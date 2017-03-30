/*
 *  Copyright (C) 2007 Austin Robot Technology, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/**
 *  @file
 *
 *  Velodyne 3D LIDAR data accessor class implementation.
 *
 *  Class for unpacking raw Velodyne LIDAR messages into useful
 *  formats.
 *
 *  Derived classes accept raw Velodyne data for either single packets
 *  or entire rotations, and provide it in various formats for either
 *  on-line or off-line processing.
 *
 */

#include "../include/velodyne16/rawdata.h"

namespace velodyne16_rawdata {
////////////////////////////////////////////////////////////////////////
//
// RawData base class implementation
//
////////////////////////////////////////////////////////////////////////

RawData::RawData()
    : timestamp_strongest_(0),
      timestamp_last_(0),
      tf_listener_(NULL),
      prev_pkt_time_(0),
      prev_pkt_seq_(-1),
      prev_azimuth_strongest_(-1),
      prev_azimuth_last_(-1) {
}

/** Update parameters: conversions and update */
void RawData::setParameters(double min_range,
                            double max_range) {
  config_.min_range = min_range;
  config_.max_range = max_range;
}

/** Set up for on-line operation. */
int RawData::setup(ros::NodeHandle private_nh, tf::TransformListener *tf_listener) {
  // whether or not to use timestamp
  private_nh.param("cloud_with_stamp", config_.cloud_with_stamp, false);
  // get path to angles.config file for this device
  if (!private_nh.getParam("calibration", config_.calibrationFile)) {
    ROS_ERROR_STREAM("No calibration angles specified! Using test values!");

    // have to use something: grab unit test version as a default
    std::string pkgPath = ros::package::getPath("velodyne16");
    config_.calibrationFile = pkgPath + "/config/calibration.yaml";
  }

  ROS_INFO_STREAM("correction angles: " << config_.calibrationFile);

  calibration_.read(config_.calibrationFile);
  if (!calibration_.initialized) {
    ROS_ERROR_STREAM("Unable to open calibration file: " <<
                                                         config_.calibrationFile);
    return -1;
  }

  ROS_INFO_STREAM("Number of lasers: " << calibration_.num_lasers << ".");
  if (calibration_.num_lasers != 16)
    ROS_ERROR_STREAM("Number of lasers in calibration file is not equal to 16.");

  // Set up cached values for sin and cos of all the possible headings
  for (uint16_t rot_index = 0; rot_index < ROTATION_MAX_UNITS; ++rot_index) {
    float rotation = angles::from_degrees(ROTATION_RESOLUTION * rot_index);
    cos_rot_table_[rot_index] = cosf(rotation);
    sin_rot_table_[rot_index] = sinf(rotation);
  }

  tf_listener_ = tf_listener;

  points_last_.resize(VLP16_SCANS_PER_FIRING);
  points_strongest_.resize(VLP16_SCANS_PER_FIRING);
  prev_last_.resize(SCANS_PER_BLOCK);

  return 0;
}

void RawData::unpack_vlp16(const velodyne16::VelodynePacket::ConstPtr &packetMsg,
                           ros::Publisher &last,
                           ros::Publisher &strongest) {
  // Check sequence of packages
  ros::Time pkt_time = packetMsg->header.stamp;
  // Check sequence id of packages
  int pkt_seq = packetMsg->header.seq;
  if (prev_pkt_time_ > pkt_time)
    ROS_WARN("The timestamp of packet is smaller than the previous packet timestamp.");
  prev_pkt_time_ = pkt_time;

  if (pkt_seq - prev_pkt_seq_ != 1 && prev_pkt_seq_ != -1)
    ROS_WARN("The sequence of packet is not different by 1. prev_pkt_seq: %i, pkt_seq: %i", prev_pkt_seq_, pkt_seq);
  prev_pkt_seq_ = pkt_seq;

  float azimuth;
  float azimuth_diff; // azimuth(N+2)-azimuth(N) with N ... number of firing in packet
  float last_azimuth_diff = 0.0;
  float azimuth_corrected_f;
  int azimuth_corrected;
  float x, y, z;
  float intensity;

  const raw_packet_vlp16_t *raw = (const raw_packet_vlp16_t *) &packetMsg->data[0];

  // Read the factory bytes to find out whether the sensor is in dual return mode.
  const bool dual_return = (raw->return_type == 0x39);

  // Calculate the index step to the next block with new azimuth value.
  // The index step depends on whether the sensor runs in single or
  // dual return mode.
  int i_diff = 1 + (int) dual_return;

  if (timestamp_last_.toSec() == 0.0) // Initialization at the start of the node
  {
    frame_id_ = packetMsg->header.frame_id;
    timestamp_last_ = packetMsg->header.stamp;
    // Calculate the amount of ns that will be lost during conversion to µs
    time_offset_last_ = (uint32_t)(timestamp_last_.nsec%1000ull);
  }

  if (dual_return && timestamp_strongest_.toSec() == 0.0) // Initialization at the start of the node
  {
    timestamp_strongest_ = packetMsg->header.stamp + ros::Duration((VLP16_BLOCK_TDURATION) * 1.0e-6);
    // Calculate the amount of ns that will be lost during conversion to µs
    time_offset_strongest_ = (uint32_t)(timestamp_strongest_.nsec%1000ull);
  }

  VPoint nan_point;
  nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.intensity = 0u;
  nan_point.time_offset_ns = 0;

  // Process each block.
  for (int block = 0; block < BLOCKS_PER_PACKET; block++) {

    // Reset previous last points
    if (dual_return && (block % 2 == 0)) std::fill(prev_last_.begin(), prev_last_.end(), nan_point);

    // Sanity check: ignore packets with mangled or otherwise different contents.
    if (UPPER_BANK != raw->blocks[block].header) {
      // Do not flood the log with messages, only issue at most one
      // of these warnings per second.
      ROS_WARN_STREAM_THROTTLE(LOG_PERIOD_, "skipping invalid VLP-16 packet: block "
          << block << " header value is "
          << raw->blocks[block].header);

      if (dual_return && (block % 2 != 0)) // Strongest return block
      {
        for (int firing = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
          for (int i = 0; i < VLP16_SCANS_PER_FIRING; ++i)
            points_strongest_[i].push_back(nan_point);
      } else { // Last return block
        for (int firing = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++)
          for (int i = 0; i < VLP16_SCANS_PER_FIRING; ++i)
            points_last_[i].push_back(nan_point);
      }

      continue;
    }

    // Calculate difference between current and next block's azimuth angle.
    azimuth = (float) (raw->blocks[block].rotation);
    if (block < (BLOCKS_PER_PACKET - i_diff) && UPPER_BANK == raw->blocks[block + i_diff].header) {
      azimuth_diff = (float) ((36000 + raw->blocks[block + i_diff].rotation
          - raw->blocks[block].rotation) % 36000);
      last_azimuth_diff = azimuth_diff;
    } else {
      azimuth_diff = last_azimuth_diff;
    }

    // Process each firing.
    for (int firing = 0, k = 0; firing < VLP16_FIRINGS_PER_BLOCK; firing++) {
      for (int dsr = 0; dsr < VLP16_SCANS_PER_FIRING; dsr++, k += RAW_SCAN_SIZE) {
        // Time of beam firing w.r.t. beginning of block in [µs].
        float t_beam = dsr * VLP16_DSR_TOFFSET + firing * VLP16_FIRING_TOFFSET;

        /** correct for the laser rotation as a function of timing during the firings **/
        azimuth_corrected_f = azimuth + (azimuth_diff * t_beam / VLP16_BLOCK_TDURATION);
        azimuth_corrected = ((int) round(azimuth_corrected_f)) % 36000;

        // Publish if we received the whole scan
        if (dual_return && (block % 2 != 0)) {
          if (azimuth_corrected_f < prev_azimuth_strongest_) {
            velodyne16_rawdata::VPointCloud::Ptr outMsgStrongest(new velodyne16_rawdata::VPointCloud());
            // Convert ros time [ns] to pcl time [µs]
            outMsgStrongest->header.stamp = timestamp_strongest_.toNSec() / 1000ull;
            outMsgStrongest->header.frame_id = frame_id_;

            // Determine the max size of points
            unsigned long max_size = points_strongest_[0].size();
            for (unsigned int i = 1; i < points_strongest_.size(); i++)
              max_size = std::max < unsigned
            long > (points_strongest_[i].size(), max_size);

            // Initialize the organized output point cloud.
            outMsgStrongest->width = max_size;
            outMsgStrongest->height = calibration_.num_lasers;
            outMsgStrongest->points.resize(outMsgStrongest->width * outMsgStrongest->height, nan_point);

            for (unsigned int i = 0; i < points_strongest_.size(); i++) {
              outMsgStrongest->points.insert(
                  outMsgStrongest->points.begin() + i * outMsgStrongest->width,
                  std::make_move_iterator(points_strongest_[i].begin()),
                  std::make_move_iterator(points_strongest_[i].end()));
            }

            strongest.publish(outMsgStrongest);

            // Set the header data for next pointcloud.
            frame_id_ = packetMsg->header.frame_id;

            timestamp_strongest_ =
                packetMsg->header.stamp + ros::Duration((block * VLP16_BLOCK_TDURATION + t_beam) * 1.0e-6);
            // Calculate the amount of ns that is lost during conversion to µs
            time_offset_strongest_ = (uint32_t)(timestamp_strongest_.nsec%1000ull);

            points_strongest_.clear();
            points_strongest_.resize(VLP16_SCANS_PER_FIRING);
          }
          prev_azimuth_strongest_ = azimuth_corrected_f;
        } else {
          if (azimuth_corrected_f < prev_azimuth_last_) {
            velodyne16_rawdata::VPointCloud::Ptr outMsgLast(new velodyne16_rawdata::VPointCloud());
            // Convert ros time [ns] to pcl time [µs]
            outMsgLast->header.stamp = timestamp_last_.toNSec() / 1000ull;
            outMsgLast->header.frame_id = frame_id_;

            // Determine the max size of points
            unsigned max_size = points_last_[0].size();
            for (unsigned int i = 1; i < points_last_.size(); i++)
              max_size = std::max<unsigned>(points_last_[i].size(), max_size);

            // Initialize the organized output point cloud.
            outMsgLast->width = max_size;
            outMsgLast->height = calibration_.num_lasers;
            outMsgLast->points.resize(outMsgLast->width * outMsgLast->height, nan_point);

            for (unsigned int i = 0; i < points_last_.size(); i++) {
              outMsgLast->points.insert(
                  outMsgLast->points.begin() + i * outMsgLast->width,
                  std::make_move_iterator(points_last_[i].begin()),
                  std::make_move_iterator(points_last_[i].end()));
            }

            last.publish(outMsgLast);

            // Set the header data for next pointcloud.
            frame_id_ = packetMsg->header.frame_id;

            timestamp_last_ =
                packetMsg->header.stamp + ros::Duration((block * VLP16_BLOCK_TDURATION + t_beam) * 1.0e-6);
            time_offset_last_ = (uint32_t)(timestamp_last_.nsec%1000ull);

            points_last_.clear();
            points_last_.resize(VLP16_SCANS_PER_FIRING);
          }
          prev_azimuth_last_ = azimuth_corrected_f;
        }

        velodyne16::LaserCorrection &corrections =
            calibration_.laser_corrections[dsr];

        /** Position Calculation */
        union two_bytes tmp;
        tmp.bytes[0] = raw->blocks[block].data[k];
        tmp.bytes[1] = raw->blocks[block].data[k + 1];

        // convert polar coordinates to Euclidean XYZ
        float distance = tmp.uint * DISTANCE_RESOLUTION;
        distance += corrections.dist_correction;

        float cos_vert_angle = corrections.cos_vert_correction;
        float sin_vert_angle = corrections.sin_vert_correction;
        float cos_rot_correction = corrections.cos_rot_correction;
        float sin_rot_correction = corrections.sin_rot_correction;

        // cos(a-b) = cos(a)*cos(b) + sin(a)*sin(b)
        // sin(a-b) = sin(a)*cos(b) - cos(a)*sin(b)
        float cos_rot_angle =
            cos_rot_table_[azimuth_corrected] * cos_rot_correction +
                sin_rot_table_[azimuth_corrected] * sin_rot_correction;
        float sin_rot_angle =
            sin_rot_table_[azimuth_corrected] * cos_rot_correction -
                cos_rot_table_[azimuth_corrected] * sin_rot_correction;

        float horiz_offset = corrections.horiz_offset_correction;
        float vert_offset = corrections.vert_offset_correction;

        // Compute the distance in the xy plane (w/o accounting for rotation)
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        float xy_distance = distance * cos_vert_angle - vert_offset * sin_vert_angle;

        // Calculate temporal X, use absolute value.
        float xx = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;
        // Calculate temporal Y, use absolute value
        float yy = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;
        if (xx < 0) xx = -xx;
        if (yy < 0) yy = -yy;

        // Get 2points calibration values,Linear interpolation to get distance
        // correction for X and Y, that means distance correction use
        // different value at different distance
        float distance_corr_x = 0;
        float distance_corr_y = 0;
        if (corrections.two_pt_correction_available) {
          distance_corr_x =
              (corrections.dist_correction - corrections.dist_correction_x)
                  * (xx - 2.4) / (25.04 - 2.4)
                  + corrections.dist_correction_x;
          distance_corr_x -= corrections.dist_correction;
          distance_corr_y =
              (corrections.dist_correction - corrections.dist_correction_y)
                  * (yy - 1.93) / (25.04 - 1.93)
                  + corrections.dist_correction_y;
          distance_corr_y -= corrections.dist_correction;
        }

        float distance_x = distance + distance_corr_x;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance = distance_x * cos_vert_angle - vert_offset * sin_vert_angle;
        x = xy_distance * sin_rot_angle - horiz_offset * cos_rot_angle;

        float distance_y = distance + distance_corr_y;
        /**the new term of 'vert_offset * sin_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        xy_distance = distance_y * cos_vert_angle - vert_offset * sin_vert_angle;
        y = xy_distance * cos_rot_angle + horiz_offset * sin_rot_angle;

        // Using distance_y is not symmetric, but the velodyne manual
        // does this.
        /**the new term of 'vert_offset * cos_vert_angle'
         * was added to the expression due to the mathemathical
         * model we used.
         */
        z = distance_y * sin_vert_angle + vert_offset * cos_vert_angle;

        /** Use standard ROS coordinate system (right-hand rule) */
        float x_coord = y;
        float y_coord = -x;
        float z_coord = z;

        /** Intensity Calculation */
        float min_intensity = corrections.min_intensity;
        float max_intensity = corrections.max_intensity;

        intensity = raw->blocks[block].data[k + 2];

        float focal_offset = 256
            * (1 - corrections.focal_distance / 13100)
            * (1 - corrections.focal_distance / 13100);
        float focal_slope = corrections.focal_slope;
        intensity += focal_slope * (abs(focal_offset - 256 *
            (1 - tmp.uint / 65535) * (1 - tmp.uint / 65535)));
        intensity = (intensity < min_intensity) ? min_intensity : intensity;
        intensity = (intensity > max_intensity) ? max_intensity : intensity;

        // Insert this point into the cloud.
        VPoint point;
        point.x = point.y = point.z = std::numeric_limits<float>::quiet_NaN();
        point.intensity = 0u;
        point.time_offset_ns = 0;
        if (config_.cloud_with_stamp) {
          ros::Time point_timestamp = packetMsg->header.stamp + ros::Duration((block * VLP16_BLOCK_TDURATION + t_beam) * 1.0e-6);
          if (dual_return && (block % 2 != 0)) {
            point.time_offset_ns = (point_timestamp - timestamp_strongest_).nsec + time_offset_strongest_;
          } else {
            point.time_offset_ns = (point_timestamp - timestamp_last_).nsec + time_offset_last_;
          }
        }

        // Compute the row index of the point.
        int row = calibration_.num_lasers - 1 - corrections.laser_ring;

        if (!pointInRange(distance)) {
          if (dual_return && (block % 2 != 0)) // Strongest
            points_strongest_[row].push_back(point);
          else                                 // Last
            points_last_[row].push_back(point);

          continue;
        }

        if (dual_return && (block % 2 != 0)) {
          //Strongest return, check the point and leave it nan if it is close to last point
          VPoint prev_last_point = prev_last_[VLP16_SCANS_PER_FIRING * firing + row];
          if (!(std::fabs(prev_last_point.x - x_coord) < STRONGEST_TRESH_) &&
              !(std::fabs(prev_last_point.y - y_coord) < STRONGEST_TRESH_) &&
              !(std::fabs(prev_last_point.z - z_coord) < STRONGEST_TRESH_)) {
            point.x = x_coord;
            point.y = y_coord;
            point.z = z_coord;
            point.intensity = (uint8_t) intensity;
          }
          points_strongest_[row].push_back(point);
        } else {
          // Last return
          point.x = x_coord;
          point.y = y_coord;
          point.z = z_coord;
          point.intensity = (uint8_t) intensity;
          points_last_[row].push_back(point);
          if (dual_return) prev_last_[VLP16_SCANS_PER_FIRING * firing + row] = point;
        }
      } // Iterate over beams
    } // Iterate over firings
  } // Iterate over blocks
}

} // namespace velodyne16_rawdata
  