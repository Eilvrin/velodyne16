/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

// Stdlib
#include <algorithm>
#include <cmath>
#include <fstream>
#include <string>
#include <vector>

// ROS
#include <angles/angles.h>
#include <ros/package.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>

// Package internal
#include <velodyne16/VelodynePacket.h>
#include "calibration.h"
#include "point_types.h"

namespace velodyne16 {


/** Log rate for throttled warnings and errors in seconds */
static const double LOG_PERIOD_ = 1.0;

/**
* Raw Velodyne packet constants and structures.
*/
static const float ROTATION_RESOLUTION = 0.01f;  // [deg]
static const uint16_t ROTATION_MAX_UNITS = 36000u;     // [deg/100]
static const float DISTANCE_RESOLUTION = 0.002f; // [m]

static const int RAW_SCAN_SIZE = 3;
static const int SCANS_PER_BLOCK = 32;
static const int BLOCK_DATA_SIZE = (SCANS_PER_BLOCK * RAW_SCAN_SIZE);

static const int BLOCKS_PER_PACKET = 12;

static const uint16_t UPPER_BANK = 0xeeff;

static const int VLP16_FIRINGS_PER_BLOCK = 2;
static const int VLP16_SCANS_PER_FIRING = 16;
static const float VLP16_BLOCK_TDURATION = 110.592f;   // [µs]
static const float VLP16_DSR_TOFFSET = 2.304f;   // [µs]
static const float VLP16_FIRING_TOFFSET = 55.296f;   // [µs]

/** Treshold for strongest return. */
static const double STRONGEST_TRESH_ = 0.001;

/** \brief Raw Velodyne data block.
 *
 *  Each block contains data from either the upper or lower laser
 *  bank.  The device returns three times as many upper bank blocks.
 *
 *  use stdint.h types, so things work with both 64 and 32-bit machines
 */
typedef struct raw_block {
  uint16_t header;        ///< UPPER_BANK or LOWER_BANK
  uint16_t rotation;      ///< 0-35999, divide by 100 to get degrees
  uint8_t data[BLOCK_DATA_SIZE];
} raw_block_t;

/** used for unpacking the first two data bytes in a block
 *
 *  They are packed into the actual data stream misaligned.  I doubt
 *  this works on big endian machines.
 */
union two_bytes {
  uint16_t uint;
  uint8_t bytes[2];
};

typedef struct raw_packet_vlp16 {
  raw_block_t blocks[BLOCKS_PER_PACKET];
  uint32_t time;        // Time in microseconds from startup. Probably overshoots each hour.
  uint8_t return_type;  // 37 - strongest return; 38 - last return; 39 - dual return
  uint8_t data_source;  // 21 for HDL-32E or 22 for VLP-16
} raw_packet_vlp16_t;

/** \brief Velodyne data conversion class */
class RawData {
 public:

  RawData();
  virtual ~RawData() {}

  /** \brief Set up for data processing.
   *
   *  Perform initializations needed before data processing can
   *  begin:
   *
   *    - read device-specific angles calibration
   *
   *  @param private_nh private node handle for ROS parameters
   *  @returns 0 if successful;
   *           errno value for failure
   */
  int setup(ros::NodeHandle private_nh);

  void setParameters(double min_range, double max_range);

  /** @brief convert raw Velodyne packets to point cloud
   *
   *  @param packetMsg raw Velodyne scan message
   *  @param pc shared pointer to organized point cloud
   */
  void unpack_vlp16(const velodyne16::VelodynePacket::ConstPtr &packetMsg,
                    ros::Publisher &last,
                    ros::Publisher &strongest);

 private:
  /** configuration parameters */
  typedef struct {
    std::string calibrationFile;     ///< calibration file name
    double max_range;                ///< maximum range to publish
    double min_range;                ///< minimum range to publish
    bool cloud_with_timestamp;           ///< Whether or not publish each point with the timestamp
  } Config;
  Config config_;

  ros::Time timestamp_strongest_;
  ros::Time timestamp_last_;
  uint32_t time_offset_strongest_ns_;       // Variables to account for loss of precision when converting to pcl type
  uint32_t time_offset_last_ns_; //TODO Timing for strongest and last identical, rewrite with one time_offset and timestamp
  std::string frame_id_;

  /**
   * Calibration file
   */
  velodyne16::Calibration calibration_;
  float sin_rot_table_[ROTATION_MAX_UNITS];
  float cos_rot_table_[ROTATION_MAX_UNITS];

  std::ofstream file_;

  ros::Time prev_pkt_time_; // previous timestamp
  int prev_pkt_seq_;

  float prev_azimuth_strongest_; // previous asymuth for strongest return
  float prev_azimuth_last_; // previous asymuth for last return

  VPointCloud out_cloud_last_;
  VPointCloud out_cloud_strongest_;

  std::vector<std::vector<VPoint> > points_last_;
  std::vector<std::vector<VPoint> > points_strongest_;

  std::vector<VPoint> prev_last_; // Points to compare to strongest return in order to set similar points to NAN

  /** in-line test whether a point is in range */
  bool pointInRange(float range) {
    return (range >= config_.min_range
        && range <= config_.max_range);
  }
};

} // namespace velodyne16_rawdata

#endif // __VELODYNE_RAWDATA_H
