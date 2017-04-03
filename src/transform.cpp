/*
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class applies tf transform to the Velodyne 16 point clouds.

*/
#include "../include/velodyne16/transform.h"

namespace velodyne16 {

/** @brief Constructor. */
Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh)
    : prev_msg_seq_strongest_(-1), prev_msg_seq_last_(-1) {
  // advertise output point cloud (before subscribing to input data)
  output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points_transformed/last", 10);
  output2_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points_transformed/strongest", 10);

  // The world coordinate frame
  private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("world"));

  velodyne_points_last_ =
      node.subscribe("velodyne16/points/last", 10, &Transform::processScansLast, this);
  velodyne_points_strongest_ =
      node.subscribe("velodyne16/points/strongest", 10, &Transform::processScansStrongest, this);
}

/** @brief Callback for messages. */
void Transform::processScansLast(const velodyne16_rawdata::VPointCloud::ConstPtr &pc) {
  // Discard messages received in the first seconds after startup, as tf might not be available
  static ros::Time start_time_ = ros::Time::now();
  if ((pcl_conversions::fromPCL(pc->header.stamp) - start_time_).toSec() < WAIT_FOR_TF_AFTER_START_) return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr outMsg(new pcl::PointCloud<pcl::PointXYZI>());
  processScan(pc, outMsg);
  output_.publish(outMsg);
}

/** @brief Callback for messages. */
void Transform::processScansStrongest(const velodyne16_rawdata::VPointCloud::ConstPtr &pc) {
  // Discard messages received in the first seconds after startup, as tf might not be available
  static ros::Time start_time_ = ros::Time::now();
  if ((pcl_conversions::fromPCL(pc->header.stamp) - start_time_).toSec() < WAIT_FOR_TF_AFTER_START_) return;

  pcl::PointCloud<pcl::PointXYZI>::Ptr outMsg(new pcl::PointCloud<pcl::PointXYZI>());
  processScan(pc, outMsg);
  output2_.publish(outMsg);
}

void Transform::processScan(const velodyne16_rawdata::VPointCloud::ConstPtr &pc,
                            pcl::PointCloud<pcl::PointXYZI>::Ptr &outMsg) {

  outMsg->header.stamp = pc->header.stamp;
  std::string frame_id = pc->header.frame_id;
  outMsg->header.frame_id = frame_id;
  outMsg->width = pc->width;
  outMsg->height = pc->height;

  pcl::PointXYZI nan_point;
  nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<float>::quiet_NaN();
  nan_point.intensity = 0u;
  outMsg->points.resize(outMsg->width * outMsg->height, nan_point);

  ros::Time target_time;
  target_time.fromNSec(pc->header.stamp * 1000ull);

  ros::Time timestamp_end_point;
  ros::Duration offset_end_point;
  offset_end_point.nsec = pc->at(pc->width - 1, 0).time_offset_ns; // height=0, because upper laser measures last
  timestamp_end_point = target_time + offset_end_point;

  // Check, whether time offset is set
  if (offset_end_point.nsec == 0) {
    ROS_ERROR_STREAM(
        "The timestamp of the last point in the point cloud is not set. Check that cloud_node provides timestamps.");
    return;
  }

  try {
    listener_.waitForTransform(frame_id, target_time,
                               frame_id, timestamp_end_point,
                               fixed_frame_id_, ros::Duration(1.0));
  } catch (std::exception &ex) {
    ROS_WARN_THROTTLE(LOG_PERIOD_, "%s", ex.what());
  }

  for (unsigned int i = 0; i < pc->points.size(); ++i) {
    if (!std::isnan(pc->points[i].x) || !std::isnan(pc->points[i].y) || !std::isnan(pc->points[i].z)) {
      ros::Time timestamp_t_point;
      ros::Duration offset_t_point;
      offset_t_point.nsec = pc->points[i].time_offset_ns;
      timestamp_t_point = target_time + offset_t_point;

      geometry_msgs::PointStamped t_point;
      t_point.header.frame_id = frame_id;
      t_point.header.stamp = timestamp_t_point;
      t_point.point.x = pc->points[i].x;
      t_point.point.y = pc->points[i].y;
      t_point.point.z = pc->points[i].z;

      try {
        listener_.transformPoint(frame_id, target_time, t_point, fixed_frame_id_, t_point);
      } catch (std::exception &ex) {
        ROS_WARN_THROTTLE(LOG_PERIOD_, "%s", ex.what());
        continue;
      }
      outMsg->points[i].x = t_point.point.x;
      outMsg->points[i].y = t_point.point.y;
      outMsg->points[i].z = t_point.point.z;
      outMsg->points[i].intensity = pc->points[i].intensity;
    }
  }
}

} // namespace velodyne16