/*
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class applies tf transform to the Velodyne 16 point clouds.

*/

#ifndef _VELODYNE_POINTCLOUD_TRANSFORM_H_
#define _VELODYNE_POINTCLOUD_TRANSFORM_H_

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include "point_types.h"

/** Log rate for throttled warnings and errors in seconds */
static const double LOG_PERIOD_ = 1.0;
/** Time to wait after startup of the node*/
static const float WAIT_FOR_TF_AFTER_START_ = 3.0;

namespace velodyne16 {

class Transform {
 public:

  Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
  virtual ~Transform() {}

 private:

  void processScansLast(const velodyne16::VPointCloud::ConstPtr &pc);
  void processScansStrongest(const velodyne16::VPointCloud::ConstPtr &pc);
  void processScan(const velodyne16::VPointCloud::ConstPtr &pc,
                   pcl::PointCloud<pcl::PointXYZI>::Ptr &outMsg);

  ros::Subscriber velodyne_points_last_;
  ros::Subscriber velodyne_points_strongest_;
  ros::Publisher output_;
  ros::Publisher output2_;

  tf::TransformListener listener_;

  std::string fixed_frame_id_;     ///<  fixed frame for tf transform

  int prev_msg_seq_strongest_;
  int prev_msg_seq_last_;

};

} // namespace velodyne16

#endif // _VELODYNE_POINTCLOUD_TRANSFORM_H_