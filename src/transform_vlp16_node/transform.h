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

#include <velodyne16/rawdata.h>
#include <velodyne16/point_types.h>

#include <dynamic_reconfigure/server.h>
#include <velodyne16/TransformVLP16NodeConfig.h>

#include <tf/transform_listener.h>

class Transform
{
public:

  Transform(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Transform() {}

private:
  
  void callback(velodyne16::TransformVLP16NodeConfig &config,
                uint32_t level);
  void processScansLast(const velodyne16_rawdata::VPointCloud::ConstPtr &scanMsg);
  void processScansStrongest(const velodyne16_rawdata::VPointCloud::ConstPtr &scanMsg);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne16::
    TransformVLP16NodeConfig> > srv_;
    
  // boost::shared_ptr<velodyne16_rawdata::RawData> data_;
  ros::Subscriber velodyne_points_last_;
  ros::Subscriber velodyne_points_strongest_;
  ros::Publisher output_;
  ros::Publisher output2_;

  tf::TransformListener listener_;

  std::string fixed_frame_id_;     ///<  fixed frame for tf transform

  int prev_msg_seq_strongest_;
  int prev_msg_seq_last_;
};


#endif // _VELODYNE_POINTCLOUD_TRANSFORM_H_