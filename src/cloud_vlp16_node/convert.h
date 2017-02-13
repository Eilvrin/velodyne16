/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#ifndef _VELODYNE_POINTCLOUD_CONVERT_H_
#define _VELODYNE_POINTCLOUD_CONVERT_H_ 

#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <velodyne16/rawdata.h>
#include <velodyne16/CloudVLP16NodeConfig.h>

#include "tf/message_filter.h"
#include "message_filters/subscriber.h"


class Convert
{
public:

  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  ~Convert() {}

private:
  
  void callback(velodyne16::CloudVLP16NodeConfig &config,
                uint32_t level);
  void processPackets(const velodyne16::VelodynePacket::ConstPtr &packetMsg);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne16::
    CloudVLP16NodeConfig> > srv_;
    
  boost::shared_ptr<velodyne16_rawdata::RawData> data_;
  //ros::Subscriber velodyne_packet_;
  ros::Publisher output_;
  ros::Publisher output2_;
  tf::TransformListener listener_;

  message_filters::Subscriber<velodyne16::VelodynePacket> velodyne_packet_;
  tf::MessageFilter<velodyne16::VelodynePacket> *tf_filter_;
  
};


#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
