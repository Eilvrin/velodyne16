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

#include <dynamic_reconfigure/server.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include "rawdata.h"
#include <velodyne16/CloudNodeConfig.h>

class Convert
{
public:

  Convert(ros::NodeHandle node, ros::NodeHandle private_nh);
  virtual ~Convert() {}

private:
  
  void callback(velodyne16::CloudNodeConfig &config,
                uint32_t level);
  void processPackets(const velodyne16::VelodynePacket::ConstPtr &packetMsg);

  ///Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne16::
    CloudNodeConfig> > srv_;
    
  boost::shared_ptr<velodyne16_rawdata::RawData> data_;
  ros::Subscriber velodyne_packet_;
  ros::Publisher output_;
  ros::Publisher output2_;
};


#endif // _VELODYNE_POINTCLOUD_CONVERT_H_
