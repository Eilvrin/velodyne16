/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 * 
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver interface for the Velodyne 3D LIDARs
 */

#ifndef _VELODYNE_DRIVER_H_
#define _VELODYNE_DRIVER_H_

// Stdlib
#include <arpa/inet.h>
#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE
#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <string>
#include <sys/socket.h>

// ROS
#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <tf/transform_listener.h>


// package internal
#include <velodyne16/DriverNodeConfig.h>
#include <velodyne16/VelodynePacket.h>

class VelodyneDriver {
 public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  virtual ~VelodyneDriver();

  bool publishPacket(void);

 private:

  // Callback for dynamic reconfigure
  void callback(velodyne16::DriverNodeConfig &config,
                uint32_t level);
  bool getPacket(velodyne16::VelodynePacketPtr pkt, const double time_offset);
  uint32_t getPacketTime(velodyne16::VelodynePacketPtr pkt);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne16::DriverNodeConfig> > srv_;

  std::string frame_id_;            ///< tf frame ID
  double time_offset_;              ///< time in seconds added to each velodyne time stamp

  int port_;
  std::string devip_str_;
  int sockfd_;
  in_addr devip_;

  ros::Publisher output_;

  uint32_t previous_pkt_time;

};

#endif // _VELODYNE_DRIVER_H_
