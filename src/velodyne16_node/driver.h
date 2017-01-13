/* -*- mode: C++ -*- */
/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
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

#include <ros/ros.h>
#include <string>
 
#include <dynamic_reconfigure/server.h>
#include <velodyne16/Velodyne16NodeConfig.h>

#include <tf/transform_listener.h>
#include <velodyne16/VelodynePacket.h>

// Socket headers 
#include <sys/socket.h>
#include <arpa/inet.h>
#include <poll.h>
#include <fcntl.h>
#include <errno.h>

#include <cstdlib> // EXIT_SUCCESS, EXIT_FAILURE


static uint16_t DATA_PORT_NUMBER = 2368;

class VelodyneDriver
{
  public:

  VelodyneDriver(ros::NodeHandle node,
                 ros::NodeHandle private_nh);
  ~VelodyneDriver();
  
  bool publishPacket(void);

  private:

  // Callback for dynamic reconfigure
  void callback(velodyne16::Velodyne16NodeConfig &config,
              uint32_t level);
  bool getPacket(velodyne16::VelodynePacketPtr pkt, const double time_offset);
  uint32_t getPacketTime(velodyne16::VelodynePacketPtr pkt);

  // Pointer to dynamic reconfigure service srv_
  boost::shared_ptr<dynamic_reconfigure::Server<velodyne16::Velodyne16NodeConfig> > srv_;

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
