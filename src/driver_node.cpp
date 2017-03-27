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
 *  ROS driver node for the Velodyne 3D LIDARs.
 */

#include "../include/velodyne16/driver.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "driver_node");
  ros::NodeHandle node;
  ros::NodeHandle private_nh("~");

  // start the driver
  VelodyneDriver dvr(node, private_nh);

  // loop until shut down
  while (ros::ok() && dvr.publishPacket()) {
    ros::spinOnce();
  }

  return 0;
}
