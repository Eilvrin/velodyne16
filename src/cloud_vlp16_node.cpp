/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
    This ROS node converts raw Velodyne LIDAR packets to PointCloud2.
*/

#include "../include/velodyne16/convert.h"

/** Main node entry point. */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "cloud_vlp16_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to raw data
  Convert conv(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}