/*
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
    This ROS node applies tf transform to the Velodyne 16 point clouds.
*/

#include "../include/velodyne16/transform.h"

/** Main node entry point. */
int main(int argc, char **argv) {
  ros::init(argc, argv, "transform_node");
  ros::NodeHandle node;
  ros::NodeHandle priv_nh("~");

  // create conversion class, which subscribes to the point clouds
  Transform trans(node, priv_nh);

  // handle callbacks until shut down
  ros::spin();

  return 0;
}