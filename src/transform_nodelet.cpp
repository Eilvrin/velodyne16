/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
    This ROS nodelet transforms raw Velodyne 3D LIDAR packets to a
    PointCloud2 in the given frame.
*/
#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "../include/velodyne16/transform.h"

namespace velodyne16
{
  class TransformNodelet: public nodelet::Nodelet
  {
  public:

    TransformNodelet() {}
    ~TransformNodelet() {}

  private:

    virtual void onInit();
    boost::shared_ptr<Transform> tf_;
  };

  /** @brief Nodelet initialization. */
  void TransformNodelet::onInit()
  {
    tf_.reset(new Transform(getNodeHandle(), getPrivateNodeHandle()));
  }

} // namespace velodyne16


// Register this plugin with pluginlib.
// parameters: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne16, TransformNodelet,
                        velodyne16::TransformNodelet, nodelet::Nodelet);