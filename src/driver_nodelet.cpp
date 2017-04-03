/*
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2017 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** \file
 *
 *  ROS driver nodelet for the Velodyne 3D LIDARs
 */

#include <string>
#include <boost/thread.hpp>

#include <ros/ros.h>
#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "../include/velodyne16/driver.h"

namespace velodyne16 {

class DriverNodelet : public nodelet::Nodelet {
 public:

  DriverNodelet() :
      running_(false) {}

  virtual ~DriverNodelet() {
    if (running_) {
      NODELET_INFO("shutting down driver thread");
      running_ = false;
      deviceThread_->join();
      NODELET_INFO("driver thread stopped");
    }
  }

 private:

  virtual void onInit(void);
  virtual void devicePoll(void);

  volatile bool running_;               ///< device thread is running
  boost::shared_ptr<boost::thread> deviceThread_;

  boost::shared_ptr<VelodyneDriver> dvr_; ///< driver implementation class
};

void DriverNodelet::onInit() {
  // start the driver
  dvr_.reset(new VelodyneDriver(getNodeHandle(), getPrivateNodeHandle()));
  // spawn device poll thread
  running_ = true;
  deviceThread_ = boost::shared_ptr<boost::thread>
      (new boost::thread(boost::bind(&DriverNodelet::devicePoll, this)));
}

/** @brief Device poll thread main loop. */
void DriverNodelet::devicePoll() {
  while (ros::ok()) {
    running_ = dvr_->publishPacket();
    if (!running_)
      break;
  }
  running_ = false;
}

} // namespace velodyne16

// Register this plugin with pluginlib
//
// parameters are: package, class name, class type, base class type
PLUGINLIB_DECLARE_CLASS(velodyne16, DriverNodelet,
                        velodyne16::DriverNodelet, nodelet::Nodelet);