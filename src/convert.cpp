/*
 *  Copyright (C) 2009, 2010 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2011 Jesse Vera
 *  Copyright (C) 2012 Austin Robot Technology, Jack O'Quin
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class converts raw Velodyne 3D LIDAR packets to PointCloud2.

*/

#include "../include/velodyne16/convert.h"

namespace velodyne16
{

/** @brief Constructor. */
Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh) :
    data_(new velodyne16_rawdata::RawData()) {
  data_->setup(private_nh);

  // advertise output point cloud (before subscribing to input data)
  output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points/last", 10);
  output2_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points/strongest", 10);

  srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne16::
                                                        CloudNodeConfig> >(private_nh);
  dynamic_reconfigure::Server<velodyne16::CloudNodeConfig>::
  CallbackType f;
  f = boost::bind(&Convert::callback, this, _1, _2);
  srv_->setCallback(f);

  //subscribe to Velodyne packets
  velodyne_packet_ =
      node.subscribe("velodyne16/packets", 500,
                     &Convert::processPackets, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
}

void Convert::callback(velodyne16::CloudNodeConfig &config,
                       uint32_t level) {
  ROS_INFO("Reconfigure request.");
  data_->setParameters(config.min_range, config.max_range);
}

/** @brief Callback for raw packet messages. */
void Convert::processPackets(const velodyne16::VelodynePacket::ConstPtr &packetMsg) {
  data_->unpack_vlp16(packetMsg, output_, output2_);
}

} // namespace velodyne16