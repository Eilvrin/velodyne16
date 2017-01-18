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

#include "convert.h"

  /** @brief Constructor. */
  Convert::Convert(ros::NodeHandle node, ros::NodeHandle private_nh):
    data_(new velodyne16_rawdata::RawData())
  {
    data_->setup(private_nh);

    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points/last", 10);
    output2_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points/strongest", 10);

    srv_ = boost::make_shared <dynamic_reconfigure::Server<velodyne16::
      CloudVLP16NodeConfig> > (private_nh);
    dynamic_reconfigure::Server<velodyne16::CloudVLP16NodeConfig>::
      CallbackType f;
    f = boost::bind (&Convert::callback, this, _1, _2);
    srv_->setCallback (f);

    // subscribe to Velodyne packets
    velodyne_packet_ =
      node.subscribe("velodyne16/packets", 10,
                     &Convert::processPackets, (Convert *) this,
                     ros::TransportHints().tcpNoDelay(true));
  }

  void Convert::callback(velodyne16::CloudVLP16NodeConfig &config,
                        uint32_t level)
  {
  ROS_INFO("Reconfigure request.");
  data_->setParameters(config.min_range, config.max_range, config.view_direction,
                       config.view_width);
  }

  /** @brief Callback for raw packet messages. */
  void Convert::processPackets(const velodyne16::VelodynePacket::ConstPtr &packetMsg)
  {
    if (output_.getNumSubscribers() == 0)         // no one listening?
      return;                                     // avoid much work

    // allocate a point cloud 
    velodyne16_rawdata::VPointCloud::Ptr outMsg(new velodyne16_rawdata::VPointCloud());
    velodyne16_rawdata::VPointCloud::Ptr outMsg2(new velodyne_rawdata::VPointCloud());

    // process all packets provided by the driver
    //data_->unpack(scanMsg, *outMsg);
    std::cout<<"Process packet" << std::endl;

    // publish the cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg->height << " x " << outMsg->width
                     << " Velodyne points, time: " << outMsg->header.stamp);
    output_.publish(outMsg);

    // publish the cloud message
    ROS_DEBUG_STREAM("Publishing " << outMsg2->height << " x " << outMsg2->width
                     << " Velodyne points, time: " << outMsg2->header.stamp);
    output2_.publish(outMsg2);
  }