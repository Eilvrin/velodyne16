/*
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class applies tf transform to the Velodyne 16 point clouds.

*/
#include "transform.h"
/** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh): prev_msg_seq_strongest_(-1), prev_msg_seq_last_(-1)
  {
    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points_transformed/last", 10);
    output2_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points_transformed/strongest", 10);

    // Set up dynamic reconfiguration.
    srv_ = boost::make_shared<dynamic_reconfigure::Server<velodyne16::
      TransformVLP16NodeConfig> >(private_nh);
    dynamic_reconfigure::Server<velodyne16::TransformVLP16NodeConfig>::CallbackType f;
    f = boost::bind (&Transform::callback, this, _1, _2);
    srv_->setCallback (f);

    velodyne_points_last_ =
      node.subscribe("velodyne16/points/last", 10, &Transform::processScansLast, this);
    velodyne_points_strongest_=
      node.subscribe("velodyne16/points/strongest", 10, &Transform::processScansStrongest, this);
  }

  void Transform::callback(velodyne16::TransformVLP16NodeConfig &config,
                        uint32_t level)
  {
    ROS_INFO("Reconfigure request.");
    fixed_frame_id_ = config.fixed_frame_id;
  }

  /** @brief Callback for messages. */
  void Transform::processScansLast(const velodyne16_rawdata::VPointCloud::ConstPtr &scanMsg)
  {
    // Check sequence id of messages
    int msg_seq_last = scanMsg->header.seq;

    if (msg_seq_last - prev_msg_seq_last_ != 1 && prev_msg_seq_last_ != -1)
      ROS_WARN("The sequence of last point clouds is not different by 1. prev_msg_seq: %i, msg_seq: %i", prev_msg_seq_last_, msg_seq_last);
    prev_msg_seq_last_ = msg_seq_last;

    velodyne16_rawdata::VPointCloud::Ptr outMsg(new velodyne16_rawdata::VPointCloud());
    outMsg->header.stamp = scanMsg->header.stamp;
    std::string frame_id = scanMsg->header.frame_id;
    outMsg->header.frame_id = frame_id;
    outMsg->width = scanMsg->width;
    outMsg->height = scanMsg->height;

    velodyne16_rawdata::VPoint nan_point;
    nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<float>::quiet_NaN();
    nan_point.intensity = 0u;
    nan_point.timestamp = -1;

    // geometry_msgs::PointStamped t_point;   
    // t_point.header.frame_id = frame_id;    
    // t_point.header.stamp =  pc->at(scanMsg->width-1, scanMsg->height-1).timestamp;   
    // t_point.point.x         = pc->at(scanMsg->width-1, scanMsg->height-1).x;    
    // t_point.point.y         = pc->at(scanMsg->width-1, scanMsg->height-1).y;    
    // t_point.point.z         = pc->at(scanMsg->width-1, scanMsg->height-1).z;

    // ros::Time target_time = 

    try {    
      tf_listener_->waitForTransform(frame_id, target_time,   
                                     frame_id, t_point.header.stamp,   
                                     fixed_frame_id_, wait_for_tf_duration);   
    } catch (std::exception& ex) {    
    // only log tf error once every second    
      ROS_WARN_THROTTLE(LOG_PERIOD_, "%s", ex.what());    
    }
    
    outMsg->points.resize(outMsg->width * outMsg->height, nan_point);
    output_.publish(outMsg);   
    
  }

    /** @brief Callback for messages. */
  void Transform::processScansStrongest(const velodyne16_rawdata::VPointCloud::ConstPtr &scanMsg)
  {
    // Check sequence id of messages
    int msg_seq_strongest = scanMsg->header.seq;

    if (msg_seq_strongest - prev_msg_seq_strongest_ != 1 && prev_msg_seq_strongest_ != -1)
      ROS_WARN("The sequence of strongest point clouds is not different by 1. prev_msg_seq: %i, msg_seq: %i", prev_msg_seq_strongest_, msg_seq_strongest);
    prev_msg_seq_strongest_ = msg_seq_strongest;

    velodyne16_rawdata::VPointCloud::Ptr outMsg(new velodyne16_rawdata::VPointCloud());
    outMsg->header.stamp = scanMsg->header.stamp;
    outMsg->header.frame_id = scanMsg->header.frame_id;
    outMsg->width = scanMsg->width;
    outMsg->height = scanMsg->height;

    velodyne16_rawdata::VPoint nan_point;
    nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<float>::quiet_NaN();
    nan_point.intensity = 0u;
    nan_point.timestamp = -1;

    outMsg->points.resize(outMsg->width * outMsg->height, nan_point);
    output2_.publish(outMsg);   
  }