/*
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file

    This class applies tf transform to the Velodyne 16 point clouds.

*/
#include "../include/velodyne16/transform.h"

/** @brief Constructor. */
  Transform::Transform(ros::NodeHandle node, ros::NodeHandle private_nh): prev_msg_seq_strongest_(-1), prev_msg_seq_last_(-1)
  {
    // advertise output point cloud (before subscribing to input data)
    output_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points_transformed/last", 10);
    output2_ =
      node.advertise<sensor_msgs::PointCloud2>("velodyne16/points_transformed/strongest", 10);

    // The world coordinate frame
    private_nh.param("fixed_frame_id", fixed_frame_id_, std::string("world"));

    velodyne_points_last_ =
      node.subscribe("velodyne16/points/last", 10, &Transform::processScansLast, this);
    velodyne_points_strongest_=
      node.subscribe("velodyne16/points/strongest", 10, &Transform::processScansStrongest, this);
  }

  /** @brief Callback for messages. */
  void Transform::processScansLast(const velodyne16_rawdata::VPointCloud::ConstPtr &pc)
  {
    // Discard messages received in the first seconds after startup, as tf might not be available
    static ros::Time start_time_ = ros::Time::now();
    if ((pcl_conversions::fromPCL(pc->header.stamp) - start_time_).toSec() < WAIT_FOR_TF_AFTER_START_) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr outMsg(new pcl::PointCloud<pcl::PointXYZI>());
    processScan(pc, 0, outMsg);
    output_.publish(outMsg);   
  }

    /** @brief Callback for messages. */
  void Transform::processScansStrongest(const velodyne16_rawdata::VPointCloud::ConstPtr &pc)
  {
    // Discard messages received in the first seconds after startup, as tf might not be available
    static ros::Time start_time_ = ros::Time::now();
    if ((pcl_conversions::fromPCL(pc->header.stamp) - start_time_).toSec() < WAIT_FOR_TF_AFTER_START_) return;

    pcl::PointCloud<pcl::PointXYZI>::Ptr outMsg(new pcl::PointCloud<pcl::PointXYZI>());
    processScan(pc, 1, outMsg);
    output2_.publish(outMsg);
  }

  void Transform::processScan(const velodyne16_rawdata::VPointCloud::ConstPtr &pc, bool return_mode,
    pcl::PointCloud<pcl::PointXYZI>::Ptr &outMsg)
  {
    // Check sequence id of messages
    int msg_seq = pc->header.seq;

    if (return_mode == 0){
      if ((msg_seq - prev_msg_seq_last_) != 1 && prev_msg_seq_last_ != -1)
        ROS_WARN("The sequence of last point clouds is not different by 1. prev_msg_seq: %i, msg_seq: %i", prev_msg_seq_last_, msg_seq);
      prev_msg_seq_last_ = msg_seq;
    }
   
    if (return_mode == 1){
      if ((msg_seq - prev_msg_seq_strongest_) != 1 && prev_msg_seq_strongest_ != -1)
        ROS_WARN("The sequence of strongest point clouds is not different by 1. prev_msg_seq: %i, msg_seq: %i", prev_msg_seq_strongest_, msg_seq);
      prev_msg_seq_strongest_ = msg_seq;
    }

    outMsg->header.stamp = pc->header.stamp;
    std::string frame_id = pc->header.frame_id;
    outMsg->header.frame_id = frame_id;
    outMsg->width = pc->width;
    outMsg->height = pc->height;

    pcl::PointXYZI nan_point;
    nan_point.x = nan_point.y = nan_point.z = std::numeric_limits<float>::quiet_NaN();
    nan_point.intensity = 0u;
    outMsg->points.resize(outMsg->width * outMsg->height, nan_point);

    if (pc->at(pc->width-1, pc->height-1).timestamp == -1.0)
    {
      ROS_ERROR_STREAM("The timestamp of the last poit in the point cloud is not set. Check that cloud_vlp16_node provides timestamps.");
      return;
    }

    ros::Time timestamp_end_point;
    timestamp_end_point.fromSec(pc->at(pc->width-1, pc->height-1).timestamp); 

    ros::Time target_time = pcl_conversions::fromPCL(pc->header.stamp);

    try {    
      listener_.waitForTransform(frame_id, target_time,   
                                 frame_id, timestamp_end_point,   
                                 fixed_frame_id_, ros::Duration(1.0));   
    } catch (std::exception& ex) {       
      ROS_WARN_THROTTLE(LOG_PERIOD_, "%s", ex.what());    
    }

    for (int i =0; i<pc->points.size();++i)
    {
      if (!std::isnan(pc->points[i].x) || !std::isnan(pc->points[i].y) || !std::isnan(pc->points[i].z))
      {
        ros::Time timestamp_t_point;
        timestamp_t_point.fromSec(pc->points[i].timestamp);

        geometry_msgs::PointStamped t_point;   
        t_point.header.frame_id = frame_id; 
        t_point.header.stamp    = timestamp_t_point;  
        t_point.point.x         = pc->points[i].x;    
        t_point.point.y         = pc->points[i].y;    
        t_point.point.z         = pc->points[i].z;

        try { 
          listener_.transformPoint(frame_id, target_time, t_point, fixed_frame_id_, t_point);
        } catch (std::exception& ex) {
          ROS_WARN_THROTTLE(LOG_PERIOD_,"%s", ex.what());
          continue;
        }
        outMsg->points[i].x = t_point.point.x;
        outMsg->points[i].y = t_point.point.y;
        outMsg->points[i].z = t_point.point.z;
        outMsg->points[i].intensity = pc->points[i].intensity;
      }
    }
  }