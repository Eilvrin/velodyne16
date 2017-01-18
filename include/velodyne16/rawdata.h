/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2007 Austin Robot Technology, Yaxin Liu, Patrick Beeson
 *  Copyright (C) 2009, 2010, 2012 Austin Robot Technology, Jack O'Quin
 *  Copyright (C) 2016 University of Freiburg
 *  License: Modified BSD Software License Agreement
 *
 *  $Id$
 */

/** @file
 *
 *  @brief Interfaces for interpreting raw packets from the Velodyne 3D LIDAR.
 *
 */

#ifndef __VELODYNE_RAWDATA_H
#define __VELODYNE_RAWDATA_H

#include <fstream>
#include <string>
#include <math.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <angles/angles.h>
#include <pcl_ros/point_cloud.h>
#include <velodyne16/point_types.h>
#include <velodyne16/calibration.h>
#include <tf/transform_listener.h>
#include <velodyne16/VelodynePacket.h>

namespace velodyne16_rawdata
{
  // Shorthand typedefs for point cloud representations
  typedef velodyne16::PointXYZIR VPoint;
  typedef pcl::PointCloud<VPoint> VPointCloud;

  
  /**
  * Raw Velodyne packet constants and structures.
  */
  static const float ROTATION_RESOLUTION      =     0.01f;  // [deg]
  static const uint16_t ROTATION_MAX_UNITS    = 36000u;     // [deg/100]
  static const float DISTANCE_RESOLUTION      =     0.002f; // [m]

  /** \brief Velodyne data conversion class */
  class RawData
  {
  public:

    RawData();
    ~RawData() {}

    /** \brief Set up for data processing.
     *
     *  Perform initializations needed before data processing can
     *  begin:
     *
     *    - read device-specific angles calibration
     *    - specify transform listener
     *
     *  @param private_nh private node handle for ROS parameters
     *  @param tf_listener_ transform listener for transforming points
     *  @returns 0 if successful;
     *           errno value for failure
     */
    int setup(ros::NodeHandle private_nh, tf::TransformListener* tf_listener = NULL);

    void setParameters(double min_range, double max_range, double view_direction, 
      double view_width, const std::string& frame_id = "", const std::string& fixed_frame_id = "");

  private:
    /** configuration parameters */
    typedef struct {
      std::string calibrationFile;     ///< calibration file name
      double max_range;                ///< maximum range to publish
      double min_range;                ///< minimum range to publish
      int min_angle;                   ///< minimum angle to publish
      int max_angle;                   ///< maximum angle to publish
      std::string frame_id;            ///< frame into which to transform points
      std::string fixed_frame_id;     ///<  fixed frame for tf transform

      double tmp_min_angle;
      double tmp_max_angle;
    } Config;
    Config config_;

    /**
     * Calibration file
     */
    velodyne16::Calibration calibration_;
    float sin_rot_table_[ROTATION_MAX_UNITS];
    float cos_rot_table_[ROTATION_MAX_UNITS];

    tf::TransformListener* tf_listener_;

    std::ofstream file_;

  };

} // namespace velodyne16_rawdata

#endif // __VELODYNE_RAWDATA_H
