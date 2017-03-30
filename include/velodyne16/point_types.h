/* -*- mode: C++ -*-
 *
 *  Copyright (C) 2011, 2012 Austin Robot Technology
 *  Copyright (C) 2016 University of Freiburg
 *
 *  License: Modified BSD Software License Agreement
 *
 *  $Id: data_base.h 1554 2011-06-14 22:11:17Z jack.oquin $
 */

/** \file
 *
 *  Point Cloud Library point structures for Velodyne data.
 *
 *  @author Jesse Vera
 *  @author Jack O'Quin
 *  @author Piyush Khandelwal
 */

#ifndef __VELODYNE_POINTCLOUD_POINT_TYPES_H
#define __VELODYNE_POINTCLOUD_POINT_TYPES_H

#include <pcl/point_types.h>

namespace velodyne16 {
/** Euclidean Velodyne coordinate, including timestamp, intensity and ring number. */
struct PointXYZTI {
  PCL_ADD_POINT4D;                    // quad-word XYZ
  uint32_t time_offset_ns;            ///< point timestamp offset
  float intensity;                    ///< laser intensity reading
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace velodyne16

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne16::PointXYZTI,
                                      (float, x, x)
                                      (float, y, y)
                                      (float, z, z)
                                      (uint32_t, time_offset_ns, time_offset_ns)
                                      (float, intensity, intensity))

#endif // __VELODYNE_POINTCLOUD_POINT_TYPES_H

