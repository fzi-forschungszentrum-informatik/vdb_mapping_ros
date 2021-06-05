// this is for emacs file handling -*- mode: c++; indent-tabs-mode: nil -*-

// -- BEGIN LICENSE BLOCK ----------------------------------------------
// Copyright 2021 FZI Forschungszentrum Informatik
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// -- END LICENSE BLOCK ------------------------------------------------

//----------------------------------------------------------------------
/*!\file
 *
 * \author  Marvin Große Besselmann grosse@fzi.de
 * \author  Lennart Puck puck@fzi.de
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------

#ifndef VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED
#define VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

/*!
 * \brief ROS wrapper class for vdb_mapping
 */
template <typename VDBMappingT>
class VDBMappingROS
{
public:
  /*!
   * \brief Creates a new VDBMappingROS instance
   */
  VDBMappingROS();
  virtual ~VDBMappingROS(){};

  /*!
   * \brief Resets the current map
   */
  void resetMap();

  /*!
   * \brief Sensor callback for scan aligned Pointclouds
   * In contrast to the normal sensor callback here an additional sensor frame has to be specified
   * as origin of the raycasting
   *
   * \param msg PointCloud message
   */
  void alignedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

  /*!
   * \brief Sensor callback for raw pointclouds. All data will be transformed into the map frame.
   *
   * \param msg
   */
  void sensorCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg);

  /*!
   * \brief Integrating the transformed pointcloud and sensor origins into the core mapping library
   *
   *
   * \param cloud Point cloud transformed into map coordinates
   * \param tf Sensor transform in map coordinates
   */
  void insertPointCloud(const typename VDBMappingT::PointCloudT::Ptr cloud,
                        const geometry_msgs::TransformStamped transform);


  /*!
   * \brief Publishes a marker array and pointcloud representation of the map
   */
  void publishMap() const;

  /*!
   * \brief Calculates a height correlating color coding using HSV color space
   *
   * \param height Gridcell height relativ to the min and max height of the complete grid. Parameter
   * can take values between 0 and 1
   *
   * \returns RGBA color of the grid cell
   */
  std_msgs::ColorRGBA heightColorCoding(const double height) const;


private:
  /*!
   * \brief Public node handle
   */
  ros::NodeHandle m_nh;

  /*!
   * \brief Private node handle
   */
  ros::NodeHandle m_priv_nh;

  /*!
   * \brief Subscriber for raw pointclouds
   */
  ros::Subscriber m_sensor_cloud_sub;

  /*!
   * \brief Subscriber for scan aligned pointclouds
   */
  ros::Subscriber m_aligned_cloud_sub;

  /*!
   * \brief Publisher for the marker array
   */
  ros::Publisher m_visualization_marker_pub;

  /*!
   * \brief Publisher for the point cloud
   */
  ros::Publisher m_pointcloud_pub;

  /*!
   * \brief Transformation buffer
   */
  tf2_ros::Buffer m_tf_buffer;

  /*!
   * \brief Transformation listener
   */
  tf2_ros::TransformListener m_tf_listener;

  /*!
   * \brief Grid cell resolution
   */
  double m_resolution;

  /*!
   * \brief Sensor frame used for raycasting of scan aligned pointclouds
   */
  std::string m_sensor_frame;

  /*!
   * \brief Map Frame
   */
  std::string m_map_frame;

  /*!
   * \brief Map pointer
   */
  std::unique_ptr<VDBMappingT> m_vdb_map;

  /*!
   * \brief Map configuration
   */

  /*!
   * \brief Specifies whether a pointcloud should be published or not
   */
  bool m_publish_pointcloud;

  /*!
   * \brief Specifies whether the map should be published as markers or not
   */
  bool m_publish_vis_marker;
  vdb_mapping::Config m_config;
};

#include "VDBMappingROS.hpp"

#endif /* VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED */
