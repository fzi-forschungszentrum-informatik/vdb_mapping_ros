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
#include <std_msgs/String.h>
#include <std_srvs/Trigger.h>
#include <vdb_mapping_msgs/GetMapSection.h>
#include <vdb_mapping_msgs/LoadMap.h>
#include <vdb_mapping_msgs/TriggerMapSectionUpdate.h>
#include <visualization_msgs/Marker.h>

#include <openvdb/io/Stream.h>
#include <vdb_mapping_ros/VDBMappingTools.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <std_srvs/Trigger.h>

#include <dynamic_reconfigure/server.h>
#include <vdb_mapping_ros/VDBMappingROSConfig.h>

struct RemoteSource
{
  ros::Subscriber map_update_sub;
  ros::Subscriber map_overwrite_sub;
  ros::ServiceClient get_map_section_client;
  bool apply_remote_updates;
  bool apply_remote_overwrites;
};

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
   * \brief Saves the current map
   */
  bool saveMap(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);


  /*!
   * \brief Load stored map
   */
  bool loadMap(vdb_mapping_msgs::LoadMap::Request& req, vdb_mapping_msgs::LoadMap::Response& res);

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
   * \brief Creates a compressed Bitstream as ROS msg from an input grid
   *
   * \param update Update grid
   *
   * \returns update msg
   */
  std_msgs::String gridToMsg(const typename VDBMappingT::UpdateGridT::Ptr update) const;

  /*!
   * \brief Creates a compressed Bitstream as string from an input grid
   *
   * \param update Update grid
   *
   * \returns bitstream
   */
  std::string gridToStr(const typename VDBMappingT::UpdateGridT::Ptr update) const;

  /*!
   * \brief Unpacks an update grid from a compressed bitstream
   *
   * \param msg Compressed Bitstream
   *
   * \returns Update Grid
   */
  typename VDBMappingT::UpdateGridT::Ptr msgToGrid(const std_msgs::String::ConstPtr& msg) const;

  /*!
   * \brief Unpacks an update grid from a ROS msg
   *
   * \param msg Compressed Bitstream as ROS msg
   *
   * \returns Update Grid
   */
  typename VDBMappingT::UpdateGridT::Ptr strToGrid(const std::string& msg) const;

  /*!
   * \brief Listens to map updates and creats a map from these
   *
   * \param update_msg Single map update from a remote mapping instance
   */
  void mapUpdateCallback(const std_msgs::String::ConstPtr& update_msg);

  /*!
   * \brief Listens to map overwrites and creates a map from these
   *
   * \param update_msg Single map overwrite from a remote mapping instance
   */
  void mapOverwriteCallback(const std_msgs::String::ConstPtr& update_msg);

  /*!
   * \brief Returns a pointer to the map
   *
   * \returns VDB grid pointer
   */
  const typename VDBMappingT::GridT::Ptr getMap();

  /*!
   * \brief Callback for requesting parts of the map
   *
   * \param req Coordinates and reference of the map section
   * \param res Result of section request, which includes the returned map
   *
   * \returns Result of section request
   */
  bool getMapSectionCallback(vdb_mapping_msgs::GetMapSection::Request& req,
                             vdb_mapping_msgs::GetMapSection::Response& res);

  /*!
   * \brief Callback for triggering a map section request on a remote source
   *
   * \param req Coordinates, reference frame and remote source identifier of the map section
   * \param res Result of triggering section request
   *
   * \returns Result of triggering section request
   */
  bool triggerMapSectionUpdateCallback(vdb_mapping_msgs::TriggerMapSectionUpdate::Request& req,
                                       vdb_mapping_msgs::TriggerMapSectionUpdate::Response& res);

  /*!
   * \brief Callback for map reset service call
   *
   * \param res result of the map reset
   * \returns result of map reset
   */
  bool mapResetCallback(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res);

  /*!
   * \brief Callback for dynamic reconfigure of parameters
   *
   * \param config new configuration
   */
  void dynamicReconfigureCallback(vdb_mapping_ros::VDBMappingROSConfig& config, uint32_t);

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
   * \brief Publisher for map updates
   */
  ros::Publisher m_map_update_pub;

  /*!
   * \brief Publisher for map overwrites
   */
  ros::Publisher m_map_overwrite_pub;

  /*!
   * \brief Saves map in specified path from parameter server
   */
  ros::ServiceServer m_save_map_service_server;

  /*!
   * \brief Loads a map from specified path from service
   */
  ros::ServiceServer m_load_map_service_server;

  /*!
   * \brief Service for map section requests
   */
  ros::ServiceServer m_get_map_section_service;

  /*!
   * \brief Service for triggering the map section request on a remote source
   */
  ros::ServiceServer m_trigger_map_section_update_service;

  /*!
   * \brief Service for reset map
   */
  ros::ServiceServer m_map_reset_service;

  /*!
   * \brief Service for dynamic reconfigure of parameters
   */
  dynamic_reconfigure::Server<vdb_mapping_ros::VDBMappingROSConfig> m_dynamic_reconfigure_service;

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
  vdb_mapping::Config m_config;

  /*!
   * \brief Specifies whether a pointcloud should be published or not
   */
  bool m_publish_pointcloud;

  /*!
   * \brief Specifies whether the map should be published as markers or not
   */
  bool m_publish_vis_marker;

  /*!
   * \brief Specifies whether the mapping publishes map updates for remote use
   */
  bool m_publish_updates;

  /*!
   * \brief Specifies whether the mapping publishes map overwrites for remote use
   */
  bool m_publish_overwrites;

  /*!
   * \brief Specifies whether the mapping applies raw sensor data
   */
  bool m_apply_raw_sensor_data;
  /*!
   * \brief Specifies whether the data reduction is enabled
   */
  bool m_reduce_data;
  /*!
   * \brief Vector of remote mapping source connections
   */
  std::map<std::string, RemoteSource> m_remote_sources;

  /*!
   * \brief Specifies the lower z bound for the visualization
   */
  double m_lower_visualization_z_limit;

  /*!
   * \brief Specifies the upper z bound for the visualization
   */
  double m_upper_visualization_z_limit;
};

#include "VDBMappingROS.hpp"

#endif /* VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED */
