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
#include <vdb_mapping_msgs/GetOccGrid.h>
#include <vdb_mapping_msgs/LoadMap.h>
#include <vdb_mapping_msgs/LoadMapFromPCD.h>
#include <vdb_mapping_msgs/Raytrace.h>
#include <vdb_mapping_msgs/TriggerMapSectionUpdate.h>
#include <vdb_mapping_msgs/UpdateGrid.h>
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
  ros::Subscriber map_section_sub;
  ros::ServiceClient get_map_section_client;
  bool apply_remote_updates;
  bool apply_remote_overwrites;
  bool apply_remote_sections;
};

struct SensorSource
{
  std::string topic;
  std::string sensor_origin_frame;
  double max_range;
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
   *
   * \param nh Node handle to use for parameters and communication
   */
  explicit VDBMappingROS(const ros::NodeHandle& nh = ros::NodeHandle("~"));

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
   * \brief Saves the active values of the current map as PCD file
   */
  bool saveMapToPCD(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& res);

  /*!
   * \brief Load stored map
   */
  bool loadMap(vdb_mapping_msgs::LoadMap::Request& req, vdb_mapping_msgs::LoadMap::Response& res);

  /*!
   * \brief Generates the map from a PCD file
   */
  bool loadMapFromPCD(vdb_mapping_msgs::LoadMapFromPCD::Request& req,
                      vdb_mapping_msgs::LoadMapFromPCD::Response& res);

  /*!
   * \brief Sensor callback for Pointclouds
   *
   * If the sensor_origin_frame is not empty it will be used instead of the frame id
   * of the input cloud as origin of the raycasting
   *
   * \param cloud_msg PointCloud message
   * \param sensor_source Sensor source corresponding to the Pointcloud
   */
  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                     const SensorSource& sensor_source);

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
   * \brief Publishes the update and overwrite grids of the current integration step
   *
   * \param update update grid
   * \param overwrite overwrite grid
   * \param stamp timestamp of the integration step
   */
  void publishUpdates(typename VDBMappingT::UpdateGridT::Ptr update,
                      typename VDBMappingT::UpdateGridT::Ptr overwrite,
                      ros::Time stamp) const;


  /*!
   * \brief Listens to map updates and creats a map from these
   *
   * \param update_msg Single map update from a remote mapping instance
   */
  void mapUpdateCallback(const vdb_mapping_msgs::UpdateGrid::ConstPtr& update_msg);

  /*!
   * \brief Listens to map overwrites and creates a map from these
   *
   * \param update_msg Single map overwrite from a remote mapping instance
   */
  void mapOverwriteCallback(const vdb_mapping_msgs::UpdateGrid::ConstPtr& update_msg);

  /*!
   * \brief Listens to map sections and overwrites the map on corresponding sections
   *
   * \param update_msg Single map section from a remote mapping instance
   */
  void mapSectionCallback(const vdb_mapping_msgs::UpdateGrid::ConstPtr& update_msg);

  /*!
   * \brief Get the map frame name
   *
   * \returns Map frame name
   */
  const std::string& getMapFrame() const;

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  VDBMappingT& getMap();

  /*!
   * \brief Returns the map
   *
   * \returns VDB map
   */
  const VDBMappingT& getMap() const;

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
   * \brief Callback for occupancy grid service call
   *
   * \param req Trigger request
   * \param res current occupancy grid
   * \returns current occupancy grid
   */
  bool occGridGenCallback(vdb_mapping_msgs::GetOccGrid::Request& req,
                          vdb_mapping_msgs::GetOccGrid::Response& res);

  /*!
   * \brief Callback for map reset service call
   *
   * \param req request of the map reset
   * \param res result of the map reset
   * \returns result of map reset
   */
  bool mapResetCallback(std_srvs::TriggerRequest& req, std_srvs::TriggerResponse& res);

  /*!
   * \brief Callback for raytrace service call
   *
   * \param req Origin and direction for raytracing
   * \param res Resulting point of the raytrace
   *
   * \returns result of raytrace service
   */
  bool raytraceCallback(vdb_mapping_msgs::Raytrace::Request& req,
                        vdb_mapping_msgs::Raytrace::Response& res);


  /*!
   * \brief Callback for dynamic reconfigure of parameters
   *
   * \param config new configuration
   */
  void dynamicReconfigureCallback(vdb_mapping_ros::VDBMappingROSConfig& config, uint32_t);

  /*!
   * \brief Timer Callback for visualizing the entire map
   *
   * \param event ROS Timer event
   */
  void visualizationTimerCallback(const ros::TimerEvent& event);

  /*!
   * \brief Timer Callback for integrating all accumulated data
   *
   * \param event ROS Timer event
   */
  void accumulationUpdateTimerCallback(const ros::TimerEvent& event);


  /*!
   * \brief Timer Callback for publishing map sections
   *
   * \param event ROS Timer event
   */
  void sectionTimerCallback(const ros::TimerEvent& event);

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
   * \brief Subscriber vector for pointclouds
   */
  std::vector<ros::Subscriber> m_cloud_subs;

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
   * \brief Publisher for map sections
   */
  ros::Publisher m_map_section_pub;

  /*!
   * \brief Saves map in specified path from parameter server
   */
  ros::ServiceServer m_save_map_service_server;

  /*!
   * \brief Saves the active values of the map as PCD file in specified path from
   * parameter server
   */
  ros::ServiceServer m_save_map_to_pcd_service_server;

  /*!
   * \brief Loads a map from specified path from service
   */
  ros::ServiceServer m_load_map_service_server;

  /*!
   * \brief Generates a map from a PCD file path specified from service
   */
  ros::ServiceServer m_load_map_from_pcd_service_server;

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
   * \brief Service to request an occupancy grid based on the current VDB map
   */
  ros::ServiceServer m_occupancy_grid_service;

  /*!
   * \brief Service for raytracing
   */
  ros::ServiceServer m_raytrace_service;

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
   * \brief Map Frame
   */
  std::string m_map_frame;
  /*!
   * \brief Robot Frame
   */
  std::string m_robot_frame;

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
   * \brief Specifies whether the mapping publishes map sections for remote use
   */
  bool m_publish_sections;

  /*!
   * \brief Specifies whether the mapping applies raw sensor data
   */
  bool m_apply_raw_sensor_data;
  /*!
   * \brief Vector of remote mapping source connections
   */
  std::map<std::string, RemoteSource> m_remote_sources;

  /*!
   * \brief Timer for map visualization
   */
  ros::Timer m_visualization_timer;

  /*!
   * \brief Specifies whether the sensor data is accumulated before updating the map
   */
  bool m_accumulate_updates;
  /*!
   * \brief Timer for integrating accumulated data
   */
  ros::Timer m_accumulation_update_timer;
  /*!
   * \brief Timer for publishing map sections
   */
  ros::Timer m_section_timer;

  /*!
   * \brief Min Coordinate of the section update bounding box
   */
  Eigen::Matrix<double, 3, 1> m_section_min_coord;
  /*!
   * \brief Max Coordinate of the section update bounding box
   */
  Eigen::Matrix<double, 3, 1> m_section_max_coord;
  /*!
   * \brief Reference Frame for the section update
   */
  std::string m_section_update_frame;

  /*!
   * \brief Specifies the lower z bound for the visualization
   */
  double m_lower_visualization_z_limit;

  /*!
   * \brief Specifies the upper z bound for the visualization
   */
  double m_upper_visualization_z_limit;

  /*!
   * \brief Specifies the number of voxels which count as occupied for the occupancy grid
   */
  int m_two_dim_projection_threshold;
  /*!
   * \brief Initialization flag used to prevent dynamic reconfigure from overriding parameter file
   */
  bool m_dynamic_reconfigure_initialized;
  /*!
   * \brief Compression level used for creating the byte array message.
   */
  unsigned int m_compression_level = 1;
};

#include "VDBMappingROS.hpp"

#endif /* VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED */
