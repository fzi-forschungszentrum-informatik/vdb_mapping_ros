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

#include <iostream>
#include <vdb_mapping_ros/VDBMappingROS.h>

template <typename VDBMappingT>
VDBMappingROS<VDBMappingT>::VDBMappingROS()
  : m_priv_nh("~")
  , m_tf_listener(m_tf_buffer)
{
  m_priv_nh.param<double>("resolution", m_resolution, 0.1);
  m_vdb_map = std::make_unique<VDBMappingT>(m_resolution);

  m_priv_nh.param<double>("max_range", m_config.max_range, 15.0);
  m_priv_nh.param<double>("prob_hit", m_config.prob_hit, 0.7);
  m_priv_nh.param<double>("prob_miss", m_config.prob_miss, 0.4);
  m_priv_nh.param<double>("prob_thres_min", m_config.prob_thres_min, 0.12);
  m_priv_nh.param<double>("prob_thres_max", m_config.prob_thres_max, 0.97);
  m_priv_nh.param<std::string>("map_save_dir", m_config.map_directory_path, "");

  // Configuring the VDB map
  m_vdb_map->setConfig(m_config);

  m_priv_nh.param<bool>("publish_pointcloud", m_publish_pointcloud, true);
  m_priv_nh.param<bool>("publish_vis_marker", m_publish_vis_marker, true);
  m_priv_nh.param<bool>("publish_updates", m_publish_updates, false);
  m_priv_nh.param<bool>("publish_overwrites", m_publish_overwrites, false);
  m_priv_nh.param<bool>("apply_raw_sensor_data", m_apply_raw_sensor_data, true);

  m_priv_nh.param<bool>("reduce_data", m_reduce_data, false);

  m_priv_nh.param<std::string>("sensor_frame", m_sensor_frame, "");
  if (m_sensor_frame.empty())
  {
    ROS_WARN_STREAM("No sensor frame specified");
  }
  m_priv_nh.param<std::string>("map_frame", m_map_frame, "");
  if (m_map_frame.empty())
  {
    ROS_WARN_STREAM("No map frame specified");
  }

  std::string raw_points_topic;
  std::string aligned_points_topic;
  m_priv_nh.param<std::string>("raw_points", raw_points_topic, "");
  m_priv_nh.param<std::string>("aligned_points", aligned_points_topic, "");


  // Setting up remote sources
  std::vector<std::string> source_ids;
  m_priv_nh.param<std::vector<std::string> >(
    "remote_sources", source_ids, std::vector<std::string>());

  for (auto& source_id : source_ids)
  {
    std::string remote_namespace;
    m_priv_nh.param<std::string>(source_id + "/namespace", remote_namespace, "");

    RemoteSource remote_source;
    m_priv_nh.param<bool>(
      source_id + "/apply_remote_updates", remote_source.apply_remote_updates, false);
    m_priv_nh.param<bool>(
      source_id + "/apply_remote_overwrites", remote_source.apply_remote_overwrites, false);

    if (remote_source.apply_remote_updates)
    {
      remote_source.map_update_sub = m_nh.subscribe(
        remote_namespace + "/vdb_map_updates", 1, &VDBMappingROS::mapUpdateCallback, this);
    }
    if (remote_source.apply_remote_overwrites)
    {
      remote_source.map_overwrite_sub = m_nh.subscribe(
        remote_namespace + "/vdb_map_overwrites", 1, &VDBMappingROS::mapOverwriteCallback, this);
    }
    remote_source.get_map_section_client =
      m_nh.serviceClient<vdb_mapping_msgs::GetMapSection>(remote_namespace + "/get_map_section");
    m_remote_sources.insert(std::make_pair(source_id, remote_source));
  }

  if (m_publish_updates)
  {
    m_map_update_pub = m_priv_nh.advertise<std_msgs::String>("vdb_map_updates", 1, true);
  }
  if (m_publish_overwrites)
  {
    m_map_overwrite_pub = m_priv_nh.advertise<std_msgs::String>("vdb_map_overwrites", 1, true);
  }

  if (m_apply_raw_sensor_data)
  {
    m_sensor_cloud_sub =
      m_nh.subscribe(raw_points_topic, 1, &VDBMappingROS::sensorCloudCallback, this);
    m_aligned_cloud_sub =
      m_nh.subscribe(aligned_points_topic, 1, &VDBMappingROS::alignedCloudCallback, this);
  }

  m_visualization_marker_pub =
    m_priv_nh.advertise<visualization_msgs::Marker>("vdb_map_visualization", 1, true);
  m_pointcloud_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("vdb_map_pointcloud", 1, true);

  m_map_reset_service =
    m_priv_nh.advertiseService("vdb_map_reset", &VDBMappingROS::mapResetCallback, this);

  m_dynamic_reconfigure_service.setCallback(
    boost::bind(&VDBMappingROS::dynamicReconfigureCallback, this, _1, _2));

  m_save_map_service_server = m_priv_nh.advertiseService("save_map", &VDBMappingROS::saveMap, this);
  m_load_map_service_server = m_priv_nh.advertiseService("load_map", &VDBMappingROS::loadMap, this);
  m_get_map_section_service =
    m_priv_nh.advertiseService("get_map_section", &VDBMappingROS::getMapSectionCallback, this);

  m_trigger_map_section_update_service = m_priv_nh.advertiseService(
    "trigger_map_section_update", &VDBMappingROS::triggerMapSectionUpdateCallback, this);
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::dynamicReconfigureCallback(
  vdb_mapping_ros::VDBMappingROSConfig& config, uint32_t)
{
  ROS_INFO("Dynamic reconfigure of parameters.");
  m_config.max_range = config.max_range;
  m_vdb_map->setConfig(m_config);

  m_publish_pointcloud          = config.publish_pointcloud;
  m_publish_vis_marker          = config.publish_vis_marker;
  m_publish_updates             = config.publish_updates;
  m_publish_overwrites          = config.publish_overwrites;
  m_lower_visualization_z_limit = config.lower_visualization_z_limit;
  m_upper_visualization_z_limit = config.upper_visualization_z_limit;
}

template <typename VDBMappingT>
bool VDBMappingROS<VDBMappingT>::mapResetCallback(std_srvs::TriggerRequest&,
                                                  std_srvs::TriggerResponse& res)
{
  resetMap();
  res.success = true;
  res.message = "Reset map successful.";
  return true;
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::resetMap()
{
  ROS_INFO_STREAM("Reseting Map");
  m_vdb_map->resetMap();
  publishMap();
}

template <typename VDBMappingT>
bool VDBMappingROS<VDBMappingT>::saveMap(std_srvs::Trigger::Request& req,
                                         std_srvs::Trigger::Response& res)
{
  (void)req;
  ROS_INFO_STREAM("Saving Map");
  res.success = m_vdb_map->saveMap();
  return res.success;
}

template <typename VDBMappingT>
bool VDBMappingROS<VDBMappingT>::loadMap(vdb_mapping_msgs::LoadMap::Request& req,
                                         vdb_mapping_msgs::LoadMap::Response& res)
{
  ROS_INFO_STREAM("Loading Map");
  bool success = m_vdb_map->loadMap(req.path);
  publishMap();
  res.success = success;
  return success;
}

template <typename VDBMappingT>
const typename VDBMappingT::GridT::Ptr VDBMappingROS<VDBMappingT>::getMap()
{
  return m_vdb_map->getMap();
}

template <typename VDBMappingT>
bool VDBMappingROS<VDBMappingT>::getMapSectionCallback(
  vdb_mapping_msgs::GetMapSection::Request& req, vdb_mapping_msgs::GetMapSection::Response& res)
{
  geometry_msgs::TransformStamped source_to_map_tf;
  try
  {
    source_to_map_tf = m_tf_buffer.lookupTransform(
      m_map_frame, req.header.frame_id, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform from source to map frame failed: " << ex.what());
    res.success = false;
    return true;
  }

  res.map     = gridToStr(m_vdb_map->getMapSection(req.bounding_box.min_corner.x,
                                               req.bounding_box.min_corner.y,
                                               req.bounding_box.min_corner.z,
                                               req.bounding_box.max_corner.x,
                                               req.bounding_box.max_corner.y,
                                               req.bounding_box.max_corner.z,
                                               tf2::transformToEigen(source_to_map_tf).matrix()));
  res.success = true;

  return true;
}

template <typename VDBMappingT>
bool VDBMappingROS<VDBMappingT>::triggerMapSectionUpdateCallback(
  vdb_mapping_msgs::TriggerMapSectionUpdate::Request& req,
  vdb_mapping_msgs::TriggerMapSectionUpdate::Response& res)
{
  auto remote_source = m_remote_sources.find(req.remote_source);
  if (remote_source == m_remote_sources.end())
  {
    std::stringstream ss;
    ss << "Key " << req.remote_source << " not found. Available sources are: ";
    for (auto& source : m_remote_sources)
    {
      ss << source.first << ", ";
    }
    ROS_INFO_STREAM(ss.str());
    res.success = false;
    return true;
  }

  vdb_mapping_msgs::GetMapSection srv;
  srv.request.header       = req.header;
  srv.request.bounding_box = req.bounding_box;
  remote_source->second.get_map_section_client.call(srv);

  if (srv.response.success)
  {
    m_vdb_map->overwriteMap(strToGrid(srv.response.map));
    publishMap();
  }

  res.success = srv.response.success;
  return true;
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::alignedCloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  geometry_msgs::TransformStamped sensor_to_map_tf;
  try
  {
    // Get sensor origin transform in map coordinates
    sensor_to_map_tf =
      m_tf_buffer.lookupTransform(m_map_frame, m_sensor_frame, cloud_msg->header.stamp);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform to map frame failed: " << ex.what());
    return;
  }

  // If aligned map is not already in correct map frame, transform it
  if (m_map_frame != cloud_msg->header.frame_id)
  {
    geometry_msgs::TransformStamped map_to_map_tf;
    try
    {
      map_to_map_tf = m_tf_buffer.lookupTransform(
        m_map_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform to map frame failed: " << ex.what());
      return;
    }
    pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(map_to_map_tf).matrix());
    cloud->header.frame_id = m_map_frame;
  }

  insertPointCloud(cloud, sensor_to_map_tf);
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::sensorCloudCallback(
  const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud);

  geometry_msgs::TransformStamped sensor_to_map_tf;
  try
  {
    // Get sensor origin transform in map coordinates
    sensor_to_map_tf =
      m_tf_buffer.lookupTransform(m_map_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform to map frame failed:" << ex.what());
    return;
  }
  // Transform pointcloud into map reference system
  pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(sensor_to_map_tf).matrix());
  cloud->header.frame_id = m_map_frame;

  insertPointCloud(cloud, sensor_to_map_tf);
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::insertPointCloud(
  const typename VDBMappingT::PointCloudT::Ptr cloud,
  const geometry_msgs::TransformStamped transform)
{
  Eigen::Matrix<double, 3, 1> sensor_to_map_eigen = tf2::transformToEigen(transform).translation();
  typename VDBMappingT::UpdateGridT::Ptr update;
  typename VDBMappingT::UpdateGridT::Ptr overwrite;
  // Integrate data into vdb grid
  m_vdb_map->insertPointCloud(cloud, sensor_to_map_eigen, update, overwrite, m_reduce_data);
  if (m_publish_updates)
  {
    m_map_update_pub.publish(gridToMsg(update));
  }
  if (m_publish_overwrites)
  {
    m_map_overwrite_pub.publish(gridToMsg(overwrite));
  }
  publishMap();
}

template <typename VDBMappingT>
std_msgs::String
VDBMappingROS<VDBMappingT>::gridToMsg(const typename VDBMappingT::UpdateGridT::Ptr update) const
{
  std_msgs::String msg;
  msg.data = gridToStr(update);
  return msg;
}

template <typename VDBMappingT>
std::string
VDBMappingROS<VDBMappingT>::gridToStr(const typename VDBMappingT::UpdateGridT::Ptr update) const
{
  openvdb::GridPtrVec grids;
  grids.push_back(update);
  std::ostringstream oss(std::ios_base::binary);
  openvdb::io::Stream(oss).write(grids);
  return oss.str();
}

template <typename VDBMappingT>
typename VDBMappingT::UpdateGridT::Ptr
VDBMappingROS<VDBMappingT>::msgToGrid(const std_msgs::String::ConstPtr& msg) const
{
  return strToGrid(msg->data);
}

template <typename VDBMappingT>
typename VDBMappingT::UpdateGridT::Ptr
VDBMappingROS<VDBMappingT>::strToGrid(const std::string& msg) const
{
  std::istringstream iss(msg);
  openvdb::io::Stream strm(iss);
  openvdb::GridPtrVecPtr grids;
  grids = strm.getGrids();
  // This cast might fail if different VDB versions are used.
  // Corresponding error messages are generated by VDB directly
  typename VDBMappingT::UpdateGridT::Ptr update_grid =
    openvdb::gridPtrCast<typename VDBMappingT::UpdateGridT>(grids->front());
  return update_grid;
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::mapUpdateCallback(const std_msgs::String::ConstPtr& update_msg)
{
  if (!m_reduce_data)
  {
    m_vdb_map->updateMap(msgToGrid(update_msg));
  }
  else
  {
    m_vdb_map->updateMap(m_vdb_map->raycastUpdateGrid(msgToGrid(update_msg)));
  }
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::mapOverwriteCallback(const std_msgs::String::ConstPtr& update_msg)
{
  m_vdb_map->overwriteMap(msgToGrid(update_msg));
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::publishMap() const
{
  if (!(m_publish_pointcloud || m_publish_vis_marker))
  {
    return;
  }

  typename VDBMappingT::GridT::Ptr grid = m_vdb_map->getMap();

  bool publish_vis_marker;
  publish_vis_marker = (m_publish_vis_marker && m_visualization_marker_pub.getNumSubscribers() > 0);
  bool publish_pointcloud;
  publish_pointcloud = (m_publish_pointcloud && m_pointcloud_pub.getNumSubscribers() > 0);

  visualization_msgs::Marker visualization_marker_msg;
  sensor_msgs::PointCloud2 cloud_msg;

  VDBMappingTools<VDBMappingT>::createMappingOutput(m_vdb_map->getMap(),
                                                    m_map_frame,
                                                    visualization_marker_msg,
                                                    cloud_msg,
                                                    publish_vis_marker,
                                                    publish_pointcloud,
                                                    m_lower_visualization_z_limit,
                                                    m_upper_visualization_z_limit);

  if (publish_vis_marker)
  {
    m_visualization_marker_pub.publish(visualization_marker_msg);
  }
  if (publish_pointcloud)
  {
    m_pointcloud_pub.publish(cloud_msg);
  }
}
