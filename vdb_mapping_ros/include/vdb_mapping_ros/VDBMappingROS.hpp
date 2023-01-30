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

#include <zstd.h>


template <typename VDBMappingT>
VDBMappingROS<VDBMappingT>::VDBMappingROS(const ros::NodeHandle& nh)
  : m_priv_nh(nh)
  , m_dynamic_reconfigure_service(ros::NodeHandle("~/vdb_mapping"))
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
  m_priv_nh.param<int>("two_dim_projection_threshold", m_two_dim_projection_threshold, 5);
  m_priv_nh.param<double>("lower_visualization_z_limit", m_lower_visualization_z_limit, 0);
  m_priv_nh.param<double>("upper_visualization_z_limit", m_upper_visualization_z_limit, 0);

  // Configuring the VDB map
  m_vdb_map->setConfig(m_config);

  m_priv_nh.param<bool>("publish_pointcloud", m_publish_pointcloud, true);
  m_priv_nh.param<bool>("publish_vis_marker", m_publish_vis_marker, true);
  m_priv_nh.param<bool>("publish_updates", m_publish_updates, false);
  m_priv_nh.param<bool>("publish_overwrites", m_publish_overwrites, false);
  m_priv_nh.param<bool>("publish_sections", m_publish_sections, false);
  m_priv_nh.param<bool>("apply_raw_sensor_data", m_apply_raw_sensor_data, true);

  m_priv_nh.param<std::string>("map_frame", m_map_frame, "");
  if (m_map_frame.empty())
  {
    ROS_WARN_STREAM("No map frame specified");
  }
  m_vdb_map->getGrid()->insertMeta("ros/map_frame", openvdb::StringMetadata(m_map_frame));
  m_priv_nh.param<std::string>("robot_frame", m_robot_frame, "");
  if (m_robot_frame.empty())
  {
    ROS_WARN_STREAM("No robot frame specified");
  }


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
    m_priv_nh.param<bool>(
      source_id + "/apply_remote_sections", remote_source.apply_remote_sections, false);

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
    if (remote_source.apply_remote_sections)
    {
      remote_source.map_section_sub = m_nh.subscribe(
        remote_namespace + "/vdb_map_sections", 1, &VDBMappingROS::mapSectionCallback, this);
    }
    remote_source.get_map_section_client =
      m_nh.serviceClient<vdb_mapping_msgs::GetMapSection>(remote_namespace + "/get_map_section");
    m_remote_sources.insert(std::make_pair(source_id, remote_source));
  }

  if (m_publish_updates)
  {
    m_map_update_pub =
      m_priv_nh.advertise<vdb_mapping_msgs::UpdateGrid>("vdb_map_updates", 1, true);
  }
  if (m_publish_overwrites)
  {
    m_map_overwrite_pub =
      m_priv_nh.advertise<vdb_mapping_msgs::UpdateGrid>("vdb_map_overwrites", 1, true);
  }
  if (m_publish_sections)
  {
    m_map_section_pub =
      m_priv_nh.advertise<vdb_mapping_msgs::UpdateGrid>("vdb_map_sections", 1, true);

    double section_update_rate;
    m_priv_nh.param<double>("section_update/rate", section_update_rate, 1);

    m_section_timer =
      m_nh.createTimer(ros::Rate(section_update_rate), &VDBMappingROS::sectionTimerCallback, this);


    m_priv_nh.param<double>("section_update/min_coord/x", m_section_min_coord.x(), -10);
    m_priv_nh.param<double>("section_update/min_coord/y", m_section_min_coord.y(), -10);
    m_priv_nh.param<double>("section_update/min_coord/z", m_section_min_coord.z(), -10);
    m_priv_nh.param<double>("section_update/max_coord/x", m_section_max_coord.x(), 10);
    m_priv_nh.param<double>("section_update/max_coord/y", m_section_max_coord.y(), 10);
    m_priv_nh.param<double>("section_update/max_coord/z", m_section_max_coord.z(), 10);
    m_priv_nh.param<std::string>("section_update/frame", m_section_update_frame, m_robot_frame);
  }

  if (m_apply_raw_sensor_data)
  {
    // Setting all aligned sources
    m_priv_nh.param<std::vector<std::string> >("sources", source_ids, std::vector<std::string>());
    for (auto& source_id : source_ids)
    {
      SensorSource sensor_source;

      m_priv_nh.param<std::string>(source_id + "/topic", sensor_source.topic, "");
      m_priv_nh.param<std::string>(
        source_id + "/sensor_origin_frame", sensor_source.sensor_origin_frame, "");
      m_priv_nh.param<double>(source_id + "/max_range", sensor_source.max_range, 0);
      ROS_INFO_STREAM("Setting up source: " << source_id);
      if (sensor_source.topic.empty())
      {
        ROS_ERROR_STREAM("No input topic specified for source: " << source_id);
        continue;
      }
      ROS_INFO_STREAM("Topic: " << sensor_source.topic);
      if (sensor_source.sensor_origin_frame.empty())
      {
        ROS_INFO_STREAM("Using frame_id of topic as raycast origin");
      }
      else
      {
        ROS_INFO_STREAM("Using " << sensor_source.sensor_origin_frame << " as raycast origin");
      }

      m_cloud_subs.push_back(m_nh.subscribe<sensor_msgs::PointCloud2>(
        sensor_source.topic,
        1,
        boost::bind(&VDBMappingROS::cloudCallback, this, _1, sensor_source)));
    }
  }

  m_visualization_marker_pub =
    m_priv_nh.advertise<visualization_msgs::Marker>("vdb_map_visualization", 1, true);
  m_pointcloud_pub = m_priv_nh.advertise<sensor_msgs::PointCloud2>("vdb_map_pointcloud", 1, true);

  m_map_reset_service =
    m_priv_nh.advertiseService("reset_map", &VDBMappingROS::mapResetCallback, this);

  m_raytrace_service =
    m_priv_nh.advertiseService("raytrace", &VDBMappingROS::raytraceCallback, this);

  m_save_map_service_server = m_priv_nh.advertiseService("save_map", &VDBMappingROS::saveMap, this);
  m_load_map_service_server = m_priv_nh.advertiseService("load_map", &VDBMappingROS::loadMap, this);
  m_load_map_from_pcd_service_server =
    m_priv_nh.advertiseService("load_map_from_pcd", &VDBMappingROS::loadMapFromPCD, this);
  m_get_map_section_service =
    m_priv_nh.advertiseService("get_map_section", &VDBMappingROS::getMapSectionCallback, this);

  m_trigger_map_section_update_service = m_priv_nh.advertiseService(
    "trigger_map_section_update", &VDBMappingROS::triggerMapSectionUpdateCallback, this);
  m_occupancy_grid_service =
    m_priv_nh.advertiseService("get_occupancy_grid", &VDBMappingROS::occGridGenCallback, this);

  double visualization_rate;
  m_priv_nh.param<double>("visualization_rate", visualization_rate, 1.0);
  m_visualization_timer = m_nh.createTimer(
    ros::Rate(visualization_rate), &VDBMappingROS::visualizationTimerCallback, this);

  m_priv_nh.param<bool>("accumulate_updates", m_accumulate_updates, false);
  if (m_accumulate_updates)
  {
    double accumulation_period;
    m_priv_nh.param<double>("accumulation_period", accumulation_period, 1);
    m_accumulation_update_timer = m_nh.createTimer(
      ros::Rate(1.0 / accumulation_period), &VDBMappingROS::accumulationUpdateTimerCallback, this);
  }
  m_dynamic_reconfigure_initialized = false;
  m_dynamic_reconfigure_service.setCallback(
    boost::bind(&VDBMappingROS::dynamicReconfigureCallback, this, _1, _2));
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::dynamicReconfigureCallback(
  vdb_mapping_ros::VDBMappingROSConfig& config, uint32_t)
{
  if (!m_dynamic_reconfigure_initialized)
  {
    config.max_range                   = m_config.max_range;
    config.publish_pointcloud          = m_publish_pointcloud;
    config.publish_vis_marker          = m_publish_vis_marker;
    config.publish_updates             = m_publish_updates;
    config.publish_overwrites          = m_publish_overwrites;
    config.lower_visualization_z_limit = m_lower_visualization_z_limit;
    config.upper_visualization_z_limit = m_upper_visualization_z_limit;
    m_dynamic_reconfigure_initialized  = true;
    return;
  }

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
bool VDBMappingROS<VDBMappingT>::raytraceCallback(vdb_mapping_msgs::Raytrace::Request& req,
                                                  vdb_mapping_msgs::Raytrace::Response& res)
{
  geometry_msgs::TransformStamped reference_tf;
  try
  {
    reference_tf = m_tf_buffer.lookupTransform(m_map_frame, req.header.frame_id, req.header.stamp);

    Eigen::Matrix<double, 4, 4> m = tf2::transformToEigen(reference_tf).matrix();
    Eigen::Matrix<double, 4, 1> origin, direction;
    origin << req.origin.x, req.origin.y, req.origin.z, 1;
    direction << req.direction.x, req.direction.y, req.direction.z, 0;

    origin    = m * origin;
    direction = m * direction;

    openvdb::Vec3d end_point;

    res.success = m_vdb_map->raytrace(openvdb::Vec3d(origin.x(), origin.y(), origin.z()),
                                      openvdb::Vec3d(direction.x(), direction.y(), direction.z()),
                                      req.max_ray_length,
                                      end_point);

    res.header.frame_id = m_map_frame;
    res.header.stamp    = req.header.stamp;
    res.end_point.x     = end_point.x();
    res.end_point.y     = end_point.y();
    res.end_point.z     = end_point.z();
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform to map frame failed:" << ex.what());
    res.success = false;
  }
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
bool VDBMappingROS<VDBMappingT>::loadMapFromPCD(vdb_mapping_msgs::LoadMapFromPCD::Request& req,
                                                vdb_mapping_msgs::LoadMapFromPCD::Response& res)
{
  ROS_INFO_STREAM("Loading Map from PCD file");
  bool success = m_vdb_map->loadMapFromPCD(req.path, req.set_background);
  publishMap();
  res.success = success;
  return success;
}

template <typename VDBMappingT>
const std::string& VDBMappingROS<VDBMappingT>::getMapFrame() const
{
  return m_map_frame;
}

template <typename VDBMappingT>
VDBMappingT& VDBMappingROS<VDBMappingT>::getMap()
{
  return *m_vdb_map;
}

template <typename VDBMappingT>
const VDBMappingT& VDBMappingROS<VDBMappingT>::getMap() const
{
  return *m_vdb_map;
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

  res.section                 = gridToMsg(m_vdb_map->getMapSectionUpdateGrid(
    Eigen::Matrix<double, 3, 1>(
      req.bounding_box.min_corner.x, req.bounding_box.min_corner.y, req.bounding_box.min_corner.z),
    Eigen::Matrix<double, 3, 1>(
      req.bounding_box.max_corner.x, req.bounding_box.max_corner.y, req.bounding_box.max_corner.z),
    tf2::transformToEigen(source_to_map_tf).matrix()));
  res.section.header.frame_id = m_map_frame;
  res.section.header.stamp    = ros::Time::now();
  res.success                 = true;

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
    m_vdb_map->overwriteMap(byteArrayToGrid(srv.response.section.map));
  }

  res.success = srv.response.success;
  return true;
}


template <typename VDBMappingT>
bool VDBMappingROS<VDBMappingT>::occGridGenCallback(vdb_mapping_msgs::GetOccGrid::Request& req,
                                                    vdb_mapping_msgs::GetOccGrid::Response& res)
{
  (void)req;
  nav_msgs::OccupancyGrid grid;
  openvdb::CoordBBox curr_bbox = m_vdb_map->getGrid()->evalActiveVoxelBoundingBox();
  grid.header.frame_id         = m_map_frame;
  grid.header.stamp            = ros::Time::now();
  grid.info.height             = curr_bbox.dim().y();
  grid.info.width              = curr_bbox.dim().x();
  grid.info.resolution         = m_resolution;
  std::vector<int> voxel_projection_grid;
  grid.data.resize(grid.info.width * grid.info.height);
  voxel_projection_grid.resize(grid.info.width * grid.info.height);

  int x_offset = abs(curr_bbox.min().x());
  int y_offset = abs(curr_bbox.min().y());

  geometry_msgs::Pose origin_pose;
  origin_pose.position.x = curr_bbox.min().x() * m_resolution;
  origin_pose.position.y = curr_bbox.min().y() * m_resolution;
  origin_pose.position.z = 0.00;

  grid.info.origin   = origin_pose;
  int world_to_index = 0;
  for (openvdb::FloatGrid::ValueOnCIter iter = m_vdb_map->getGrid()->cbeginValueOn(); iter; ++iter)
  {
    if (iter.isValueOn())
    {
      world_to_index =
        (iter.getCoord().y() + y_offset) * curr_bbox.dim().x() + (iter.getCoord().x() + x_offset);
      voxel_projection_grid[world_to_index] += 1;
    }
  }

  for (size_t i = 0; i < voxel_projection_grid.size(); i++)
  {
    if (voxel_projection_grid[i] > m_two_dim_projection_threshold)
    {
      grid.data[i] = 100;
    }
    else if (voxel_projection_grid[i] == 0)
    {
      grid.data[i] = -1;
    }
    else
    {
      grid.data[i] = 0;
    }
  }
  res.occupancy_grid = grid;
  return true;
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg,
                                               const SensorSource& sensor_source)
{
  typename VDBMappingT::PointCloudT::Ptr cloud(new typename VDBMappingT::PointCloudT);
  pcl::fromROSMsg(*cloud_msg, *cloud);
  geometry_msgs::TransformStamped cloud_origin_tf;

  std::string sensor_frame = sensor_source.sensor_origin_frame.empty()
                               ? cloud_msg->header.frame_id
                               : sensor_source.sensor_origin_frame;

  // Get the origin of the sensor used as a starting point of the ray cast
  try
  {
    cloud_origin_tf = m_tf_buffer.lookupTransform(
      m_map_frame, sensor_frame, cloud_msg->header.stamp, ros::Duration(0.1));
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform from " << sensor_frame << " to " << m_map_frame
                                       << " frame failed:" << ex.what());
    return;
  }

  // Transform the input pointcloud to the correct map frame
  if (m_map_frame != cloud_msg->header.frame_id)
  {
    geometry_msgs::TransformStamped origin_to_map_tf;
    try
    {
      origin_to_map_tf = m_tf_buffer.lookupTransform(
        m_map_frame, cloud_msg->header.frame_id, cloud_msg->header.stamp);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform from " << cloud_msg->header.frame_id << " to " << m_map_frame
                                         << " frame failed:" << ex.what());
      return;
    }
    pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(origin_to_map_tf).matrix());
    cloud->header.frame_id = m_map_frame;
  }
  m_vdb_map->accumulateUpdate(
    cloud, tf2::transformToEigen(cloud_origin_tf).translation(), sensor_source.max_range);
  if (!m_accumulate_updates)
  {
    typename VDBMappingT::UpdateGridT::Ptr update;
    typename VDBMappingT::UpdateGridT::Ptr overwrite;
    m_vdb_map->integrateUpdate(update, overwrite);
    m_vdb_map->resetUpdate();
    publishUpdates(update, overwrite, cloud_msg->header.stamp);
  }
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::publishUpdates(typename VDBMappingT::UpdateGridT::Ptr update,
                                                typename VDBMappingT::UpdateGridT::Ptr overwrite,
                                                ros::Time stamp) const

{
  static unsigned int sequence_number = 0;
  std_msgs::Header header;
  header.frame_id = m_map_frame;
  header.stamp    = stamp;
  header.seq      = sequence_number++;
  if (m_publish_updates)
  {
    vdb_mapping_msgs::UpdateGrid msg = gridToMsg(update);
    msg.header                       = header;
    m_map_update_pub.publish(msg);
  }
  if (m_publish_overwrites)
  {
    vdb_mapping_msgs::UpdateGrid msg = gridToMsg(overwrite);
    msg.header                       = header;
    m_map_overwrite_pub.publish(msg);
  }
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
  m_vdb_map->insertPointCloud(cloud, sensor_to_map_eigen, update, overwrite);
  publishUpdates(update, overwrite, transform.header.stamp);
}

template <typename VDBMappingT>
vdb_mapping_msgs::UpdateGrid
VDBMappingROS<VDBMappingT>::gridToMsg(const typename VDBMappingT::UpdateGridT::Ptr update) const
{
  vdb_mapping_msgs::UpdateGrid msg;
  msg.map = gridToByteArray(update);
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
std::vector<uint8_t> VDBMappingROS<VDBMappingT>::gridToByteArray(
  const typename VDBMappingT::UpdateGridT::Ptr update) const
{
  std::string map_str = gridToStr(update);
  auto uncompressed   = std::vector<uint8_t>(map_str.begin(), map_str.end());

  // Create buffer with enough size for worst case scenario
  size_t len = ZSTD_compressBound(uncompressed.size());
  std::vector<uint8_t> compressed(len);

  int ret = ZSTD_compress(
    compressed.data(), len, uncompressed.data(), uncompressed.size(), m_compression_level);


  if (ZSTD_isError(ret))
  {
    ROS_ERROR("Compression using ZSTD failed: '%s', sending uncompressed byte array",
              ZSTD_getErrorName(ret));

    return uncompressed;
  }

  // Resize compressed buffer to actual compressed size
  compressed.resize(ret);
  return compressed;
}

template <typename VDBMappingT>
typename VDBMappingT::UpdateGridT::Ptr
VDBMappingROS<VDBMappingT>::msgToGrid(const vdb_mapping_msgs::UpdateGrid::ConstPtr& msg) const
{
  return byteArrayToGrid(msg->map);
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
typename VDBMappingT::UpdateGridT::Ptr
VDBMappingROS<VDBMappingT>::byteArrayToGrid(const std::vector<uint8_t>& msg) const
{
  std::size_t len = ZSTD_getDecompressedSize(msg.data(), msg.size());
  std::vector<uint8_t> uncompressed(len);

  std::size_t size = ZSTD_decompress(uncompressed.data(), len, msg.data(), msg.size());


  if (ZSTD_isError(size))
  {
    ROS_ERROR("Could not decompress map using ZSTD: %s. Returning raw data.",
              ZSTD_getErrorName(size));
    std::string map_str(msg.begin(), msg.end());
    return strToGrid(map_str);
  }

  std::string map_str(uncompressed.begin(), uncompressed.end());
  return strToGrid(map_str);
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::mapUpdateCallback(
  const vdb_mapping_msgs::UpdateGrid::ConstPtr& update_msg)
{
  static unsigned int sequence_number = 0;
  if (sequence_number != update_msg->header.seq)
  {
    ROS_WARN_STREAM("Missed " << (update_msg->header.seq - sequence_number) << " update(s)");
    sequence_number = update_msg->header.seq;
  }
  sequence_number++;

  m_vdb_map->updateMap(msgToGrid(update_msg));
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::mapOverwriteCallback(
  const vdb_mapping_msgs::UpdateGrid::ConstPtr& update_msg)
{
  static unsigned int sequence_number = 0;
  if (sequence_number != update_msg->header.seq)
  {
    ROS_WARN_STREAM("Missed " << (update_msg->header.seq - sequence_number) << " overwrite(s)");
    sequence_number = update_msg->header.seq;
  }
  sequence_number++;
  m_vdb_map->overwriteMap(msgToGrid(update_msg));
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::mapSectionCallback(
  const vdb_mapping_msgs::UpdateGrid::ConstPtr& update_msg)
{
  static unsigned int sequence_number = 0;
  if (sequence_number != update_msg->header.seq)
  {
    ROS_WARN_STREAM("Missed " << (update_msg->header.seq - sequence_number) << " section(s)");
    sequence_number = update_msg->header.seq;
  }
  sequence_number++;

  m_vdb_map->applyMapSectionUpdateGrid(msgToGrid(update_msg));
}


template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::visualizationTimerCallback(const ros::TimerEvent& event)
{
  (void)event;
  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::accumulationUpdateTimerCallback(const ros::TimerEvent& event)
{
  // static unsigned int sequence_number = 0;
  typename VDBMappingT::UpdateGridT::Ptr update;
  typename VDBMappingT::UpdateGridT::Ptr overwrite;
  m_vdb_map->integrateUpdate(update, overwrite);

  publishUpdates(update, overwrite, event.current_real);
  m_vdb_map->resetUpdate();
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::sectionTimerCallback(const ros::TimerEvent& event)
{
  static unsigned int sequence_number = 0;
  geometry_msgs::TransformStamped map_to_robot_tf;
  try
  {
    map_to_robot_tf = m_tf_buffer.lookupTransform(
      m_map_frame, m_section_update_frame, ros::Time(0), ros::Duration(1.0));
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform from source to map frame failed: " << ex.what());
  }

  typename VDBMappingT::UpdateGridT::Ptr section = m_vdb_map->getMapSectionUpdateGrid(
    m_section_min_coord, m_section_max_coord, tf2::transformToEigen(map_to_robot_tf).matrix());

  vdb_mapping_msgs::UpdateGrid msg;
  msg.header.frame_id = m_map_frame;
  msg.header.stamp    = event.current_real;
  msg.header.seq      = sequence_number++;
  msg.map             = gridToByteArray(section);
  m_map_section_pub.publish(msg);
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::publishMap() const
{
  if (!(m_publish_pointcloud || m_publish_vis_marker))
  {
    return;
  }

  bool publish_vis_marker;
  publish_vis_marker = (m_publish_vis_marker && m_visualization_marker_pub.getNumSubscribers() > 0);
  bool publish_pointcloud;
  publish_pointcloud = (m_publish_pointcloud && m_pointcloud_pub.getNumSubscribers() > 0);

  visualization_msgs::Marker visualization_marker_msg;
  sensor_msgs::PointCloud2 cloud_msg;

  VDBMappingTools<VDBMappingT>::createMappingOutput(m_vdb_map->getGrid(),
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
