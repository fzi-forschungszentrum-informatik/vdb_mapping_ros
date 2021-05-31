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

  // Configuring the VDB map
  m_vdb_map->setConfig(m_config);

  m_priv_nh.param<bool>("publish_pointcloud", m_publish_pointcloud, true);
  m_priv_nh.param<bool>("publish_vis_marker", m_publish_vis_marker, true);
  m_priv_nh.param<bool>("publish_updates", m_publish_updates, true);

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

  std::string raw_points_topic, aligned_points_topic;
  m_priv_nh.param<std::string>("raw_points", raw_points_topic, "");
  m_priv_nh.param<std::string>("aligned_points", aligned_points_topic, "");

  m_sensor_cloud_sub =
    m_nh.subscribe(raw_points_topic, 1, &VDBMappingROS::sensorCloudCallback, this);

  m_aligned_cloud_sub =
    m_nh.subscribe(aligned_points_topic, 1, &VDBMappingROS::alignedCloudCallback, this);

  m_visualization_marker_pub =
    m_nh.advertise<visualization_msgs::Marker>("vdb_map_visualization", 1, true);
  m_pointcloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("vdb_map_pointcloud", 1, true);

  m_update_pub = m_nh.advertise<std_msgs::String>("vdb_map_update", 1, true);
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::resetMap()
{
  ROS_INFO_STREAM("Reseting Map");
  m_vdb_map->resetMap();
  publishMap();
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
  // Integrate data into vdb grid
  openvdb::FloatGrid::Ptr update = m_vdb_map->createUpdate(cloud, sensor_to_map_eigen);

  if (m_publish_updates)
  {
    publishUpdate(update);
  }

  m_vdb_map->updateMap(update);

  publishMap();
}

template <typename VDBMappingT>
void VDBMappingROS<VDBMappingT>::publishUpdate(openvdb::FloatGrid::Ptr update) const
{
  openvdb::GridPtrVec grids;
  grids.push_back(update);
  std::ostringstream oss(std::ios_base::binary);
  openvdb::io::Stream(oss).write(grids);
  std_msgs::String msg;
  msg.data = oss.str();
  m_update_pub.publish(msg);
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

  visualization_msgs::Marker visualization_marker;

  sensor_msgs::PointCloud2 bla;

  VDBMappingTools::createVisualizationMsgs(m_vdb_map->getMap(),
                                           m_resolution,
                                           m_map_frame,
                                           visualization_marker,
                                           bla,
                                           publish_vis_marker,
                                           publish_pointcloud);

  if (publish_vis_marker)
  {
    m_visualization_marker_pub.publish(visualization_marker);
  }
  if (publish_pointcloud)
  {
    m_visualization_marker_pub.publish(bla);
  }
}
