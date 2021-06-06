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
 * \date    2021-05-26
 *
 */
//----------------------------------------------------------------------

#include <vdb_mapping_ros/RemoteVDBMappingROS.h>

template <typename VDBMappingT>
RemoteVDBMappingROS<VDBMappingT>::RemoteVDBMappingROS()
  : m_priv_nh("~")
{
  openvdb::initialize();

  m_priv_nh.param<double>("resolution", m_resolution, 0.1);
  m_vdb_map = std::make_unique<VDBMappingT>(m_resolution);
  m_priv_nh.param<double>("max_range", m_config.max_range, 15.0);
  m_priv_nh.param<double>("prob_hit", m_config.prob_hit, 0.7);
  m_priv_nh.param<double>("prob_miss", m_config.prob_miss, 0.4);
  m_priv_nh.param<double>("prob_thres_min", m_config.prob_thres_min, 0.12);
  m_priv_nh.param<double>("prob_thres_max", m_config.prob_thres_max, 0.97);

  m_vdb_map->setConfig(m_config);

  m_priv_nh.param<bool>("publish_pointcloud", m_publish_pointcloud, true);
  m_priv_nh.param<bool>("publish_vis_marker", m_publish_vis_marker, true);


  m_priv_nh.param<std::string>("map_frame", m_map_frame, "map");
  if (m_map_frame.empty())
  {
    ROS_WARN_STREAM("No map frame specified");
  }

  m_pointcloud_pub = m_nh.advertise<sensor_msgs::PointCloud2>("remote_vdb_map_pointcloud", 1, true);
  m_visualization_marker_pub =
    m_nh.advertise<visualization_msgs::Marker>("remote_vdb_map_visualization", 1, true);

  m_update_sub = m_nh.subscribe("vdb_map_update", 1, &RemoteVDBMappingROS::updateCallback, this);
}

template <typename VDBMappingT>
void RemoteVDBMappingROS<VDBMappingT>::updateCallback(const std_msgs::String::ConstPtr& update_msg)
{
  std::istringstream iss(update_msg->data);
  openvdb::io::Stream strm(iss);
  openvdb::GridPtrVecPtr grids;
  grids                               = strm.getGrids();
  openvdb::FloatGrid::Ptr update_grid = openvdb::gridPtrCast<openvdb::FloatGrid>(grids->front());
  m_vdb_map->updateMap(update_grid);
  publishMap();
}


template <typename VDBMappingT>
void RemoteVDBMappingROS<VDBMappingT>::publishMap() const
{
  if (!(m_publish_pointcloud || m_publish_vis_marker))
  {
    return;
  }

  openvdb::FloatGrid::Ptr grid = m_vdb_map->getMap();

  bool publish_vis_marker;
  publish_vis_marker = (m_publish_vis_marker && m_visualization_marker_pub.getNumSubscribers() > 0);
  bool publish_pointcloud;
  publish_pointcloud = (m_publish_pointcloud && m_pointcloud_pub.getNumSubscribers() > 0);

  visualization_msgs::Marker visualization_marker_msg;
  sensor_msgs::PointCloud2 pointcloud_msg;

  VDBMappingTools<VDBMappingT>::createVisualizationMsgs(m_vdb_map->getMap(),
                                           m_map_frame,
                                           visualization_marker_msg,
                                           pointcloud_msg,
                                           publish_vis_marker,
                                           publish_pointcloud);

  if (publish_vis_marker)
  {
    m_visualization_marker_pub.publish(visualization_marker_msg);
  }
  if (publish_pointcloud)
  {
    m_visualization_marker_pub.publish(pointcloud_msg);
  }
}
