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

RemoteVDBMappingROS::RemoteVDBMappingROS()
  : m_priv_nh("~")
{

  openvdb::initialize();


  m_priv_nh.param<double>("resolution", m_resolution, 0.1);
  m_vdb_map = std::make_unique<vdb_mapping::OccupancyVDBMapping>(m_resolution);
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

void RemoteVDBMappingROS::updateCallback(const std_msgs::String::ConstPtr& update_msg)
{
  std::istringstream iss(update_msg->data);
  openvdb::io::Stream strm(iss);
  openvdb::GridPtrVecPtr grids;
  grids                               = strm.getGrids();
  openvdb::FloatGrid::Ptr update_grid = openvdb::gridPtrCast<openvdb::FloatGrid>(grids->front());
  m_vdb_map->updateMap(update_grid);
  publishMap();
}


void RemoteVDBMappingROS::publishMap() const
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

  visualization_msgs::Marker visualization_marker;
  vdb_mapping::OccupancyVDBMapping::PointCloudT::Ptr cloud(
    new vdb_mapping::OccupancyVDBMapping::PointCloudT);

  openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
  double min_z, max_z;
  openvdb::Vec3d min_world_coord = grid->indexToWorld(bbox.getStart());
  openvdb::Vec3d max_world_coord = grid->indexToWorld(bbox.getEnd());

  min_z = min_world_coord.z();
  max_z = max_world_coord.z();


  for (vdb_mapping::OccupancyVDBMapping::GridT::ValueOnCIter iter = grid->cbeginValueOn(); iter;
       ++iter)
  {
    openvdb::Vec3d world_coord = grid->indexToWorld(iter.getCoord());

    if (publish_vis_marker)
    {
      geometry_msgs::Point cube_center;
      cube_center.x = world_coord.x();
      cube_center.y = world_coord.y();
      cube_center.z = world_coord.z();
      visualization_marker.points.push_back(cube_center);
      // Calculate the relative height of each voxel.
      double h = (1.0 - ((world_coord.z() - min_z) / (max_z - min_z)));
      visualization_marker.colors.push_back(heightColorCoding(h));
    }
    if (publish_pointcloud)
    {
      cloud->points.push_back(vdb_mapping::OccupancyVDBMapping::PointT(
        world_coord.x(), world_coord.y(), world_coord.z()));
    }
  }

  if (publish_vis_marker)
  {
    double size                             = m_resolution;
    visualization_marker.header.frame_id    = m_map_frame;
    visualization_marker.header.stamp       = ros::Time::now();
    visualization_marker.id                 = 0;
    visualization_marker.type               = visualization_msgs::Marker::CUBE_LIST;
    visualization_marker.scale.x            = size;
    visualization_marker.scale.y            = size;
    visualization_marker.scale.z            = size;
    visualization_marker.color.a            = 1.0;
    visualization_marker.pose.orientation.w = 1.0;
    visualization_marker.frame_locked       = true;

    if (visualization_marker.points.size() > 0)
    {
      visualization_marker.action = visualization_msgs::Marker::ADD;
    }
    else
    {
      visualization_marker.action = visualization_msgs::Marker::DELETE;
    }
    m_visualization_marker_pub.publish(visualization_marker);
  }
  if (publish_pointcloud)
  {
    cloud->width  = cloud->points.size();
    cloud->height = 1;
    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(*cloud, cloud_msg);
    cloud_msg.header.frame_id = m_map_frame;
    cloud_msg.header.stamp    = ros::Time::now();
    m_pointcloud_pub.publish(cloud_msg);
  }
}

// Conversion from Hue to RGB Value
std_msgs::ColorRGBA RemoteVDBMappingROS::heightColorCoding(const double height) const
{
  // The factor of 0.8 is only for a nicer color range
  double h = height * 0.8;

  int i    = (int)(h * 6.0);
  double f = (h * 6.0) - i;
  double q = (1.0 - f);
  i %= 6;

  auto toMsg = [](double v1, double v2, double v3) {
    std_msgs::ColorRGBA rgba;
    rgba.a = 1.0;
    rgba.r = v1;
    rgba.g = v2;
    rgba.b = v3;
    return rgba;
  };

  switch (i)
  {
    case 0:
      return toMsg(1.0, f, 0.0);
      break;
    case 1:
      return toMsg(q, 1.0, 0.0);
      break;
    case 2:
      return toMsg(0.0, 1.0, f);
      break;
    case 3:
      return toMsg(0.0, q, 1.0);
      break;
    case 4:
      return toMsg(f, 0.0, 1.0);
      break;
    case 5:
      return toMsg(1.0, 0.0, q);
      break;
    default:
      return toMsg(1.0, 0.5, 0.5);
      break;
  }
}
