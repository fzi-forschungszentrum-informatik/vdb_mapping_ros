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

#ifndef VDBMAPPING_REMOTE_VDBMAPPING_INCLUDED_H_
#define VDBMAPPING_REMOTE_VDBMAPPING_INCLUDED_H_

#include <ros/ros.h>
#include <vdb_mapping/OccupancyVDBMapping.h>
#include <vdb_mapping_ros/VDBMappingTools.h>
#include <std_msgs/String.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <openvdb/openvdb.h>
#include <openvdb/io/Stream.h>

#include <pcl_conversions/pcl_conversions.h>

class RemoteVDBMappingROS
{
public:
  /*!
   * \brief Creates a new RemoteVDBMappingROS instance
   */
  RemoteVDBMappingROS();
  virtual ~RemoteVDBMappingROS(){};

  /*!
   * \brief Callback that listens to map updates from the VDBMappingROS side
   *
   * \param update_msg Update grid encoded as serialized string
   */
  void updateCallback(const std_msgs::String::ConstPtr& update_msg);

  /*!
   * \brief Publishes a marker array and pointcloud representation of the map
   */
  void publishMap() const;

private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;
  ros::Subscriber m_update_sub;

  std::unique_ptr<vdb_mapping::OccupancyVDBMapping> m_vdb_map;
  vdb_mapping::Config m_config;
  double m_resolution;
  bool m_publish_vis_marker;
  bool m_publish_pointcloud;
  std::string m_map_frame;

  ros::Publisher m_pointcloud_pub;
  ros::Publisher m_visualization_marker_pub;

};

#endif /* VDBMAPPING_REMOTE_VDBMAPPING_INCLUDED_H_ */
