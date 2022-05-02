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
 * \date    2020-12-23
 *
 */
//----------------------------------------------------------------------

#include <ros/ros.h>
#include <vdb_mapping/OccupancyVDBMapping.h>
#include <vdb_mapping_ros/VDBMappingROS.h>

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "vdb_mapping_node");
  ros::NodeHandle nh;
  VDBMappingROS<vdb_mapping::OccupancyVDBMapping> vdb_mapping;
  ros::spin();
  return 0;
}
