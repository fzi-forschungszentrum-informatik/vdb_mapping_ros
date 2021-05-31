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
 * \date    2020-05-31
 *
 */
//----------------------------------------------------------------------
#include <vdb_mapping_ros/VDBMappingTools.h>

void VDBMappingTools::createVisualizationMsgs(openvdb::FloatGrid::Ptr grid,
                                      visualization_msgs::Marker& marker,
                                      sensor_msgs::PointCloud2& cloud,
                                      bool createMarker,
                                      bool createPointcloud)
{
  std::cout << "i visualize stuff" << std::endl;
}

std_msgs::ColorRGBA VDBMappingTools::heightColorCoding(const double height)
{
  std::cout << " i do height" << std::endl;
}
