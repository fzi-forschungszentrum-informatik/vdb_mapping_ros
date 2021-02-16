#include <iostream>
#include <vdb_mapping_ros/VDBMappingROS.h>

VDBMappingROS::VDBMappingROS()
  : m_priv_nh("~")
  , m_tf_listener(m_tf_buffer)
{
  m_priv_nh.param<double>("resolution", m_resolution, 0.1);
  m_vdb_map = new VDBMapping(m_resolution);

  m_priv_nh.param<double>("max_range", m_config.max_range, 15.0);
  m_priv_nh.param<double>("prob_hit", m_config.prob_hit, 0.7);
  m_priv_nh.param<double>("prob_miss", m_config.prob_miss, 0.4);
  m_priv_nh.param<double>("prob_thres_min", m_config.prob_thres_min, 0.12);
  m_priv_nh.param<double>("prob_thres_max", m_config.prob_thres_max, 0.97);


  m_vdb_map->setConfig(m_config);


  m_priv_nh.param<std::string>("sensor_frame", m_sensor_frame, "");
  if (m_sensor_frame == "")
  {
    // TODO maybe handle this as error
    ROS_WARN_STREAM("No sensor frame specified");
  }
  m_priv_nh.param<std::string>("map_frame", m_map_frame, "");
  if (m_map_frame == "")
  {
    // TODO maybe handle this as error
    ROS_WARN_STREAM("No sensor frame specified");
  }

  m_sensor_cloud_sub = m_nh.subscribe("points", 1, &VDBMappingROS::sensorCloudCallback, this);

  m_aligned_cloud_sub =
    m_nh.subscribe("scan_matched_points2", 1, &VDBMappingROS::alignedCloudCallback, this);
  m_vis_pub = m_nh.advertise<visualization_msgs::MarkerArray>("vdb_map", 1, true);
}

void VDBMappingROS::alignedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ros::Time a, b;
  a = ros::Time::now();
  geometry_msgs::TransformStamped sensor_to_map_tf;
  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);
  try
  {
    sensor_to_map_tf = m_tf_buffer.lookupTransform(m_map_frame, m_sensor_frame, msg->header.stamp);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform to map frame failed: " << ex.what());
    return;
  }

  if (m_map_frame != msg->header.frame_id)
  {
    pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(sensor_to_map_tf).matrix());
    cloud->header.frame_id = m_map_frame;
  }

  b = ros::Time::now();
  std::cout << "Preprocessing: " << (b - a).toSec() << std::endl;


  processCloud(cloud, sensor_to_map_tf);
}

void VDBMappingROS::sensorCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ros::Time a, b;


  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);

  geometry_msgs::TransformStamped sensor_to_map_tf;
  try
  {
    sensor_to_map_tf =
      m_tf_buffer.lookupTransform(m_map_frame, msg->header.frame_id, msg->header.stamp);
  }
  catch (tf2::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform to map frame failed:" << ex.what());
    return;
  }
  // Transform pointcloud into map reference system
  pcl::transformPointCloud(*cloud, *cloud, tf2::transformToEigen(sensor_to_map_tf).matrix());
  cloud->header.frame_id = m_map_frame;

  b = ros::Time::now();
  std::cout << "Preprocessing: " << (b - a).toSec() << std::endl;

  processCloud(cloud, sensor_to_map_tf);
}

void VDBMappingROS::processCloud(const PointCloudT::Ptr cloud, geometry_msgs::TransformStamped tf)
{
  ros::Time a, b;
  a                                               = ros::Time::now();
  Eigen::Matrix<double, 3, 1> sensor_to_map_eigen = tf2::transformToEigen(tf).translation();
  m_vdb_map->insertPointCloud(cloud, sensor_to_map_eigen);
  b = ros::Time::now();
  std::cout << "Raycasting: " << (b - a).toSec() << std::endl;
  a = ros::Time::now();
  m_vis_pub.publish(createVDBVisualization(m_vdb_map->getMap(), m_map_frame));
  b = ros::Time::now();
  std::cout << "Visualization: " << (b - a).toSec() << std::endl;
}


visualization_msgs::MarkerArray VDBMappingROS::createVDBVisualization(openvdb::FloatGrid::Ptr grid,
                                                                      std::string frame_id)
{
  visualization_msgs::MarkerArray occupied_nodes_vis;
  occupied_nodes_vis.markers.resize(1);

  openvdb::CoordBBox bbox = grid->evalActiveVoxelBoundingBox();
  double minX, minY, minZ, maxX, maxY, maxZ;
  openvdb::Vec3d min_world_coord = grid->indexToWorld(bbox.getStart());
  openvdb::Vec3d max_world_coord = grid->indexToWorld(bbox.getEnd());

  minX = min_world_coord.x();
  minY = min_world_coord.y();
  minZ = min_world_coord.z();
  maxX = max_world_coord.x();
  maxY = max_world_coord.y();
  maxZ = max_world_coord.z();


  for (openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter)
  {
    openvdb::Vec3d world_coord = grid->indexToWorld(iter.getCoord());

    geometry_msgs::Point cube_center;
    cube_center.x = world_coord.x();
    cube_center.y = world_coord.y();
    cube_center.z = world_coord.z();

    occupied_nodes_vis.markers[0].points.push_back(cube_center);
    double h = (1.0 - std::min(std::max((cube_center.z - minZ) / (maxZ - minZ), 0.0), 1.0)) * 0.8;
    occupied_nodes_vis.markers[0].colors.push_back(heightColorCoding(h));
  }

  double size                                   = m_resolution;
  occupied_nodes_vis.markers[0].header.frame_id = frame_id;
  occupied_nodes_vis.markers[0].header.stamp    = ros::Time::now();
  occupied_nodes_vis.markers[0].id              = 0;
  occupied_nodes_vis.markers[0].type            = visualization_msgs::Marker::CUBE_LIST;
  occupied_nodes_vis.markers[0].scale.x         = size;
  occupied_nodes_vis.markers[0].scale.y         = size;
  occupied_nodes_vis.markers[0].scale.z         = size;
  occupied_nodes_vis.markers[0].color.a         = 1.0;
  occupied_nodes_vis.markers[0].color.r         = 1.0;
  occupied_nodes_vis.markers[0].frame_locked    = true;

  if (occupied_nodes_vis.markers[0].points.size() > 0)
  {
    occupied_nodes_vis.markers[0].action = visualization_msgs::Marker::ADD;
  }
  else
  {
    occupied_nodes_vis.markers[0].action = visualization_msgs::Marker::DELETE;
  }
  return occupied_nodes_vis;
}

// Conversion from Hue to RGB Value
std_msgs::ColorRGBA VDBMappingROS::heightColorCoding(double h)
{
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
