#include <iostream>
#include <vdb_mapping_ros/VDBMappingROS.h>

VDBMappingROS::VDBMappingROS()
  : m_priv_nh("~")
  , m_tf_listener(m_tf_buffer)
{
  m_priv_nh.param<double>("max_range", m_max_range, 5.0);
  m_priv_nh.param<double>("submap_size", m_submap_size, 15.0);
  m_priv_nh.param<double>("linear_submap_error", m_linear_submap_error, 0.1);
  m_priv_nh.param<double>("angular_submap_error", m_angular_submap_error, 0.05);
  m_priv_nh.param<std::string>("sensor_frame", m_sensor_frame, "");
  if (m_sensor_frame == "")
  {
    // TODO maybe handle this as error
    ROS_WARN_STREAM("No sensor frame specified");
  }
  m_priv_nh.param<std::string>("map_frame", m_map_frame, "map");

  m_priv_nh.param<double>("resolution", m_resolution, 0.1);


  m_sensor_cloud_sub = m_nh.subscribe("scan", 1, &VDBMappingROS::sensorCloudCallback, this);
  m_aligned_cloud_sub =
    m_nh.subscribe("scan_matched_points2", 1, &VDBMappingROS::alignedCloudCallback, this);
  m_vis_pub = m_nh.advertise<visualization_msgs::MarkerArray>("subvdbs", 1, true);
}

void VDBMappingROS::alignedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  ROS_INFO_STREAM("Processing new aligned PointCloud");
  geometry_msgs::TransformStamped sensor_to_cloud_tf;

  PointCloudT::Ptr cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *cloud);
  try
  {
    sensor_to_cloud_tf =
      m_tf_buffer.lookupTransform(msg->header.frame_id, m_sensor_frame, msg->header.stamp);
  }
  catch (tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform to submap frame failed: " << ex.what());
    return;
  }

  Eigen::Matrix<double, 3, 1> blub(sensor_to_cloud_tf.transform.translation.x,
                                   sensor_to_cloud_tf.transform.translation.y,
                                   sensor_to_cloud_tf.transform.translation.z);
  std::cout << "Updating map" << std::endl;
  m_vdb_map.insertPointCloud(cloud, blub);
  m_vis_pub.publish(
    createSubVDBVisualization((0 * 17), m_vdb_map.getMap(), true, "map"));
}

void VDBMappingROS::sensorCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
  // Transform pointcloud into map reference frame
  // Transform sensor origin in map reference frame
  // raycast this bitch

  // TODO is this really necessary????
  PointCloudT::Ptr sensor_cloud(new PointCloudT);
  // PointCloudT::Ptr submap_cloud(new PointCloudT);
  pcl::fromROSMsg(*msg, *sensor_cloud);

  // TODO naming
  geometry_msgs::TransformStamped map_to_sensor_link_tf;

  // TODO this is just for cartographers weird frame problem maybe fix it a different way...
  // TODO THIS SHOULD NOT BE IN THE FINISHED VERSION ....
  // Transform pointcloud from map to sensor frame...
  // TODO Check if pointcloud is in map frame
  // If not transform it

  if (msg->header.frame_id != m_map_frame)
  {
    // TODO Transform cloud to map frame
    // get source frame from header of msg
    // lookup transform from map to header frame

    ROS_INFO_STREAM("Sensor Data is not in map frame. Let me transform that for you");
    try
    {
      map_to_sensor_link_tf =
        m_tf_buffer.lookupTransform(m_sensor_frame, m_map_frame, msg->header.stamp);
      // pcl::transformPointCloud(
      //*sensor_cloud, *sensor_cloud, tf2::transformToEigen(map_to_sensor_link_tf).matrix());
      // sensor_cloud->header.frame_id = m_sensor_frame;
    }
    catch (tf2::TransformException& ex)
    {
      ROS_ERROR_STREAM("Transform from map to base_link failed: " << ex.what());
      return;
    }
  }
  else
  {
    // lookup transform from map to m_sensor_frame
  }

  // geometry_msgs::TransformStamped sensor_to_submap_tf;
  // try
  //{
  // sensor_to_submap_tf =
  // m_tf_buffer.lookupTransform(ss.str(), sensor_cloud->header.frame_id, msg->header.stamp);
  //}
  // catch (tf::TransformException& ex)
  //{
  // ROS_ERROR_STREAM("Transform to submap frame failed: " << ex.what());
  // return;
  //}
}

visualization_msgs::MarkerArray VDBMappingROS::createSubVDBVisualization(
  int id_offset, openvdb::FloatGrid::Ptr grid, bool color, std::string frame_id)
{
  // TODO not sure if tiles are handled correctly
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

    if (color)
    {
      occupied_nodes_vis.markers[0].colors.push_back(heightColorCoding(h));
    }
    else
    {
      std_msgs::ColorRGBA white;
      white.a = 1.0;
      white.r = 1.0;
      white.g = 1.0;
      white.b = 1.0;
      occupied_nodes_vis.markers[0].colors.push_back(white);
    }
  }

  // finish marker array ... kind of inefficient
  for (unsigned i = 0; i < occupied_nodes_vis.markers.size(); ++i)
  {
    double size                                   = m_resolution;
    occupied_nodes_vis.markers[i].header.frame_id = frame_id;
    occupied_nodes_vis.markers[i].header.stamp    = ros::Time::now();
    occupied_nodes_vis.markers[i].id              = id_offset + i;
    occupied_nodes_vis.markers[i].type            = visualization_msgs::Marker::CUBE_LIST;
    occupied_nodes_vis.markers[i].scale.x         = size;
    occupied_nodes_vis.markers[i].scale.y         = size;
    occupied_nodes_vis.markers[i].scale.z         = size;

    occupied_nodes_vis.markers[i].color.a      = 1.0;
    occupied_nodes_vis.markers[i].color.r      = 1.0;
    occupied_nodes_vis.markers[i].frame_locked = true;

    if (occupied_nodes_vis.markers[i].points.size() > 0)
    {
      occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::ADD;
    }
    else
    {
      occupied_nodes_vis.markers[i].action = visualization_msgs::Marker::DELETE;
    }
  }
  return occupied_nodes_vis;
}
// Copied from octomap_server
std_msgs::ColorRGBA VDBMappingROS::heightColorCoding(double h)
{
  std_msgs::ColorRGBA color;
  color.a  = 1.0;
  double s = 1.0;
  double v = 1.0;

  h -= floor(h);
  h *= 6;
  int i;
  double m, n, f;

  i = floor(h);
  f = h - i;
  if (!(i & 1))
    f = 1 - f; // if i is even
  m = v * (1 - s);
  n = v * (1 - s * f);

  switch (i)
  {
    case 6:
    case 0:
      color.r = v;
      color.g = n;
      color.b = m;
      break;
    case 1:
      color.r = n;
      color.g = v;
      color.b = m;
      break;
    case 2:
      color.r = m;
      color.g = v;
      color.b = n;
      break;
    case 3:
      color.r = m;
      color.g = n;
      color.b = v;
      break;
    case 4:
      color.r = n;
      color.g = m;
      color.b = v;
      break;
    case 5:
      color.r = v;
      color.g = m;
      color.b = n;
      break;
    default:
      color.r = 1;
      color.g = 0.5;
      color.b = 0.5;
      break;
  }
  return color;
}
