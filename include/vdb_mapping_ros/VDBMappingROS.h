#ifndef VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED
#define VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>

#include <vdb_mapping/VDBMapping.h>

#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

// TODO which one is needed?
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

class VDBMappingROS
{
  using PointT      = pcl::PointXYZ;
  using PointCloudT = pcl::PointCloud<PointT>;

public:
  VDBMappingROS();
  virtual ~VDBMappingROS(){};

  void alignedCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);
  void sensorCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg);

  visualization_msgs::MarkerArray createSubVDBVisualization(int id_offset,
                                                            openvdb::FloatGrid::Ptr grid,
                                                            bool color,
                                                            std::string frame_id);
  std_msgs::ColorRGBA heightColorCoding(double h);

private:
  ros::NodeHandle m_nh;
  ros::NodeHandle m_priv_nh;
  ros::Subscriber m_sensor_cloud_sub;
  ros::Subscriber m_aligned_cloud_sub;
  ros::Publisher m_vis_pub;

  tf2_ros::Buffer m_tf_buffer;
  tf2_ros::TransformListener m_tf_listener;

  double m_max_range;
  double m_resolution;
  double m_submap_size;
  double m_linear_submap_error;
  double m_angular_submap_error;

  std::string m_sensor_frame;
  std::string m_map_frame;

  visualization_msgs::MarkerArray m_vis_marker;

  VDBMapping m_vdb_map;
};

#endif /* VDB_MAPPING_ROS_VDBMAPPINGROS_H_INCLUDED */
