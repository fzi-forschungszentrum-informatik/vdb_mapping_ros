#include <vdb_mapping_ros/VDBMappingROS.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "vdb_mapping_node");
  ros::NodeHandle nh;
  VDBMappingROS vdbMapping;;
  ros::spin();

  return 0;
}
