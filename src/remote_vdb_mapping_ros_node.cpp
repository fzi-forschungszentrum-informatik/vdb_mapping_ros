#include <ros/ros.h>
#include <vdb_mapping_ros/RemoteVDBMappingROS.h>

int main(int argc, char *argv[])
{
  ros::init(argc,argv, "remote_mapping");
  RemoteVDBMappingROS remote_mapping_node;
  ros::spin();
  return 0;
}
