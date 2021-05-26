#include <ros/ros.h>
#include <vdb_mapping_ros/RemoteVDBMappingROS.h>

int main(int argc, char *argv[])
{
  ros::init(argc,argv, "remote_mapping");
  std::cout << "hi im a remote mapping node" << std::endl;

  RemoteVDBMappingROS test;
  ros::spin();
  return 0;
}
