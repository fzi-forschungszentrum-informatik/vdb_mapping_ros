VDB Mapping ROS Package 
===
DISCLAIMER: This library is still under development. Be warned that some interfaces will be changed and/or extended in the future.

The VDB Mapping ROS Package is a ROS wrapper around [VDB Mapping](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping)

## Getting Started

### Requirements
This library requires [OpenVDB](https://www.openvdb.org/) as it is build around it. This library was initially developed using Version 5.0 and should work with all versions above.  
Either use the apt package which will be automatically installed via rosdep or compile the package from source using the provided [build instructions](https://github.com/AcademySoftwareFoundation/openvdb)

### Build instructions

Since the required VDB Mapping library is a plain C++ package, you cannot use catkin_make directly.
Instead you have to use [catkin build](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) or [catkin_make_isolated](http://docs.ros.org/independent/api/rep/html/rep-0134.html) to build the workspace.

``` bash
# source global ros
source /opt/ros/<your_ros_version>/setup.bash

# create a catkin workspace
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src

# clone packages
git clone https://github.com/fzi-forschungszentrum-informatik/vdb_mapping
git clone https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros

# install dependencies
sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -y

# build the workspace.  
catkin build

# source the workspace
source devel/setup.bash
```

## ROS API
### Advertised ROS Topics
``` 
~/vdb_map_visualization (type: visualization_msgs/Marker)
```
Publishes the resulting map as voxel marker
``` 
~/vdb_map_pointcloud (type: sensor_msgs/PointCloud2)
```
Publishes the resulting map as pointcloud 

### Subscribed ROS Topics
```
~/Parameter:aligned_points (type: sensor_msgs/PointCloud2)
```
Subscriber for pointclouds which are already aligned to a specific frame
```
~/Parameter:raw_points (type: sensor_msgs/PointCloud2)
```
Subscriber for pointclouds in sensor coordinates

### ROS Parameters
All parameters can be passed as commandline arguments to the launch file.
| Parameter Name     | Type    | Default              | Information
| ------------------ | ------- | -------------------- | -----------
| aligned_points     | string  | scan_matched_points2 | Pointclouds which are already aligned to a specific frame (e.g. /map)
| raw_points         | string  | raw_points           | Pointclouds in sensor frame
| sensor_frame       | string  | velodyne             | Sensor frame for raycasting aligned pointclouds
| map_frame          | string  | map                  | Coordinate frame of the map
| max_range          | double  | 10.0                 | Maximum raycasting range
| resolution         | double  | 0.07                 | Map resolution
| prob_hit           | double  | 0.8                  | Probability update if a beam hits a voxel
| prob_miss          | double  | 0.1                  | Probability update if a beam misses a voxel
| prob_thres_min     | double  | 0.12                 | Lower occupancy threshold of a voxel
| prob_thres_max     | double  | 0.8                  | Upper occupancy threshold of a voxel
| publish_pointcloud | boolean | true                 | Specify whether the map should be published as pointcloud
| publish_vis_marker | boolean | true                 | Specify whether the map should be published as visual marker 


## Acknowledgement

The research leading to this package has received funding from the German Federal Ministry of Education and Research under grant agreement No. 13N14679:  

<a href="https://www.bmbf.de/">
  <img src="https://robdekon.de/user/themes/robdekon/images/BMBF_gefoerdert_2017_web.de.svg"
  alt="bmbf" height="80">
</a>  
  
ROBDEKON - Robotic systems for decontamination in hazardous environments  
More information: [robdekon.de](https://robdekon.de/)  


<a href="https://robdekon.de/">
  <img src="https://robdekon.de/user/themes/robdekon/images/robdekon_logo_web.svg"
  alt="robdekon_logo" height="40">
</a>  
