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
### ROS Parameters
VDB Mapping is highly configurable using ROS parameters. Below is a complete list of all available parameters. Internally we store them in a yaml configuration file. An example can be found [here](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros/blob/devel/vdb_mapping_ros/cfg/vdb_params.yaml).

#### Basic Parameters
Listed below are the general parameters to configure the basic behavior of vdb_mapping

| Parameter Name        | Type    | Default            | Information
| ------------------    | ------- | ------------------ | -----------
| map_frame             | string  | ' '                | Coordinate frame of the map
| robot_frame           | string  | ' '                | Coordinate frame of the robot
| max_range             | double  | 15.0               | Global maximum raycasting range (can also be set for each sensor source individually)
| resolution            | double  | 0.07               | Map resolution
| prob_hit              | double  | 0.7                | Probability update if a beam hits a voxel
| prob_miss             | double  | 0.4                | Probability update if a beam misses a voxel
| prob_thres_min        | double  | 0.12               | Lower occupancy threshold of a voxel
| prob_thres_max        | double  | 0.97                | Upper occupancy threshold of a voxel
| map_save_dir          | string  | ' '                  | Storage location for saved maps
| accumulate_updates    | boolean | false                  | Specifies whether the data of multiple sensor measurement should be accumulated before integrating it into the map.
| accumulation_period   | double  | 1 | Specifies how long updates should be accumulated before integration
| visualization_rate    | double  | 1                  | Specifies in which rate the visualization is published
| publish_pointcloud    | boolean | true                  | Specifies whether the map should be published as pointcloud
| publish_vis_marker    | boolean | true                  | Specifies whether the map should be published as visual marker 
| apply_raw_sensor_data | boolean | true                  | Specify whether raw sensor data (e.g. Pointclouds) should be integrated into the map. This flag becomes relevant when the user wants to use a pure remote instance. If set to true all sensor sources will be periodically integrated into the map.
| sources               | string[]| []                 | List of sensor sources
| remote_sources        | string[]| []                 | List of remote sources

#### Sensor Sources
VDB Mapping is able to integrate an arbitrary amount of different sensor sources (given they are available as PointCloud2 msg). To add a new sensor source the user has to add it to the list of sources in the basic parameters. Below are listed the parameters that can be used to configure the individual sensor source. These have to be put into the corresponding namespace in the yaml file. For an example see this [configuration file](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros/blob/devel/vdb_mapping_ros/cfg/vdb_params.yaml).

| Parameter Name      | Type    | Default      | Information
| ------------------  | ------- | -------------| -----------
| topic               | string  | ' '            | Topic name of the sensor msg
| sensor_origin_frame | string  | ' '            | Sensor frame of the measurement. This parameter is optional and in general the frame id of the msg is used. However in some cases like already frame aligned point clouds this info is no longer the origin of the sensor. For this purpose the user can specify here from which frame the raycasting should be performed.
| max_range           | double  | 0            | Individual per sensor max raycasting range. This parameter is optional and per default the global max range is used.

#### Remote Sources
VDB Mapping provides a remote operation mode. Here the map data is shared between multiple instances to generate the same or similar maps (depending on the chosen parameter settings). For this remote sources can be specified over the parameter server.

##### Local side
| Parameter Name                   | Type      | Default      | Information
| ------------------               | -------   | -------------| -----------
| publish_updates                  | boolean   | false            | Specifies whether updates are published
| publish_overwrites               | boolean   | false            | Specifies whether overwrites are published
| publish_sections                 | boolean   | false            | Specifies whether sections are published
| section_update/rate              | double    | 1            | Rate in which the section update is published
| section_update/frame             | string    | robot_frame  | Center of the update section 
| section_update/min_coord/{x,y,z} | double    | -10          | Min coordinate of the section bounding box centered around the section frame
| section_update/max_coord/{x,y,z} | double    | 10           | Max coordinate of the section bounding box centered around the section frame

##### Remote side
Below are listed the parameters that can be used to configure the individual remote source. These have to be put into the corresponding namespace in the yaml file. An example config file can be found [here](https://github.com/fzi-forschungszentrum-informatik/vdb_mapping_ros/blob/devel/vdb_mapping_ros/cfg/vdb_remote_params.yaml).

| Parameter Name         | Type    | Default      | Information
| ------------------     | ------- | -------------| -----------
| namespace              | string  | ' '            | Namespace of the remote mapping instance
| apply_remote_updates   | boolean | false            | Specifies whether remote updates should be applied to the map
| apply_remote_overwrites| boolean | false            | Specifies whether remote overwrites should be applied to the map
| apply_remote_sections  | boolean | false            | Specifies whether remote sections should be applied to the map

### Advertised ROS Topics

| Topic Name              | Type    |  Information
| ------------------      | ------- | -----------
| ~/vdb_map_visualization | visualization_msgs/Marker   | Map visualization topic as voxel marker
| ~/vdb_map_pointcloud    | sensor_msgs/PointCloud2     | Map visualization topic as pointcloud
| ~/vdb_map_updates       | vdb_mapping_msgs/UpdateGrid | Map updates topic
| ~/vdb_map_overwrites    | vdb_mapping_msgs/UpdateGrid | Map overwrites topic
| ~/vdb_map_sections      | vdb_mapping_msgs/UpdateGrid | Map sections topic

### Subscribed ROS Topics

| Topic Name                            | Type    |  Information
| ------------------                    | ------- | -----------
| {Parameter:sensor_source}/{Parameter:topic} | sensor_msgs/PointCloud2 | Pointcloud sensor message
| {Parameter:remote_source}/vdb_map_updates    | vdb_mapping_msgs/UpdateGrid | Map updates topic
| {Parameter:remote_source}/vdb_map_overwrites | vdb_mapping_msgs/UpdateGrid | Map overwrites topic
| {Parameter:remote_source}/vdb_map_sections   | vdb_mapping_msgs/UpdateGrid | Map sections topic

### ROS Services

| Service Name                            | Type    |  Information
| ------------------                    | ------- | -----------
| ~/load_map | vdb_mapping_msgs/LoadMap | Loads the specified map
| ~/save_map | std_srvs/Trigger | Saves the current map
| ~/reset_map| std_srvs/Trigger | Resets the current map
| ~/raytrace| vdb_mapping_msgs/Raytrace | Raytraces a point into the map
| ~/trigger_map_section_update| vdb_mapping_msgs/TriggerMapSectionUpdate | Triggers a map section update on a remote instance

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
