## Map creation

### 1. Environment mapping

In order to create a environment map you will need to run ndt_mapper node witch creates a .pcd map based on lidar data.
We strongly recommend to record your data to rosbag first and then run the mapper alongside rosbag file.

To run ndt_maper:

```
ros2 launch panther_real_demo ndt_mapper_ouster.launch.py
```

### 2. Pcl map conversion

Unfortunetly the ndt_mapper provides pcl file in X Y Z Intensity format but ndt_localizer supports only X Y Z files.
To fix the .pcl simplest wau is to remove intensity fields manually

Steps:
- Remove last element from fields: FIELDS, SIZE, TYPE, COUNT.
- Nex use regular expression `"0$"` to find zeros at end of lines.
- Add previously removed 0 in field VIEWPOINT

Nex you will need to compress the .pcd map. To do so just run:

```
#If you don't have pcl-tools installed
sudo apt install pcl-tools 

pcl_converter file_in.pcd file_out.pcd -f binary_compressed
```

### Lenelet map creation

For more info: https://answers.ros.org/question/376240/how-to-build-correct-lanelet2-map-for-autowareauto/

In order co create the map you will need to ude the vector map builder available [here](https://tools.tier4.jp/feature/vector_map_builder_ll2/)

Steps to create lanelet2 map:
- set MGRS of your current location
- load ndt map (.pcd) created in previous step
- create desired lanes using the tool

### Setting up parameters

Copy the custom_map.yaml file and change latitude, longitude values to those according to your MGRS zone you added to lanelet map.
For example for MGRS: 32U PB, the lat, lon is: 50.54338, 10.41134 

