#  Sim demo of Autoware.Auto on Panther robot


## File structure

Main folders are:
- `Panther_simulator` - folder containing all simulator code
- `adehome` - home folder of ADE environment. Here all the development is done


### Main code:

All of the main code is located inside `adehome/AutowareDemo/src/`. Its the source directory of a ROS2 project.

There are two main ROS2 packages:
- `panther_description` - contains Panther robot mesh and urdf.
- `panther_sim_demo` - contains launch files and config files required for the system to run. launch files are based on launches for AVP demo from [autoware_demos](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/tree/master/src/launch/autoware_demos) package.
 


