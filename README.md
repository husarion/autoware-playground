# autoware-playground
notes and code sample how to run Panther in Autoware.auto

## Launching Demo



### 1. Start ADE

```
#To make sure no ade proces is in the background
docker container kill $(docker ps -q)
#Start ade
cd your_path/ade_autoware_home/AutowareAuto
ade start
```

### 2. Start SVL Simulator

```
ade enter
AutowareAuto/src/autoware-playground/Panther_simulator/simulator
```
Next choose your simulation with panther in SVL websiter and launch it

### 3. Start AutowareAuto

Terminal 1
```
ade enter
source AutowareAuto/install/setup.bash
lgsvl_bridge
```

Terminal 2

```
ade enter
source AutowareAuto/install/setup.bash
ros2 launch panther_autoware_demo autoware_auto_visualization.launch.py
```

Terminal 3

```
ade enter
source AutowareAuto/install/setup.bash
stdbuf -o L ros2 launch panther_autoware_demo avp_sim_panther.launch.py
```

### 4. Set pose estimate in rviz

- Click play button in SVl
- Select pose estimate t ocorrect place on the map, Panther should jump to this position and detected objects should become visible in rviz.

### 5. Choose goal pose to drive around

Select a goal pose somewhere on the map and watch Panther go there.

## Instalation


### 1. 

### 2. Install ade, but dont launch it yet, instructions [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements)

### 3. Setup ade with autoware using [this](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-ade.html) tutorial.

### 4. Pull latest Autyoware version

```
cd adehome
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
```

### 5. Pull rosbridge and panther demo

```
cd adehome/AutowareAuto/src

# Panther demo
git clone https://github.com/husarion/autoware-playground/

# Svl bridge
git clone https://github.com/lgsvl/ros2-lgsvl-bridge.git
cd ros2-lgsvl-bridge
git checkout ${ROS_DISTRO}-devel
```

### 6. Launch ade
```
sudo ade update-cli
ade start --update --enter
cd AutowareAuto
git pull 
vcs import < autoware.auto.foxy.repos
cd AutowareAuto/
colcon_build --symlink-install
```

### 7. Launch and setup SVL



```
AutowareAuto/src/autoware-playground/Panther_simulator/simulator
```

instructions [here](https://www.svlsimulator.com/docs/installation-guide/installing-simulator/#linktocloud)

## TODO - Panther + sensor config in svl store and 

When errors occure: https://answers.ros.org/question/375372/autowareauto-master-branch-source-code-failed/
