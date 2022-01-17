# Simulation Demo
notes and code sample how to run Panther in Autoware.auto


## Installation

### 1. Install ade, but don't launch it yet
```
cd /usr/local/bin/
sudo wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
sudo mv ade+x86_64 ade
sudo chmod +x ade
sudo ./ade update-cli
```
More information [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements)


### 2. Pull panther demo

```
cd path/you/want

git clone https://github.com/husarion/autoware-playground/
```

### 3. Build ADE software

```
cd autoware-playground/adehome/AutowareDemo/
sudo ade update-cli
ade start --update --enter
cd AutowareDemo
vcs import < autoware.auto.foxy.repos
colcon build --symlink-install

echo 'source /opt/AutowareAuto/setup.bash' >> ~/.bashrc
echo 'source ~/AutowareDemo/install/setup.bash' >> ~/.bashrc
```


## Setting up SVL simulator

### 1. Launch the simulator
```
autoware-playground/Panther_simulator/simulator
```

### 2. Link the simulator to cloud 
Use instructions [here](https://www.svlsimulator.com/docs/installation-guide/installing-simulator/#linktocloud)

### 3. Add panther vehicle to your simulator, you can do so by serching in vechicles store

### 4. Create a new simulation


 - in General tab: select name and cluster
 - in test case celetc Autonomous Stuff map and Vechicle panther with sensor configuration autoware
 - in Autopilot tab select Autoware.Auto with bridge connection `localhost:9090`

## Launching the demo

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
autoware-playground/Panther_simulator/simulator
```

Next choose your simulation with panther in SVL web app and launch it

### 3. Start AutowareAuto

Terminal 1
```
ade enter
lgsvl_bridge
```

Terminal 2

```
ade enter
ros2 launch panther_sim_demo sim_panther.launch.py
```

### 4. Set pose estimate in rviz

- Click play button in SVl
- Select pose estimate t correct place on the map, Panther should jump to this position and detected objects should become visible in rviz.

### 5. Choose goal pose to drive around

Select a goal pose somewhere on the map and watch Panther go there.

