## REQUIREMENTS

Install Docker

Make sure you added $USER to docker group.
instructions [here](https://docs.docker.com/engine/install/linux-postinstall/)


## INSTALATION

1. Install ade, but dont launch it yet, instructions [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements)


```
cd /usr/local/bin/
sudo wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
sudo mv ade+x86_64 ade
sudo chmod +x ade
sudo ./ade update-cli
```

2. Pull panther demo

### TODO REPLACE PLAYGROUND

```
cd path/you/want

git clone https://github.com/husarion/autoware-playground/
```

3. Build ADE software
```
cd adehome/AutowareDemo/
sudo ade update-cli
ade start --update --enter
cd AutowareDemo
vcs import < autoware.auto.foxy.repos
colcon build --symlink-install

## replace playground dir
echo 'source /opt/AutowareAuto/setup.bash' >> ~/.bashrc
echo 'source ~/AutowareDemo/install/setup.bash' >> ~/.bashrc
```

4. Build lidar software


## RUNNING SIM DEMO




## RUNNING REAL DEMO

Terminal 1 - insde ADE:

```
ros2 launch panther_real_demo panther.launch.py
```

Terminal 2 - on PC:

```
ros2 launch panther_sim_demo autoware_auto_visualization.launch.py
```

Terminal 3 - on Panther, ros1_ws sourced

```
roslaunch ouster_panther ouster_odom_fusion.launch
```

Termial 4 - on Panther, inside demo repo. Launching bridge

```
docker-compose up
```