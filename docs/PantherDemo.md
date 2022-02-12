

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