## REQUIREMENTS

Install Docker

Make sure you added $USER to docker group.
instructions [here](https://docs.docker.com/engine/install/linux-postinstall/)


## ADE INSTALATION

1. Install ade, but dont launch it yet, instructions [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements)


```
cd /usr/local/bin/
sudo wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
sudo mv ade+x86_64 ade
sudo chmod +x ade
sudo ./ade update-cli
```

2. Setup ade with autoware using [this](https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/installation-ade.html) tutorial.

```
mkdir -p ~/adehome
cd ~/adehome
touch .adehome
```

3. Pull latest Autoware version

### TODO REPLACE LINK TO PROPER REPO -NOT PLAYGROUND

```
cd adehome
git clone https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto.git
```


5. Pull panther demo

```
cd adehome/AutowareAuto/src

git clone https://github.com/husarion/autoware-playground/
```

6. Pul lgsvl_brigde (USE ONLY WITH SIMULATION)

```
cd adehome/AutowareAuto/src

git clone https://github.com/lgsvl/ros2-lgsvl-bridge.git
cd ros2-lgsvl-bridge
git checkout ${ROS_DISTRO}-devel
```

6. Build software
```
cd adehome/AutowareAuto/
sudo ade update-cli
ade start --update --enter
cd AutowareAuto
git pull 
vcs import < autoware.auto.foxy.repos
colcon build --symlink-install

## replace playground dir
echo 'source /opt/AutowareAuto/setup.bash' >> ~/.bashrc
echo 'source ~/AutowarePlayground/install/setup.bash' >> ~/.bashrc
```
