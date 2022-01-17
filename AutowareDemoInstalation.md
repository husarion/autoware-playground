## REQUIREMENTS

Install Docker

Make sure you added $USER to docker group.
instructions [here](https://docs.docker.com/engine/install/linux-postinstall/)


## INSTALATION

1. Install ade, but don't launch it yet, instructions [here](https://ade-cli.readthedocs.io/en/latest/install.html#requirements)


```
cd /usr/local/bin/
sudo wget https://gitlab.com/ApexAI/ade-cli/-/jobs/1341322851/artifacts/raw/dist/ade+x86_64
sudo mv ade+x86_64 ade
sudo chmod +x ade
sudo ./ade update-cli
```

2. Pull panther demo

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

echo 'source /opt/AutowareAuto/setup.bash' >> ~/.bashrc
echo 'source ~/AutowareDemo/install/setup.bash' >> ~/.bashrc
```