# PX4 Drone Autopilot

### 1. Suggestions: Clone to ~/src
```bash
$ cd ~/src
$ git clone https://github.com/zhenfu128/PX4-Autopilot.git --recursive
```
### 2. Some dependencies
```bash
$ cd ~/src/PX4-Autopilot
$ bash ./Tools/setup/ubuntu.sh
```
### 3. Install MAVROS 
https://docs.px4.io/main/en/ros/mavros_installation.html

This installation assumes you have a catkin workspace located at ~/catkin_ws If you don't create one with:
```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
$ wstool init src
```
You will be using the ROS Python tools: wstool (for retrieving sources), rosinstall, and catkin_tools (building) for this installation. While they may have been installed during your installation of ROS you can also install them with:
```bash
$ sudo apt-get install python-catkin-tools python-rosinstall-generator -y
```
If this is your first time using wstool you will need to initialize your source space with:
```bash
$ wstool init ~/catkin_ws/src
```
Now you are ready to do the build:

Install MAVLink:
```bash
# We use the Kinetic reference for all ROS distros as it's not distro-specific and up to date
$ rosinstall_generator --rosdistro kinetic mavlink | tee /tmp/mavros.rosinstall
```
nstall MAVROS from source using either released or latest version:
Released/stable
```bash
$ rosinstall_generator --upstream mavros | tee -a /tmp/mavros.rosinstall
```
Create workspace & deps
```bash
$ wstool merge -t src /tmp/mavros.rosinstall
$ wstool update -t src -j4
$ rosdep install --from-paths src --ignore-src -y
```
Install GeographicLib datasets:
```bash
$ ./src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```
Build source
```bash
$ catkin_make
```
Make sure that you use setup.bash or setup.zsh from workspace.
```bash
$ source devel/setup.bash
```
### 4. Make Project 
```bash
$ cd ~/src/PX4-Autopilot
$ make px4_sitl gazebo
```

### 5. Clone husky and build
https://github.com/CWEzio/Husky_Pursuit

### 6. Clone world model
```bash
$ mkdir src
$ cd src
$ git clone https://github.com/zhenfu128/jacabot.git
```
add to .bashrc
```bash
$ export GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/src/jacabot/jacabot_simulation/models
$ export GAZEBO_WORLD_PATH=${GAZEBO_WORLD_PATH}:$HOME/src/jacabot/jacabot_simulation/worlds
```

### 7. Run the simulation
```bash
$ source $HOME/src/PX4-Autopilot/Tools/setup_gazebo.bash $HOME/src/PX4-Autopilot $HOME/src/PX4-Autopilot/build/px4_sitl_default
$ export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$HOME/src/PX4-Autopilot:$HOME/src/PX4-Autopilot/Tools/sitl_gazebo
```

```bash
$ roslaunch husky_gazebo multi_lawn_unicycle.launch
```
open a new terminal
```bash
$ cd ~/src/PX4-Autopilot/integrationtests/python_src/px4_it/mavros/
$ python mavros_offboard_posctl_test.py
```