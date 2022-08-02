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