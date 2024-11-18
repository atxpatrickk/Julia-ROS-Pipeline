Assumptions
===========

1. ros2 installed on your machine
2. `.bashrc` with relevant bits:

```
# main ros2 stuff
source /opt/ros/humble/setup.bash
# your own ros2 stuff
source ~/ros2_ws/install/setup.bash
# identify yourself on the network
export ROS_DOMAIN_ID=30 #TURTLEBOT3
``` 

Prerequisites
=============

1. Install julia for your user - `curl -fsSL https://install.julialang.org | sh`
2. Install colcon - `sudo apt install python3-colcon-common-extensions`
3. Install pip3 - `sudo apt install python3-pip`
4. Install julia python package with pip3 - `pip3 install julia`
5. Run the julia package's installer - `python3 -m julia.install` or

```
$ python3
>>> import julia
>>> julia.install()
```

Building
========

1. Must build from ros2 workspace root!
2. `cd ~/ros2_ws/`
3. `colcon build --packages-select julia_turtlebot_circle_test`

Running
=======

1. Ensure that turtlebot is brought up. Run the following on the turtlebot:

```
$ ros2 launch turtlebot3_bringup robot.launch.py
```

2. Make sure you can see the turtlebot on your machine:

```
$ ros2 node list
/diff_drive_controller
/ld08_driver
/robot_state_publisher
/turtlebot3_node
```

3. Make sure you can see the topics on your machine:

```
$ ros2 topic list
/battery_state
/cmd_vel
/imu
/joint_states
/magnetic_field
/odom
/parameter_events
/robot_description
/rosout
/scan
/sensor_state
/tf
/tf_static
```

4. Run the following command to start the package and have turtlebot move based
   on calculations from a julia script:

```
$ ros2 run julia_turtlebot_circle_test launch_julia_turtlebot
```

Future Efforts
==============

The existing `launch_julia_turtlebot.py` script only runs a basic julia script
and reads data back. The v2, `launch_julia_turtlebot_v2.py`, will work with
vicon data and run more complex julia scripts.
