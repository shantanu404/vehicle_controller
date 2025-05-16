# Vehicle Control System Experiments

Developed for lane following applications and ADAS using monocular vision. Tested on both ROS2 humble and jazzy.

### How to run

```bash
$ source /opt/ros/jazzy/setup.bash # Or use ros2 `humble`
$ cd ~/ros2_ws/src
$ git clone https://github.com/shantanu404/vehicle_controller.git
$ pip3 install requirments.txt # You can use `virtualenv` too
$ cd ~/ros2_ws
$ colcon build # do not use symlink-install if you are using `virtualenv`
$ source install/local_setup.bash
$ ros2 launch vehicle_controller mpc.launch.py
```
