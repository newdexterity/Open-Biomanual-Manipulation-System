# Code for the RPi4 CAN passover

- [ ] Implement C++ version of CAN bus communication with ROS
- [ ] Improve control loop time

## Usage

Download and extract `moa_odrv_ros` folder to `catkin_ws/src/` on your RPi4
run catkin_make on your RPi4

## Acknowledgements

Adapted from following code sources:

- [ODrive main repo](https://github.com/neomanic/odrive_ros)
- [https://github.com/belovictor](https://github.com/belovictor/odrive_can_ros_driver)

# Note:

Change txquelen to 1 for the canbus settings to prevent delayed motion.

#
