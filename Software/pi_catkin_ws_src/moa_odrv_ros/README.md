# Odrive CAN passover from ROS topic

The Modular Open-source Arm (MOA) uses the cansimple DBC to transfer data from ROS topics to the RPi4 hat for CAN
Please make sure the CAN hat is configured correctly before proceeding.
This is a basic pass at a ROS driver to operate the ODrive. It's Python-based, so not super-fast.
A faster C++ version is being examined but it won't be optimised anytime soon - dev branch. 

### Folder Directory
    moa_odrv_ros
    ├── ..
    ├── config
    │   └── odrive.yaml                          # Setting file for socketcan_bridge method
    ├── launch                                  
    │   ├── joint_pos_to_armhand.launch          # Launch file for playback rosbag files (Need USB_interface_code moved to src)
    │   ├── odrive_can_ros.launch                # Launch file for socketcan_bridge communication method with Odrive (Need USB_interface_code moved to src)
    │   ├── serial_node_left.launch              # Launch file for left hand serial connection (needs rosserial installed)
    │   └── serial_nodes.launch                  # Launch file for both serial connection to hands (needs rosserial installed)
    ├── nodes
    │   └── odrive_node                          # Node file for starting Odrive node(Need USB_interface_code moved to src)
    ├── src/moa_odrv_ros
    │   ├── USB_interface_code                   # Experimental code for socketcan_bridge
    │   │   ├── __init__.py
    │   │   ├── bag2armhand.py                   # Rosbag playback for arm and hand
    │   │   ├── __init__.py
    │   │   ├── __init__.py
    │   │   ├── __init__.py
    │   │   ├── __init__.py
    │   │   ├── __init__.py
    │   │   ├── __init__.py

    
    │   ├──CaptoGloveDocker                     # Captoglove Docker scripts for installing and interfacing with ROS (Bluetooth)
    │   ├── Odrive_Settings                      # ODrive sample settings for setting up the ODrives (WIP)
    │   ├── moah_src                             # Main package for simulation, visualization, and operation of the robot
    │   ├── pi_catkin_ws_src                     # Code for the RPi4 CAN passover module and setup of the RPi4 for multimachine ROS
    │   ├── README.md                            # *** Use this for install instructions ***
    │   ├── Screenshot 2023-10-12 15_46_23.png
    │   ├── config.txt                           # RPi4 config for SPI bridge to CANBUS controller
    │   ├── syscfg.txt                           # RPi4 config for SPI bridge to CANBUS controller
    │   └── usercfg.txt                          # RPi4 config for SPI bridge to CANBUS controller and hdmi output settings
    ├── LICENSE
    └── README.md



# odrive_ros testing
It's a good idea to install the official odrive_ros code for testing:

ROS driver for the [ODrive motor driver](https://odriverobotics.com/)
odrive_ros repository by neomanic: [odrive_ros](https://github.com/neomanic/odrive_ros)

## Acknowledgements
- [ODrive homepage](https://odriverobotics.com)
- [ODrive getting started](https://docs.odriverobotics.com)
- [ODrive main repo](https://github.com/madcowswe/ODrive)
- [Odrive_ros repo by neomanic](https://github.com/neomanic/odrive_ros)

