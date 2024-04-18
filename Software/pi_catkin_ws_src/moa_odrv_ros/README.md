# Odrive CAN passover from ROS topic

The Modular Open-source Arm (MOA) uses the cansimple DBC to transfer data from ROS topics to the RPi4 hat for CAN
Please make sure the CAN hat is configured correctly before proceeding.
This is a basic pass at a ROS driver to operate the ODrive. It's Python-based, so not super-fast.
A faster C++ version is being examined but it won't be optimised anytime soon - dev branch. 

### Folder Directory
    moa_odrv_ros
    ├── ..
    ├── launch                                  
    │   ├── joint_pos_to_armhand.launch          # Launch file for playback rosbag files
    │   ├── serial_node_left.launch              # Launch file for left hand serial connection (needs rosserial installed)
    │   └── serial_nodes.launch                  # Launch file for both serial connection to hands (needs rosserial installed)
    ├── src/moa_odrv_ros
    │   ├── bag2armhand.py                       # Rosbag playback for arm and hand
    │   ├── can_dbc_example.py                     # Captoglove Docker scripts for installing and interfacing with ROS (Bluetooth)
    │   ├── can_example.py                     # ODrive sample settings for setting up the ODrives (WIP)
    │   ├── can_odrive_passover.py                           # Main package for simulation, visualization, and operation of the robot
    │   ├── create_can_dbc.py                     # Code for the RPi4 CAN passover module and setup of the RPi4 for multimachine ROS
    │   ├── direct_motor_test.py                          # *** Use this for install instructions ***
    │   ├── odrive-cansimple.dbc
    │   ├── readCSV.py                         # RPi4 config for SPI bridge to CANBUS controller
    │   ├── readcsv2motor.py                          # RPi4 config for SPI bridge to CANBUS controller
    │   ├── test.csv
    │   └── writeCSV.py                          # RPi4 config for SPI bridge to CANBUS controller and hdmi output settings
    ├── CHANGELOG
    ├── CMakeLists.txt
    ├── LICENSE
    ├── README.md
    ├── canbus_debug
    ├── package.xml
    └── setup.py



# odrive_ros testing
It's a good idea to install the official odrive_ros code for testing:

ROS driver for the [ODrive motor driver](https://odriverobotics.com/)
odrive_ros repository by neomanic: [odrive_ros](https://github.com/neomanic/odrive_ros)

## Acknowledgements
- [ODrive homepage](https://odriverobotics.com)
- [ODrive getting started](https://docs.odriverobotics.com)
- [ODrive main repo](https://github.com/madcowswe/ODrive)
- [Odrive_ros repo by neomanic](https://github.com/neomanic/odrive_ros)

