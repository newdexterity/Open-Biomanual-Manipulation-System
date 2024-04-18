# Odrive CAN passover from ROS topic

The Modular Open-source Arm (MOA) uses the cansimple DBC to transfer data from ROS topics to the RPi4 hat for CAN
Please make sure the CAN hat is configured correctly before proceeding.
This is a basic pass at a ROS driver to operate the ODrive. It's Python-based, so not super-fast.
A faster C++ version is being examined but it won't be optimised anytime soon - dev branch. 


> [!Note]
> Remember to run chmod +x for the python scripts such as can_odrive_passover.py and bag2armhand.py

### Folder Directory
    moa_odrv_ros
    ├── ..
    ├── launch                                  
    │   ├── joint_pos_to_armhand.launch          # Launch file for playback rosbag files
    │   ├── serial_node_left.launch              # Launch file for left hand serial connection (needs rosserial installed)
    │   └── serial_nodes.launch                  # Launch file for both serial connection to hands (needs rosserial installed)
    ├── scripts
    │   ├── bag2armhand.py                       # Rosbag playback for arm and hand
    │   ├── can_dbc_example.py                   # Example can dbc from odrive
    │   ├── can_example.py                       # Example can code
    │   ├── can_odrive_passover.py               # Main code for passing ros topic to odrive canbus
    │   ├── create_can_dbc.py                    # Create custom can dbc
    │   ├── direct_motor_test.py                 # Publish test commands to rostopic - use this to test motors
    │   ├── odrive-cansimple.dbc                 # Odrive cansimple protocol dbc file
    │   ├── readCSV.py                           # Motion playback from CSV
    │   ├── readcsv2motor.py                     # Read saved csv and playback to ros topics
    │   ├── test.csv                             # Empty sample csv
    │   └── writeCSV.py                          # Records rostopics to csv
    ├── CHANGELOG
    ├── CMakeLists.txt
    ├── LICENSE
    ├── README.md
    ├── canbus_debug                            # snippets for canbus debugging
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

