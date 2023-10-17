# Captoglove_ros_msgs

Package that contains Captoglove ROS msgs. Purpose of this package is to 
decouple [captoglove_ros_wrapper](https://github.com/fzoric8/captoglove_ros_msgs) (driver wrapper for CaptoGlove) and 
[captoglove_ros_control](TODO:Add link) (higher level UAV control based on input from CaptoGlove).

Currently contains: 

 * FingerFeedbackMsg -> raw measurements from CaptoGlove bending sensors
 * BatteryLevel -> raw measurement for Battery Level from CaptoGlove 
 * DeviceInfo -> device name (to make sure that we're using right/left glove)  

