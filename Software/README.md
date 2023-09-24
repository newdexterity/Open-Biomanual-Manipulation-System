# ISO image for RPi4
a pre-configured ISO for the RPi4. Steps for custom configurations such as boot-previlages, network settings, SPI settings, and setup of the RPI4 is currently a WIP.

# moah_ws

Example ROS interface to control the bimanual manupulator system.

## Dependencies


# CaptoGloveDocker

The bluetooth communication is setup within a docker and can be installed follwing the steps listed below :

* WIP

## Setting Up ODrive Controllers
Please make sure to go through [getting started](https://docs.odriverobotics.com/v/0.5.6/getting-started.html) page for setting up and tuning the ODrive motors

### Setting commands and motor numbering


## System Networking
### Main Computer (with moah_ws)

```
ROS_MASTER =
```

```
ROS_IP =
 ```
### RPi4 ROS to CANbus passover

```ROS_MASTER = ```
```ROS_IP = ```

## Virtual Interface Usage
### On the Main Computer (with moah_ws)
* ToDo - add a screenshot of the terminator split screen
First start roscore

```
roscore
```

Then run the following command to bring up the visualizer.

```
roslaunch
```

If you wish to test the robot model, edit the `view_arm.launch` files and uncomment the GUI controls for slider control of the robot arms


To bringup the controllers, run the following

```
roslaunch
```

To start the motion remapping via functional anthropomorphism, run the following commands to start the controller for each of the arms.

```
roslaunch
```

```
roslaunch
```

### Connection to Input Interfaces
The robot system is designed for the hybrid glove setup with individual controls for the hand and arm. 
The Captoglove bluetooth interface is setup within a Dockerfile. 

If the Captoglove is not showing blue, (most likely orange or magneta) press and hold the button until an orange light flashes after the light turns off (approx, 9 secs). This should set the captoglove into a white light indicator for it to be in pairing mode.

change your directory to the installed docker folder
```
cd CaptoGloveDocker
```

start docker

```
./start melodic_11
```

make sure dbus connection is successful

#### Connecting Captoglove
```
bluetoothctl
```

```
scan on
```

Once the MAC address and captoglove appears, stop the scan and connect to the glove
```
scan off
```

```
connect XX:XX:XX:XX:XX
```

Remember to pair the device before exiting bluetoothctl
```
pair
```

```
exit
```


Now run the captoglove wrapper. The finger flexion positions are published to `\leftservo_recorded` topic
```
roslaunch
```

To start the Polhemus, please use the provided package and run the following command.
NOTE : For calibration purposes, only turn on the Polhemus machine once the user wearing the sensor is in the initial position. Once the liberty system indication light turns green, run the following command to start the Polhemus system.

```
roslaunch
```

Once the markers are found, you should see the markers published under `\tf` in RViz


## Connecting to Physical Robot
On the RPi4, once multi-machine ROS is setup, run the following command to start passing the commands from the topics `\leftarm\position_controller\command` and `\rightarm\position_controller\command` over onto the CANbus network where the ODrive boards will try to move to the target positions.

```
rosrun
```

For faster communications, remember to change the `txqlen` to 1 for the can interface

```
sudo ip .....
```
