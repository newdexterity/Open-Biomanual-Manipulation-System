# ARCHIVE NOTICE 2021-02-04
I have taken on a new position no longer have access to an ODrive for testing. Happy to add a maintainer role if anyone wants to step up to it, otherwise this repo will just stay up here as a reference for other implementations. 

# odrive_ros
ROS driver for the [ODrive motor driver](https://odriverobotics.com/)

This is a basic pass at a ROS driver to operate the ODrive as a differential drive. It's Python-based, so not super-fast, but it'll get you started. Maybe in time this will have an optimised C++ version, but I woudn't count on it anytime soon. ;)

Future plans include adding a simulation mode so you can continue to use this driver in a simulation mode within gazebo.

Feedback, issues and pull requests welcome.

## Usage

You will need the main ODrive Python tools installed.

To install:
```sh
git clone https://github.com/madcowswe/ODrive
cd ODrive/tools
sudo pip install monotonic # required for py < 3

# sudo python setup.py install # doesn't work due to weird setup process, so do the following:
python setup.py sdist
sudo pip install dist/odrive-xxx.tar.gz
```

then `rosrun odrive_ros odrive_node` will get you on your way. 

There is a sample launch file showing the use of the wheelbase, tyre circumference, and encoder count parameters too, try `roslaunch odrive_ros odrive.launch` or copy it into your own package and edit the values to correspond to your particular setup.

If you want to test out the driver separate from ROS, you can also install as a regular Python package.

```sh
roscd odrive_ros
sudo pip install -e .
```

If you want to use a bare Python shell, see below for the import which exposes the ODrive with setup and drive functions. You can use either ODriveInterfaceAPI (uses USB) or ODriveInterfaceSerial (requires the /dev/ttyUSBx corresponding to the ODrive as parameter, but not tested recently).

```python
from odrive_ros import odrive_interface
od = odrive_interface.ODriveInterfaceAPI()
od.connect()
od.calibrate()      # does a calibration
od.engage()         # get ready to drive
od.drive(1000,1000) # drive axis0 and axis1
od.release()        # done
```

You can now (as of v0.5) use this wrapper from within an `odrivetool shell`. Once the shell has launched and connected to your ODrive, run the following, then you can use the commands as if in a bare Python interactive session above, and still get the help provided by odrivetool.

```python
import odrive_interface
od = odrive_interface.ODriveInterfaceAPI(active_odrive=odrv0)
```


## Acknowledgements

Thanks to the ODrive team for a great little bit of hardware and the active community support. Hope this helps!

- [ODrive homepage](https://odriverobotics.com)
- [ODrive getting started](https://docs.odriverobotics.com)
- [ODrive main repo](https://github.com/madcowswe/ODrive)

Thanks to Simon Birrell for his tutorial on [how to package a Python-based ROS package](http://www.artificialhumancompanions.com/structure-python-based-ros-package/).

# odrive_can_ros_drive

## Overview

ROS driver for the [ODrive](https://odriverobotics.com) motor controller connecting through CAN bus

This is a basic driver node for ROS which builds an interface to control multiple [ODrive](https://odriverobotics.com) motor controllers through a CAN bus. This driver depends on [socketcan_bridge](http://wiki.ros.org/socketcan_bridge) package to connect to physical CAN bus.

Right now driver only supports velocity control mode as this was my prime area of interest for my own project but it can be extended to provide support for other modes of control. I would be happy to accept any pull requests extending funtionality of this driver.

This driver was successfully tested with multiple [ODrive](https://odriverobotics.com) controllers on a CAN bus connected to [Raspberry Pi 4](https://www.raspberrypi.com/products/raspberry-pi-4-model-b/) through [Waveshare CAN hat](https://www.waveshare.com/rs485-can-hat.htm) on Ubuntu 18.04 and 20.04 with ROS melodic and noetic.

## Usage

### Getting the code

To use the driver just check it out to your ROS workspace:
```
git clone https://github.com/belovictor/odrive_can_ros_driver.git
```

### Dependancies

The driver depends on socketcan_bridge package. Please install it prior to using the driver:
```
sudo apt install ros-<ros distribution name>-socketcan-bridge
```
The package name would depend on your ROS version. This driver has been tested and is known to work with melodic and noetic.

### Configuration

Driver is configured with the following configuration paramters:

| Parameter name          | Type     | Description                                                                        |
|-------------------------|----------|------------------------------------------------------------------------------------|
| ```can_device```        | String   | CAN device name as it appears on ifconfig interface list, default is can0          |
| ```can_rx_topic```      | String   | Topic name through which driver receives CAN messages from socketcan_bridge        |
| ```can_tx_ropic```      | String   | Topic name through which driver sends CAN messages to socketcan_bridge             |
| ```update_rate```       | Double   | Update rate in Hz of how frequently driver gets data from ODrive controllers       |
| ```engage_on_startup``` | Boolean  | If driver should engage all motors on startup (not implemented yet)                |
| ```axis_names```        | String[] | Array of names of axises which are then used to name topics corresponding to axises|
| ```axis_can_ids```      | Int[]    | Array of axis CAN ids as they are configured on ODrive controllers for every axis  |
| ```axis_directions```   | String[] | Axis direction either ```forward``` or ```backward```, ```backward``` reverses axis control, angle and current velocity |

An example configuration is presented in config/odrive.yaml.

### Running the driver

```
roslaunch odrive_can_ros_driver odrive_can_ros.launch
```

### How it works?

On the high level this driver has very simple architecture. On one hand it interacts with CAN bus through 2 topics (one for receiving CAN frames and one for transmitting them) provided by socketcan_bridge node. On the other hand it provides several topics for every ODrive axis defined in the configuration for other nodes like controllers and hardware interfaces to interact with. The example configuration file provided with the driver shows how to connect to a 6WD chasis based on 3 ODrive controller boards. The axises are addressed through CAN bus according to their axis IDs configured on ODrive controller through ```<odrv>.<axis>.config.can.node_id``` attribute

When driver is started it creates the following topics for every axis configured:

| Topic name                        | Message type           | Direction | Description                            |
|-----------------------------------|------------------------|-----------|----------------------------------------|
|```<axis name>/angle```            | ```std_msgs/Float64``` | Outbound  | Angle of the axis in rad               |
|```<axis name>/current```          | ```std_msgs/Float64``` | Outbound  | Axis current consumption               |
|```<axis name>/current_velocity``` | ```std_msgs/Float64``` | Outbound  | Current axis angular velocity in rad/s |
|```<axis name>/target_velocity```  | ```std_msgs/Float64``` | Inbound   | Target axis angular velocity in rad/s  |
|```<axis name>/voltage```          | ```std_msgs/Float64``` | Outbound  | Axis voltage of power supply in V      |

So basically you can control every axis by setting the target velocity and monitor axis activity/performance through current angle and velocity and also monitor power supply and consumption through voltage and current.

