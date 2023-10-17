# Polhemus Liberty ROS Driver

This package publishes coordinate transforms for the Polhemus Liberty sensors (stations) using the [`tf2` package](http://wiki.ros.org/tf2).

The user manual for the Polhemus Liberty can be found [here](http://polhemus.com/_assets/img/LIBERTY_User_Manual_URM03PH156-H.pdf).

# Install 

Do not make changes to any files in `polhemus/etc`. Install requirements using:

```
$ cd ./etc
$ ./install.bash
```

Note that you will require the `sudo` password. 

# Usage

## Basic

1. `$ roscore`.
2. Open a new terminal.
3. `$ roslaunch polhemus_ros_driver start.launch`

Note: it may take 2-3 attempts to successfully start broadcasting `tf2` frames.

## View frames

Open a new terminal. Start RVIZ (`$ rosrun rviz rviz`).

1. Global options: change *Fixed Frame* to `polhemus_base`. Note that the Polhemus z-frame faces downwards and you need to publish a static transform to flip the orientation e.g. to polhemus_world (cf. example in launch).
2. Add -> *By display type* -> rviz -> TF.

# Launch file

The launch file `start.launch` allows you to change the zenith of the hemisphere.

# Requirements

* ROS Indigo, see `package.xml`
* `libusb-dev`
* `fxload`
