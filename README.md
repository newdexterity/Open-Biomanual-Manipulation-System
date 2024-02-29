# The New Dexterity Open-Source Bimanual Manipulation System for Education and Research
Open Source Anthropomorphic Bimanual Manipulator Robot project initiated by New Dexterity

<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_arm_main.jpg" width="800">

## Repository Contents
This repository contains the following material:

* Assembly guide folder - [Wiki](../../wiki)
* CAD folder - Design files of the bimanual platform for manufacturing
* Software folder - Interface for joint control - WIP (Sorting through code)

### Folder Directory
    .
    ├── ..
    ├── Assembly guide
    │   ├── Bill_of_Materials.xlsx               # Bill of Materials (WIP)        
    │   └── README.md
    ├── CAD                                      # Software folder for various subsystems
    │   ├── ...                                  
    │   ├──                                      # Various submodules of the OpenBMP system
    │   ├── ...   
    │   ├── README.md
    │   └── open_bmp_asm.7z                      # Assembled robot CAD
    ├── Software                                 # Software folder for various subsystems
    │   ├── Arduino_ESP32                        # Arduino code for operating the forearm and human-like hands for the ESP32
    │   ├── CaptoGloveDocker                     # Captoglove Docker scripts for installing and interfacing with ROS (Bluetooth)
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

# About the Modular Open-source Arm Hand (MOAH) Hybrid-drive Bimanual Arm Hand System
## The Mission

The modular, dexterous, anthropomorphic bimanual manipulation system is a collaborative open-source robot manipulator project initiated by the New Dexterity group aimed to provide a low-cost research platform for studying bimanual manipulator systems and experimental actuating systems or kinematic configurations. It follows the footsteps of ARoA, a humanoid Autonomous Robotic Assistant, and the Modular Open-source Arm Hand (MOAH) is a single-arm manipulator system upgraded from the original ARoA manipulators.

## System Overview
The system utilizes rapid prototyping technologies such as FDM printing and waterjet cutting to manufacture robust custom parts through carbon fiber plate-reinforced 

The bimanual platform is equipped with a pair of modular, 5 DoF upper, mixed-drive manipulators with four axially driven joints and one tendon-driven joint in each arm. A 1:100 reduction harmonic drive from HanZhen Technology was used in each direct axial drive joint, enabling standard connectors between the various modules. The Bowden cable tendon-driven elbows allow the actuating mechanism to be placed away from the joint, reducing the arm's inertial moments. Each upper arm shoulder unit joint is driven by a dual shaft brushless DC (BLDC) motor, the D6374-150KV, from ODrive robotics. The position feedback employs absolute encoders, AMT232B-V from CUI Devices, to provide compact, accurate, and fast position controls. The torso structure was designed to be mounted on a pair of opposing linear rails to access a larger reachable workspace similar to the variable torso of the New Dexterity ARoA humanoid. 

* Lightweight and reconfigurable structure
* Bowden drive joints
* Box-style structure for easy access and maintenance
* Open-source ODrive Controllers for BLDC motors

## Robot Hardware Overview

### Foldable Shoulder
The foldable shoulder enables re-orientation of the manipulator bases, allowing for a larger intersecting bimanual workspace. This demonstrates how the modular platform can be modified for different kinematic designs through different actuator orientations. With detachable modules and 3D-printed sandwiched plate structures, the layout of the system can be quickly reconfigured. For example, a raised shoulder abduction module can be used to fit different types of robotic configurations and designs. 

<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_folding_image.jpg" width="800">

### Joint Locking
A specialized joint locking mechanism is used in both the shoulder and elbow flexion joints, mitigating motor stress during high-load manipulation tasks such as holding extended poses for long periods. The ratchet lock systems employed enable the joints to maintain their position with minimal energy expenditure, thereby reducing the overall torque exerted on the motors. The curved design also allows gravity-assisted locking modifications for future iterations.
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_elbow_image.jpg" width="800">

### Low-inertial Manipulators and Experimental Transmissions
The system incorporates two types of reducer transmission mechanisms that were used to demonstrate the versatility and capability of the platform. One is a directly coupled tendon drive onto the harmonic drive output with a total reduction of approximately 61.17. The other is a simple pulley system routed around central idler wheels for the compactness of a single-joint drive. The pulley system consists of an antagonistic 1:6 reduction pulley set with an output tendon driver reduction of 2.6. The total reduction is approximately 15.6. Both transmission systems use Bowden cables to route the tendons to the elbow joint and are fitted with in-line cable tensioners. The design enables easy exchange or modification of the elbow module without disassembling the shoulder modules and also allows for the replacement of Bowden cable sets through slotted in-line cable tensioners without requiring disassembly of the robot. 

## The Design

The torso and bi-manual upper arm unit were constructed using 3D printed polylactic acid (PLA) high-strength filament parts, which were reinforced with carbon fibre composite plates in regions subjected to high loads. This choice of materials enables rapid prototyping and facilitates easy design modifications via Fused Deposition Modeling (FDM) 3D printing and waterjet cutting. The rigidity of the carbon fibre plates prevents structural deformation of the PLA parts, and the conductive nature provides grounding planes in the chassis. Additionally, certain modules were structurally reinforced using aluminium standoffs, and a machined keyed shaft adapter drove each motor-harmonic drive module.

Here is an overview of the structural parts required for assembling the torso and one of the upper arm units:
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_parts_full.jpg" width="800">

### Torso Base Unit
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_base_cad.jpg" width="800">

### Torso Clavicle Unit
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_shoulder_exp.jpg" width="800">

### Shoulder Unit
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_cad_shoulder_abd.jpg" width="800">

### Shoulder Rotation Unit
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_cad_shoulder_rot.jpg" width="800">

### Elbow Unit
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_elbow_cad_explode.jpg" width="800">

### Forearm Modules
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_forearm_rot_modules.jpg" width="800">

### Wrist/Hand Modules
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_hand_wrist_modules.jpg" width="800">

## Robot Manipulator specifications
* Reachable arm length: 1000 mm
* Single arm weight: 12.4 kg 
* Software requirements: ROS Melodic/Noetic
* Multimachine communication: Ethernet via router
* CAN bus communication: CANbus hat via SPI with RPi4 using ROS noetic for arm64
  
### Hardware costs (approximate) (USD):
>    ### Bimanual robot arm platform (without forearm module):
>         $11600
>    ### Single ASL hand (servo Ver.)
>         $450
>    ### Single Dynamixel hand
>         $1920
>    ### Single unit of robotic wrist module
>         $1230

### Theoretical capabilities:
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/Openbmp_Specs.jpg" width="800">

## Computer and Electronics

* Robot CAN controller node: Raspberry Pi 4 Model B with custom MCP2515 CAN controller and MCP2551 CAN transceiver hat
* Main Computer: Intel NUCi7 or similar with Ubuntu 18.04 with ROS melodic - 20.04 noetic should work
* Router: TP-link Archer VR300 (any router would do)
* Power supplies: assorted 6V, 12V, 24V or 48V power supplies
* Microcontrollers: ESP32 boards for ROS control for forearms and hands
* MAX485 transceiver boards for RS485 communication with Dynamixel motors
* Vero boards for custom interfacing
* Tri-state buffer (74AHC1G125S) for AMT232 SPI communication with ODrive
* Old Ethernet cables for signal wiring
* Assorted cables for power wiring

## Teleoperation
The robot can accept a variety of input controls via the ROS framework. The base system demonstrates the integration of different telemanipulation controllers with the Captoglove and the Polhemus magnetic system.

## Power Distribution System - WIP
The 56V ODrive controllers can operate motors with voltages up to 56V and peak currents of 120A per motor. However, for continuous operation, it is recommended to operate at a much lower current.

Heatsink only in still air: 40A
Heatsink with basic fan cooling: 75A
Heatsink with overkill fan cooling: 90A

Retrieved from :
[Details](https://discourse.odriverobotics.com/t/odrive-mosfet-temperature-rise-measurements-using-the-onboard-thermistor/972)

The base system limits current supply at 10A and voltage at 24V for safety in testing and operation. A higher current and voltage bypass is possible via the optional 48V battery pack. 
Here's a link to a nice article on power source selection:
[Details for custom powersources](https://things-in-motion.blogspot.com/2018/12/how-to-select-right-power-source-for.html)

https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet

<p align="justify"> The project is distributed under the Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0) </p>

[CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/)
