# The New Dexterity Open-Source Bimanual Manipulation System for Education and Research
Open Source Anthropomorphic Bimanual Manipulator Robot project initiated by New Dexterity

<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_arm_main.jpg" width="800">

# About the Modular Open-Architecture Hybrid-drive (MOAH) Bimanual Arm Hand System
## The Mission

The modular, dexterous, anthropomorphic bimanual manipulation system is a collaborative open source robot manipulator project initiated by the New Dexterity group aimed to provide a low cost research platform for studying bimanual manipulator systems and experimental actuating systems or kinematic configurations. It follows the footsteps of ARoA, a humanoid Autonomous Robotic Assistant.

## System Overview
The system utilizes rapid prototyping technologies such as FDM printing and waterjet cutting to manufacture robust custom parts through carbon fiber plate reinforced 

The bimanual platform is equipped with a pair of modular, 5 DoF upper, mixed-drive manipulators with four axially driven joints and one tendon-driven joint in each arm. A 1:100 reduction harmonic drive from HanZhen Technology was used in each direct axial drive joint, enabling standard connectors between the various modules. The Bowden cable tendon-driven elbows allow the actuating mechanism to be placed away from the joint, reducing the arm's inertial moments. Each upper arm shoulder unit joint is driven by a dual shaft brushless DC (BLDC) motor, the D6374-150KV, from ODrive robotics. The position feedback employs absolute encoders, AMT232B-V from CUI Devices, to provide compact, accurate, and fast position controls. The torso structure was designed to be mounted on a pair of opposing linear rails to access a larger reachable workspace similarly to the variable torso of the New Dexterity ARoA humanoid. 

* Lightweight and reconfigurable structure
* Bowden drive joints
* Box style structure for easy access and maintainence
* Open-Source ODrive Controllers for BLDC motors

## Repository Contents - WIP
This repository contains the following material:

* Assembly guide and bill of materials - WIP
* CAD files of the bimanual platform - WIP
* ROS interface for joint control - WIP

### Foldable Shoulder
The foldable shoulder enables re-orientation of the manipulator bases allowing for a larger intersecting bimanual workspace. This also demonstrates how the modular platform can be modified for different kinematic designs through different actuator orientation. With detachable modules and 3D-printed sandwiched plate structures, the layout of the system can be quickly reconfigured. For example, a raised shoulder abduction module can be used to fit different types of robotic configurations and designs. 

### Joint Locking
A specialized joint locking mechanism is used in both the shoulder and elbow flexion joints, mitigating motor stress during high-load manipulation tasks such as holding extended poses for long periods of time. The ratchet lock systems employed enable the joints to maintain their position with minimal energy expenditure, thereby reducing the overall torque exerted on the motors. 
<img src="https://raw.githubusercontent.com/newdexterity/Open-Biomanual-Manipulation-System/master/images/readme/moah_elbow_image.jpg" width="800">

### Low-inertial Manipulators and Experimental Transmissions
The system incorporates two types of reducer transmission mechanisms that were used to demonstrate the versatility and capability of the platform. One is a directly coupled tendon drive onto the harmonic drive output with a total reduction of approximately 61.17. The other is a simple pulley system routed around central idler wheels for the compactness of a single-joint drive. The pulley system consists of an antagonistic 1:6 reduction pulley set with an output tendon driver reduction of 2.6. The total reduction is approximately 15.6. Both transmission systems use Bowden cables to route the tendons to the elbow joint and are fitted with in-line cable tensioners. The design enables easy exchange or modification of the elbow module without disassembling the shoulder modules and also allows for the replacement of Bowden cable sets through slotted in-line cable tensioners without requiring disassembly of the robot. 

## The Design

The torso and bi-manual upper arm unit were constructed using 3D printed polylactic acid (PLA) high-strength filament parts, which were reinforced with carbon fibre composite plates in regions subjected to high loads. This choice of materials enables rapid prototyping and facilitates easy design modifications via Fused Deposition Modeling (FDM) 3D printing and waterjet cutting. The rigidity of the carbon fibre plates prevents structural deformation of the PLA parts, and the conductive nature provides grounding planes in the chassis. Additionally, certain modules were structurally reinforced using aluminium standoffs, and a machined keyed shaft adapter drove each motor-harmonic drive module.

Here is an overview of the structural parts required for assembling the torso and one of the upper arm unit:
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
WIP

## Robot Manipulator specifications
* Reachable arm length: 1000 mm
* Single arm weight: 12.4 kg
* Hardware cost: ~$11000 USD
* Software requirements: ROS Melodic/Noetic
* Multimachine communication: Ethernet via router
* CAN bus communication: MCP2515 via RPi4 with ROS noetic for arm64

## Hardware - WIP
* ??x M3x12 mm Hex screw

## Software - WIP

## Teleoperation - WIP
The robot can accept a variety of input controls via the ROS framework. The base system demonstrates the integration of different telemanipulation controllers with the Captoglove and the Polhemus magnetic system.

## Power Distribution System - WIP
The 56V ODrive controllers can operate motors with voltages up to 56V and peak currents of 120A per motor. However for continous operation, it is recommended to operate at a much lower current.

Heatsink only in still air: 40A
Heatsink with basic fan cooling: 75A
Heatsink with overkill fan cooling: 90A
[Details](https://discourse.odriverobotics.com/t/odrive-mosfet-temperature-rise-measurements-using-the-onboard-thermistor/972)

The base system limits current supply at 10A and voltage at 24V for safety in testing and operation. A higher current and voltage bypass is possible via the optional 48V battery pack. 
[Details for custom powersources](https://things-in-motion.blogspot.com/2018/12/how-to-select-right-power-source-for.html)

<p align="justify"> The project is distributed under the Creative Commons Attribution-NonCommercial 2.0 Generic (CC BY-NC 2.0) </p>
[https://creativecommons.org/licenses/by-nc/2.0/](https://creativecommons.org/licenses/by-nc/2.0/)
