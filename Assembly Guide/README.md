# Materials

## Components
The Bill_of_Materials.xlsx document contains the list of required components to assemble the bimanual robot. It also includes product numbers, prices and links to reseller webpages.
Most of the structure is designed to be 3D printed with reinforcing parts cut from carbon fibre plates. 
The original version was manufactured using the Prusa mk3 printers and standard filaments from Esun. Carbon fibre plates were cut using the desktop WAZER water jet cutter.
Some components are custom and require machining capabilities. These can be obtained from rapid prototyping services or from local machine shops. The BOM uses PCBWays for estimating the costs of the parts hence the price is more expensive than if the parts were locally printed. Additionally, the prices recorded are retail prices.
The BOM is organised with different pages for different modules :

* Bimanual_Robot_Arm_TypeA : The cost of building both arms, including the elbow and forearm rotation module TypeA (Power system not included)
* Stand : The base stand for the robot
* One_Unit_of_Forearm_ASL_Hand : Cost of building ONE ASL hand module
* One_Unit_of_Dynamixel_Hand : Cost of building ONE Dynamixel hand module
* One_Unit_of_Robotic_Wrist : Cost of building ONE Robotic wrist module
* Alternative_actuators : Miscellaneous items
* Power_supply : WIP

### WARNING: It is recommended to use the AMT102 series incremental encoders instead of the AMT232V absolute encoders
This is to make life easier without having to deal with noise in the SPI line. Otherwise a custom tristate buffer is required for correct operation of the encoder. (74AHC1G125S)

# Getting Started
## Things you will need
* Main computer (intel NUC7)
* DCHP router
* Raspberry pi 4
* Ethernet cables
  
## Multi-machine ROS setup
* Please follow the wiki for software setup
Connect the ethernet as depicted in the following diagram:
WIP
<p align="center">
  <img src = ../docs/connection_schematic.png>
</p>

goto your router dchp configuration page and reserve the IP for each machine using their MAC addresses
Setup ROS_IP and ROS_MASTER accordingly
link to ros multimachine tutorial page:

## ODriver Connection
The ODrive motor drivers can be connected directly to the BLDC motors and require a parallel CAN bus connection to the CANbus interface with the Raspberry Pi GPIO hat.
It is recommended to create a small connector hub for the connections for easier assembly and wire routing.
Motor power can be supplied directly from the battery but also from an isolated power supply. The following image shows the power schematic and the communication schematic of the system

WIP
<p align="center">
  <img src = ../docs/connection_schematic.png>
</p>

## Forearm Module Electronics

WIP

## Teleoperation controls

### Polhemus

### Captoglove
