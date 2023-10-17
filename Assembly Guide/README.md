# Getting Started

## Components
The Bill_of_Materials.xlsx document contains the list of required components to assemble the bimanual robot.
It also includes product numbers, prices and links to reseller webpages.
Most of the structure is designed to be 3D printed with reinforcing parts cut from carbon fiber plates. 
The original version were manufactured using the Prusa mk3 printers and standard filaments from esun. Carbon fiber plates were cut using the desktop WAZER water jet cutter.
Some components are custom and requires machining capabilities, these can be obtained from rapid prototyping services or local machine shops.

## ODriver Connection
The ODrive motor drivers can be connected directly to the BLDC motors and requires a parallel CAN bus connection to the CANbus interface with the Raspberry Pi GPIO hat.
It is recommended to create a small connector hub for the connections for easier assembly and wire routing.
Motor power can be supplied directly from the battery but also from an isolated power supply.

<p align="center">
  <img src = ../docs/connection_schematic.png>
</p>
