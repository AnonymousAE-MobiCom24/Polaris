# Hardware Design of Polaris' Sensor Array

## Description
This directory contains the comprehensive PCB design for Polaris' sensor array, including the sensor bar for fiducial detection and a flashing board designed to program the MCU (). 

"SensorArray_RobotCar" refers to the PCB design of the sensor array for the robot car.
"SensorArray_MiniCar" referes to the PCB design of the sensor array for the mini car.
"Flashing_Module" refers to the flashing board for programming the MCU to obtain data readings of each magnetometers.

## Hardware requirements
The hardware setup requires various components during the design and fabrication process. 
Below is a list of components we used for designing and manufacturing our prototype.

For the sensor array:
| Component  | Link |
| ------------- | ------------- |
| FR4 PCB sheet  |  |
| xxx  | xxx  |

For the flashing module:
| Component  | Link |
| ------------- | ------------- |
| FR4 PCB sheet  |  |
| xxx  | xxx  |

For each module, we also provide an interactive HTML BOM file for reference.


## Design tool and library
We utilize [Altium Designer](https://www.altium.com/altium-designer) (version 20.0.13) to design PCB boards.
The component library of capacitive and resistance relies on the standard library.
The footprint of the MLX90393 magnetometer we used, can see https://www.snapeda.com/parts/MLX90393SLW-ABA-011-RE/Melexis%20Technologies/view-part/551380/?ref=search&t=MLX90393