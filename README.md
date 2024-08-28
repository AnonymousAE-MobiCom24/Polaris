# The source code of the artifact submission for Polaris in MobiCom'24

This repository contains the artifacts and guidelines for ACM Badging in the MobiCom 2024 proceedings.

# Overview
We present Polaris, the first vision-free fiducial marking system, based on a novel, full-stack magnetic sensing design. 
Polaris can achieve reliable and accurate pose estimation and contextual perception, even in NLOS scenarios. 

Its core design includes: (1) a novel digital modulation scheme, Magnetic Orientation-shift Keying (MOSK) that can encode key information like waypoints and coordinates with passive magnets; (2) a robust and lightweight magnetic sensing framework to decode and localize the magnetic tags. 
Our design also equips Polaris with three key features: sufficient encoding capacity, robust detection accuracy, and low energy consumption. 
We have built an end-to-end system of Polaris and tested it extensively in real-world scenarios. The testing results have shown Polaris to achieve an accuracy of up to 0.58 mm and 1&deg; in posture estimation with a power consumption of only 25.08 mW.

![plot](./Imgs/illustration.png)


# Setup
To use Polaris, the following steps are required:
* [Building the hardware](#building-the-hardware)
* [Programing the sensor array](#programing-the-sensor-array)
* [Running the sensing pipeline](#running-the-sensing-pipeline)

## Building the hardware
We provide the hardware requirements and manufacturing details of Polaris' sensing array in `/PCBs`, please see the related [README](./PCBs/README.md) file.

## Programing the sensor array
Based on the hardware, we use Arduino IDE to program the sensor array.
### Arduino IDE setup
1. Download and install [Arduino IDE](https://www.arduino.cc/en/software)
2. Set up the Arduino IDE for the nRF52 Development Board, according to the [official instruction](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/arduino-bsp-setup).
Once completed, you will be able to access the board information by clicking on the 'Tools' menu
  ![plot](./Imgs/nRF52_arduino_configuration.png)
3. Install the Adafruit MLX90393 Library for the Arduino IDE:
    - Click 'Sketch' --> 'Include Library' --> 'Manage Libraries...'
    ![plot](./Imgs/sensor_arduino_library.png)
    - Search for Adafruit MLX90393, and install the Adafruit MLX90393 library:
    ![plot](./Imgs/install_MLX90393_library.png)


### Programing the sensor array
to activate the 


## Running the sensing pipeline


# Run a quick demo
You can find a quick demo in the `/Quick_Demo` directory. For more information, please refer to the related [README](./Quick_Demo/README.md) file.