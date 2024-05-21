# Overview
This file contains code for the arbiter teensy. This microcontroller acts as a bus controller between the jetson and all of the other arduinos on C1C0. Due to serial port conflicts, the IMU from the teensy_sensors arduino has been moved here.

# Required libraries
Install the teensyduino library found here.
also install the Adafruit_BN055 library from the arduino ide library manager.

# IMU
the imu.cpp and imu.h files contain the library for the BN055 IMU.

The way this library works is that the imu is initialized with imu_begin().
At the top of the file, set the "CALIBRATION" macro to 1 or 0 based on if you
want to recalibrate the imu. If calibrating, open the serial port while still attached to the teensy
and follow the serial monitor's instructions.

# Main Program

## Overview

The arbiter needs to communicate jetson commands to the subsystem arduinos based on the "type" field hidden within
each message. At the same time, the arbiter needs to send any messages coming from the subsystems up to the jetson. To do this all in a timely manner, serial interrupts are used. This way, any message processing comes second to actually receiving serial messages. This prevents the serial buffers from filling up and thus losing messages.

## Interrupts
