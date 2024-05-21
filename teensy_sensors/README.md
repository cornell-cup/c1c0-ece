# Overview
This directory contains code for the teensy that controls C1C0's sensors and sends the sensor data the jetson periodically.
The sensors are an RPLidar and a strip of terabees. The source code will show the currently attached serial port.

# Required libraries
Install the teensyduino library found here.

https://www.pjrc.com/teensy/td_download.html

# Lidar
The Lidar sensor works through the lidar library contains in the auxiliary header and cpp files.

First,
```c
lidar.init()
```
is called with with serial port the lidar sensor is connected to. Then,

```c
IS_OK(lidar.waitPoint())
```
returns if the lidar has data points available. If not, we haven't started the scan, so call 
```c
lidar.startScan();
```
to start the lidar sensor.

If waitPoint returns true, then the points are collected using

```c
uint16_t distance = (uint16_t)lidar.getCurrentPoint().distance; // distance value in mm unit
uint16_t angle = (uint16_t)lidar.getCurrentPoint().angle;    
```

# Terabees
The terbees do not run off a library. The terabees simply send data as long as they are connected

find_msg is used to read serial data to see if the terabee has received data. Once the end byte is received, the data is stored.

# Program

The program works by initializing the terabee serial port and lidar object. Then in the loop, the lidar data is collected as well as terabee data in tandem. Once the lidar data has been received, the current terabee data and lidar data are sent over the jetson serial port.

# Debugging
To debug the system, First, set the macro at the top of the file, "DEBUG" to 1. If the teensy doesn't print out
"NUMBER OF BYTES WRITTEN! READ ME", then one of the sensors has been disconnected.
