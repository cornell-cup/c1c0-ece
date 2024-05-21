C1C0 has four subsystems, 

# Subsystems
This repo: code for the teensy sensors and arbiter for bus communication

c1c0-movement: code for locomotion arduino (https://github.com/cornell-cup/c1c0-movement)

c1c0-precision-arm: code for precise arm arduino and blue arm used to test object detection off of C1C0 (https://github.com/cornell-cup/c1c0-precision-arm)

c1c0-strong-arm: code for strong arm arduino  (https://github.com/cornell-cup/c1c0-strong-arm)

c1c0-scheduler: Code for scheduler which runs on the main Jetson nano.

To learn more about these subsystems, visit these repositories and read their ReadMe files.

# High level diagram


<img src="images/C1C0CommBlockDiagram v8-1.png" alt="Page 1 from my PDF">

See the images directory for a higher quality image.

# Contained Files

This repository holds the sensor code, high level documents, as well as code to communicate arduinos between each-other

teensy_sensors/teensy_sensors.ino: Holds code to connect teensy to LIDAR, 3 terabee strips, and IMU

arbiter_teensy/arbiter_teensy.ino: Holds code for arbiter which acts as serial intermediate between jetson and other arduinos

libs/R2Protocol: Holds R2Protocol which is a library providing methods to pack, encode and decode data to send over serial. All arduino code uses this library.
    R2Protocol.h: For use in arduino code, only C/C++
    R2Protocol.py: For use in Jetson code, only python