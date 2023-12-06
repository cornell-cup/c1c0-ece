This repository holds the sensor code, high level documents, as well as code to communicate arduinos between each-other

teensy_sensors/teensy_sensors.ino: Holds code to connect teensy to LIDAR, 3 terabee strips, and IMU

arbiter_teensy/arbiter_teensy.ino: Holds code for arbiter which acts as serial intermediate between jetson and other arduinos

libs/R2Protocol: Holds R2Protocol which packs encodes and decodes data to send over serial
    R2Protocol.h: For use in arduino code, only C/C++
    R2Protocol.py: For use in Jetson code, only python