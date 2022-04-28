# Example File for Jetson Sending R2Protocol Messages to the Arduino
# In coordination with arduino_receiving.ino
# For use with c1c0 communication systems
# See the FAQ on the Cornell Cup Google Drive, located Here: 
# https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

import serial
import time
import sys

sys.path.append('/home/c1c0-main/c1c0-movement/c1c0-movement/Locomotion')
#Resolve 'cornellcup' to be the username of this Jetson, make sure c1c0-movement is cloned
import R2Protocol2 as r2p

ser = serial.Serial(
    port = '/dev/ttyTHS1', #Jetson hardware serial port (pins 8/10 on header)
    baudrate = 115200, #Bits/s data rate
)

data = "123"

while(True):

    msg = r2p.encode(b"DUE1", bytearray(data, "utf-8"))

    ser.write(msg) 
    
    msg = r2p.encode(b"sprt", bytearray(data, "utf-8")) 

    ser.write(msg) 
   
    msg = r2p.encode(b"DUE2", bytearray(data, "utf-8"))

    ser.write(msg)
    
    print("Sent messages")
    time.sleep(1)
