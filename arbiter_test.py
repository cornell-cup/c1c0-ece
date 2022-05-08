# Example File for Jetson Sending R2Protocol Messages to the Arduino
# In coordination with arduino_receiving.ino
# For use with c1c0 communication systems
# See the FAQ on the Cornell Cup Google Drive, located Here: 
# https://docs.google.com/document/d/1cKZGTqvyOFYL5ugZGULJ74xGuEwgBEWSXJAztM2qnaE/edit

import serial
import time
import sys
import TEST_API

sys.path.append('/home/c1c0-main/c1c0-movement/c1c0-movement/Locomotion')
#Resolve 'cornellcup' to be the username of this Jetson, make sure c1c0-movement is cloned
import R2Protocol2 as r2p

# ~ ser = serial.Serial(
    # ~ port = '/dev/ttyTHS1', #Jetson hardware serial port (pins 8/10 on header)
    # ~ baudrate = 115200, #Bits/s data rate
# ~ )

ser = TEST_API.init_serial('/dev/ttyTHS1', 115200)

data1 = [0,5,10,15,20,25] # precise arm data
data2 = "1234567"
data3 = "(35, 12, 10 )"

ser.reset_input_buffer()

def send_token(msg_type, token):
    req = r2p.encode(msg_type, token.to_bytes(1, 'big'))
    ser.write(req)
    
def convert_8_to_16(msg, length):
    data = []
    for i in range(0,length,2):
        data.append((msg[i] << 8) | msg[i+1]) 
    return data

while(True):
    
    msg = r2p.encode(b"PRM", bytes(data1))
    ser.write(msg) 
    print("Wrote PRM Data Downstream")
    print("")
    
    msg = r2p.encode(b"LOC", bytearray(data3, "utf-8")) 
    ser.write(msg) 
    print("Wrote LOC Data Downstream")
    print("")
    
    send_token(b"PRMR", 1)
    print("Requested PRM Data Downstream")
    ser_msg = ser.read(28)
    mtype, data, status = r2p.decode(ser_msg)
    print("Received PRM Data Upstream")
    print(ser_msg)
    print("")
    print(convert_8_to_16(data,12))
    
    TEST_API.sensor_token(b"SNSR",1)
    print("Requested SNS Data Downstream")
    TEST_API.decode_arrays()
    imu = TEST_API.get_array('IMU')
    print(imu)
    print("Received SNS Data Upstream")
    print("")
    
    time.sleep(1)
