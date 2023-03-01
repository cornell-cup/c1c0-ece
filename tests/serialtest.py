import serial
import time

ser = serial.Serial(
	port = '/dev/ttyTHS1',
	baudrate = 9600,
)

print(ser.name)

while(1):
	for i in range(6):
		ser.write(bytes([i]))
		time.sleep(1)

