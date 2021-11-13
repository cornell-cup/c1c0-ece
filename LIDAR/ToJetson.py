import serial
import time

ser = serial.Serial(
	port = '/dev/ttyTHS1',
	baudrate = 9600
)

print(ser.name)

while(True):
	
	serial4 = ser.read(400)
	
	print(serial4)
	
	for i in range(400):
		print("Angle ", i, ": ", serial4[i])
	# for i in range(100):
		# if i%2 == 0:
			# print("Angle:")
		# else:
			# print("Distance:")
		# print(serial4[i])
		# print("---------------")
	
