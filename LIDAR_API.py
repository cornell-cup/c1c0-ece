import serial
import time
import sys

"""
To be extended into Lidar API, maybe extended for terabees/IMU as well

"""
sys.path.append('../c1c0-movement/c1c0-movement/Locomotion')
import R2Protocol2 as r2p

ser = serial.Serial('/dev/ttyTHS1',
					9600)
startseq = (16777215).to_bytes(3, 'big')
endseq = (16777214).to_bytes(3, 'big')

print(ser)

def pack(tup):
	return(tup[0]<<8 | tup[1])
	

try:
	while True:
		ser_msg = ser.read(444)
			
		mtype, lidar_data, status = r2p.decode(ser_msg)
		#print(mtype, msg, status
		
		print(lidar_data)
		
		start = lidar_data.index(startseq) + 3
		end = lidar_data.index(endseq, start)
		
		lidar_data = lidar_data[start:end]
		
		
		for i in range(0, len(lidar_data), 4):
			#print("Here")
			angle_msbs = lidar_data[i]
			angle_lsbs = lidar_data[i+1]
			distance_msbs = lidar_data[i+2]
			distance_lsbs = lidar_data[i+3]
			print("Packet: " + str(i))
			print("Angle MSB: " + str(angle_msbs) + " Angle LSB: " + str(angle_lsbs))
			print("Distance MSB: " + str(distance_msbs) + " Distance LSB: " + str(distance_lsbs))
			angle = pack((angle_msbs, angle_lsbs))
			distance = pack((distance_msbs, angle_msbs))
			print("Angle: " + str(angle))
			print("Distance: " + str(distance))
			print("")
		
except KeyboardInterrupt:
	ser.close()

	
