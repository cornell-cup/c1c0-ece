import serial
import time
import sys

"""
IMU API for use with path_planning. 

"""
sys.path.append('../c1c0-movement/c1c0-movement/Locomotion') #Might need to be resolved
import R2Protocol2 as r2p

ser = None

terabee_array_1 = []
terabee_array_2 = []
terabee_array_3 = []

def init_serial(port, baud):
	"""
	Opens serial port for LIDAR communication
	port should be a string linux port: Ex dev/ttyTHS1
	Baud is int the data rate, commonly multiples of 9600
	"""
	global ser, startseq, endseq
	
	ser = serial.Serial(port,
						baud)
	
	

def get_imu_tuples():
	"""

	
	"""
	global ser, terabee_array_1, terabee_array_2, terabee_array_3
	
	good_data = False
	
	while(not good_data):
		gyro_data = []
		ser_msg = ser.read(22) # make 96 for three terabees
			
		for i in range(0, len(ser_msg), 32): 
			
			mtype, terabee_data, status = r2p.decode(ser_msg[i:i+32])
			
			print(mtype)
			
			if(status == 1):
				good_data = True
				if(mtype==b'IR\x00\x00'):
					for i in range(0, len(terabee_data), 2):
						terabee_value = (terabee_data[i]<<8) | terabee_data[i+1]
						terabee_array_1.append(terabee_value)
				elif(mtype==b'IR2\x00'):
					for i in range(0, len(terabee_data), 2):
						terabee_value = (terabee_data[i]<<8) | terabee_data[i+1]
						terabee_array_2.append(terabee_value)
				elif(mtype==b'IR3\x00'):
					for i in range(0, len(terabee_data), 2):
						terabee_value = (terabee_data[i]<<8) | terabee_data[i+1]
						terabee_array_3.append(terabee_value)
					
			else:
				ser.reset_input_buffer()

		
	return gyro_array
		


if __name__ == '__main__':
	init_serial('/dev/ttyTHS1', 38400)
	
	f = open("terabeeprint.txt", "a")
	
	try:
		while True:

			start = time.time()
			arr = get_imu_tuples()
			for i in arr:
				print(i)
			print("End of seg")
			end = time.time() - start
			print(end)
			
	except KeyboardInterrupt:
		ser.close()
		f.close()
