import serial
import time
import sys

"""
Terabee API for use with path_planning. 

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
	

def close_serial():
	global ser
	ser.close()

def get_terabee_array():
	"""
	Acquires and returns one array of terabee data, for each of the three
	terabee sensor strips (three strips of 8 sensors each)
	This will return all three arrays, each element in the array denotes
	a 16-bit integer measurement in mm

	
	"""
	global ser, terabee_array_1, terabee_array_2, terabee_array_3, ldr_array
	
	good_data = False
	
	while(not good_data):
		terabee_array_1 = []
		terabee_array_2 = []
		terabee_array_3 = []
		ldr_array = []

		ser_msg = ser.read(280) # make 96 for three terabees
		
		mtype, data, status = r2p.decode(ser_msg)
		
		print(len(data))
		
		print("TYPE: \n" + str(mtype))
		#print("DATA:" + str(data))
		
		if (mtype == b'IR\x00\x00'):
			decode_from_ir(data)
			good_data = True
			
		elif (mtype == b'IR2\x00'):
			#decode_from_ir2(data)
			good_data = True
			
		elif (mtype == b'LDR\x00'):
			good_data = True
			print("DECODE FROM LDR")
			
		else:
			print("NO GOOD")
			ser.reset_input_buffer()
			

def decode_from_ir(data):
	
	terabee1_data = data[0:16]
	terabee2_data = data[32:32+16]
	ldr_data = data[64:]
	
	print(terabee1_data)
	print(terabee2_data)
	print(ldr_data)
	
	terabee_array_append(terabee1_data, terabee_array_1)
	terabee_array_append(terabee2_data, terabee_array_2)
	lidar_tuple_array_append(ldr_data, ldr_array)
	
	print("TERABEE 1:")
	for item in terabee_array_1:
		print(item)
	
	print("TERABEE 2:")
	for item in terabee_array_2:
		print(item)
		
	print("LIDAR:")
	print(ldr_array)
		
# ~ def decode_from_ir2(data):
	
	# ~ terabee2_data = data[0:16]
	
	# ~ mtype2, ldr_data, status2 = r2p.decode(data[16+3:])
	
	# ~ mtype3, terabee1_data, status3 = r2p.decode(ldr_data[200+3:])
	
	# ~ print(terabee1_data)
	# ~ print(terabee2_data)
	# ~ print(ldr_data)
	
	# ~ array_append(terabee1_data, terabee_array_1, 16)
	# ~ array_append(terabee2_data, terabee_array_2, 16)
	# ~ array_append(ldr_data, ldr_array, 200)
	
	# ~ print("TERABEE 1:")
	# ~ for item in terabee_array_1:
		# ~ print(item)
	
	# ~ print("TERABEE 2:")
	# ~ for item in terabee_array_2:
		# ~ print(item)
		
	# ~ print("LIDAR:")
	# ~ print(ldr_array)

def terabee_array_append(data, target_array):
	
	for i in range(0, 16, 2):
		value = (data[i]<<8) | data[i+1]
		target_array.append(value)
		
def lidar_tuple_array_append(data, target_array):

	for i in range(0, 200, 4):
		angle_msbs = data[i]
		angle_lsbs = data[i+1]
		distance_msbs = data[i+2]
		distance_lsbs = data[i+3]
		angle = pack((angle_msbs, angle_lsbs))
		distance = pack((distance_msbs, angle_msbs))
		target_array.append((angle,distance))
		
def pack(tup):
	return(tup[0]<<8 | tup[1])

if __name__ == '__main__':
	init_serial('/dev/ttyTHS1', 38400)
	
	f = open("terabeeprint.txt", "a")
	
	try:
		while True:

			start = time.time()
			get_terabee_array()
			end = time.time() - start
			print(end)
			
	except KeyboardInterrupt:
		ser.close()
		f.close()
