import TEST_API.py

def decodetime():
	start = time.time()
	decode_arrays()
	end = time.time()
	time_diff = end-start
	print("Decode Arrays Time: " + str(time_diff) + "seconds.")

if __name__ == '__main__':
	init_serial('/dev/ttyTHS1', 115200)
	decode_time()
	close_serial()
