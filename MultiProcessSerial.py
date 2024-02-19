import threading, multiprocessing

import serial
import time
import sys

sys.path.append('/home/c1c0-main/c1c0-movement/c1c0-movement/Locomotion')
import R2Protocol2 as r2p



def read_data(queue, serialPort, stopped):
    
    while not stopped.is_set():
        fio2_data = serialPort.read_until(expected = b'\xd2\xe2\xf2')
        mtype,mdata,mstatus = r2p.decode(fio2_data)
        queue.put(mdata)
    
    
def display_data(queue, stopped):
    
    while not stopped.is_set():
        
        if not queue.empty():
            fio2_data = queue.get()
            #print(queue.qsize())
            print(fio2_data)

        

if __name__ == "__main__":
    
    ser = serial.Serial('/dev/ttyTHS1', 38400)
    queue = multiprocessing.Queue()
    
    stopped = threading.Event()
    p1 = threading.Thread(target=read_data, args=(queue, ser, stopped))
    p2 = threading.Thread(target=display_data, args = (queue, stopped))
    
    p1.start()
    p2.start()
    
    loopcnt = 20
    while (loopcnt > 0) and (not stopped.is_set()):
        loopcnt -= 1
        try:
            time.sleep(10)
        
        except KeyboardInterrupt:
            loopcnt=0
            stopped.set()
            
    stopped.set()
    loopcnt=0
    
    p1.join()
    p2.join()
    
    ser.close()
    print("Done")
