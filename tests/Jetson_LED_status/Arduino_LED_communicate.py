import Jetson.GPIO as GPIO

# 03/19/22 - Initialized this file! Jetson GPIO dependency resolved
#set mode
GPIO.setmode(GPIO.BOARD)

#set pins
channels = [11, 13, 15]
GPIO.setup(channels, GPIO.OUT)

#variables
#state is the current number that we are testing
#signal: should be the incoming signal, like which part of c1c0 is moving
#LED1: the signal that controlls pin 11
#LED2: the signal that controlls pin 13
#LED3: the signal that controlls pin 15
#t is just a test variable, but it's useless right now
'''
for signal in range(0,5):
    state = str(bin(signal+1)[2:].zfill(3))
    if state == '001':
        LED1,LED2,LED3 = 0,0,1
    elif state == '010':
        LED1,LED2,LED3 = 0,1,0
    elif state == '011':
        LED1,LED2,LED3 = 0,1,1
    elif state == '100':
        LED1,LED2,LED3 = 1,0,0
    elif state == '101':
        LED1,LED2,LED3 = 1,0,1
    else:
        LED1,LED2,LED3 = 0,0,0
    #print(LED1,LED2,LED3)
'''
#receiving signal
signal = eval(input('enter:'))
state = str(bin(signal+1)[2:].zfill(3))
if state == '001':
    LED1,LED2,LED3 = 0,0,1
elif state == '010':
    LED1,LED2,LED3 = 0,1,0
elif state == '011':
    LED1,LED2,LED3 = 0,1,1
elif state == '100':
    LED1,LED2,LED3 = 1,0,0
elif state == '101':
    LED1,LED2,LED3 = 1,0,1
else:
    LED1,LED2,LED3 = 0,0,0

#set high and low
if LED1 == 1:
    GPIO.output(11, GPIO.HIGH)
    print('led1 HIGH')
else:
    GPIO.output(11, GPIO.LOW)
    print('led1 LOW')
if LED2 == 1:
    GPIO.output(13, GPIO.HIGH)
    print('led2 HIGH')
else:
    GPIO.output(13, GPIO.LOW)
    print('led2 LOW')
if LED3 == 1:
    GPIO.output(15, GPIO.HIGH)
    print('led3 HIGH')
else:
    GPIO.output(15, GPIO.LOW)
    print('led3 LOW')

GPIO.cleanup()
