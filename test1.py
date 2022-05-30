import RPi.GPIO as GPIO
import time
#test 1
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

en_lst = [11, 33] 
in13_lst = [13, 29]
in24_lst = [15, 31]

GPIO.setup(11, GPIO.OUT)
GPIO.setup(33, GPIO.OUT)
GPIO.setup(13, GPIO.OUT)
GPIO.setup(15, GPIO.OUT)
GPIO.setup(29, GPIO.OUT)
GPIO.setup(31, GPIO.OUT)

GPIO.output(13, GPIO.LOW)
GPIO.output(15, GPIO.LOW)
GPIO.output(29, GPIO.LOW)
GPIO.output(31, GPIO.LOW)

GPIO.output(11, GPIO.LOW)
GPIO.output(33, GPIO.LOW)

time.sleep(0.2)

GPIO.output(13, GPIO.LOW)
GPIO.output(15, GPIO.HIGH)
GPIO.output(29, GPIO.LOW)
GPIO.output(31, GPIO.HIGH)

GPIO.output(11, GPIO.HIGH)
GPIO.output(33, GPIO.HIGH)

time.sleep(2)

try :
    while 1 :
        GPIO.output(13, GPIO.LOW)
        GPIO.output(15, GPIO.LOW)
        GPIO.output(29, GPIO.LOW)
        GPIO.output(31, GPIO.LOW)
        
        GPIO.output(11, GPIO.LOW)
        GPIO.output(33, GPIO.LOW)

        time.sleep(0.2)
        
        GPIO.output(13, GPIO.HIGH)
        GPIO.output(15, GPIO.LOW)
        GPIO.output(29, GPIO.HIGH)
        GPIO.output(31, GPIO.LOW)
        
        GPIO.output(11, GPIO.HIGH)
        GPIO.output(33, GPIO.HIGH)
        
        time.sleep(1)
        
        GPIO.output(13, GPIO.LOW)
        GPIO.output(15, GPIO.LOW)
        GPIO.output(29, GPIO.LOW)
        GPIO.output(31, GPIO.LOW)
        
        GPIO.output(11, GPIO.LOW)
        GPIO.output(33, GPIO.LOW)

        time.sleep(0.2)
        
        GPIO.output(13, GPIO.LOW)
        GPIO.output(15, GPIO.HIGH)
        GPIO.output(29, GPIO.LOW)
        GPIO.output(31, GPIO.HIGH)
        
        GPIO.output(11, GPIO.HIGH)
        GPIO.output(33, GPIO.HIGH)
        
        time.sleep(1)
    
except KeyboardInterrupt :
    GPIO.cleanup()
    