import RPi.GPIO as GPIO
import time
#test 1
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

en_lst = [11, 33]
in13_lst = [13, 29]
in24_lst = [15, 31]

GPIO.setup(en_lst, GPIO.OUT)
GPIO.setup(in13_lst, GPIO.OUT)
GPIO.setup(in24_lst, GPIO.OUT)

try :
    while 1 :
        GPIO.output(in13_lst, GPIO.LOW)
        GPIO.output(in24_lst, GPIO.LOW)
        
        GPIO.output(en_lst, GPIO.LOW)

        time.sleep(0.2)
        
        GPIO.output(in13_lst, GPIO.HIGH) #
        GPIO.output(in24_lst, GPIO.LOW)
        
        GPIO.output(en_lst, GPIO.HIGH)
        
        time.sleep(1)
        
        GPIO.output(in13_lst, GPIO.LOW)
        GPIO.output(in24_lst, GPIO.LOW)
        
        GPIO.output(en_lst, GPIO.LOW)

        time.sleep(0.2)
        
        GPIO.output(in13_lst, GPIO.LOW)
        GPIO.output(in24_lst, GPIO.HIGH)
        
        GPIO.output(en_lst, GPIO.HIGH)
        
        time.sleep(1)
    
except KeyboardInterrupt :
    GPIO.cleanup()
    