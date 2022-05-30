import RPi.GPIO as GPIO
import time
#test 1
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

GPIO.setup(33, GPIO.OUT) # enA
GPIO.setup(11, GPIO.OUT) # enB
GPIO.setup(31, GPIO.OUT) # in1
GPIO.setup(29, GPIO.OUT) # in2
GPIO.setup(15, GPIO.OUT) # in3
GPIO.setup(13, GPIO.OUT) # in4


while True :
    GPIO.output(33, GPIO.LOW) # enA
    GPIO.output(11, GPIO.LOW) # enB
    
    GPIO.output(31, GPIO.LOW) # in1
    GPIO.output(29, GPIO.LOW) # in2
    GPIO.output(15, GPIO.LOW) # in3
    GPIO.output(13, GPIO.LOW) # in4

    time.sleep(0.2)

    GPIO.output(33, GPIO.HIGH)
    GPIO.output(11, GPIO.HIGH)
    
    GPIO.output(31, GPIO.HIGH)
    GPIO.output(29, GPIO.LOW)
    GPIO.output(15, GPIO.HIGH)
    GPIO.output(13, GPIO.LOW)
    
    time.sleep(1)
    
    GPIO.output(33, GPIO.LOW)
    GPIO.output(11, GPIO.LOW)
    
    GPIO.output(31, GPIO.LOW)
    GPIO.output(29, GPIO.LOW)
    GPIO.output(15, GPIO.LOW)
    GPIO.output(13, GPIO.LOW)

    time.sleep(0.2)

    GPIO.output(33, GPIO.HIGH)
    GPIO.output(11, GPIO.HIGH)
    
    GPIO.output(31, GPIO.LOW)
    GPIO.output(29, GPIO.HIGH)
    GPIO.output(15, GPIO.LOW)
    GPIO.output(13, GPIO.HIGH)
    
    
    time.sleep(1)
    
    if KeyboardInterrupt :
        GPIO.cleanup()
    