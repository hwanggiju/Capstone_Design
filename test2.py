import RPi.GPIO as GPIO
import time
#test 1
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)###
enA = 13
enB = 17
in1 = 6
in2 = 5
in3 = 22
in4 = 27
GPIO.cleanup()
GPIO.setup(enA, GPIO.OUT, initial=0) # enA
GPIO.setup(enB, GPIO.OUT, initial=0) # enB
GPIO.setup(in1, GPIO.OUT, initial=0) # in1
GPIO.setup(in2, GPIO.OUT, initial=0) # in2
GPIO.setup(in3, GPIO.OUT, initial=0) # in3
GPIO.setup(in4, GPIO.OUT, initial=0) # in4


while True :
    GPIO.output(enA, GPIO.LOW) # enA
    GPIO.output(enB, GPIO.LOW) # enB

    GPIO.output(in1, GPIO.LOW) # in1
    GPIO.output(in2, GPIO.LOW) # in2
    GPIO.output(in3, GPIO.LOW) # in3
    GPIO.output(in4, GPIO.LOW) # in4

    time.sleep(0.2)

    GPIO.output(enA, 0)
    GPIO.output(enB, 0)
    
    GPIO.output(in1, GPIO.HIGH)
    GPIO.output(in2, GPIO.LOW)
    GPIO.output(in3, GPIO.HIGH)
    GPIO.output(in4, GPIO.LOW)
    
    time.sleep(1)

    GPIO.output(enA, GPIO.LOW)  # enA
    GPIO.output(enB, GPIO.LOW)  # enB

    GPIO.output(in1, GPIO.LOW)  # in1
    GPIO.output(in2, GPIO.LOW)  # in2
    GPIO.output(in3, GPIO.LOW)  # in3
    GPIO.output(in4, GPIO.LOW)  # in4

    time.sleep(0.2)

    GPIO.output(enA, GPIO.HIGH)
    GPIO.output(enB, GPIO.HIGH)

    GPIO.output(in1, GPIO.LOW)
    GPIO.output(in2, GPIO.HIGH)
    GPIO.output(in3, GPIO.LOW)
    GPIO.output(in4, GPIO.HIGH)

    time.sleep(1)

    
    if KeyboardInterrupt :
        GPIO.cleanup()
    