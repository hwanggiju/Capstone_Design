import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

gpio.setup(33, gpio.OUT) # enA
gpio.setup(11, gpio.OUT) # enB
gpio.setup(31, gpio.OUT) # in1
gpio.setup(29, gpio.OUT) # in2
gpio.setup(15, gpio.OUT) # in3
gpio.setup(13, gpio.OUT) # in4


while True :
    gpio.output(33, gpio.LOW) # enA
    gpio.output(11, gpio.LOW) # enB
    gpio.output(31, gpio.LOW) # in1
    gpio.output(29, gpio.LOW) # in2
    gpio.output(15, gpio.LOW) # in3
    gpio.output(13, gpio.LOW) # in4

    time.sleep(0.2)

    gpio.output(33, gpio.HIGH)
    gpio.output(11, gpio.HIGH)
    gpio.output(31, gpio.HIGH)
    gpio.output(29, gpio.LOW)
    gpio.output(15, gpio.HIGH)
    gpio.output(13, gpio.LOW)
    
    time.sleep(1)
    
    gpio.output(33, gpio.LOW)
    gpio.output(11, gpio.LOW)
    gpio.output(31, gpio.LOW)
    gpio.output(29, gpio.LOW)
    gpio.output(15, gpio.LOW)
    gpio.output(13, gpio.LOW)

    time.sleep(0.2)

    gpio.output(33, gpio.HIGH)
    gpio.output(11, gpio.HIGH)
    gpio.output(31, gpio.LOW)
    gpio.output(29, gpio.HIGH)
    gpio.output(15, gpio.LOW)
    gpio.output(13, gpio.HIGH)
    
    
    time.sleep(1)
    
    if KeyboardInterrupt :
        gpio.cleanup()
    