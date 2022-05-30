import RPi.GPIO as gpio
import time

gpio.setmode(gpio.BCM)
gpio.setup(13, gpio.OUT) # enA
gpio.setup(6, gpio.OUT) # in1
gpio.setup(5, gpio.OUT) # in2
gpio.setup(22, gpio.OUT) # in3
gpio.setup(27, gpio.OUT) # in4
gpio.setup(17, gpio.OUT) # enB

while True :
    gpio.output(13, gpio.LOW)
    gpio.output(17, gpio.LOW)
    gpio.output(6, gpio.LOW)
    gpio.output(5, gpio.LOW)
    gpio.output(22, gpio.LOW)
    gpio.output(27, gpio.LOW)

    time.sleep(0.2)

    gpio.output(6, gpio.HIGH)
    gpio.output(5, gpio.LOW)
    gpio.output(22, gpio.HIGH)
    gpio.output(27, gpio.LOW)
    gpio.output(13, gpio.HIGH)
    gpio.output(17, gpio.HIGH)
    
    time.sleep(1)
    
    gpio.output(13, gpio.LOW)
    gpio.output(17, gpio.LOW)
    gpio.output(6, gpio.LOW)
    gpio.output(5, gpio.LOW)
    gpio.output(22, gpio.LOW)
    gpio.output(27, gpio.LOW)

    time.sleep(0.2)

    gpio.output(6, gpio.LOW)
    gpio.output(5, gpio.HIGH)
    gpio.output(22, gpio.LOW)
    gpio.output(27, gpio.HIGH)
    gpio.output(13, gpio.HIGH)
    gpio.output(17, gpio.HIGH)
    
    time.sleep(1)
    ###