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
    gpio.OUTPUT(6, gpio.LOW)
    gpio.OUTPUT(5, gpio.LOW)
    gpio.OUTPUT(22, gpio.LOW)
    gpio.OUTPUT(27, gpio.LOW)

    time.sleep(0.2)

    gpio.OUTPUT(6, gpio.HIGH)
    gpio.OUTPUT(5, gpio.LOW)
    gpio.OUTPUT(22, gpio.HIGH)
    gpio.OUTPUT(27, gpio.LOW)

    time.sleep(1)
    
    gpio.OUTPUT(6, gpio.LOW)
    gpio.OUTPUT(5, gpio.LOW)
    gpio.OUTPUT(22, gpio.LOW)
    gpio.OUTPUT(27, gpio.LOW)

    time.sleep(0.2)

    gpio.OUTPUT(6, gpio.LOW)
    gpio.OUTPUT(5, gpio.HIGH)
    gpio.OUTPUT(22, gpio.LOW)
    gpio.OUTPUT(27, gpio.HIGH)

    time.sleep(1)