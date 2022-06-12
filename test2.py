import RPi.GPIO as GPIO
import time

# Motor Driver [enA/in1/in2/in3/in4/enB]
driver = [35, 13, 15, 29, 31, 33]

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

for i in range(len(driver)) :
    GPIO.setup(driver[i], GPIO.OUT)

GPIO.output(driver[0], 0)
GPIO.output(driver[5], 0)
GPIO.output(driver[1], 0)
GPIO.output(driver[2], 0)
GPIO.output(driver[3], 0)
GPIO.output(driver[4], 0)
time.sleep(0.3)

GPIO.output(driver[0], 1)
GPIO.output(driver[5], 1)
GPIO.output(driver[1], 0)
GPIO.output(driver[2], 1)
GPIO.output(driver[3], 0)
GPIO.output(driver[4], 1)
time.sleep(5)

GPIO.output(driver[0], 1)
GPIO.output(driver[5], 1)
GPIO.output(driver[1], 0)
GPIO.output(driver[2], 0)
GPIO.output(driver[3], 0)
GPIO.output(driver[4], 0)
time.sleep(0.3)

GPIO.output(driver[0], 1)
GPIO.output(driver[5], 1)
GPIO.output(driver[1], 1)
GPIO.output(driver[2], 0)
GPIO.output(driver[3], 1)
GPIO.output(driver[4], 0)
time.sleep(5)

GPIO.output(driver[0], 0)
GPIO.output(driver[5], 0)
GPIO.output(driver[1], 0)
GPIO.output(driver[2], 0)
GPIO.output(driver[3], 0)
GPIO.output(driver[4], 0)
GPIO.cleanup()
