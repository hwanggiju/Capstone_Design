import RPi.GPIO as GPIO

driver = [19, 27, 22, 5, 6, 13]
GPIO.setmode(GPIO.BCM)
GPIO.setwarnings(False)

for i in range(len(driver)): #모터 드라이버 핀
    GPIO.setup(driver[i], GPIO.OUT)
    
for i in range(len(driver)):
    GPIO.output(driver[i], 0)
try :    
    GPIO.output(driver[0], 1)
    GPIO.output(driver[0], 1)
    GPIO.output(driver[1], 1)
    GPIO.output(driver[2], 0)
    GPIO.output(driver[3], 1)
    GPIO.output(driver[4], 0)
    
except KeyboardInterrupt :
    GPIO.cleanup()