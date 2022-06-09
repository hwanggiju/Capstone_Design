import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
# Motor Driver [enA/in1/in2/in3/in4/enB]
driver = [35, 13, 15, 29, 31, 33]

#하드웨어 초기설정
for i in range(len(driver)):
    GPIO.setup(driver[i], GPIO.OUT)
    
pwmA = GPIO.PWM(driver[0], 50)
pwmB = GPIO.PWM(driver[5], 50)

pwmA.start(3)
pwmB.start(3)
    
try :
    for cnt in range(0,3) :
        pwmA.ChangeDutyCycle(3.0) # 0도
        pwmB.ChangeDutyCycle(3.0) # 0도
        
        for i in range(1, len(driver)-1):
            GPIO.output(driver[i], 0)
        time.sleep(0.5)
        GPIO.output(driver[1], 0)
        GPIO.output(driver[2], 1)
        GPIO.output(driver[3], 0)
        GPIO.output(driver[4], 1)
        
        time.sleep(1.0)
        
        pwmA.ChangeDutyCycle(7.5) # 90도
        pwmB.ChangeDutyCycle(7.5) # 90도
        
        for i in range(1, len(driver)-1):
            GPIO.output(driver[i], 0)
        time.sleep(0.5)
        GPIO.output(driver[1], 0)
        GPIO.output(driver[2], 1)
        GPIO.output(driver[3], 0)
        GPIO.output(driver[4], 1)
        
        time.sleep(1.0)
        
        pwmA.ChangeDutyCycle(12.5) # 180도
        pwmB.ChangeDutyCycle(12.5) # 180도
        
        for i in range(1, len(driver)-1):
            GPIO.output(driver[i], 0)
        time.sleep(0.5)
        GPIO.output(driver[1], 0)
        GPIO.output(driver[2], 1)
        GPIO.output(driver[3], 0)
        GPIO.output(driver[4], 1)
        
        time.sleep(1.0)
        
except KeyboardInterrupt :
    GPIO.cleanup()