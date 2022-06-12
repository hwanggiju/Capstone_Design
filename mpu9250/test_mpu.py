from mpu9250_i2c import *
import math
import time
import RPi.GPIO as GPIO

time.sleep(1)

# Motor Driver [enA/in1/in2/in3/in4/enB]
driver = [35, 13, 15, 29, 31, 33]

def setPinConfig() :
    for i in range(len(driver)) :
        GPIO.setup(driver[i], GPIO.OUT)

    pwmA = GPIO.PWM(driver[0], 100)
    pwmB = GPIO.PWM(driver[5], 100)
    
    pwmA.start(0)
    pwmB.start(0)
    return pwmA, pwmB

def setMotorControl(pwmA, pwmB, speed, stat) :
    pwmA.ChangeDutyCycle(speed)
    pwmB.ChangeDutyCycle(speed)
    
    if stat == 1 :      # 위로
        GPIO.output(driver[1], 0)
        GPIO.output(driver[2], 1)
        
    elif stat == 2 :     # 아래로
        GPIO.output(driver[1], 1)
        GPIO.output(driver[2], 0)
        
    elif stat == 0 :     # 정지
        GPIO.output(driver[1], 0)
        GPIO.output(driver[2], 0)
        
    if stat == 1 :
        GPIO.output(driver[3], 0)
        GPIO.output(driver[4], 1)
        
    elif stat == 2 :
        GPIO.output(driver[3], 1)
        GPIO.output(driver[4], 0)
    
    elif stat == 0 :
        GPIO.output(driver[3], 0)
        GPIO.output(driver[4], 0)

def dist(a, b) :
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x, y, z) :
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z) :
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

pwmA, pwmB = setPinConfig()

setMotorControl(pwmA, pwmB, 0, 0)
time.sleep(2)

setMotorControl(pwmA, pwmB, 70, 1)
time.sleep(5)

setMotorControl(pwmA, pwmB, 100, 2)
time.sleep(5)

setMotorControl(pwmA, pwmB, 50, 1)
time.sleep(5)

setMotorControl(pwmA, pwmB, 0, 0)
'''
print('start')
while True :
    try :
        ax, ay, az = MPU6050_conv()
        
        accel_xout_scaled = ax / 16384.0
        accel_yout_scaled = ay / 16384.0
        accel_zout_scaled = az / 16384.0
        
        # mpu6050 데이터 가공
        x_angle = get_x_rotation(accel_xout_scaled, accel_yout_scaled, \
                                accel_zout_scaled)
        y_angle = get_y_rotation(accel_xout_scaled, accel_yout_scaled, \
                                accel_zout_scaled)
        
        print('angle x y = {}   {}'.format(x_angle, y_angle))
        
        
        time.sleep(1)
    
    except KeyboardInterrupt:
        break
    
    finally :
        pwmA.stop()
        pwmB.stop()
        GPIO.cleanup()
'''
pwmA.stop()
pwmB.stop()
GPIO.cleanup()
print('finish')