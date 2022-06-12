from mpu9250_i2c import *
import math

def dist(a, b) :
    return math.sqrt((a*a)+(b*b))

def get_y_rotation(x, y, z) :
    radians = math.atan2(x, dist(y, z))
    return -math.degrees(radians)

def get_x_rotation(x, y, z) :
    radians = math.atan2(y, dist(x, z))
    return math.degrees(radians)

print('start')
while True :
    try :
        ax, ay, az = MPU6050_conv() 
        
    except :
        continue
    
    accel_xout_scaled = ax / 16384.0
    accel_yout_scaled = ay / 16384.0
    accel_zout_scaled = az / 16384.0
    
    # mpu6050 데이터 가공
    x_angle = get_x_rotation(accel_xout_scaled, accel_yout_scaled, \
                            accel_zout_scaled)
    y_angle = get_y_rotation(accel_xout_scaled, accel_yout_scaled, \
                            accel_zout_scaled)
    
    print(x_angle, y_angle)