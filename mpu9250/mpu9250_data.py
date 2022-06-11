from mpu9250_i2c import *

time.sleep(1) # i2c setting delay

print('데이터 값')
while True :
    try :
        ax, ay, az, wx, wy, wz = MPU6050_conv()
    
    except :
        continue
    
    print('{}'.format('-'*30))
    print('accel [g] : x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(ax, ay, az))
    print('gyro [dps] : x = {0:2.2f}, y = {1:2.2f}, z = {2:2.2f}'.format(wx, wy, wz))
    print('{}'.format('-'*30))
    time.sleep(1)