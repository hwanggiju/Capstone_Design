import time
from mpu9250_jmdev.registers import *
from mpu9250_jmdev.mpu_9250 import MPU9250

mpu = MPU9250(
    address_ak=AK8963_ADDRESS, 
    address_mpu_master=MPU9050_ADDRESS_68, # In 0x68 Address
    address_mpu_slave=None, 
    bus=1,
    gfs=GFS_1000, 
    afs=AFS_8G, 
    mfs=AK8963_BIT_16, 
    mode=AK8963_MODE_C100HZ)

mpu.configure() # mpu 레지스터 설정

while True:

    print("|.....MPU9250 in 0x68 Address.....|")
    # print("Accelerometer", mpu.readAccelerometerMaster()) # 가속도
    print("Gyroscope", mpu.readGyroscopeMaster()) # 자이로
    # print("Magnetometer", mpu.readMagnetometerMaster()) # 자력계
    # print("Temperature", mpu.readTemperatureMaster()) # 온도
    print("\n")

    time.sleep(1)
    
    
    # mpu.calibrate() # 센서값 교정