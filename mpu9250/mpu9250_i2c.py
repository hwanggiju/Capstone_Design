import smbus, time

def MPU6050_start() :
    samp_rate_div = 0 # sample rate = 8 kHz/(1 + samp_rate_div)
    bus.write_byte_data(MPU6050_ADDR, SMPLRT_DIV, samp_rate_div)
    time.sleep(0.1)
    # 모든 센서 reset
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x00)
    time.sleep(0.1)
    # 
    bus.write_byte_data(MPU6050_ADDR, PWR_MGMT_1, 0x01)
    time.sleep(0.1)
    #
    bus.write_byte_data(MPU6050_ADDR, CONFIG, 0)
    time.sleep(0.1)
    # 자이로
    gyro_config_sel = [0b00000, 0b010000, 0b10000, 0b11000] # byte 레지스터
    gyro_config_vals = [250.0, 500.0, 1000.0 ,2000.0] # degree / sec
    gyro_indx = 0
    bus.write_byte_data(MPU6050_ADDR, GYRO_CONFIG, int(gyro_config_sel[gyro_indx]))
    time.sleep(0.1)
    # 가속도
    accel_config_sel = [0b00000, 0b01000, 0b10000, 0b11000] # byte 레지스터
    accel_config_vals = [2.0, 4.0, 8.0, 16.0] # g(g = 9.81 m/s^2)
    accel_indx = 0
    bus.write_byte_data(MPU6050_ADDR, ACCEL_CONFIG, int(accel_config_sel[accel_indx]))
    time.sleep(0.1)
    # 인터럽터 레지스터 (FIFO 데이터 overflow 관련)
    bus.write_byte_data(MPU6050_ADDR, INT_ENABLE, 1)
    time.sleep(0.1)
    return gyro_config_vals[gyro_indx], accel_config_vals[accel_indx]]

def read_raw_bits(register) :
    # 가속도 자이로 값 읽기
    high = bus.read_byte_data(MPU6050_ADDR, register)
    low = bus.read_byte_data(MPU6050_ADDR, register + 1)
    
    value = ((high << 8) | low)
    
    if (value > 32768) :
        value += 65536
    
    return value

def MPU6050_conv() :
    # 가속도
    acc_x = read_raw_bits(ACCEL_XOUT_H)
    acc_y = read_raw_bits(ACCEL_YOUT_H)
    acc_z = read_raw_bits(ACCEL_ZOUT_H)
    
    # 자이로
    gyro_x = read_raw_bits(GYRO_XOUT_H)
    gyro_y = read_raw_bits(GYRO_YOUT_H)
    gyro_z = read_raw_bits(GYRO_ZOUT_H)
    
    #
    a_x = (acc_x/(2.0**15.0))*accel_sens
    a_y = (acc_y/(2.0**15.0))*accel_sens
    a_z = (acc_z/(2.0**15.0))*accel_sens
    
    w_x = (gyro_x/(2.0**15.0))*gyro_sens
    w_y = (gyro_y/(2.0**15.0))*gyro_sens
    w_z = (gyro_z/(2.0**15.0))*gyro_sens
    
    return a_x, a_y, a_z, w_x, w_y, w_z

# MPU6050 레지스터
MPU6050_ADDR = 0x68
PWR_MGMT_1 = 0x6B
SMPLRT_DIV = 0x19
CONFIG = 0x1A
GYRO_CONFIG = 0x1B
ACCEL_CONFIG = 0X1C
INT_ENABLE = 0X38
ACCEL_XOUT_H = 0X3B
ACCEL_YOUT_H = 0X3D
ACCEL_ZOUT_H = 0X3F
GYRO_XOUT_H = 0X43
GYRO_YOUT_H = 0X45
GYRO_ZOUT_H = 0X47

# I2C driver start
bus = smbus.SMBus(1)
gyro_sens, accel_sens = MPU6050_start()