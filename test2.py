import smbus			#import SMBus module of I2C
from time import sleep          #import
from time import time 

PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

# 각도(deg) = Gyro값(step) / DEGREE_PER_SECOND(step*sec/deg) * dt(sec) 의 누적...
DEGREE_PER_SECOND = 32767 / 250  # Gyro의 Full Scale이 250인 경우
                                 # Full Scale이 1000인 경우 32767/1000

past = 0      # 현재 시간(sec)
baseAcX = 0   # 기준점(가만히 있어도 회전이 있나???)
baseAcY = 0
baseAcZ = 0
baseGyX = 0
baseGyY = 0
baseGyZ = 0

GyX_deg = 0   # 측정 각도
GyY_deg = 0
GyZ_deg = 0

def cal_angle_gyro(GyX, GyY, GyZ):
    """
    Gyro를 이용한 현재 각도 계산
    누적 방식이라... 회전하는 방향에 따라 양수/음수가 정해진다.
    :param y: 현재 Gyro 출력
    :return: 현재 각도, 기준 시간 -> past
    """
    global GyX_deg, GyY_deg, GyZ_deg

    now = time()
    dt = now - past     # 초단위

    GyX_deg += ((GyX - baseGyX) / DEGREE_PER_SECOND) * dt
    GyY_deg += ((GyY - baseGyY) / DEGREE_PER_SECOND) * dt
    GyZ_deg += ((GyZ - baseGyZ) / DEGREE_PER_SECOND) * dt

    return now      # 다음 계산을 위해 past로 저장되어야 한다.

def sensor_calibration():
    """
    1초동안의 평균을 이용하여 기준점 계산
    :return: Accel과 Gyro의 기준점 -> baseAcX ~ basGyZ
    """
    SumAcX = 0
    SumAcY = 0
    SumAcZ = 0
    SumGyX = 0
    SumGyY = 0
    SumGyZ = 0

    for i in range(10):
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
        SumAcX += AcX
        SumAcY += AcY
        SumAcZ += AcZ
        SumGyX += GyX
        SumGyY += GyY
        SumGyZ += GyZ
        time.sleep(0.1)

    avgAcX = SumAcX / 10
    avgAcY = SumAcY / 10
    avgAcZ = SumAcZ / 10
    avgGyX = SumGyX / 10
    avgGyY = SumGyY / 10
    avgGyZ = SumGyZ / 10

    return avgAcX, avgAcY, avgAcZ, avgGyX, avgGyY, avgGyZ


def set_MPU6050_init(dlpf_bw=DLPF_BW_256,
                     gyro_fs=GYRO_FS_250, accel_fs=ACCEL_FS_2,
                     clk_pll=CLOCK_PLL_XGYRO):
    global baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ, past

    # MPU6050 초기값 setting
    write_byte(PWR_MGMT_1, SLEEP_EN | clk_pll)      # sleep mode(bit6), clock(bit2:0)은 XGyro 동기
    write_byte(CONFIG, dlpf_bw)                     # bit 2:0
    write_byte(GYRO_CONFIG, gyro_fs)                # Gyro Full Scale bit 4:3
    write_byte(ACCEL_CONFIG, accel_fs)              # Accel Full Scale Bit 4:3
    write_byte(PWR_MGMT_1, SLEEP_DIS | clk_pll)     # Start

    # sensor 계산 초기화
    baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ \
        = sensor_calibration()
    past = time.time()

    return read_byte(PWR_MGMT_1)    # 정말 시작되었는지 확인

if __name__ == '__main__':
    ''' -----------------------------------'''
    ''' Gyro 테스트 '''
    ''' -----------------------------------'''
    # 1) MPU 6050 초기화
    test = set_MPU6050_init(dlpf_bw=DLPF_BW_98)   # BW만 변경, 나머지는 default 이용
    print("Gyro PWR_MGMT_1 Register = ", test)

    # 2) Gyro 기준값 계산(Gyro 이용시)
    sensor_calibration()    # Gyro의 기준값 계산

    cnt = 0

    while True:
        # 3) accel, gyro의 Raw data 읽기, 
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
     
        # 4-1) Accel을 이용한 각도 계산
        AcX_deg, AcY_deg = cal_angle_acc(AcX, AcY, AcZ)

        # 4-2) Gyro를 이용한 각도 계산 
        past = cal_angle_gyro(GyX, GyY, GyZ)

        # 5) 0.01초 간격으로 값 읽기
        time.sleep(0.01)
        cnt += 1

        # 1초에 한번씩 display
        if cnt%100 == 0:
            print("GyX,Y,Z_deg = ", GyX_deg, ',', GyY_deg, ',', GyZ_deg)
            # print("AcX_deg, AcY_deg = ", AcX_deg, ',', AcY_deg)



'''
def read_raw_data(addr):
	#Accelero and Gyro value are 16-bit
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr+1)

    #concatenate higher and lower value
    value = ((high << 8) | low)
    
    #to get signed value from mpu6050
    if(value > 32768):
        value = value - 65536
    return value


bus = smbus.SMBus(1) 	# or bus = smbus.SMBus(0) for older version boards
Device_Address = 0x68   # MPU6050 device address

MPU_Init()

print (" Reading Data of Gyroscope and Accelerometer")

while True:
	
	#Read Accelerometer raw value
	acc_x = read_raw_data(ACCEL_XOUT_H)
	acc_y = read_raw_data(ACCEL_YOUT_H)
	acc_z = read_raw_data(ACCEL_ZOUT_H)
	
	#Read Gyroscope raw value
	gyro_x = read_raw_data(GYRO_XOUT_H)
	gyro_y = read_raw_data(GYRO_YOUT_H)
	gyro_z = read_raw_data(GYRO_ZOUT_H)
	
	#Full scale range +/- 250 degree/C as per sensitivity scale factor
	Ax = acc_x/16384.0
	Ay = acc_y/16384.0
	Az = acc_z/16384.0
	
	Gx = gyro_x/131.0
	Gy = gyro_y/131.0
	Gz = gyro_z/131.0
	

	print ("Gx=%.2f" %Gx, u'\u00b0'+ "/s", "\tGy=%.2f" %Gy, u'\u00b0'+ "/s", "\tGz=%.2f" %Gz, u'\u00b0'+ "/s", "\tAx=%.2f g" %Ax, "\tAy=%.2f g" %Ay, "\tAz=%.2f g" %Az) 	
'''