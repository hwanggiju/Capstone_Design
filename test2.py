import smbus			#import SMBus module of I2C
import time
import math
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# 레지스터 값 설정
CONFIG       = 0x1A     # LowPassFilter bit 2:0
GYRO_CONFIG  = 0x1B     # FS_SEL bit 4:3
ACCEL_CONFIG = 0x1C     # FS_SEL bit 4:3
PWR_MGMT_1   = 0x6B     # sleep bit 6, clk_select bit 2:0

# CONFIG: Low Pass Filter 설정(bit 2:0)
DLPF_BW_256 = 0x00      # Acc: BW-260Hz, Delay-0ms, Gyro: BW-256Hz, Delay-0.98ms
DLPF_BW_188 = 0x01
DLPF_BW_98  = 0x02
DLPF_BW_42  = 0x03
DLPF_BW_20  = 0x04
DLPF_BW_10  = 0x05
DLPF_BW_5   = 0x06      # Acc: BW-5Hz, Delay-19ms, Gyro: BW-5Hz, Delay-18.6ms

# GYRO_CONFIG: Gyro의 Full Scale 설정(bit 4:3)
GYRO_FS_250  = 0x00 << 3    # 250 deg/sec
GYRO_FS_500  = 0x01 << 3
GYRO_FS_1000 = 0x02 << 3
GYRO_FS_2000 = 0x03 << 3    # 2000 deg/sec

# ACCEL_CONFIG: 가속도센서의 Full Scale 설정(bit 4:3)
ACCEL_FS_2  = 0x00 << 3     # 2g
ACCEL_FS_4  = 0x01 << 3
ACCEL_FS_8  = 0x02 << 3
ACCEL_FS_16 = 0x03 << 3     # 16g

# PWR_MGMT_1: sleep(bit 6)
SLEEP_EN        = 0x01 << 6
SLEEP_DIS       = 0x00 << 6
# PWR_MGMT_1: clock(bit 2:0)
CLOCK_INTERNAL  = 0x00  # internal clk(8KHz) 이용 (Not! Recommended)
CLOCK_PLL_XGYRO = 0x01  # XGyro와 동기
CLOCK_PLL_YGYRO = 0x02  # YGyro와 동기
CLOCK_PLL_ZGYRO = 0x03  # ZGyro와 동기

# Data 읽기
ACCEL_XOUT_H = 0x3B     # Low는 0x3C
ACCEL_YOUT_H = 0x3D     # Low는 0x3E
ACCEL_ZOUT_H = 0x3F     # Low는 0x40
GYRO_XOUT_H  = 0x43     # Low는 0x44
GYRO_YOUT_H  = 0x45     # Low는 0x46
GYRO_ZOUT_H  = 0x47     # Low는 0x48

################################
# I2C 읽고 쓰기
################################
# I2C Bus 초기화
I2C_bus = smbus.SMBus(1)
MPU6050_addr = 0x68

# 한바이트 쓰기
def write_byte(adr, data):
    I2C_bus.write_byte_data(MPU6050_addr, adr, data)

# 한바이트 읽기
def read_byte(adr):
    return I2C_bus.read_byte_data(MPU6050_addr, adr)

# 두바이트 읽기
def read_word(adr):
    high = I2C_bus.read_byte_data(MPU6050_addr, adr)
    low = I2C_bus.read_byte_data(MPU6050_addr, adr+1)
    val = (high << 8) + low
    return val

# 두바이트를 2's complement로 읽기(-32768~32767)
# 아두이노는 변수를 signed 16bit로 선언해서 처리하지만
# 라즈베리파이는 이렇게 변환해 줘야 한다. 
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
        return val
    
# -----------------------------------------------    

def get_raw_data():
    """
    가속도(accel)와 각속도(gyro)의 현재 값 읽기
    :return: accel x/y/z, gyro x/y/z
    """
    gyro_xout = read_word_2c(GYRO_XOUT_H)
    gyro_yout = read_word_2c(GYRO_YOUT_H)
    gyro_zout = read_word_2c(GYRO_ZOUT_H)
    accel_xout = read_word_2c(ACCEL_XOUT_H)
    accel_yout = read_word_2c(ACCEL_YOUT_H)
    accel_zout = read_word_2c(ACCEL_ZOUT_H)
    return accel_xout, accel_yout, accel_zout,\
           gyro_xout, gyro_yout, gyro_zout

# 가속도 앵글
def cal_angle_acc(AcX, AcY, AcZ):
    """
    Accel값만 이용해서 X, Y의 각도 측정
    (고정 좌표 기준?)
    그런데... 각도가 0 -> 90 -> 0 -> -90 -> 0으로 바뀐다. 왜?
    0도 -> 90도 -> 180도 -> 270도 -> 360도
    즉, 30도와 120도가 모두 30도로 표시된다. 왜?
    :param AcX: Accel X
    :param AcY: Accel Y
    :param AcZ: Accel Z
    :return: X, Y angle in degree
    """
    y_radians = math.atan2(AcX, math.sqrt((AcY*AcY) + (AcZ*AcZ)))
    x_radians = math.atan2(AcY, math.sqrt((AcX*AcX) + (AcZ*AcZ)))
    return math.degrees(x_radians), -math.degrees(y_radians)

# 각속도 각도 계산
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
    global past
    """
    Gyro를 이용한 현재 각도 계산
    누적 방식이라... 회전하는 방향에 따라 양수/음수가 정해진다.
    :param y: 현재 Gyro 출력
    :return: 현재 각도, 기준 시간 -> past
    """
    global GyX_deg, GyY_deg, GyZ_deg, baseGyX, baseGyY, baseGyZ

    now = time.time()
    dt = now - past     # 초단위
    # if abs(((GyX - baseGyX) / DEGREE_PER_SECOND) * dt) > 0.02:
    GyX_deg += ((GyX - baseGyX) / DEGREE_PER_SECOND) * dt
    GyY_deg += ((GyY - baseGyY) / DEGREE_PER_SECOND) * dt
    GyZ_deg += ((GyZ - baseGyZ) / DEGREE_PER_SECOND) * dt
    baseGyX = GyX
    baseGyY = GyY
    baseGyZ = GyZ
    past = now      # 다음 계산을 위해 past로 저장되어야 한다.


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

    for i in range(20):
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
        SumAcX += AcX
        SumAcY += AcY
        SumAcZ += AcZ
        SumGyX += GyX
        SumGyY += GyY
        SumGyZ += GyZ
        

    avgAcX = SumAcX / 20
    avgAcY = SumAcY / 20
    avgAcZ = SumAcZ / 20
    avgGyX = SumGyX / 20
    avgGyY = SumGyY / 20
    avgGyZ = SumGyZ / 20

    return avgAcX, avgAcY, avgAcZ, avgGyX, avgGyY, avgGyZ

def set_MPU_init(dlpf_bw=DLPF_BW_256,
                gyro_fs=GYRO_FS_250, accel_fs=ACCEL_FS_2,
                clk_pll=CLOCK_PLL_XGYRO):
    global baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ, past

    write_byte(PWR_MGMT_1, SLEEP_EN | clk_pll)      # sleep mode(bit6), clock(bit2:0)은 XGyro 동기
    write_byte(CONFIG, dlpf_bw)                     # bit 2:0
    write_byte(GYRO_CONFIG, gyro_fs)                # Gyro Full Scale bit 4:3
    write_byte(ACCEL_CONFIG, accel_fs)              # Accel Full Scale Bit 4:3
    write_byte(PWR_MGMT_1, SLEEP_DIS | clk_pll)     # Start

    # sensor 계산 초기화
    # baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ \
    #    = sensor_calibration()
    past = time.time()

    return read_byte(PWR_MGMT_1)

def animate(i):
    x_value.append(next(nowtime))
    y_value.append(GyX_deg)
    plt.cla()
    plt.plot(x_value, y_value)
 

if __name__ == '__main__':
    ''' -----------------------------------'''
    ''' Gyro 테스트 '''
    ''' -----------------------------------'''
    test = set_MPU_init(dlpf_bw=DLPF_BW_98)   # BW만 변경, 나머지는 default 이용
    print("Gyro PWR_MGMT_1 Register = ", test)

    # 2) Gyro 기준값 계산(Gyro 이용시)
    sensor_calibration()    # Gyro의 기준값 계산

    cnt = 0
    nowtime=time.time()
    x_value = []
    y_value = []
    
    ani = FuncAnimation(plt.gcf(), animate, interval = 1000)
    
    plt.show()

    while True:
        # 3) accel, gyro의 Raw data 읽기, 
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
     
        # 4-1) Accel을 이용한 각도 계산
        AcX_deg, AcY_deg = cal_angle_acc(AcX, AcY, AcZ)

        # 4-2) Gyro를 이용한 각도 계산 
        cal_angle_gyro(GyX, GyY, GyZ)

        # 5) 0.01초 간격으로 값 읽기
        time.sleep(0.01)
        cnt += 10

        # 1초에 한번씩 display
        if cnt%100 == 0:
            print("GyX,Y,Z_deg = ", round(GyX_deg, 4), ',', round(GyY_deg, 4), ',',round(GyZ_deg, 4))
            # print("AcX_deg, AcY_deg = ", AcX_deg, ',', AcY_deg)
    
    ani = FuncAnimation(plt.gcf(), animate, interval = 1000)
            
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
'''