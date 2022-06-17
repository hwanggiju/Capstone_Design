from asyncio.unix_events import _UnixSelectorEventLoop
import face_recognition
import cv2
import RPi.GPIO as GPIO
import time
import sys
import numpy as np
import math
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import os
import smbus
from imusensor.MPU9250 import MPU9250
import FaBo9Axis_MPU9250
mpu9250 = FaBo9Axis_MPU9250.MPU9250()
#그래프
import matplotlib.pyplot as plt
graphRow = 200
x_val = [i for i in range(graphRow)]
y_val = [130 for i in range(graphRow)]
y_valAVG = [130 for i in range(graphRow)]
y_valDesk = [130 for i in range(graphRow)]
y_valPID = [0 for i in range(graphRow)]
gyrosensorX = [0 for i in range(graphRow)]
gyrosensorY = [0 for i in range(graphRow)]
ENA_PWM = [100 for i in range(graphRow)]
ENB_PWM = [100 for i in range(graphRow)]

angleLine = np.linspace(-2,2,graphRow)
heightLine = np.linspace(70, 200, graphRow)
pidLine = np.linspace(-200,200,graphRow)
pwmLine = np.linspace(0,100,graphRow)

plt.ion()
figure, ax = plt.subplots(2, 2 ,figsize=(10, 8))

line_labels = ['User Heght', 'complementary Filter', 'Desk Height', 'PID', 'Angle-X', 'Angle-Y', 'PWM-LEFT', 'PWM-RIGHT']
line1 = ax[0][0].plot(x_val, heightLine, color='red')[0]     # height
line2 = ax[0][0].plot(x_val, heightLine, color='orange')[0]    # height average
line3 = ax[0][0].plot(x_val, heightLine, color='yellow')[0]   # desk height
line4 = ax[0][1].plot(x_val, pidLine, color='green')[0]      # pid
line5 = ax[1][0].plot(x_val, angleLine, color='blue')[0]      # angleX
line6 = ax[1][0].plot(x_val, angleLine, color='navy')[0]     # angleY
line7 = ax[1][1].plot(x_val, pwmLine, color='purple')[0]        # pwm A
line8 = ax[1][1].plot(x_val, pwmLine, color='crimson')[0]       # pwm B

figure.legend([line1, line2, line3, line4, line5, line6, line7], labels= line_labels)
plt.title("SMART DESK", fontsize=20)
plt.xlabel("TIME")
'''
해야할 것
- 정확한 각도 도출
- 각도연산을 통한 모터 속도제어 DC PWM
- OLED 작동 SPI
- 스위치 작동
- GUI 제작 

질문1. 논문을 찾아내서 적정한 높이를 왜 정했는가
질문2. 높이를 자유롭게 정할 수 있는가?
질문3. 앉고 일어서는 자세밖에 없는데 카메라는 너무 과한 테크 아닌가
질문4. 스위치로 올리는 것보다 더 편한가? feat. 김성수교수님

'''
TESTMODE = True
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
MPU_addr = 0x68

################################
# SPI Setting
################################
WIDTH = 128
HEIGHT = 64  # Change to 64 if needed
BORDER = 5

# SPI 선언
spi = board.SPI()
oled_reset = digitalio.DigitalInOut(board.D25)
oled_cs = digitalio.DigitalInOut(board.D8)
oled_dc = digitalio.DigitalInOut(board.D17)
oled = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, oled_dc, oled_reset, oled_cs)

# 초기화
oled.fill(0)
oled.show()

# 하드웨어 연결 구성
# Motor Driver [enA/in1/in2/in3/in4/enB] -> [35, 13, 15, 29, 31, 33]
driver = [19, 27, 22, 5, 6, 13]
# driver = [35, 13, 15, 29, 31, 33]
# I2C [SDA/SCL]
iic_arr = [2, 3]
# UART [TXD/RXD]
uart_arr = [14, 15]
# SPI [SCK, D1, RST, DC, CS] -> GPIO 핀번호 사용 -> [11, 10, 25, 17, 8]
# spi_arr = [23, 19, 22, 11, 24]
spi_arr = [11, 10, 25, 17, 8]
# switch[left/center/right]
switch = [16, 20, 21]
# ultra wave[trig, echo]
wave = [24, 23]

# 사용자 정의 변수
UserTall = 0      # 키는 입력으로 받는다
# 앉았을 때 적정 책상 높이
bestDeskTall = 0 # (UserTall * 0.23) + (UserTall * 0.18)
# 일어섰을 때 적정 높이 = UserTall - bestDeskTall -> 초음파 거리
# minHeight = 80
# seatdownHeight = 0

nowTime = time.time()
preTime = nowTime
initial = True

#기초 데이터 얼굴폭, 거리
#가까울때
userDistanceMin = 70 #cm
faceWidthMax    = 111.3077 #pixel
#멀때
userDistanceMax = 137 #cm
faceWidthMin    = faceWidthMax - 59 #pixel # 54

deskHeight       = 117.5 # 수정

fixAngleX       = 0 #모터 작동시 고정되는 각도값
fixAngleY       = 0
userDistance    = 0

actionNow = 0  # 0:down 1:stop 2:up
actionPre = 1

timeNum = 10 #계산값 평균화 횟수, 클수록 안정되지만 반응이 느려짐
faceWidthAverage = [((faceWidthMax + faceWidthMin)/2) for col in range(timeNum)]

#카메라
cameraWidth = 480
cameraHeight = 640
cameraAngle = 56 # 카메라 수평 화각
cameraWaveDifference = 46 # 카메라 - 센서간 높이차이

# 각도
deskAngle = 0 #28 # 책상 판과 카메라 중심까지의 각도
deskUserAngle = -1 # 책상 판과 사용자 높이 사이의 각도
cameraUserAngle = 0 # 카메라 앵글 안의 사용자 높이 각도

##################################
#하드웨어 초기설정
for i in range(len(driver)): #모터 드라이버 핀
    GPIO.setup(driver[i], GPIO.OUT)
for i in range(len(switch)): #사용자 인터페이스 스위치핀
    GPIO.setup(switch[i], GPIO.IN)
# 초기 시작은 모두 OFF (PWM 핀 제외)
for i in range(1, len(driver)-1):
    GPIO.output(driver[i], GPIO.LOW)
# PWM핀 초기설정
enA_pwm = GPIO.PWM(driver[0], 50) # channel, frequency 50Hz
enB_pwm = GPIO.PWM(driver[5], 50) # channel, frequency 50Hz
enA_pwm.start(0)    # enableA pin start dutycycle 0%
enB_pwm.start(0)    # enableB pin start dutycycle 0%
# 초음파 핀 setup
GPIO.setup(wave[0], GPIO.OUT)
GPIO.setup(wave[1], GPIO.IN)
GPIO.output(wave[0], False)
################################

# OLED 초기설정
oled.fill(0)
oled.show()

font = ImageFont.truetype('malgun.ttf', 15)
font1 = ImageFont.truetype('malgun.ttf', 20)
font2 = ImageFont.truetype('malgun.ttf', 10)

image = Image.new('1', (oled.width, oled.height), 255)
logoImage = Image.open('logo.bmp')
draw = ImageDraw.Draw(image)

# 한바이트 쓰기
def write_byte(adr, data):
    I2C_bus.write_byte_data(MPU_addr, adr, data)

# 한바이트 읽기
def read_byte(adr):
    return I2C_bus.read_byte_data(MPU_addr, adr)

# 두바이트 읽기
def read_word(adr):
    val = 0
    try:
        high = I2C_bus.read_byte_data(MPU_addr, adr)
        low = I2C_bus.read_byte_data(MPU_addr, adr+1)
        val = (high << 8) + low
    except 121:
        print("warnning!")
        time.sleep(2)
        pass
    finally:
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
    Accel값만 이용해서 X, Y의 각도 측정 /현한: 직선 가속도만으로 각도를 측정할 수 있나?
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
DEGREE_PER_SECOND = 32767 / 1000  # Gyro의 Full Scale이 250인 경우
                                 # Full Scale이 1000인 경우 32767/1000

past = 0      # 현재 시간(sec)
past1 = 0 # nani test용 calGyro() 용 충돌예방 (필요없으면 없애도됨)
baseAcX = 0   # 기준점(가만히 있어도 회전이 있나???)
baseAcY = 0
baseAcZ = 0
baseGyX = 0
baseGyY = 0
baseGyZ = 0

GyX_deg = 0   # 측정 각도
GyY_deg = 0
GyZ_deg = 0

average = [ 0 for i in range(100)]
def cal_angle_gyro(GyX, GyY, GyZ):
    # 이 사이트를 참고하면 좋을 듯.
    # https://hs36.tistor   y.com/32
    """
    Gyro를 이용한 현재 각도 계산
    누적 방식이라... 회전하는 방향에 따라 양수/음수가 정해진다.
    :param y: 현재 Gyro 출력
    :return: 현재 각도, 기준 시간 -> past
    """
    global GyX_deg, GyY_deg, GyZ_deg, past

    now = time.time()
    dt = (now - past) / 1000.0
    #GyY_deg -= val
    GyX_deg += ((GyX - baseGyX) / DEGREE_PER_SECOND) * dt
    GyY_deg += ((GyY - baseGyY) / DEGREE_PER_SECOND) * dt
    GyZ_deg += ((GyZ - baseGyZ) / DEGREE_PER_SECOND) * dt
    average[0] = GyY_deg
    val = sum(average)/100
    for i in range(len(average)-1):
        average[len(average) - i - 1] = average[len(average) - i - 2]
        
    past = now      # 다음 계산을 위해 past로 저장되어야 한다.
    return val

# PID 제어식 nani 개발중
# Kp 조절 시
# 오차가 줄어듬 하지만 정상상태 오차로 수렴해 예상값과 오차발생
# Ki 조절 시
# 최초의 목표값과 정상상태 오차의 적분을 비교 : 정상상태오차 및 상승시간 개선
# Kd 조절 시
# 미분을 통해 정상상태로 가는 속도 조절 : 오버슈트 개선
pastPID = time.time() # 초기 셋팅
preError = 0
Ki_term = 0
# 인수 > 센서값 , 목표치
# 출력 > PID값
def PID(currentVal,setVal):
    global pastPID, preError, Kp_term, Ki_term, Kd_term
    Kp = 35.0 #비례
    Ki = 10.11 #적분
    Kd = 25.5 #미분
    now = time.time()
    dt = (now - pastPID) / 1.0
    errorGap_P = setVal - currentVal
    Kp_term = Kp * errorGap_P

    errorGap_I = errorGap_P * dt
    Ki_term += Ki * errorGap_I

    errorGap_D = (errorGap_P - preError) / dt
    Kd_term = Kd * errorGap_D

    preError = errorGap_P
    pastPID = now
    return Kp_term + Ki_term + Kd_term

# 가속도, 각속도를 이용해서 각 도출 (계산 및 상보필터)
def calGyro(accelX, accelY, accelZ, GyroAccX, GyroAccY, GyroAccZ):
    global GyX_deg, GyY_deg, GyZ_deg
    global past1 #기존 시간값 충돌방지
    FS_CEL = 131 #3 Full-Scale Range 0=131, 1=65.5, 2=32.8, 3=16.4
    now = time.time()
    dt = (now - past) / 1000.0
    # convert gyro val to degree
    gyro_x = (GyroAccX - baseGyX) / FS_CEL
    gyro_y = (GyroAccY - baseGyY) / FS_CEL
    gyro_z = (GyroAccZ - baseGyZ) / FS_CEL
    # compute
    gyroAngleX = gyro_x * dt + GyX_deg
    gyroAngleY = gyro_y * dt + GyY_deg
    gyroAngleZ = gyro_z * dt + GyZ_deg
    # calculate Gyro
    RADIANS_TO_DEGREES = 180 / math.pi
    accelAngleX = math.atan(accelX / math.sqrt(math.pow(accelX, 2) + math.pow(accelZ, 2))) * RADIANS_TO_DEGREES
    accelAngleY = math.atan(-1 * accelX / math.sqrt(math.pow(accelY, 2) + math.pow(accelZ, 2))) * RADIANS_TO_DEGREES
    accelAngleZ = 0
    # complementary Filter
    alpha = 0.56
    GyX_deg = alpha * gyroAngleX + (1.0 - alpha) * accelAngleX
    GyY_deg = alpha * gyroAngleY + (1.0 - alpha) * accelAngleY
    GyZ_deg = gyroAngleZ

    past1 = now
    return GyX_deg, GyY_deg, GyZ_deg

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

    avgAcX = SumAcX / 10
    avgAcY = SumAcY / 10
    avgAcZ = SumAcZ / 10
    avgGyX = SumGyX / 10
    avgGyY = SumGyY / 10
    avgGyZ = SumGyZ / 10

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
    baseAcX, baseAcY, baseAcZ, baseGyX, baseGyY, baseGyZ \
        = sensor_calibration()
    past = time.time()

    return read_byte(PWR_MGMT_1)

def getUserHeight(faceWidth, pixelX, pixelY, nowHeight):
    faceWidthAverage[0] = faceWidth
    sumHeight = 0
    for i in range(len(faceWidthAverage)):
        sumHeight = faceWidthAverage[i] + sumHeight
    widthAverage = sumHeight / timeNum
    fullHorizontalAngle = cameraAngle
    fullVerticalAngle = fullHorizontalAngle * cameraHeight / cameraWidth
    faceDifference = faceWidthMax - faceWidthMin
    distanceDifference = userDistanceMax - userDistanceMin
    calUserDistance = ((faceWidthMax) - widthAverage) / faceDifference * distanceDifference + userDistanceMin
    userTopAngle = abs(pixelX - cameraWidth/2) / cameraWidth * fullHorizontalAngle
    userSideAngle = abs(cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
    userDistance = (calUserDistance / np.cos(userTopAngle * np.pi/180))/ np.cos(userSideAngle * np.pi/180)
    gap = calUserDistance / userDistance
    calUserDistance = ((faceWidthMax + (1 - gap) * 3) - widthAverage) / faceDifference * distanceDifference + userDistanceMin
    userDistance = (calUserDistance / np.cos(userTopAngle * np.pi / 180)) / np.cos(userSideAngle * np.pi / 180)
    cameraUserAngle = (cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
    calHeight = np.sin((cameraUserAngle + deskAngle) * np.pi/180) * userDistance# abs(np.sin((cameraUserAngle + deskAngle) * np.pi/180))* 15
    for i in range(timeNum-1):#shift array
        faceWidthAverage[timeNum-1-i] = faceWidthAverage[timeNum-2-i]
    return nowHeight + calHeight

# PWM 값만 바꿀 때
def changePWM(enA, enB):
    if enA < 0 or enA > 100: #range over check
        return False
    elif enB < 0 or enB > 100:
        return False
    enA_pwm.ChangeDutyCycle(0)  # enableA pin start dutycycle 0%
    enB_pwm.ChangeDutyCycle(0)  # enableB pin start dutycycle 0%
    enA_pwm.ChangeDutyCycle(enA)
    enB_pwm.ChangeDutyCycle(enB)
    return True

#각도 자세유지 코드
pwmA = 100
pwmB = 100
pwmA_AVG = 100
pwmB_AVG = 100
preMotorState = 0
def HorizontalHoldTEST(nowAngle, compareAngle):
    global pwmA, pwmB, preMotorState, pwmB_AVG, pwmA_AVG
    angleDiff = nowAngle - compareAngle
    if actionPre == 2:
        if angleDiff > 0 and preMotorState == 1:
            preMotorState = 0 #각 차값이 양수
            pwmA = 100
            pwmB = 100
        elif angleDiff > 0:
            if pwmA > 20:
                pwmA -= 5
            preMotorState = 0
        elif angleDiff < 0 and preMotorState == 0:
            preMotorState = 1 #각 차값이 음수
            pwmA = 100
            pwmB = 100
        elif angleDiff < 0:
            if pwmB > 20:
                pwmB -= 5
            preMotorState = 1
    elif actionPre == 0:
        if angleDiff > 0 and preMotorState == 1:
            preMotorState = 0 #각 차값이 양수
            pwmA = 100
            pwmB = 100
        elif angleDiff > 0:
            if pwmB > 20:
                pwmB -= 5
            preMotorState = 0
        elif angleDiff < 0 and preMotorState == 0:
            preMotorState = 1 #각 차값이 음수
            pwmA = 100
            pwmB = 100
        elif angleDiff < 0:
            if pwmA > 20:
                pwmA -= 5
            preMotorState = 1
    alpha = 0.7
    pwmA_AVG = alpha * pwmA_AVG + (1 - alpha) * pwmA
    pwmB_AVG = alpha * pwmB_AVG + (1 - alpha) * pwmB
    changePWM(pwmA_AVG, pwmB_AVG)
    return pwmA_AVG, pwmB_AVG
# driver set
# 0 : stop
# 1 : down
# 2 : up
def driverSet(enA, motorA, motorB, enB):
    global initial, nowTime, preTime
    if initial == True:
        preTime = time.time()
        initial = False
    changePWM(0, 0)
    for i in range(1, len(driver)-1):
        GPIO.output(driver[i], 0)

    if nowTime - preTime > 0.5: # 작동 딜레이
        if motorA == 2:#up
            GPIO.output(driver[1], 0)
            GPIO.output(driver[2], 1)
        elif motorA == 1 :
            GPIO.output(driver[1], 1)
            GPIO.output(driver[2], 0)
        else:#stop
            GPIO.output(driver[1], 0)
            GPIO.output(driver[2], 0)
            
        if motorB == 2:#up
            GPIO.output(driver[3], 0)
            GPIO.output(driver[4], 1)
        elif motorB == 1 :
            GPIO.output(driver[3], 1)
            GPIO.output(driver[4], 0)
        else:#stop
            GPIO.output(driver[3], 0)
            GPIO.output(driver[4], 0)
        changePWM(enA, enB)
        initial = True
        preTime = nowTime
        return True
    else:
        return False

def waveFun() :
    GPIO.output(wave[0], True)
    time.sleep(0.00001)
    GPIO.output(wave[0], False)
    
    while GPIO.input(wave[1]) == 0 :
        pulse_start = time.time()
    # pulse_end = pulse_start
    while GPIO.input(wave[1]) == 1 :
        pulse_end = time.time()
        # if pulse_end - pulse_start > 1:
        #     return 0
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = round(distance, 5)
    
    return distance
    
def OLED_initial_setting_Height(CHANGE_HEIGHT) :
    draw.text((5, 0), '-First Setting-', font = font, fill = 0)
    draw.text((5, 20), 'Input your Height', font = font, fill = 0)
    draw.text((5, 40), str(CHANGE_HEIGHT), font = font, fill = 0)
    oled.image(image)
    oled.show()

def OLED_initial_setting_Height1(CHANGE_HEIGHT) :
    draw.text((5, 0), '-First Setting-', font = font, fill = 255)
    draw.text((5, 20), 'Input your Height', font = font, fill = 255)
    draw.text((5, 40), str(CHANGE_HEIGHT), font = font, fill = 255)
    oled.image(image)
    oled.show()
    
deskDistance = 0
def drawDisplay() :
        deskDistance = waveFun()
        draw.text((100, 0), 'Up', font=font2, fill=0)
        draw.text((100, 20), 'Okay', font=font2, fill=0)
        draw.text((100, 40), 'Down', font=font2, fill=0)
        draw.text((5, 0), 'Desk Tall', font=font, fill=0)
        draw.text((5, 10), str(deskDistance), font = font, fill = 0)
        preTime = nowTime
        oled.image(image)
        oled.show()
    
def eraseDisplay() :
    draw.text((100, 0), 'Up', font=font2, fill=255)
    draw.text((100, 15), 'Okay', font=font2, fill=255)
    draw.text((100, 30), 'Down', font=font2, fill=255)
    draw.text((5, 0), 'Desk Tall', font=font, fill=255)
    draw.text((5, 10), str(deskDistance), font = font, fill = 255)
    oled.image(image)
    oled.show()

# main code
def main():
    global actionNow, actionPre, bestDeskTall, fixAngleX, fixAngleY
    global nowTime, preTime
    global deskAngle, Ki_term
    # 디스플레이 초기 설정
    try :
        SET_HEIGHT = 170
        oled.image(logoImage)
        oled.show()
        time.sleep(2)
        oled.fill(0)
        oled.show()
        OLED_initial_setting_Height(SET_HEIGHT)
        # if TESTMODE == False: #test 모드일때는 작동 안함
        while True :
            if GPIO.input(switch[2]) == 1 :     # up
                draw.text((5, 0), 'Complete set', font = font, fill = 255)
                draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 255)
                OLED_initial_setting_Height1(SET_HEIGHT)
                SET_HEIGHT = SET_HEIGHT + 1
                OLED_initial_setting_Height(SET_HEIGHT)
                time.sleep(0.2)
            
            elif GPIO.input(switch[0]) == 1:    # okay
                draw.text((5, 0), 'Complete set', font = font, fill = 255)
                draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 255)
                OLED_initial_setting_Height1(SET_HEIGHT)
                SET_HEIGHT = SET_HEIGHT - 1
                OLED_initial_setting_Height(SET_HEIGHT)
                time.sleep(0.2)
                
            elif GPIO.input(switch[1]) == 1:    # down
                OLED_initial_setting_Height1(SET_HEIGHT)
                draw.text((5, 0), 'Your height', font = font, fill = 0)
                draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 0)
                draw.text((5, 40), 'Right?', font = font, fill = 0)
                oled.image(image)
                oled.show()
                time.sleep(0.2)
                while True :
                    if GPIO.input(switch[2]) == 1:
                        draw.text((5, 0), 'Your height', font = font, fill = 255)
                        draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 255)
                        draw.text((5, 40), 'Right?', font = font, fill = 255)
                        OLED_initial_setting_Height1(SET_HEIGHT)
                        SET_HEIGHT = SET_HEIGHT + 1
                        OLED_initial_setting_Height(SET_HEIGHT)
                        time.sleep(0.2)
                        break
                    elif GPIO.input(switch[0]) == 1:
                        draw.text((5, 0), 'Your height', font = font, fill = 255)
                        draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 255)
                        draw.text((5, 40), 'Right?', font = font, fill = 255)
                        OLED_initial_setting_Height1(SET_HEIGHT)
                        SET_HEIGHT = SET_HEIGHT - 1
                        OLED_initial_setting_Height(SET_HEIGHT)
                        time.sleep(0.2)
                        break
                    elif GPIO.input(switch[1]) == 1 :
                        draw.text((5, 0), 'Your height', font = font, fill = 255)
                        draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 255)
                        draw.text((5, 40), 'Right?', font = font, fill = 255)
                        time.sleep(0.2)
                        draw.text((5, 0), 'Success Set Height', font = font2, fill = 0)
                        time.sleep(1)
                        break
                draw.text((5, 0), 'Success Set Height', font = font2, fill = 255)
                break

        cap = cv2.VideoCapture(0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, cameraHeight)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cameraWidth)
        test = set_MPU_init(dlpf_bw=DLPF_BW_98)
        # 2) Gyro 기준값 계산(Gyro 이용시)
        sensor_calibration()    # Gyro의 기준값 계산
        if not cap.isOpened() :
            print('Camera open failed!')
            sys.exit()

        model = 'res10_300x300_ssd_iter_140000.caffemodel'
        config = 'deploy.prototxt.txt'

        net = cv2.dnn.readNet(model, config)
        if net.empty() :
            print('Net open failed!')

        fixAngleY = 0
        waveSensorMean = 0
        waveSensorHeight = 70 # 최소 길이 초기화 71.5
        stop = False
        HeightAVG = [130 for i in range(15)]
        WaveAVG = [waveSensorHeight for i in range(15)]



        while True:
            drawDisplay()
            accel = mpu9250.readAccel()
            gyro = mpu9250.readGyro()
            if TESTMODE == False:
                time.sleep(0.005)
            nowTime = time.time()
            _, frame = cap.read()
            
            rotate_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
            if frame is None:
                print("fail")
                break
            
            blob = cv2.dnn.blobFromImage(rotate_frame,  # image
                                        1,  # scalefactor
                                        (200, 200),  # image Size
                                        (104, 177, 123)  # Scalar
                                        )
            net.setInput(blob)
            detect = net.forward()
            (h, w) = rotate_frame.shape[:2]
            detect = detect[0, 0, :, :]
            rotate_frame = cv2.resize(rotate_frame, (0, 0), fx=0.4, fy=0.4)
                
            waveSensorHeight = waveFun() # 책상 높이
            WaveAVG[0] = waveSensorHeight
            for i in range(len(WaveAVG) - 1) :
                WaveAVG[len(WaveAVG) - i - 1] = WaveAVG[len(WaveAVG) - i - 2]
            waveSensorMean = np.mean(WaveAVG) # 초음파 평균 거리

            #print('test')
            # 4-2) Gyro를 이용한 각도 계산 
            #Gy_Angle = cal_angle_gyro(GyX, GyY, GyZ)
            #print('test')
            # nani 각도 코드 테스트
            angleX, angleY, angleZ = calGyro(accel['x'], accel['y'], accel['z'] ,gyro['x'] , gyro['y'], gyro['z'])
            deskAngle = angleX
            print("nani = ", round(angleY, 4))

            #수평 자세 유지 코드 (현재 각도, 작동시 각도)
            ENA_PWM[0], ENB_PWM[0] = HorizontalHoldTEST(angleY, fixAngleY)
            
            userNum = 0
            for i in range(detect.shape[0]):
                confidence = detect[i, 2]
                if confidence < 0.5:
                    break
                userNum = userNum + 1
                x1 = int(detect[i, 3] * w)
                y1 = int(detect[i, 4] * h)
                x2 = int(detect[i, 5] * w)
                y2 = int(detect[i, 6] * h)

                cv2.rectangle(rotate_frame, (x1, y1), (x2, y2), (0, 255, 0))  # green ractangle
                
            if userNum == 1: #인식된 얼굴 수
                # 책상 다리 모터 제어에 활용되는 값
                widthLength = x2 - x1
                heightLength = y2 - y1
                area = widthLength * heightLength  # 사용자 인식 넓이
                center_x = x1 + (x2 - x1) / 2
                center_y = y1 + (y2 - y1) / 2  # 인식된 부분 중심 좌표 x, y 값

                print(" 가로 :" + str(widthLength) + " 세로:" + str(heightLength), end='')
                print(' area : %d    center_x : %d   center_y : %d \n'
                    % (area, center_x, center_y))

                # 얼굴폭 / 계산 좌표 X / 계산 좌표 Y / 카메라 높이
                userHeight = getUserHeight(widthLength,center_x,center_y-heightLength/2, waveSensorHeight+cameraWaveDifference+2)
                HeightAVG[0] = userHeight
                for i in range(len(HeightAVG) - 1):
                    HeightAVG[len(HeightAVG) - i - 1] = HeightAVG[len(HeightAVG) - i - 2]
                # 사용자의 현재 키
                # 현재 키의 값 변화를 천천히 바꿔주기 위함
                userHeightAVG = np.mean(HeightAVG)
                if TESTMODE == True:
                    print("현재 키값 :" + str(round(userHeight, 2)))
                    print("테스트식 결과 :" + str(round(userHeightAVG, 2)))
                    print("차값 :" + str(userHeight - userHeightAVG))
                    #val = PID(userHeightAVG, userHeight)
                    #print("PID 계산값 " + str(round(val, 5)))
                    # 그래프 값 입력부
                    y_val[0] = userHeight
                    y_valAVG[0] = userHeightAVG
                    y_valDesk[0] = waveSensorHeight + 2
                    # 쉬프트 그래프
                    for i in range(graphRow - 1):
                        y_val[graphRow - i - 1] = y_val[graphRow - i - 2]
                        y_valAVG[graphRow - i - 1] = y_valAVG[graphRow - i - 2]
                        y_valDesk[graphRow - i - 1] = y_valDesk[graphRow - i - 2]
                # 실제 책상 높이는 78cm인데, 키를 바탕으로한 최적의 높이 식을 대입하면 키가 190cm 사람이 최적의 책상 높이가 77.9 ????
                # 책상의 최적 높이와 사용자의 현재 키를 빼서 최적의 값을 알아낸다 
                #높이에 따른 모터작동
                if stop != True: # 드라이버 pin Set 변경 후 반복 변경 방지
                    # 앉았을 때, 책상의 최적 높이 설정
                        # down
                    if userHeightAVG < 140 :
                        stop = driverSet(100, 1, 1, 100)  
                        actionPre = 0#down
                        fixAngleY = angleY  # 현재 각도고정
                        fixAngleX = angleX
                        Ki_term = 0
                        print("down")
                    # up    
                    elif userHeightAVG > 150 :
                        stop = driverSet(100, 2, 2, 100)
                        actionPre = 2#up
                        fixAngleY = angleY  # 현재 각도고정
                        fixAngleX = angleX
                        Ki_term = 0
                        print("up")
                    else:
                        stop = driverSet(0, 0, 0, 0)  # stay
                        actionPre = 1#stop
                        fixAngleY = angleY
                        fixAngleX = angleX
                        Ki_term = 0
                        print("stop")
                else:
                    if userHeightAVG < 140 and actionPre != 0:
                        stop = False
                    elif userHeightAVG > 150 and actionPre != 2:
                        stop = False
                    elif userHeightAVG > 140 and userHeightAVG < 150 and actionPre != 1 :
                        stop = False
            print("초음파 측정 거리 : %d\n" % (waveSensorMean+3))
            #그래프 표시

            gyrosensorX[0] = angleX - fixAngleX
            gyrosensorY[0] = angleY - fixAngleY
            #gyrosensorY[0] = angleY
            y_valPID[0] = 0
            # 쉬프트
            for i in range(graphRow - 1):
                gyrosensorX[graphRow - i - 1] = gyrosensorX[graphRow - i - 2]
                gyrosensorY[graphRow - i - 1] = gyrosensorY[graphRow - i - 2]
                ENA_PWM[graphRow - i - 1] = ENA_PWM[graphRow - i - 2]
                ENB_PWM[graphRow - i - 1] = ENB_PWM[graphRow - i - 2]
                y_valPID[graphRow - i - 1] = y_valPID[graphRow - i - 2]

            if TESTMODE == True:
                line1.set_ydata(y_val)
                line2.set_ydata(y_valAVG)
                line3.set_ydata(y_valDesk)
                line4.set_ydata(y_valPID)
                line5.set_ydata(gyrosensorX)
                line6.set_ydata(gyrosensorY)
                line7.set_ydata(ENA_PWM)
                line8.set_ydata(ENB_PWM)
                #figure.canvas.draw()
                figure.canvas.flush_events()
            cv2.imshow("Camera", rotate_frame)
    except KeyboardInterrupt :
        pass
    
if __name__ == "__main__":
    main()
    enA_pwm.stop()
    enB_pwm.stop()
    GPIO.cleanup()
    cv2.destroyAllWindows()

