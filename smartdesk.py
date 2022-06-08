from asyncio.unix_events import _UnixSelectorEventLoop
import face_recognition
import cv2
import RPi.GPIO as GPIO
import time
import sys
import numpy as np
import math

# spi SSD1306 OLED code
# pip install spidev 설치
# import spidev

# MPU9250 gyro sensor code
import os
import smbus
from imusensor.MPU9250 import MPU9250

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

# spi = spidev.SpiDev(0, spi_ch)
# spi.max_speed_hz = 1200000



# Motor Driver [enA/in1/in2/in3/in4/enB]
driver = [35, 13, 15, 29, 31, 33]
# I2C [SDA/SCL]
iic_arr = [3, 5]
# UART [TXD/RXD]
uart_arr = [8, 10]
# SPI [MOSI/MISO/SCK/CE0/CE1]
spi_arr = [19, 21, 23, 24, 26]
# switch[left/center/right]
switch = [36, 38, 40]
# ultra wave[trig, echo]
wave = [18, 16]

# 사용자 정의 변수
# maxHeight = 170
# minHeight = 80
# seatdownHeight = 0

nowTime = time.time()
preTime = nowTime
initial = True
#가까울때
userDistanceMin = 60 #cm
faceWidthMax    = 110 #pixel
#멀때
userDistanceMax = 114 #cm
faceWidthMin    = 72 #pixel

deskHeight = 117.5# 수정
waveSensorHeight = 70 # 최소 길이 초기화 71.5

userDistance    = 0

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

timeNum = 10 #평균횟수 클수록 둔화됨, 하지만 반응이 느려짐
faceWidthAverage = [((faceWidthMax + faceWidthMin)/2) for col in range(timeNum)]

#카메라
cameraWidth = 480
cameraHeight = 640
cameraAngle = 56 # 카메라 수평 화각
cameraWaveDifference = 46 #카메라 - 센서간 높이차이

# 각도
deskAngle = -3 #28 # 책상 판과 카메라 중심까지의 각도
deskUserAngle = -1 # 책상 판과 사용자 높이 사이의 각도
cameraUserAngle = 0 # 카메라 앵글 안의 사용자 높이 각도


#하드웨어 초기설정
for i in range(len(driver)):
    GPIO.setup(driver[i], GPIO.OUT)
for i in range(len(switch)):
    GPIO.setup(switch[i], GPIO.IN)
# initial system down
for i in range(len(driver)):
    GPIO.output(driver[i], GPIO.LOW)
# 초음파 핀 setup
GPIO.setup(wave[0], GPIO.OUT)
GPIO.setup(wave[1], GPIO.IN)
GPIO.output(wave[0], False)

# enA_pwm = GPIO.PWM(driver[0], 1)  # channel, frequecy
# enB_pwm = GPIO.PWM(driver[5], 1)

# 한바이트 쓰기
def write_byte(adr, data):
    I2C_bus.write_byte_data(MPU_addr, adr, data)

# 한바이트 읽기
def read_byte(adr):
    return I2C_bus.read_byte_data(MPU_addr, adr)

# 두바이트 읽기
def read_word(adr):
    high = I2C_bus.read_byte_data(MPU_addr, adr)
    low = I2C_bus.read_byte_data(MPU_addr, adr+1)
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
    
def get_raw_data():
    """
    가속도(accel)와 각속도(gyro)의 현재 값 읽기
    :return: accel x/y/z, gyro x/y/z
    """
    gyro_xout = 0 # read_word_2c(GYRO_XOUT_H)
    gyro_yout = read_word_2c(GYRO_YOUT_H)
    gyro_zout = 0 # read_word_2c(GYRO_ZOUT_H)
    accel_xout = 0 # read_word_2c(ACCEL_XOUT_H)
    accel_yout = 0 # read_word_2c(ACCEL_YOUT_H)
    accel_zout = 0 # read_word_2c(ACCEL_ZOUT_H)
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

average = [ 0 for i in range(10)]
def cal_angle_gyro(GyX, GyY, GyZ):
    global past
    """
    Gyro를 이용한 현재 각도 계산
    누적 방식이라... 회전하는 방향에 따라 양수/음수가 정해진다.
    :param y: 현재 Gyro 출력
    :return: 현재 각도, 기준 시간 -> past
    """
    global GyX_deg, GyY_deg, GyZ_deg

    now = time.time()
    dt = now - past
    GyX_deg += ((GyX - baseGyX) / DEGREE_PER_SECOND) * dt
    GyY_deg += ((GyY - baseGyY) / DEGREE_PER_SECOND) * dt
    GyZ_deg += ((GyZ - baseGyZ) / DEGREE_PER_SECOND) * dt
    average[0] = GyY_deg
    val = sum(average)/10
    for i in range(len(average)-1):
        average[len(average) - i - 1] = average[len(average) - i - 2]
        
    past = now      # 다음 계산을 위해 past로 저장되어야 한다.
    return val
    
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

def getUserHeight_nani(faceWidth, pixelX, pixelY, nowHeight):
    faceWidthAverage[0] = faceWidth
    sumHeight = 0
    for i in range(len(faceWidthAverage)):
        sumHeight = faceWidthAverage[i] + sumHeight
    widthAverage = sumHeight / timeNum
    fullHorizontalAngle = cameraAngle
    fullVerticalAngle = fullHorizontalAngle * cameraHeight / cameraWidth
    faceDifference = faceWidthMax - faceWidthMin
    distanceDifference = userDistanceMax - userDistanceMin
    calUserDistance = (faceWidthMax - widthAverage) / faceDifference * distanceDifference + userDistanceMin
    userTopAngle = abs(pixelX - cameraWidth/2) / cameraWidth * fullHorizontalAngle
    userDistance = calUserDistance / np.cos(userTopAngle * np.pi/180)
    cameraUserAngle = (cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
    calHeight = np.tan((cameraUserAngle + deskAngle) * np.pi/180) * (userDistance + 10)
    for i in range(timeNum-1):#shift array
        faceWidthAverage[timeNum-1-i] = faceWidthAverage[timeNum-2-i]
    return nowHeight + calHeight

def getUserHeight_nani1(faceWidth, pixelX, pixelY, nowHeight):
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
    calUserDistance = ((faceWidthMax + (1 - gap) * 10) - widthAverage) / faceDifference * distanceDifference + userDistanceMin
    userDistance = (calUserDistance / np.cos(userTopAngle * np.pi / 180)) / np.cos(userSideAngle * np.pi / 180)
    cameraUserAngle = (cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
    calHeight = np.sin((cameraUserAngle + deskAngle) * np.pi/180) * (userDistance+15)
    for i in range(timeNum-1):#shift array
        faceWidthAverage[timeNum-1-i] = faceWidthAverage[timeNum-2-i]
    return nowHeight + calHeight

# driver
# 0 : stop
# 1 : down
# 2 : up
def driverSet(enA, motorA, motorB, enB):
    global initial, nowTime, preTime
    if initial == True:
        preTime = time.time()
        initial = False
    # enA_pwm.start(0)
    # enB_pwm.start(0)
    for i in range(len(driver)):
        GPIO.output(driver[i], 0)
    if nowTime - preTime > 0.5:
        if motorA == 2:#up
            GPIO.output(driver[1], 0)
            GPIO.output(driver[2], 1)
        elif motorA == 1:#down
            GPIO.output(driver[1], 1)
            GPIO.output(driver[2], 0)
        else:#stop
            GPIO.output(driver[1], 0)
            GPIO.output(driver[2], 0)
        if motorB == 2:#up
            GPIO.output(driver[3], 0)
            GPIO.output(driver[4], 1)
        elif motorB == 1:#down
            GPIO.output(driver[3], 1)
            GPIO.output(driver[4], 0)
        else:#stop
            GPIO.output(driver[3], 0)
            GPIO.output(driver[4], 0)
        GPIO.output(driver[0], enA)
        GPIO.output(driver[5], enB)
        initial = True
        preTime = nowTime
        return True
    else:
        return False
    # enA_pwm.start(100)
    # enB_pwm.start(100)
    
def waveFun() :
    GPIO.output(wave[0], True)
    time.sleep(0.00001)
    GPIO.output(wave[0], False)
    
    while GPIO.input(wave[1]) == 0 :
        pulse_start = time.time()
        
    while GPIO.input(wave[1]) == 1 :
        pulse_end = time.time()
        
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration   * 17000
    distance = round(distance, 2)
    
    return distance

# main code


def main():
    global nowTime, preTime
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
        sys.exit()

    # 픽셀 최대 최소값 초기화
    # maxHeightPixel = 0
    # minHeightPixel = 1000


    # cnt = 0
    AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
    
    actionNow = 0  # 0:down 1:stop 2:up
    actionPre = 1
    #driverSet(1, 1, 1, 1)  # down
    #time.sleep(5)
    stop = False
    while True:
        time.sleep(0.005)
        nowTime = time.time()
        _, frame = cap.read()
        rotate_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)
        if frame is None:
            break
        blob = cv2.dnn.blobFromImage(rotate_frame,  # image
                                     1,  # scalefactor
                                     (150, 150),  # image Size
                                     (104, 177, 123)  # Scalar
                                     )
        net.setInput(blob)
        detect = net.forward()
        (h, w) = rotate_frame.shape[:2]
        detect = detect[0, 0, :, :]
        userNum = 0
        
        waveSensorHeight = waveFun() # 책상 높이
        
        # 3) accel, gyro의 Raw data 읽기, 
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
        
        # 4-1) Accel을 이용한 각도 계산
        AcX_deg, AcY_deg = cal_angle_acc(AcX, AcY, AcZ)

        # 4-2) Gyro를 이용한 각도 계산 
        Gy_Angle = cal_angle_gyro(GyX, GyY, GyZ)
        
        print("GyY = ", round(Gy_Angle,4))
        # print("AcX_deg, AcY_deg = ", AcX_deg, ',', AcY_deg)
        
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
            
        if userNum == 1:
            # 책상 다리 모터 제어에 활용되는 값
            area = (x2 - x1) * (y2 - y1)  # 사용자 인식 넓이
            center_x = x1 + (x2 - x1) / 2
            center_y = y1 + (y2 - y1) / 2  # 인식된 부분 중심 좌표 x, y 값
            width = x2 - x1
            height = y2 - y1
            print(" 가로 :" + str(width) + "  세로:" + str(height), end='')
            print('  area : %d    center_x : %d   center_y : %d \n'
                % (area, center_x, center_y))
            #Height = getUserHeight_nani(width,center_x,center_y-height/2, deskHeight)
            Height = getUserHeight_nani1(width,center_x,center_y-height/2, waveSensorHeight+cameraWaveDifference+1.5)
            print("테스트 nani 식 :" + str(Height) + "\n")
            if waveSensorHeight < 72 :
                stop = driverSet(0, 0, 0, 0)
                
            #높이에 따른 모터작동
            if stop != True:
                if Height < 120:
                    stop = driverSet(1, 1, 1, 1)  # down
                    actionPre = 0#down
                    print("down\n")
                elif Height > 130:
                    stop = driverSet(1, 2, 2, 1)  # up
                    actionPre = 2#up
                    print("up\n")
                else:
                    stop = driverSet(0, 0, 0, 0)  # stay
                    actionPre = 1#stop
                    print("stop")
            else:
                if Height < 120 and actionPre != 0:
                    stop = False
                elif Height > 130 and actionPre != 2:
                    stop = False
                elif Height >= 120 and Height <= 130 and actionPre != 1:
                    stop = False

        print("초음파 측정 거리 : %d\n" % (waveSensorHeight))
        # cv2.imshow('Facerec_Video', rotate_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            # enA_pwm.stop()
            # enB_pwm.stop()
            GPIO.cleanup()
            break

if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()