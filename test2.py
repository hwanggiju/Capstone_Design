from tkinter import CENTER
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import os
import time
import RPi.GPIO as GPIO
import numpy as np
import smbus
from imusensor.MPU9250 import MPU9250
import math


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

WIDTH = 128
HEIGHT = 64 
BORDER = 5

switch = [16, 20, 21]
wave = [24, 23]
driver = [19, 27, 22, 5, 6, 13]

# Use for SPI
spi = board.SPI()
oled_reset = digitalio.DigitalInOut(board.D25)
oled_cs = digitalio.DigitalInOut(board.D8)
oled_dc = digitalio.DigitalInOut(board.D17)
oled = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, oled_dc, oled_reset, oled_cs)

GPIO.setup(switch[0], GPIO.IN)
GPIO.setup(switch[1], GPIO.IN)
GPIO.setup(switch[2], GPIO.IN)

for i in range(len(driver)): #모터 드라이버 핀
    GPIO.setup(driver[i], GPIO.OUT)

# 초음파 핀 setup
GPIO.setup(wave[0], GPIO.OUT)
GPIO.setup(wave[1], GPIO.IN)
GPIO.output(wave[0], False)

enA_pwm = GPIO.PWM(driver[0], 50) # channel, frequency 50Hz
enB_pwm = GPIO.PWM(driver[5], 50) # channel, frequency 50Hz
enA_pwm.start(0)    # enableA pin start dutycycle 0%
enB_pwm.start(0)    # enableB pin start dutycycle 0%

# 초기 디스플레이 설정
oled.fill(0)
oled.show()

font = ImageFont.truetype('malgun.ttf', 15)
font1 = ImageFont.truetype('malgun.ttf', 20)
font2 = ImageFont.truetype('malgun.ttf', 10)

image = Image.new('1', (oled.width, oled.height), 255)
logoImage = Image.open('logo.bmp')
draw = ImageDraw.Draw(image)

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
    distance = int(distance)
    
    return distance

actionPre = 1
def changePWM(enA, enB):
    if enA < 0 or enA > 100: #range over check
        return False
    elif enB < 0 or enB > 100:
        return False
    enA_pwm.ChangeDutyCycle(enA)
    enB_pwm.ChangeDutyCycle(enB)
    return True

def HorizontalHold(nowAngle, compareAngle):
    pwmA = 80
    pwmB = 80
    diffPwm = int(20 * np.sin((64 * (nowAngle-compareAngle)) * np.pi/180))
    if actionPre == 2 :
        if (nowAngle > compareAngle) : 
            pwmA += diffPwm
            pwmB -= diffPwm
        elif nowAngle < compareAngle:
            pwmA -= diffPwm
            pwmB += diffPwm
        changePWM(pwmA, pwmB)
        
    elif actionPre == 0 :
        if (nowAngle > compareAngle) : 
            pwmA += diffPwm
            pwmB -= diffPwm
        elif (nowAngle < compareAngle):
            pwmA -= diffPwm
            pwmB += diffPwm
        changePWM(pwmA, pwmB)

def btn_driverSet(enA, motorA, motorB, enB):
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
    
def write_byte(adr, data):
    I2C_bus.write_byte_data(MPU_addr, adr, data)

def read_byte(adr):
    return I2C_bus.read_byte_data(MPU_addr, adr)
    
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
    
def read_word_2c(adr):
    val = read_word(adr)
    if (val >= 0x8000):
        return -((65535 - val) + 1)
    else:
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
    alpha = 0.86
    GyX_deg = alpha * gyroAngleX + (1.0 - alpha) * accelAngleX
    GyY_deg = alpha * gyroAngleY + (1.0 - alpha) * accelAngleY
    GyZ_deg = gyroAngleZ

    past1 = now
    return GyX_deg, GyY_deg, GyZ_deg    

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

def OLED_initial_setting_Height(CHANGE_HEIGHT) :
    draw.text((5, 0), 'First Setting', font = font, fill = 0)
    draw.text((5, 20), 'Input your Height', font = font, fill = 0)
    draw.text((5, 40), str(CHANGE_HEIGHT), font = font, fill = 0)
    oled.image(image)
    oled.show()

def OLED_initial_setting_Height1(CHANGE_HEIGHT) :
    draw.text((5, 0), 'First Setting', font = font, fill = 255)
    draw.text((5, 20), 'Input your Height', font = font, fill = 255)
    draw.text((5, 40), str(CHANGE_HEIGHT), font = font, fill = 255)
    oled.image(image)
    oled.show()
    
deskDistance = 0
preTime = 0
timeTest = True
def drawDisplay() :
    if timeTest == True :
        preTime = nowTime
        timeTest = False
    if nowTime - preTime > 1 :
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
        
try :
    SET_HEIGHT = 170
    oled.image(logoImage)
    oled.show()
    time.sleep(2)
    oled.fill(0)
    oled.show()
    OLED_initial_setting_Height(SET_HEIGHT)
    
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
        
    AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
    angleX, angleY, angleZ = calGyro(AcX, AcY, AcZ ,GyX , GyY, GyZ)
    fixAngle = angleY
    while True :
        AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
        angleX, angleY, angleZ = calGyro(AcX, AcY, AcZ ,GyX , GyY, GyZ)
        nowTime = time.time()
        drawDisplay()
        if GPIO.input(switch[2]) == 1 :     # up
            AcX, AcY, AcZ, GyX, GyY, GyZ = get_raw_data()
            angleX, angleY, angleZ = calGyro(AcX, AcY, AcZ ,GyX , GyY, GyZ)
            HorizontalHold(angleY, fixAngle)
            btn_driverSet(0, 2, 2, 0)
        else :
            btn_driverSet(0, 0, 0, 0)
            pass 
        eraseDisplay()
            
     
except KeyboardInterrupt :
    print('end')
    enA_pwm.stop()
    enB_pwm.stop()
    GPIO.cleanup()
    pass