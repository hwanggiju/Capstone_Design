from asyncio.unix_events import _UnixSelectorEventLoop
import face_recognition
import cv2
import RPi.GPIO as GPIO
import time
import sys
import numpy as np

# spi SSD1306 OLED code
# pip install spidev 설치
# import spidev

# MPU9250 gyro sensor code
# pip3 install imusensor 설치
# import os
# import smbus
# from imusensor.MPU9250 import MPU9250

# sensorAddress = 0x68
# bus = smbus.SMBus(1)
# imu = MPU9250.MPU9250(bus, sensorAddress)
# imu.begin()

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



#카메라
cameraWidth = 480
cameraHeight = 640
cameraAngle = 56 # 카메라 수평 화각
cameraWaveDifference = 46 #카메라 - 센서간 높이차이

# 각도
deskAngle = -3 #28 # 책상 판과 카메라 중심까지의 각도
deskUserAngle = 0 # 책상 판과 사용자 높이 사이의 각도
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

timeNum = 30 #평균횟수 클수록 둔화됨, 하지만 반응이 느려짐
faceWidthAverage = [((faceWidthMax + faceWidthMin)/2) for col in range(timeNum)]
def getUserHeight_nani(faceWidth, pixelX, pixelY, nowHeight):
    faceWidthAverage[0] = faceWidth
    widthAverage = sum(faceWidthAverage) / timeNum
    fullHorizontalAngle = cameraAngle
    fullVerticalAngle = fullHorizontalAngle * cameraHeight / cameraWidth
    faceDifference = faceWidthMax - faceWidthMin
    distanceDifference = userDistanceMax - userDistanceMin
    calUserDistance = (faceWidthMax - widthAverage) / faceDifference * distanceDifference + userDistanceMin
    userTopAngle = abs(pixelX - cameraWidth/2) / cameraWidth * fullHorizontalAngle
    userDistance = calUserDistance / np.cos(userTopAngle * np.pi/180)
    cameraUserAngle = (cameraHeight/2 - pixelY) / cameraHeight * fullVerticalAngle
    calHeight = np.tan((cameraUserAngle + deskAngle) * np.pi/180) * userDistance + np.tan(cameraUserAngle * np.pi/180)*10
    for i in range(timeNum-1):#shift array
        faceWidthAverage[timeNum-1-i] = faceWidthAverage[timeNum-2-i]
    return nowHeight + calHeight

# driver
# 0 : stop
# 1 : down
# 2 : up
def driverSet(enA, motorA, motorB, enB):
    # enA_pwm.start(0)
    # enB_pwm.start(0)
    for i in range(len(driver)):
        GPIO.output(driver[i], 0)
    time.sleep(0.2)
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

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, cameraHeight)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, cameraWidth)
    # _, frame = cap.read(0)
    # print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),
    #       cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # rotate_frame = cv2.rotate(frame, cv2.ROTATE_90_CLOCKWISE)

    if not cap.isOpened() :
        print('Camera open failed!')
        sys.exit()

    model = 'res10_300x300_ssd_iter_140000.caffemodel'
    config = 'deploy.prototxt.txt'

    net = cv2.dnn.readNet(model, config)
    
    if net.empty() :
        print('Net open failed!')
        sys.exit()

    #small_frame = cv2.resize(rotate_frame, (0, 0), fx=0.25, fy=0.25)
    #rgb_small_frame = small_frame[:, :, ::-1]
    #face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)

    # 픽셀 최대 최소값 초기화
    # maxHeightPixel = 0
    # minHeightPixel = 1000


    actionNow = 0  # 0:down 1:stop 2:up
    actionPre = 0
    driverSet(1, 1, 1, 1)  # down
    time.sleep(5)

    while True:
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
            Height = getUserHeight_nani(width,center_x,center_y-height/2, waveSensorHeight+cameraWaveDifference)
            print("테스트 nani 식 :" + str(Height) + "\n")
            '''
            #높이에 따른 모터작동
            if Height < 120:
                actionNow = 0#down
                print("down\n")
            elif Height > 130:
                actionNow = 2#up
                print("up\n")
            else:
                actionNow = 1#stop
                print("stop")

            if actionNow != actionPre :
                if actionNow == 0:
                    driverSet(1,1,1,1)# down
                elif actionNow == 1:
                    driverSet(0,0,0,0) # stay
                elif actionNow == 2:
                    driverSet(1,2,2,1)# up
                actionPre = actionNow
            '''
        print("초음파 측정 거리 : %d" % (waveSensorHeight))
        
        cv2.imshow('Facerec_Video', rotate_frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            # enA_pwm.stop()
            # enB_pwm.stop()
            GPIO.cleanup()
            break

if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()