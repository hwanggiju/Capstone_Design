import face_recognition
import cv2
import RPi.GPIO as GPIO
import time
from scipy.spatial import distance as dist
import sys
import numpy as np


# Motor Driver [enA/in1/in2/in3/in4/enB]
driver = [11, 13, 15, 29, 31, 33]
# I2C [SDA/SCL]
iic_arr = [3, 5]
# UART [TXD/RXD]
uart_arr = [8, 10]
# SPI [MOSI/MISO/SCK/CE0/CE1]
spi_arr = [19, 21, 23, 24, 26]
# switch[left/center/right]
switch = [36, 38, 40]

# 사용자 정의 변수
maxHeight = 170
minHeight = 80
seatdownHeight = 0

GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)

# 각도
cameraAngle = 50 # 카메라 각도
deskAngle = 30 # 책상 판과 카메라 중심까지의 각도
deskUserAngle = 0 # 책상 판과 사용자 높이 사이의 각도
cameraUserAngle = 0 # 카메라 앵글 안의 사용자 높이 각도

def initHardware():
    #input/output setting
    for i in range(len(driver)):
        GPIO.setup(driver[i], GPIO.OUT)
    for i in range(len(switch)):
        GPIO.setup(switch[i], GPIO.IN)
    #initial system down
    for i in range(len(driver)):
        GPIO.output(driver[i], GPIO.LOW)

# driver
def driverSet(enA, motorA, motorB, enB):
    for i in range(len(driver)):
        GPIO.output(driver[i], 0)
    time.sleep(0.2)
    if motorA == 1:
        GPIO.output(driver[0], enA)
        GPIO.output(driver[1], 0)
        GPIO.output(driver[2], 1)
    else:
        GPIO.output(driver[0], enA)
        GPIO.output(driver[1], 1)
        GPIO.output(driver[2], 0)
    if motorB == 1:
        GPIO.output(driver[3], 0)
        GPIO.output(driver[4], 1)
        GPIO.output(driver[5], enB)
    else:
        GPIO.output(driver[3], 1)
        GPIO.output(driver[4], 0)
        GPIO.output(driver[5], enB)

# main code
def main():
    cap = cv2.VideoCapture(0)
    _, frame = cap.read(0)
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),
          cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    if not cap.isOpened() :
        print('Camera open failed!')
        sys.exit()

    model = 'res10_300x300_ssd_iter_140000.caffemodel'
    config = 'deploy.prototxt.txt'

    net = cv2.dnn.readNet(model, config)
    
    if net.empty() :
        print('Net open failed!')
        sys.exit()

    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = small_frame[:, :, ::-1]
    face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)

    # 픽셀 최대 최소값 초기화
    maxHeightPixel = 0
    minHeightPixel = 1000

    # driverSet(1, 0, 0, 1)
    # time.sleep(5)
    # driverSet(0, 0, 0, 0)

    while True:
        _, frame = cap.read(0)
        if frame is None:
            break
        blob = cv2.dnn.blobFromImage(frame,  # image
                                     1,  # scalefactor
                                     (150, 150),  # image Size
                                     (104, 177, 123)  # Scalar
                                     )
        net.setInput(blob)
        detect = net.forward()
        (h, w) = frame.shape[:2]
        detect = detect[0, 0, :, :]

        for i in range(detect.shape[0]):
            confidence = detect[i, 2]
            if confidence < 0.5:
                break

            x1 = int(detect[i, 3] * w)
            y1 = int(detect[i, 4] * h)
            x2 = int(detect[i, 5] * w)
            y2 = int(detect[i, 6] * h)

            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))  # green ractangle

            # 책상 다리 모터 제어에 활용되는 값
            area = (x2 - x1) * (y2 - y1)  # 사용자 인식 넓이
            center_x = x1 + (x2 - x1) / 2
            center_y = y1 + (y2 - y1) / 2  # 인식된 부분 중심 좌표 x, y 값
            width = x2 - x1
            height = y2 - y1
            print(" 가로 :" + str(width) + "  세로:" + str(height), end='')
            print('  area : %d    center_x : %d   center_y : %d'
                  % (area, center_x, center_y))
            
            '''
            if width > 70 and width < 90:  # 카메라 사용자 거리 : 70 ~ 100(cm)
                # 움직임으로 최대 최소점 고정
                if maxHeightPixel < center_y:
                    maxHeightPixel = center_y
                if minHeightPixel > center_y:
                    minHeightPixel = center_y
                if (maxHeightPixel-minHeightPixel) > 100:
                    currentHeight = (center_y - maxHeightPixel) / (minHeightPixel - maxHeightPixel) * (
                                maxHeight - minHeight) + minHeight
                    print(currentHeight)
                    #높이에 따른 모터작동
                    if currentHeight < 140:
                        driverSet(1,0,0,1)# down
                        print("down\n")
                    elif currentHeight > 160:
                        driverSet(1,1,1,1)#up
                        print("up\n")
                    else :
                        driverSet(0, 0, 0, 0) # stay
            '''
        
            cameraUserAngle = ((480 - center_y)*cameraAngle) / 480
            deskUserAngle = deskAngle - (cameraAngle / 2) + cameraUserAngle
            print("cameraUserAngle = %d\tdeskUserAngle = %d"%(cameraUserAngle, deskUserAngle), end='')
            seatdownHeight = np.sin(deskUserAngle * np.pi/180) # np.pi/180 degree -> radian 변환
            print("앉은 키 : %f"%(seatdownHeight))
        
        cv2.imshow('Facerec_Video', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            GPIO.cleanup()
            break

if __name__ == "__main__":
    initHardware()
    main()
    cv2.destroyAllWindows()