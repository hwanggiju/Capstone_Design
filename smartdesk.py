import face_recognition
import cv2
import RPi.GPIO as GPIO
import time
from scipy.spatial import distance as dist
import sys
import numpy as np
import RPi.GPIO as gpio

# Motor Driver [enA/in1/in2/in3/in4/enB]
driver = [ 11, 13, 15, 29, 31, 33]

def initDriver():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setwarnings(False)
    #input/output setting
    for i in range(len(driver)):
        GPIO.setup(driver[i], GPIO.OUT)
    #initial system down
    for i in range(len(driver)):
        GPIO.output(driver[i], GPIO.LOW)

# driver
def driverSet(enA, In1, In2, In3, In4, enB):
    for i in range(len(driver)):
        GPIO.output(driver[i], 0)
    time.sleep(0.1)
    GPIO.setup(driver[0], enA)
    GPIO.setup(driver[1], In1)
    GPIO.setup(driver[2], In2)
    GPIO.setup(driver[3], In3)
    GPIO.setup(driver[4], In4)
    GPIO.setup(driver[5], enB)

# main code
def main():
    cap = cv2.VideoCapture(0)
    _, frame = cap.read(0)
    print(cap.get(cv2.CAP_PROP_FRAME_WIDTH),
          cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    if not cap.isOpened() :
        print('Camera open failed!')
        sys.exit()

    small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
    rgb_small_frame = small_frame[:, :, ::-1]
    face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)

    # 픽셀 최대 최소값 초기화
    maxHeightPixel = 0

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
            
        cv2.imshow('Facerec_Video', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

if __name__ == "__main__":
    initDriver()
    main()
    cv2.destroyAllWindows()