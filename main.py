# 필요한 라이브러리 선언
import face_recognition
import cv2
import time
from scipy.spatial import distance as dist
import sys
import numpy as np
#import RPI.GPIO as gpio

# 졸음 인식 판단 카운트 값
EYES_CLOSED_SECONDS = 5

# 카페 학습된 이미지 모델 선언
model = 'res10_300x300_ssd_iter_140000.caffemodel'
config = 'deploy.prototxt.txt'

# 네트워크 구성
net = cv2.dnn.readNet(model, config)

if net.empty() :
    print('Net open failed!')
    sys.exit()

def main():
    closed_count = 0    # 카운트 비교 변수 선언
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
    
    process = True    
    
    # 사용자 인식 구현 부분 --------------------------------------------------------
    while True:
        _, frame = cap.read(0)
        if frame is None :
            break
        
        blob = cv2.dnn.blobFromImage(frame,             #image
                                     1,                 #scalefactor
                                     (10, 10),        #image Size
                                     (104, 177, 123)    #Scalar
                                     )
        net.setInput(blob)
        detect = net.forward()
        
        (h, w) = frame.shape[:2]
        detect = detect[0, 0, :, :]
        
        for i in range(detect.shape[0]) : 
            confidence = detect[i, 2]
            if confidence < 0.5 :
                break
            
            x1 = int(detect[i, 3] * w)
            y1 = int(detect[i, 4] * h)
            x2 = int(detect[i, 5] * w)
            y2 = int(detect[i, 6] * h)
            
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0)) #green ractangle
            
            # 책상 다리 모터 제어에 활용되는 값
            area = (x2-x1) * (y2-y1) # 사용자 인식 넓이
            center_x = x1 + (x2-x1)/2 
            center_y = y1 + (y2-y1)/2 # 인식된 부분 중심 좌표 x, y 값
            
            print('area : %d    center_x : %d   center_y : %d' 
                %(area, center_x, center_y))
        # -----------------------------------------------------------------------------
            '''
            ret, frame = cap.read(0)
            small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)
            rgb_small_frame = small_frame[:, :, ::-1]
            
            if process :
                face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
            
            # 사용자 눈 인식 구현 부분 ----------------------------------------
                for face_landmark in face_landmarks_list:
                    left_eye = face_landmark['left_eye']
                    right_eye = face_landmark['right_eye']

                    ear_left = get_ear(left_eye)
                    ear_right = get_ear(right_eye)
                    
                    print('left_eye : %lf   left_eye : %lf' 
                        %(ear_left, ear_right))

                    # 졸음 방지 알림에 활용될 코드
                    closed = 0
                    closed = ear_left < 0.2 and ear_right < 0.2
                    if (closed):
                        closed_count += 1

                    else:
                        closed_count = 0

                    if (closed_count >= EYES_CLOSED_SECONDS):
                        asleep = True
                        while (asleep):
                            print("EYES CLOSED")

                            if cv2.waitKey(1) == 32: 
                                asleep = False
                                print("EYES OPENED")
                        closed_count = 0
            # -------------------------------------------------------------'''
        cv2.imshow('Facerec_Video', frame)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            break

# 특징점 좌표값을 받은 후 거리값 도출 함수
def get_ear(eye):
	A = dist.euclidean(eye[1], eye[5])
	B = dist.euclidean(eye[2], eye[4])
	C = dist.euclidean(eye[0], eye[3])
	ear = (A + B) / (2.0 * C)
	return ear

if __name__ == "__main__":
    main()
    cv2.destroyAllWindows()

