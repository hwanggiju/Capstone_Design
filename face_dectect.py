# 라이브러리 불러오기
import sys
import numpy as np
import cv2

# 카페 얼굴인식 모델
model = 'res10_300x300_ssd_iter_140000.caffemodel'
config = 'deploy.prototxt.txt'

cap = cv2.VideoCapture(0)
print(cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

if not cap.isOpened() :
    print('Camera open failed!')
    sys.exit()

# opencv 딥러닝 네트워크로 카페 모델 학습
net = cv2.dnn.readNet(model, config)

if net.empty() :
    print('Net open failed!')
    sys.exit()


while True :
    _, frame = cap.read()
    if frame is None :
        break
    
    # 입력 영상 블롭 객체 생성
    blob = cv2.dnn.blobFromImage(frame, 1, (300, 300), (104, 177, 123)) # cv2.dnn.blobFromImage(입력, *, size, 각 채널에 뺄 평균값)
    net.setInput(blob) # 네트워크 입력 설정
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
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))
        area = (x2-x1) * (y2-y1)
        center_x = x1 + (x2-x1)/2 
        center_y = y1 + (y2-y1)/2
        
        print('area : %d    center_x : %d   center_y : %d' %(area, center_x, center_y))
        
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) == 27:
        break
    
cv2.destroyAllWindows()