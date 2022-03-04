import sys
import numpy as np
import cv2

model = 'res10_300x300_ssd_iter_140000.caffemodel'
config = 'deploy.prototxt.txt'

cap = cv2.VideoCapture(0)

if not cap.isOpened() :
    print('Camera open failed!')
    sys.exit()
    
net = cv2.dnn.readNet(model, config)

if net.empty() :
    print('Net open failed!')
    sys.exit()
    
while True :
    _, frame = cap.read()
    if frame is None :
        break
    
    blob = cv2.dnn.blobFromImage(frame, 1, (300, 300), (104, 177, 123))
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
        
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0))
        
    area = (x2-x1) * (y2-y1)
    center_x = (x2-x1)/2
    center_y = (y2-y1)/2
    print('area : %d    center_x : %d   center_y : %d' %(area, center_x, center_y))
    cv2.imshow('frame', frame)
    
    if cv2.waitKey(1) == 27:
        break
    
cv2.destroyAllWindows()