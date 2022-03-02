import cv2
import sys
import numpy as np
import random

cap = cv2.VideoCapture(0)

if cap.isOpened() :
    print("Camera Is Opened")
    
    hog = cv2.HOGDescriptor()
    hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
    
    delay = int(1000 / cap.get(cv2.CAP_PROP_FPS))
    
    while True :
        reg, frame = cap.read()
        if reg :
            detected, _ = hog.detectMultiScale(frame)
            
            for (x, y , w, h) in detected :
                c = (random.randint(0, 255), random.randint(0, 255), random.randint(0, 255))
                cv2.rectangle(frame, (x, y), (x+w, y+h), c, 3)

            cv2.imshow("Video", frame)
            
            if cv2.waitKey(delay) & 0xFF == 27 :
                print("ESC Key Pressed")
                break
        else :
            print(reg, frame)
            break
else :
    print("Camera Is Not Opened!!!")
    
cap.release()
cv2.destroyAllWindows()
