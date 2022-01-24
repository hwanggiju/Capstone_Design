import cv2

cap = cv2.VideoCapture(0)

if cap.isOpened() :
    print("Camera Is Opened")
    delay = int(1000 / cap.get(cv2.CAP_PROP_FPS))
    while True :
        ret, frame = cap.read()
        frame = cv2.resize(frame, (0, 0), fx = 0.25, fy = 0.25)
        if ret :
            frame = cv2.flip(frame, 1)
            cv2.imshow("Video", frame)
            if cv2.waitKey(delay) & 0xFF == 27:
                break
            
        else :
            print(ret, frame)
            break
    
else :
    print("Camera Isn't Opened")
    
cap.release()
cv2.destroyAllWindows()         