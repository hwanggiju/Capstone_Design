import cv2
from cv2 import CAP_V4L2

cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

if cap.isOpened() :
    try:
        print("Camera Is Opened")
        while True :
            ret, frame = cap.read()
            print(ret, frame)
            img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_gray = cv2.flip(img_gray, 1)
            cv2.imshow("Video", img_gray)
            if cv2.waitKey(delay) & 0xFF == 27:
                print("ESC Key pressed")
                break
    except:
        pass
    
else :
    print("Camera Isn't Opened")
    
cap.release()
cv2.destroyAllWindows()
#