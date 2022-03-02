import cv2

cap = cv2.VideoCapture(0)

if cap.isOpened() :
    fourcc = cv2.VideoWriter_fourcc(*'DIVX')
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    size = (int(width), int(height))
    fps = cap.get(cv2.CAP_PROP_FPS)
    out = cv2.VideoWriter("video.avi", fourcc, fps, size)
    
    delay = int(1000 / cap.get(cv2.CAP_PROP_FPS))
    while True:
        ret, img = cap.read()
        if ret :
            img_gray = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
            cv2.imshow('img', img_gray)
            out.write(cv2.cvtColor(img_gray, cv2.COLOR_GRAY2BGR))
            if cv2.waitKey(delay) & 0xFF == 27 :
                print("ESC KEY pressed")
                break
            
        else :
            print(ret, img)
            break
        
else :
    print("Camera not Opened")

out.release()
cap.release()
cv2.destroyAllWindows()