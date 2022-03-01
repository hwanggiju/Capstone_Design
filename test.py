"""
# 비디오 영상 코드
import cv2

cap = cv2.VideoCapture(0)

if cap.isOpened() :
    print("Camera Is Opened")
    delay = int(1000 / cap.get(cv2.CAP_PROP_FPS))
    while True :
        ret, frame = cap.read()
        if ret :
            img_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            img_gray = cv2.flip(img_gray, 1)
            cv2.imshow("Video", img_gray)
            if cv2.waitKey(delay) & 0xFF == 27:
                print("ESC Key pressed")
                break
            
        else :
            print(ret, frame)
            break
    
else :
    print("Camera Isn't Opened")
    
cap.release()
cv2.destroyAllWindows()
"""     

"""
# 동영상 속성
import cv2

cap = cv2.VideoCapture(0)
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FRAME_WIDTH))
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 320)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 240)
print(cap.get(cv2.CAP_PROP_FRAME_HEIGHT), cap.get(cv2.CAP_PROP_FRAME_WIDTH))

if cap.isOpened() :
    delay = int(1000 / cap.get(cv2.CAP_PROP_FPS))
    
    while True :
        ret, img = cap.read()
        if ret :
            cv2.imshow("Camera", img)
            if cv2.waitKey(delay) & 0xFF == 27 :
                print("ESC KEY pressed")
                break
            
        else :
            print(ret, img)
            break
        
else :
    print("Camera not Opened")
    
cap.release()
cv2.destroyAllWindows()
"""

'''
# 동영상 프레임 저장
import cv2
import numpy as np
from datetime import datetime

def mouseHandler(event, x, y, flags, param) :
    if event == cv2.EVENT_LBUTTONDOWN :
        print(event, x, y)
        print(datetime.today())
        filename = str(datetime.today().microsecond) + ".jpg"
        cv2.imwrite(filename, img)
        
cv2.namedWindow("Camera")
cv2.setMouseCallback("Camera", mouseHandler)

cap = cv2.VideoCapture(0)

img = None
if cap.isOpened() :
    delay = int(1000 / cap.get(cv2.CAP_PROP_FPS))
    while True :
        ret, img = cap.read()
        if ret :
            img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            cv2.imshow('Camera', img_gray)
            
            if cv2.waitKey(delay) & 0xFF == 27 :
                print("ESC KEY pressed")
                break
        else :
            print(ret, img)
            break
else :
    print("Camera not Opened")
    
cap.release()
cv2.destroyAllWindows()
'''

'''
# 비디오 저장하기
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
'''     

'''
# 마우스 이벤트
import cv2
import numpy as np

def draw_circle(event, x, y, flags, param) :
    if event == cv2.EVENT_LBUTTONDOWN :
        if flags & cv2.EVENT_FLAG_CTRLKEY :
            cv2.rectangle(img, (x, y), (x+20, y+20), (0, 0, 255), -1)
        else :
            cv2.rectangle(img, (x, y), (x+20, y+20), (0, 255, 0), 2)
            
    elif event == cv2.EVENT_RBUTTONDOWN :
        cv2.circle(img, (x, y), 10, (255, 0, 0), 2)
    elif event == cv2.EVENT_LBUTTONDBLCLK :
        cv2.circle(img, (x, y), 20, (255, 0, 0), -1)
        
img = np.full((512, 512, 3), 255, dtype = np.uint8)
cv2.namedWindow('image')
cv2.setMouseCallback('image', draw_circle)

while True :
    cv2.imshow('image', img)
    if cv2.waitKey(20) & 0xFF == 27 :
        break
        
cv2.destroyAllWindows()
'''

'''
# 키보드 이벤트
import cv2
import numpy as np

img = np.full((100, 100, 3), 255, np.uint8)
while True :
    cv2.imshow('img', img)
    key = cv2.waitKey()
    print(f'Code : {key}, Char : {chr(key)}')
    if key & 0xFF == 27 :
        break
    
cv2.destroyAllWindows()
'''

'''
# 트랙 바
import cv2
import numpy as np

img = np.zeros((200, 500, 3), np.uint8)
cv2.namedWindow('image')

def set_background(x):
    global img
    r = cv2.getTrackbarPos('Red', 'image')
    g = cv2.getTrackbarPos('Green', 'image')
    b = cv2.getTrackbarPos('Blue', 'image')
    s = cv2.getTrackbarPos('0 or 1', 'image')
    if s == 0:
        img[:] = 0
    else :
        img[:] = [b, g, r]
        
cv2.createTrackbar('Red', 'image', 0, 255, set_background)
cv2.createTrackbar('Green', 'image', 0, 255, set_background)
cv2.createTrackbar('Blue', 'image', 0, 255, set_background)
cv2.createTrackbar('0 or 1', 'image', 0, 1, set_background)

while True :
    cv2.imshow('image', img)
    if cv2.waitKey(1) & 0xFF == 27 :
        break
cv2.destroyAllWindows()
'''

'''
# ROI(Region Of Interest)
import cv2

img = cv2.imread('lena.jpg')
temp = img[200:380, 200:360]
cv2.imshow('img', img)
cv2.imshow('img_ROI', temp)
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# 이진화
import cv2
import numpy as np

def two_tone(img, threshold=128) :
    output = (img > threshold) * 255
    return output.astype(np.uint8)

img = cv2.imread('lena.jpg', 0)
new_img = two_tone(img, threshold=120)

cv2.imshow('img', new_img)
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# 디더링
import cv2
import numpy as np

def minmax(pixel) :
    if pixel > 255 :
        pixel = 255
    if pixel < 0 :
        pixel = 0
    return pixel

def dithering(img) :
    height, width = img.shape
    for y in range(0, height-1) :
        for x in range(0, width-1) :
            p = img[y, x]
            new_p = np.round(p/255) * 255
            img[y, x] = new_p
            error = p - new_p
            img[y, x+1] = minmax(img[y, x+1] + error * 7/16)
            img[y+1, x-1] = minmax(img[y+1, x-1] + error * 3/16)
            img[y+1, x] = minmax(img[y+1, x] + error * 5/16)
            img[y+1, x+1] = minmax(img[y+1, x+1] + error * 1/16)
    return img

lena = cv2.imread('lena.jpg', cv2.IMREAD_GRAYSCALE)
lena_dithering = dithering(lena.copy())
cv2.imshow('Lena grayscale', lena)
cv2.imshow('Lena dithering', lena_dithering)
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# 히스토그램
import cv2
import matplotlib.pyplot as plt
import numpy as np

def histogram(img) :
    height, width = img.shape
    hist_ = np.zeros(256)
    for y in range(height) :
        for x in range(width) :
            hist_[img[y, x]] = hist_[img[y, x]] + 1
    
    return hist_

lena_gray = cv2.imread('lena.jpg', 0)

plt.figure(figsize=(12, 4))
plt.subplot(121)
plt.bar(x=range(256), height=histogram(lena_gray), width=1)

plt.subplot(122)
hist = cv2.calcHist(images=[lena_gray],channels=[0], mask=None, histSize=[256], ranges=[0, 256])
plt.plot(hist.flatten())
plt.show()
'''

'''
# 입력 영상 기준 크기변환
import cv2
import numpy as np

def scale_nogood(img, scale_x=1, scale_y=1) :
    height, width = img.shape
    height_n, width_n = int(height * scale_y), int(width * scale_x)
    img_ = np.zeros((height_n, width_n), dtype=np.uint8)
    
    for y in range(height):
        for x in range(width) :
            img_[int(y*scale_y), int(x*scale_x)] = img[y, x]
    
    return img_

img = cv2.imread("lena.jpg", cv2.IMREAD_GRAYSCALE)
result = scale_nogood(img, 0.5, 0.5)

cv2.imshow('Origin', img)
cv2.imshow('Scale', result)
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# 출력 영상 기준 크기변환
import cv2
import numpy as np

def scale_nogood2(img, scale_x=1, scale_y=1) :
    height, width = img.shape
    height_n, width_n = int(height/scale_y), int(width/scale_x)
    img_ = np.zeros((height_n, width_n), dtype=np.uint8)
    
    for y in range(height_n) :
        for x in range(width_n) :
            img_[y, x] = img[int(y/scale_y), int(x/scale_x)]
            
    return img_

img = cv2.imread('lena.jpg', cv2.IMREAD_GRAYSCALE)
result2 = scale_nogood2(img, 1.5, 1.0)

cv2.imshow('Origin', img)
cv2.imshow('Scale', result2)
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# 보간법 : 최근방법
import numpy as np
import cv2

def scale_nearset(img, scale_x=1, scale_y=1) :
    height, width = img.shape
    img_= np.zeros((int(height*scale_y), int(width*scale_x)), dtype=np.uint8)
    for y in range(int(height*scale_y)) :
        for x in range(int(width*scale_x)) :
            try :
                img_[y, x] = img[round(y/scale_y), round(x/scale_x)]
            except :
                pass
    return img_

lena_roi = cv2.imread('lena.jpg', cv2.IMREAD_GRAYSCALE)
cv2.imshow('image', lena_roi)
cv2.imshow('Scale_nearset', scale_nearset(lena_roi, 3, 3))
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# 보간법 : 선형방법
import numpy as np
import cv2

def scale_bilinear(img, scale_x=1, scale_y=1) :
    height, width = img.shape
    img_ = np.zeros((int(height*scale_y), int(width*scale_x)), dtype=np.uint8)
    for y in range(int(height*scale_y)):
        for x in range(int(width*scale_x)) :
            q = x/scale_x - int(x/scale_x)
            p = y/scale_y - int(y/scale_y)
            try : 
                X = int(x/scale_x)
                Y = int(y/scale_y)
                value = (1-p)*(1-q)*img[Y,X] + p*(1-q)*img[Y+1,X] + (1-p)*q*img[Y,X+1] + p*q*img[Y+1,X+1]
                if value > 255 :
                    img_[y, x] = 255
                else :
                    img_[y, x] = int(value)
            except :
                pass
    return img_

img = cv2.imread('lena.jpg', cv2.IMREAD_GRAYSCALE)

cv2.imshow('image', img)
cv2.imshow('Nearest', scale_bilinear(img, 3, 3))
cv2.waitKey()
cv2.destroyAllWindows()
'''

'''
# numpy 푸리에 변환
import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('lena.jpg', cv2.IMREAD_GRAYSCALE)

f = np.fft.fft2(img)
fshift = np.fft.fftshift(f)

magnitude_spectrum = 20*np.log(abs(fshift))

rows, cols = img.shape
crows, ccols = int(rows/2), int(cols/2)

d = 10

fshift[crows-d:crows+d, ccols-d:ccols+d] = 1

f_ishift = np.fft.ifftshift(fshift)
img_back = np.fft.ifft2(f_ishift)
img_back = np.abs(img_back)

img_new = np.uint8(img_back)
_, thresh = cv2.threshold(img_new, 30, 255, cv2.THRESH_BINARY_INV)

plt.subplot(221), plt.imshow(img, cmap='gray')
plt.title('Input Image')
plt.xticks([]), plt.yticks([])

plt.subplot(222), plt.imshow(magnitude_spectrum, cmap='gray')
plt.title('Spectrum')
plt.xticks([]), plt.yticks([])

plt.subplot(223), plt.imshow(img_back, cmap='gray')
plt.title('FT')
plt.xticks([]), plt.yticks([])

plt.subplot(224), plt.imshow(thresh, cmap='gray')
plt.title('Threshold With FT')
plt.xticks([]), plt.yticks([])
plt.show()
'''

'''
# OPENCV 푸리에 변환
import cv2
import numpy as np
import matplotlib.pyplot as plt

img = cv2.imread('lena.jpg', cv2.IMREAD_GRAYSCALE)
dft = cv2.dft(np.float32(img), flags=cv2.DFT_COMPLEX_OUTPUT)

dft_shift = np.fft.fftshift(dft)

rows, cols = img.shape
crow, ccol = int(rows/2), int(cols/2)

d=20
mask = np.ones((rows, cols, 2), np.uint8) # 저주파 영역 제거 # mask = np.zeros((rows, cols, 2), np.uint8) # 고주파 영역 제거 
mask[crow-d:crow+d, ccol-d:ccol+d] = 0 # 저주파 영역 제거# mask[crow-d:crow+d, ccol-d:ccol+d] = 1 # 고주파 영역 제거

fshift = dft_shift*mask
f_ishift = np.fft.ifftshift(fshift)
img_back = cv2.idft(f_ishift)
img_back = cv2.magnitude(img_back[:,:,0], img_back[:,:,1])
plt.subplot(121)
plt.imshow(img, cmap='gray')
plt.title('Input image')
plt.xticks([]), plt.yticks([])
plt.subplot(122)
plt.imshow(img_back, cmap='gray')
plt.xticks([]), plt.yticks([])
plt.show()
'''

'''
# 문서 스캔
import cv2
import numpy as np
from datetime import datetime
from time import sleep

points = np.zeros((4, 2), dtype=np.float32)
count = 0

def mouseHandler(event, x, y, flags, param) :
    global count
    if event == cv2.EVENT_LBUTTONDOWN :
        cv2.circle(img, (x,y), 5, (0, 0, 255), -1)
        cv2.imshow("Capture", img)
        try :
            points[count] = [x, y]
            count += 1
            if count == 4 :
                sum_ = points.sum(axis=1)
                diff = np.diff(points, axis=1)
                
                top_left = points[np.argmin(sum_)]
                bottom_right = points[np.argmin(sum_)]
                top_right = points[np.argmin(diff)]
                bottom_left = points[np.argmin(diff)]
                
                pts1 = np.float32([top_left, top_right, bottom_right, bottom_left])
                width_bottom = abs(bottom_right[0]-bottom_left[0])
                width_top = abs(top_right[0]-top_left[0])
                height_right = abs(top_right[1]-bottom_right[1])
                height_left = abs(top_left[1]-bottom_left[1])
                
                width = int(max([width_bottom, width_top]))
                height = int(max([height_right, height_left]))
                
                pts2 = np.float32([[0, 0], [width-1,0], [width-1, height-1], [0, height-1]])
                
                M = cv2.getPerspectiveTransform(pts1, pts2)
                
                dst = cv2.warpPerspective(img, M, (width, height))
                cv2.imshow("Capture", img)
                cv2.imshow("Scanned", dst)
        except Exception as e:
            print(e)
            
cap = cv2.VideoCapture(0)
captured = False

if cap.isOpened():
   delay = int(1000/cap.get(cv2.CAP_PROP_FPS))
   while True :
        ret, img = cap.read()
        if ret :
           cv2.imshow("Capture", img)
           key = cv2.waitKey(delay)
           if key & 0xFF == 27 :
               print("아무 작업도 하지 않고 종료함")
               break
           elif key == ord('c') :
               captured = True
               break
        else:
            print(ret, img)
            break
else :
    print("File not opened")
    
if captured :
    cap.release()
    while True :
        cv2.imshow("Capture", img)
        cv2.setMouseCallback("Capture", mouseHandler)
        key = cv2.waitKey(delay)
        if key & 0xFF == 27 :
             print("ESC Key Pressed")
             break
cap.release()
cv2.destroyAllWindows()
'''

