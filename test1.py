from cv2 import UMAT_DATA_DEVICE_COPY_OBSOLETE
import RPi.GPIO as GPIO

# GPIO 번호 사용
switch =  [16, 20, 21] # -> 실제 핀 번호[36, 38, 40]
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)


GPIO.setup(switch[0], GPIO.IN)
GPIO.setup(switch[1], GPIO.IN)
GPIO.setup(switch[2], GPIO.IN)

try:
    print('button test')
    while 1 :
        if GPIO.input(switch[0]) == 1 :
            print('u')
            
        elif GPIO.input(switch[1]) == 1 :
            print('d')
        
        elif GPIO.input(switch[2]) == 1 :
            print('o')
            
except KeyboardInterrupt :
    print('end')
    GPIO.cleanup()