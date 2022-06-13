from cv2 import UMAT_DATA_DEVICE_COPY_OBSOLETE
import RPi.GPIO as GPIO

# GPIO 번호 사용
switch =  [16, 20, 21] # -> 실제 핀 번호[36, 38, 40]
GPIO.setmode(GPIO.BCM)


GPIO.setup(switch[0], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch[1], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch[2], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

up_btn = GPIO.input(switch[0])
okay_btn = GPIO.input(switch[1])
down_btn = GPIO.input(switch[2])

try:
    print('button test')
    while 1 :
        if up_btn == 1 :
            print('u')
            
        elif down_btn == 1 :
            print('d')
        
        elif okay_btn == 1 :
            print('o')
            
except KeyboardInterrupt :
    print('end')
    GPIO.cleanup()