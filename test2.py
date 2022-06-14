import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import RPi.GPIO as GPIO
import os

# GPIO 번호 사용
switch =  [16, 20, 21] # -> 실제 핀 번호[36, 38, 40]

GPIO.setup(switch[0], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch[1], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)
GPIO.setup(switch[2], GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

up_btn = GPIO.input(switch[0])
okay_btn = GPIO.input(switch[1])
down_btn = GPIO.input(switch[2])

WIDTH = 128
HEIGHT = 64 
BORDER = 5

SET_HEIGHT = 170
FACEWIDTHMAX = 0
FACEWIDTHMIN = 0

# Use for SPI
spi = board.SPI()
oled_reset = digitalio.DigitalInOut(board.D25)
oled_cs = digitalio.DigitalInOut(board.D8)
oled_dc = digitalio.DigitalInOut(board.D17)
oled = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, oled_dc, oled_reset, oled_cs)

# Clear display.
oled.fill(0)
oled.show()
font = ImageFont.load_default()

image = Image.new('1', (oled.width, oled.height), 255)
draw = ImageDraw.Draw(image)

def OLED_initial_setting_Height(CHANGE_HEIGHT) :
    draw.text((0, 0), 'First Setting', fill = 0)
    draw.text((0, 20), 'Input your Height', fill = 0)
    draw.text((0, 40), str(CHANGE_HEIGHT), fill = 0)
    oled.image(image)
    oled.show()  

def OLED_initial_setting_NearFaceSize(FACEWIDTHMIN) :
    draw.text((0, 0), 'Next Setting', fill = 0)
    draw.text((0, 20), 'Your Face Size', fill = 0)      
    draw.text((0, 40), str(FACEWIDTHMIN), fill = 0)        # widthLength 값 전달

OLED_initial_setting_Height(HEIGHT)
# def OLED_initial_setting_NearFaceSize(FACEWIDTHMIN)

'''
try :
    OLED_initial_setting_Height(SET_HEIGHT)
    while True :
        if up_btn == 1 :
            SET_HEIGHT = SET_HEIGHT + 5
            OLED_initial_setting_Height(SET_HEIGHT)
        elif down_btn == 1:
            SET_HEIGHT = SET_HEIGHT - 5
            OLED_initial_setting_Height(SET_HEIGHT)
        elif okay_btn == 1:
            SET_HEIGHT = SET_HEIGHT
            oled.fill(0)
            oled.show()
            break
    
    while True :
        OLED_initial_setting_NearFaceSize(FACEWIDTHMIN)
        if okay_btn == 1:
            FACEWIDTHMIN = widthLength     # 현재 얼굴 값 

except KeyboardInterrupt:
    pass
    '''