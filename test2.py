from tkinter import CENTER
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import os
import time
import RPi.GPIO as GPIO

WIDTH = 128
HEIGHT = 64 
BORDER = 5

SET_HEIGHT = 170
switch = [16, 20, 21]
wave = [24, 23]

# Use for SPI
spi = board.SPI()
oled_reset = digitalio.DigitalInOut(board.D25)
oled_cs = digitalio.DigitalInOut(board.D8)
oled_dc = digitalio.DigitalInOut(board.D17)
oled = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, oled_dc, oled_reset, oled_cs)

GPIO.setup(switch[0], GPIO.IN)
GPIO.setup(switch[1], GPIO.IN)
GPIO.setup(switch[2], GPIO.IN)

# 초음파 핀 setup
GPIO.setup(wave[0], GPIO.OUT)
GPIO.setup(wave[1], GPIO.IN)
GPIO.output(wave[0], False)

# 초기 디스플레이 설정
oled.fill(0)
oled.show()

font = ImageFont.truetype('malgun.ttf', 15)
font1 = ImageFont.truetype('malgun.ttf', 20)
font2 = ImageFont.truetype('malgun.ttf', 10)

image = Image.new('1', (oled.width, oled.height), 255)
draw = ImageDraw.Draw(image)

def waveFun() :
    GPIO.output(wave[0], True)
    time.sleep(0.00001)
    GPIO.output(wave[0], False)
    
    while GPIO.input(wave[1]) == 0 :
        pulse_start = time.time()
    # pulse_end = pulse_start
    while GPIO.input(wave[1]) == 1 :
        pulse_end = time.time()
        # if pulse_end - pulse_start > 1:
        #     return 0
    pulse_duration = pulse_end - pulse_start
    distance = pulse_duration * 17000
    distance = int(distance)
    
    return distance

def OLED_initial_setting_Height(CHANGE_HEIGHT) :
    draw.text((5, 0), 'First Setting', font = font, fill = 0)
    draw.text((5, 20), 'Input your Height', font = font, fill = 0)
    draw.text((5, 40), str(CHANGE_HEIGHT), font = font, fill = 0)
    oled.image(image)
    oled.show()

def OLED_initial_setting_Height1(CHANGE_HEIGHT) :
    draw.text((5, 0), 'First Setting', font = font, fill = 255)
    draw.text((5, 20), 'Input your Height', font = font, fill = 255)
    draw.text((5, 40), str(CHANGE_HEIGHT), font = font, fill = 255)
    oled.image(image)
    oled.show()
    
deskDistance = 0
def drawDisplay() :
    deskDistance = waveFun()
    draw.text((100, 0), 'Up', font=font2, fill=0)
    draw.text((100, 15), 'Okay', font=font2, fill=0)
    draw.text((100, 30), 'Down', font=font2, fill=0)
    draw.text((5, 0), 'Desk Tall', font=font2, fill=0)
    draw.text((5, 10), str(deskDistance), font = font2, fill = 0)
    oled.image(image)
    oled.show()

def eraseDisplay() :
    draw.text((100, 0), 'Up', font=font2, fill=255)
    draw.text((100, 15), 'Okay', font=font2, fill=255)
    draw.text((100, 30), 'Down', font=font2, fill=255)
    draw.text((5, 0), 'Desk Tall', font=font2, fill=255)
    draw.text((5, 10), str(deskDistance), font = font2, fill = 255)
    oled.image(image)
    oled.show()
    
OLED_initial_setting_Height(SET_HEIGHT)     
try :
    while True :
        if GPIO.input(switch[2]) == 1 :
            draw.text((5, 0), 'Complete set', font = font, fill = 255)
            draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 255)
            OLED_initial_setting_Height1(SET_HEIGHT)
            SET_HEIGHT = SET_HEIGHT + 1
            OLED_initial_setting_Height(SET_HEIGHT)
            time.sleep(0.2)
            
        elif GPIO.input(switch[0]) == 1:
            draw.text((5, 0), 'Complete set', font = font, fill = 255)
            draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 255)
            OLED_initial_setting_Height1(SET_HEIGHT)
            SET_HEIGHT = SET_HEIGHT - 1
            OLED_initial_setting_Height(SET_HEIGHT)
            time.sleep(0.2)
            
        elif GPIO.input(switch[1]) == 1:
            OLED_initial_setting_Height1(SET_HEIGHT)
            draw.text((5, 0), 'Your height', font = font, fill = 0)
            draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 0)
            draw.text((5, 40), 'Right?', font = font, fill = 0)
            oled.image(image)
            oled.show()
            while True :
                if GPIO.input(switch[2]) == 1:
                    draw.text((5, 0), 'Your height', font = font, fill = 255)
                    draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 255)
                    draw.text((5, 40), 'Right?', font = font, fill = 255)
                    OLED_initial_setting_Height1(SET_HEIGHT)
                    SET_HEIGHT = SET_HEIGHT + 1
                    OLED_initial_setting_Height(SET_HEIGHT)
                    time.sleep(0.2)
                    break
                elif GPIO.input(switch[0]) == 1:
                    draw.text((5, 0), 'Your height', font = font, fill = 255)
                    draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 255)
                    draw.text((5, 40), 'Right?', font = font, fill = 255)
                    OLED_initial_setting_Height1(SET_HEIGHT)
                    SET_HEIGHT = SET_HEIGHT - 1
                    OLED_initial_setting_Height(SET_HEIGHT)
                    time.sleep(0.2)
                    break
                elif GPIO.input(switch[1]) == 1 :
                    draw.text((5, 0), 'Your height', font = font, fill = 255)
                    draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 255)
                    draw.text((5, 40), 'Right?', font = font, fill = 255)
                    time.sleep(0.2)
                    draw.text((5, 0), 'Success Set Height', font = font, fill = 0)
                    break
            drawDisplay()
            break
     
except KeyboardInterrupt :
    print('end')
    GPIO.cleanup()
    pass