'''
디스플레이 관련 라이브러리 설치
$ sudo apt-get install python-imaging python-smbus

$ sudo apt-get install git
$ git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
$ cd Adafruit_Python_SSD1306
$ sudo python3 setup.py install
'''
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

# Change these
# to the right size for your display!
WIDTH = 128
HEIGHT = 64  # Change to 64 if needed
BORDER = 5

SET_HEIGHT = 170

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

draw.text((0, 0), 'First Setting', fill = 0)
draw.text((0, 20), 'Setting your Height', fill = 0)

oled.image(image)
oled.show()  

try :
    while True :
        draw.text((0, 40), str(SET_HEIGHT), fill = 0)  
        if  up_btn == True :
            SET_HEIGHT = SET_HEIGHT + 5
        elif down_btn == True :
            SET_HEIGHT = SET_HEIGHT - 5
        elif okay_btn == True :
            oled.fill(0)
            draw.text((0, 0), 'Your Height', fill = 0)
            draw.text((0, 20), str(SET_HEIGHT), fill = 0)
            oled.show()
            
        oled.image(image)
        oled.show()
except KeyboardInterrupt:
    pass