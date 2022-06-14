from tkinter import CENTER
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import os
import time

WIDTH = 128
HEIGHT = 64 
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

def OLED_initial_setting_Height(CHANGE_HEIGHT) :
    draw.text((5, 0), 'First Setting', font = font, fill = 0)
    draw.text((5, 20), 'Input your Height', font = font, fill = 0)
    #draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 0)
    oled.image(image)
    oled.show()

OLED_initial_setting_Height(SET_HEIGHT)     

while True :
    if digitalio.DigitalInOut(board.D16) == 1 :
        SET_HEIGHT = SET_HEIGHT + 5
        OLED_initial_setting_Height(SET_HEIGHT)
        oled.image(image)
        oled.show()
        time.sleep(0.2)
    elif digitalio.DigitalInOut(board.D20) == 1:
        SET_HEIGHT = SET_HEIGHT - 5
        OLED_initial_setting_Height(SET_HEIGHT)
        oled.image(image)
        oled.show()
        time.sleep(0.2)
    elif digitalio.DigitalInOut(board.D21) == 1:
        SET_HEIGHT = SET_HEIGHT
        oled.fill(0)
        oled.show()
        break
