from tkinter import CENTER
import board
import digitalio
from PIL import Image, ImageDraw, ImageFont
import adafruit_ssd1306
import os
import time
import RPi.GPIO as GPIO
import numpy as np

WIDTH = 128
HEIGHT = 64 
BORDER = 5

SET_HEIGHT = 170
switch = [16, 20, 21]
wave = [24, 23]
driver = [19, 27, 22, 5, 6, 13]

# Use for SPI
spi = board.SPI()
oled_reset = digitalio.DigitalInOut(board.D25)
oled_cs = digitalio.DigitalInOut(board.D8)
oled_dc = digitalio.DigitalInOut(board.D17)
oled = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, oled_dc, oled_reset, oled_cs)

GPIO.setup(switch[0], GPIO.IN)
GPIO.setup(switch[1], GPIO.IN)
GPIO.setup(switch[2], GPIO.IN)

for i in range(len(driver)): #모터 드라이버 핀
    GPIO.setup(driver[i], GPIO.OUT)

# 초음파 핀 setup
GPIO.setup(wave[0], GPIO.OUT)
GPIO.setup(wave[1], GPIO.IN)
GPIO.output(wave[0], False)

enA_pwm = GPIO.PWM(driver[0], 50) # channel, frequency 50Hz
enB_pwm = GPIO.PWM(driver[5], 50) # channel, frequency 50Hz
enA_pwm.start(0)    # enableA pin start dutycycle 0%
enB_pwm.start(0)    # enableB pin start dutycycle 0%

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

actionPre = 1
def changePWM(enA, enB):
    if enA < 0 or enA > 100: #range over check
        return False
    elif enB < 0 or enB > 100:
        return False
    enA_pwm.ChangeDutyCycle(enA)
    enB_pwm.ChangeDutyCycle(enB)
    return True

def HorizontalHold(nowAngle, compareAngle, waveSensorMean):
    pwmA = 80
    pwmB = 80
    diffPwm = int(20 * np.sin((64 * (nowAngle-compareAngle)) * np.pi/180))
    if actionPre == 2 :
        if (nowAngle > compareAngle) : 
            pwmA += diffPwm
            pwmB -= diffPwm
            changePWM(pwmA, pwmB)
            print(str(pwmA) + '/' + str(pwmB))
        elif nowAngle < compareAngle:
            pwmA -= diffPwm
            pwmB += diffPwm
            changePWM(pwmA, pwmB)
            print(str(pwmA) + '/' + str(pwmB))
        else : 
            fixpwmA = diffPwm + pwmA
            fixpwmB = diffPwm + pwmB
            changePWM(fixpwmA, fixpwmB)
            print(str(fixpwmA) + '/' + str(fixpwmB))
    elif actionPre == 0 :
        if (nowAngle > compareAngle) : 
            pwmA += diffPwm
            pwmB -= diffPwm
            changePWM(pwmA, pwmB)
            print(str(pwmA) + '/' + str(pwmB))
        elif (nowAngle < compareAngle):
            pwmA -= diffPwm
            pwmB += diffPwm
            changePWM(pwmA, pwmB)
            print(str(pwmA) + '/' + str(pwmB))
        else :
            fixpwmA = diffPwm + pwmA
            fixpwmB = diffPwm + pwmB
            changePWM(fixpwmA, fixpwmB)
            print(str(fixpwmA) + '/' + str(fixpwmB))

def btn_driverSet(enA, motorA, motorB, enB):
    global initial, nowTime, preTime
    if initial == True:
        preTime = time.time()
        initial = False
    changePWM(0, 0)
    for i in range(1, len(driver)-1):
        GPIO.output(driver[i], 0)

    if nowTime - preTime > 0.5: # 작동 딜레이
        if motorA == 2:#up
            GPIO.output(driver[1], 0)
            GPIO.output(driver[2], 1)
        elif motorA == 1 :
            GPIO.output(driver[1], 1)
            GPIO.output(driver[2], 0)
        else:#stop
            GPIO.output(driver[1], 0)
            GPIO.output(driver[2], 0)
            
        if motorB == 2:#up
            GPIO.output(driver[3], 0)
            GPIO.output(driver[4], 1)
        elif motorB == 1 :
            GPIO.output(driver[3], 1)
            GPIO.output(driver[4], 0)
        else:#stop
            GPIO.output(driver[3], 0)
            GPIO.output(driver[4], 0)
        changePWM(enA, enB)
        initial = True
        preTime = nowTime
        return True
    else:
        return False

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
    draw.text((100, 20), 'Okay', font=font2, fill=0)
    draw.text((100, 40), 'Down', font=font2, fill=0)
    draw.text((5, 0), 'Desk Tall', font=font2, fill=0)
    draw.text((5, 10), str(deskDistance), font = font2, fill = 0)
    oled.image(image)
    oled.show()

def eraseDisplay() :
    draw.text((100, 0), 'Up', font=font2, fill=255)
    draw.text((100, 15), 'Okay', font=font2, fill=255)
    draw.text((100, 30), 'Down', font=font2, fill=255)
    draw.text((5, 0), 'Desk Tall', font=font, fill=255)
    draw.text((5, 10), str(deskDistance), font = font, fill = 255)
    oled.image(image)
    oled.show()
    
OLED_initial_setting_Height(SET_HEIGHT)     
try :
    while True :
        if GPIO.input(switch[2]) == 1 :     # up
            draw.text((5, 0), 'Complete set', font = font, fill = 255)
            draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 255)
            OLED_initial_setting_Height1(SET_HEIGHT)
            SET_HEIGHT = SET_HEIGHT + 1
            OLED_initial_setting_Height(SET_HEIGHT)
            time.sleep(0.2)
            
        elif GPIO.input(switch[0]) == 1:    # okay
            draw.text((5, 0), 'Complete set', font = font, fill = 255)
            draw.text((5, 40), str(SET_HEIGHT), font = font, fill = 255)
            OLED_initial_setting_Height1(SET_HEIGHT)
            SET_HEIGHT = SET_HEIGHT - 1
            OLED_initial_setting_Height(SET_HEIGHT)
            time.sleep(0.2)
            
        elif GPIO.input(switch[1]) == 1:    # down
            OLED_initial_setting_Height1(SET_HEIGHT)
            draw.text((5, 0), 'Your height', font = font, fill = 0)
            draw.text((5, 20), str(SET_HEIGHT), font = font, fill = 0)
            draw.text((5, 40), 'Right?', font = font, fill = 0)
            oled.image(image)
            oled.show()
            time.sleep(0.2)
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
                    draw.text((5, 0), 'Success Set Height', font = font2, fill = 0)
                    time.sleep(1)
                    break
            draw.text((5, 0), 'Success Set Height', font = font2, fill = 255)
            drawDisplay()
            break
        
    while True :
        eraseDisplay()
        drawDisplay()
        if GPIO.input(switch[2]) == 1 :     # up
            pass 
        
        break
            
     
except KeyboardInterrupt :
    print('end')
    GPIO.cleanup()
    pass