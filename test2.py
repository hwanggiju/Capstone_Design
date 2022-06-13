'''
디스플레이 관련 라이브러리 설치
$ sudo apt-get install python-imaging python-smbus

$ sudo apt-get install git
$ git clone https://github.com/adafruit/Adafruit_Python_SSD1306.git
$ cd Adafruit_Python_SSD1306
$ sudo python3 setup.py install
'''
# oled 라이브러리
import Adafruit_GPIO.SPI as SPI
import Adafruit_SSD1306
# PIL image 라이브러리
from PIL import Image
from PIL import ImageDraw
from PIL import ImageFont

# oled display 오브젝트 셋업
RST = 24
DC = 25
SPI_PORT = 0
SPI_DEVICE = 0

disp = Adafruit_SSD1306.SSD1306_128_64(rst=RST, dc=DC, spi=SPI.SpiDev(SPI_PORT, SPI_DEVICE, max_speed_hz=8000000))
disp.begin()

# clear display
disp.clear()
disp.display()

# 빈 이미지 준비하기
width = disp.width
height = disp.height
image = Image.new('1',(width, height)) # 1bit-black&white  이미지이므로 1.
draw = ImageDraw.Draw(image)

#font 준비
font = ImageFont.truetype("malgun.ttf",15)

# draw a text
draw.text(
(10,10),
'''세종대왕
만만세''',
font=font, fill=255)

# oled에 보이기
disp.image(image)
disp.display()