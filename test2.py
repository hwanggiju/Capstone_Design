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

# Change these
# to the right size for your display!
WIDTH = 128
HEIGHT = 64  # Change to 64 if needed
BORDER = 5

# Use for SPI
spi = board.SPI()
oled_reset = digitalio.DigitalInOut(board.D11)
oled_cs = digitalio.DigitalInOut(board.D7)
oled_dc = digitalio.DigitalInOut(board.D8)
oled = adafruit_ssd1306.SSD1306_SPI(WIDTH, HEIGHT, spi, oled_dc, oled_reset, oled_cs)

# Clear display.
oled.fill(0)
oled.show()

# Alternatively load a different format image, resize it, and convert to 1 bit color.
image = Image.open('spi_test_img.png').resize((oled.width, oled.height), Image.ANTIALIAS).convert('1')

# Display image.
oled.image(image)
oled.show()