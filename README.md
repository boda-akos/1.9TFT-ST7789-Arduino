# 1.9TFT 170x320 dots ST7789 Arduino

Arduino files for 1.9 TFT recently available on Aliexpress. The resolution of this TFT is 170x320 dots. On the FPC printed : NFP190B-21AF.  Adafruit has an example for the ST7789 driver chip, however it seems to be fixed for the 240x240 resolution despite there is a "void Adafruit_ST7789::init(uint16_t width, uint16_t height)" command but this is channelled into fixed values of width and height. This is done probably the RAM biasing of the chip and the actual TFT is totally hardware dependant anyway, set by the ST7789_170x320_XSTART and ST7789_170x320_YSTART variables in the CPP file. I had to find out these values experimentally. Please unzip all files in the same directory. 

The ino file is written for the Raspberry Pi Pico, modify the ports for other MCUs.  Find the default 4 wire SPI pins with "read_actual_pins.ino".

Note : My excuses for changing the original file names, the reason is to avoid any confusion with the original 240x240 drivers.
