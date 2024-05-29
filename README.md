# 1.9TFT 170x320 pixels ST7789 Arduino

Arduino files for 1.9 TFT recently available on Aliexpress. The resolution of this TFT is 170x320 pixels. On the FPC printed : NFP190B-21AF.  Adafruit has an example for the ST7789 driver chip, however it seems to be fixed for the 240x240 resolution despite there is a "void Adafruit_ST7789::init(uint16_t width, uint16_t height)" command but this is channeled into fixed values of width and height. Two hardware dependant constants are ST7789_170x320_XSTART and ST7789_170x320_YSTART variables in the CPP file. I had to find out these values experimentally. 

Please unzip all files in the same directory. 

The ino file is written for the Raspberry Pi Pico, modify the ports for other MCUs.  Find the default 4 wire SPI pins with "read_actual_pins.ino".

Interesting fact that the 0,0 coordinate is the bottom left of the TFT, on all other displays to my knowledge this point is the upper left corner.

Note : My excuses for changing the original file names, the reason is to avoid any confusion with the original 240x240 drivers.
