#include "Afruit_ST77xx.h"
#include "Afruit_ST7789.h"

// CONSTRUCTORS ************************************************************

Adafruit_ST7789::Adafruit_ST7789(int8_t cs, int8_t dc, int8_t mosi,
  int8_t sclk, int8_t rst) : Adafruit_ST77xx(cs, dc, mosi, sclk, rst) {
}

Adafruit_ST7789::Adafruit_ST7789(int8_t cs, int8_t dc, int8_t rst) :
  Adafruit_ST77xx(cs, dc, rst) {
}

#if !defined(ESP8266)

Adafruit_ST7789::Adafruit_ST7789(SPIClass *spiClass, int8_t cs, int8_t dc,
  int8_t rst) : Adafruit_ST77xx(spiClass, cs, dc, rst) {
}
#endif // end !ESP8266

// SCREEN INITIALIZATION ***************************************************

#define ST7789_170x320_XSTART 0x24
#define ST7789_170x320_YSTART 0x00

static const uint8_t
  cmd_170x320[] =  {                // Init commands for 7789 screens
    9,                              //  9 commands in list:
    ST77XX_SWRESET,   ST_CMD_DELAY, //  1: Software reset, no args, w/delay
      150,                          //    150 ms delay
    ST77XX_SLPOUT ,   ST_CMD_DELAY, //  2: Out of sleep mode, no args, w/delay
      255,                          //     255 = 500 ms delay
    ST77XX_COLMOD , 1+ST_CMD_DELAY, //  3: Set color mode, 1 arg + delay:
      0x55,                         //     16-bit color
      10,                           //     10 ms delay
    ST77XX_MADCTL , 1,              //  4: Mem access ctrl (directions), 1 arg:
      0x08,                         //     Row/col addr, bottom-top refresh
    ST77XX_CASET  , 4,              //  5: Column addr set, 4 args, no delay:
      0x00,
      0x00,        //     XSTART = 0
      0x00,
      0x1a,  //     XEND = 170
    ST77XX_RASET  , 4,              //  6: Row addr set, 4 args, no delay:
      0x00,
      0x00,             //     YSTART = 0
      0x01,
      0x3f,  //     YEND = 320
    ST77XX_INVON  ,   ST_CMD_DELAY,  //  7: hack
      10,
    ST77XX_NORON  ,   ST_CMD_DELAY, //  8: Normal display on, no args, w/delay
      10,                           //     10 ms delay
    ST77XX_DISPON ,   ST_CMD_DELAY, //  9: Main screen turn on, no args, delay
    255 };                          //     255 = max (500 ms) delay

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST7789 displays
    @param  width  Display width
    @param  height Display height
*/
/**************************************************************************/
void Adafruit_ST7789::init(uint16_t width, uint16_t height) {
  commonInit(NULL);

  _colstart = ST7789_170x320_XSTART;
  _rowstart = ST7789_170x320_YSTART;
  _height   = height;
  _width    = width;

  displayInit(cmd_170x320);

  setRotation(0);
}
/**************************************************************************/
void Adafruit_ST7789::setRotation(uint8_t m) {
  uint8_t madctl = 0;

  rotation = m & 3; // can't be higher than 3

  switch (rotation) {
   case 0:
     madctl  = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     madctl  = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     _xstart = _rowstart;
     _ystart = _colstart;
     break;
  case 2:
     madctl  = ST77XX_MADCTL_RGB;
     _xstart = 0;
     _ystart = 0;
     break;
   case 3:
     madctl  = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     _xstart = 0;
     _ystart = 0;
     break;
  }

  sendCommand(ST77XX_MADCTL, &madctl, 1);
}
