

#include "Afruit_ST77xx.h"
#include <limits.h>
#ifndef ARDUINO_STM32_FEATHER
  #include "pins_arduino.h"
  #include "wiring_private.h"
#endif
#include <SPI.h>

#define SPI_DEFAULT_FREQ 16000000 ///< Default SPI data clock frequency

/**************************************************************************/
Adafruit_ST77xx::Adafruit_ST77xx(int8_t cs, int8_t dc, int8_t mosi,
  int8_t sclk, int8_t rst, int8_t miso) : Adafruit_SPITFT(
  ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160, cs, dc, mosi, sclk, rst, miso) {
}


/**************************************************************************/
Adafruit_ST77xx::Adafruit_ST77xx(int8_t cs, int8_t dc, int8_t rst) :
  Adafruit_SPITFT(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160, cs, dc, rst) {
}

#if !defined(ESP8266)
/**************************************************************************/

/**************************************************************************/
Adafruit_ST77xx::Adafruit_ST77xx(SPIClass *spiClass, int8_t cs, int8_t dc,
  int8_t rst) : Adafruit_SPITFT(ST7735_TFTWIDTH_128, ST7735_TFTHEIGHT_160,
  spiClass, cs, dc, rst) {
}
#endif // end !ESP8266

/**************************************************************************/
/*!
    @brief  Companion code to the initiliazation tables. Reads and issues
            a series of LCD commands stored in PROGMEM byte array.
    @param  addr  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_ST77xx::displayInit(const uint8_t *addr) {

  uint8_t  numCommands, cmd, numArgs;
  uint16_t ms;

  numCommands = pgm_read_byte(addr++);   // Number of commands to follow
  while(numCommands--) {                 // For each command...
    cmd = pgm_read_byte(addr++);         // Read command
    numArgs  = pgm_read_byte(addr++);    // Number of args to follow
    ms       = numArgs & ST_CMD_DELAY;   // If hibit set, delay follows args
    numArgs &= ~ST_CMD_DELAY;            // Mask out delay bit
    sendCommand(cmd, addr, numArgs);
    addr += numArgs;
    SPI_CS_HIGH(); SPI_CS_LOW();  // ST7789 needs chip deselect after each

    if(ms) {
      ms = pgm_read_byte(addr++); // Read post-command delay time (ms)
      if(ms == 255) ms = 500;     // If 255, delay for 500 ms
      delay(ms);
    }
  }
}

/**************************************************************************/
/*!
    @brief  Initialize ST77xx chip. Connects to the ST77XX over SPI and
            sends initialization procedure commands
    @param  freq  Desired SPI clock frequency
*/
/**************************************************************************/
void Adafruit_ST77xx::begin(uint32_t freq) {
  if(!freq) {
    freq = SPI_DEFAULT_FREQ;
  }
  _freq = freq;

  invertOnCommand  = ST77XX_INVON;
  invertOffCommand = ST77XX_INVOFF;

  initSPI(freq);
}

/**************************************************************************/
/*!
    @brief  Initialization code common to all ST77XX displays
    @param  cmdList  Flash memory array with commands and data to send
*/
/**************************************************************************/
void Adafruit_ST77xx::commonInit(const uint8_t *cmdList) {
  begin();

  if(cmdList) {
    displayInit(cmdList);
  }
}

/**************************************************************************/
/*!
  @brief  SPI displays set an address window rectangle for blitting pixels
  @param  x  Top left corner x coordinate
  @param  y  Top left corner x coordinate
  @param  w  Width of window
  @param  h  Height of window
*/
/**************************************************************************/
void Adafruit_ST77xx::setAddrWindow(uint16_t x, uint16_t y, uint16_t w,
  uint16_t h) {
  x += _xstart;
  y += _ystart;
  uint32_t xa = ((uint32_t)x << 16) | (x+w-1);
  uint32_t ya = ((uint32_t)y << 16) | (y+h-1); 

  writeCommand(ST77XX_CASET); // Column addr set
  SPI_WRITE32(xa);

  writeCommand(ST77XX_RASET); // Row addr set
  SPI_WRITE32(ya);

  writeCommand(ST77XX_RAMWR); // write to RAM
}

/**************************************************************************/
/*!
    @brief  Set origin of (0,0) and orientation of TFT display
    @param  m  The index for rotation, from 0-3 inclusive
*/
/**************************************************************************/
void Adafruit_ST77xx::setRotation(uint8_t m) {
  uint8_t madctl = 0;

  rotation = m % 4; // can't be higher than 3

  switch(rotation) {
   case 0:
     madctl  = ST77XX_MADCTL_MX | ST77XX_MADCTL_MY | ST77XX_MADCTL_RGB;
     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 1:
     madctl  = ST77XX_MADCTL_MY | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  case 2:
     madctl  = ST77XX_MADCTL_RGB;
     _xstart = _colstart;
     _ystart = _rowstart;
     break;
   case 3:
     madctl  = ST77XX_MADCTL_MX | ST77XX_MADCTL_MV | ST77XX_MADCTL_RGB;
     _ystart = _colstart;
     _xstart = _rowstart;
     break;
  }
  
  sendCommand(ST77XX_MADCTL, &madctl, 1);
}


/**************************************************************************/
void Adafruit_ST77xx::setColRowStart(int8_t col, int8_t row) {
  _colstart = col;
  _rowstart = row;
}

/*!
    @brief  Set scroll def
    @param  
    @param  
*/
void Adafruit_ST77xx::setScroll(int8_t col) {
   writeCommand(0x37);  //writeCommand(ST77XX_VSCRSADD);
  spiWrite(0x00);
  spiWrite(col);
}

void Adafruit_ST77xx::setScrollDef(uint8_t top_fix_height, uint8_t bottom_fix_height, uint8_t _scroll_direction){

  uint8_t scroll_height;
  scroll_height = 160 - top_fix_height - bottom_fix_height; 
  
  writeCommand(ST77XX_VSCRDEF);
  
  spiWrite(0x00);
  spiWrite(top_fix_height);
  spiWrite(0x00);
  spiWrite(scroll_height);
  spiWrite(0x00);
  spiWrite(bottom_fix_height);
  writeCommand(ST77XX_MADCTL);
  if(_scroll_direction){
  spiWrite(0xD8);

  }
  else{ spiWrite(0xC8);

  } 
}
