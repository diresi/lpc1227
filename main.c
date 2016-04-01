/**************************************************************************/
/*!
    @file     main.c
    @author   K. Townsend (microBuilder.eu)

    @section LICENSE

    Software License Agreement (BSD License)

    Copyright (c) 2011, microBuilder SARL
    All rights reserved.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:
    1. Redistributions of source code must retain the above copyright
    notice, this list of conditions and the following disclaimer.
    2. Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.
    3. Neither the name of the copyright holders nor the
    names of its contributors may be used to endorse or promote products
    derived from this software without specific prior written permission.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ''AS IS'' AND ANY
    EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
/**************************************************************************/
#include "projectconfig.h"
#include "sysinit.h"

#include "core/gpio/gpio.h"
#include "core/systick/systick.h"
#include <string.h>

#ifdef CFG_INTERFACE
  #include "core/cmd/cmd.h"
#endif

#include "core/ssp/ssp.h"

void Delay(volatile unsigned long);
void assert_crc4(uint16_t *buf, uint16_t len);
void panic(void);
void LCDStr(unsigned char row, const unsigned char *dataPtr, unsigned char inv );

// olimex LPC_P1227 buttons
#define BTN_WKUP  GPIO_IO_P3    // GPIO_GPIO1
#define BTN_USER1 GPIO_IO_P12   // GPIO_GPIO2
#define BTN_USER2 GPIO_IO_P11   // GPIO_GPIO2
#define BTN_USER3 GPIO_IO_P10   // GPIO_GPIO2

////////////////////////////////////////////////////////////////////////////////
// MS5803
#define MS_CS GPIO_IO_P19

void ms_reset(void)
{
  //ms_send(0x1e);
  uint8_t buf[] = {0x1e};
  GPIO_GPIO0CLR = MS_CS;
  sspSend(0, buf, 1);
  Delay(10000); // wait ~2.8ms
  GPIO_GPIO0SET = MS_CS;
}

uint16_t ms_consts[8];
void ms_read_prom(void)
{
  uint8_t buf[2];

  for (int i=0; i<8; ++i)
  {
    buf[0] = 0xa0 | 2*i;

    GPIO_GPIO0CLR = MS_CS;
    sspSend(0, buf, 1);

    sspReceive(0, buf, 2);
    ms_consts[i] = (buf[0] << 8) | buf[1];
    GPIO_GPIO0SET = MS_CS;
  }
  assert_crc4(ms_consts, sizeof(ms_consts)/sizeof(ms_consts[0]));
}

void ms_init(void)
{
  GPIO_GPIO0DIR |= MS_CS;
  ms_reset();
  ms_read_prom();
}

uint32_t ms_adc_cmd(uint8_t cmd) 
{
  // osr: 0 = 256, 1 = 512, 2 = 1024, 3 = 2048, 4 = 4096
  int osr = 4;

  uint8_t buf[3] = {0};

  buf[0] = cmd | 2*osr;
  GPIO_GPIO0CLR = MS_CS;
  sspSend(0, buf, 1);

  // osr 4096 => 10.0ms timeout
  // osr  256 =>  0.6ms timeout
  Delay(4000); // wait ~1.2ms
  GPIO_GPIO0SET = MS_CS;

  Delay(200); // wait ~0.6ms
  GPIO_GPIO0CLR = MS_CS;

  uint32_t val = 0;

  buf[0] = 0;
  sspSend(0, buf, 1);
  sspReceive(0, buf, 3);
  GPIO_GPIO0SET = MS_CS;

  val |= buf[0] << 16;
  val |= buf[1] <<  8;
  val |= buf[2] <<  0;
  return val;
}

uint32_t ms_pressure()
{
  return ms_adc_cmd(0x40);
}

uint32_t ms_temperature()
{
  return ms_adc_cmd(0x50);
}

void ms_display_prom()
{
  unsigned char buf[18] = {0};

  for (int i = 0; i < 8; i+=2) {
    snprintf((char *)buf, sizeof(buf)/sizeof(buf[0]), "0x%04x 0x%04x    ", ms_consts[i], ms_consts[i+1]);
    LCDStr(1+i/2, buf, 0);
  }
}

void ms_update_screen()
{
  unsigned char buf[20];
  uint32_t d1 = ms_temperature();
  uint32_t d2 = ms_pressure();

  uint32_t dT = d2 - (ms_consts[5] << 8);
  uint32_t t = 2000 + ((dT * ms_consts[6]) >> 23);

  // 1st order temperature compensation
  uint32_t off = (ms_consts[2] << 16) + ((ms_consts[4] * dT) >> 7);
  uint32_t sens = (ms_consts[1] << 15) + ((ms_consts[3] * dT) >> 8);

  // FIXME: 2nd order compensation

  uint32_t p = ((d1 * (sens / (1 << 21))) - off) / (1 << 15);

  double dt = ((double) t) / 100.;
  snprintf((char *)buf, 20, "T: %d.%d", (int)dt, (int)(100* (dt - ((int)dt))));
  LCDStr(2, buf, 0);


  double dp = ((double) p) / 10.;
  snprintf((char *)buf, 20, "P: %d.%d", (int)dp, (int)(100* (dp - ((int)dp))));
  LCDStr(3, buf, 0);
}

////////////////////////////////////////////////////////////////////////////////
// Nokia 3310 display via SPI  // GPIO_GPIO2 13, 14, 15
#define LCD_RES GPIO_IO_P13
#define LCD_CS  GPIO_IO_P14
#define LCD_DC  GPIO_IO_P15

#define SEND_CMD                   0
#define SEND_CHR                   1

#define LCD_X_RES                  84
#define LCD_Y_RES                  48

#define PIXEL_OFF                  0
#define PIXEL_ON                   1
#define PIXEL_XOR                  2

#define FONT_1X                    1
#define FONT_2X                    2
#define LCD_START_LINE_ADDR (66-2)

#define LCD_CACHE_SIZE     504
unsigned char  LcdMemory[LCD_CACHE_SIZE];

const unsigned char  FontLookup [][5] =
{
    { 0x00, 0x00, 0x00, 0x00, 0x00 },   // sp
    { 0x00, 0x00, 0x2f, 0x00, 0x00 },   // !
    { 0x00, 0x07, 0x00, 0x07, 0x00 },   // "
    { 0x14, 0x7f, 0x14, 0x7f, 0x14 },   // #
    { 0x24, 0x2a, 0x7f, 0x2a, 0x12 },   // $
    { 0xc4, 0xc8, 0x10, 0x26, 0x46 },   // %
    { 0x36, 0x49, 0x55, 0x22, 0x50 },   // &
    { 0x00, 0x05, 0x03, 0x00, 0x00 },   // '
    { 0x00, 0x1c, 0x22, 0x41, 0x00 },   // (
    { 0x00, 0x41, 0x22, 0x1c, 0x00 },   // )
    { 0x14, 0x08, 0x3E, 0x08, 0x14 },   // *
    { 0x08, 0x08, 0x3E, 0x08, 0x08 },   // +
    { 0x00, 0x00, 0x50, 0x30, 0x00 },   // ,
    { 0x10, 0x10, 0x10, 0x10, 0x10 },   // -
    { 0x00, 0x60, 0x60, 0x00, 0x00 },   // .
    { 0x20, 0x10, 0x08, 0x04, 0x02 },   // /
    { 0x3E, 0x51, 0x49, 0x45, 0x3E },   // 0
    { 0x00, 0x42, 0x7F, 0x40, 0x00 },   // 1
    { 0x42, 0x61, 0x51, 0x49, 0x46 },   // 2
    { 0x21, 0x41, 0x45, 0x4B, 0x31 },   // 3
    { 0x18, 0x14, 0x12, 0x7F, 0x10 },   // 4
    { 0x27, 0x45, 0x45, 0x45, 0x39 },   // 5
    { 0x3C, 0x4A, 0x49, 0x49, 0x30 },   // 6
    { 0x01, 0x71, 0x09, 0x05, 0x03 },   // 7
    { 0x36, 0x49, 0x49, 0x49, 0x36 },   // 8
    { 0x06, 0x49, 0x49, 0x29, 0x1E },   // 9
    { 0x00, 0x36, 0x36, 0x00, 0x00 },   // :
    { 0x00, 0x56, 0x36, 0x00, 0x00 },   // ;
    { 0x08, 0x14, 0x22, 0x41, 0x00 },   // <
    { 0x14, 0x14, 0x14, 0x14, 0x14 },   // =
    { 0x00, 0x41, 0x22, 0x14, 0x08 },   // >
    { 0x02, 0x01, 0x51, 0x09, 0x06 },   // ?
    { 0x32, 0x49, 0x59, 0x51, 0x3E },   // @
    { 0x7E, 0x11, 0x11, 0x11, 0x7E },   // A
    { 0x7F, 0x49, 0x49, 0x49, 0x36 },   // B
    { 0x3E, 0x41, 0x41, 0x41, 0x22 },   // C
    { 0x7F, 0x41, 0x41, 0x22, 0x1C },   // D
    { 0x7F, 0x49, 0x49, 0x49, 0x41 },   // E
    { 0x7F, 0x09, 0x09, 0x09, 0x01 },   // F
    { 0x3E, 0x41, 0x49, 0x49, 0x7A },   // G
    { 0x7F, 0x08, 0x08, 0x08, 0x7F },   // H
    { 0x00, 0x41, 0x7F, 0x41, 0x00 },   // I
    { 0x20, 0x40, 0x41, 0x3F, 0x01 },   // J
    { 0x7F, 0x08, 0x14, 0x22, 0x41 },   // K
    { 0x7F, 0x40, 0x40, 0x40, 0x40 },   // L
    { 0x7F, 0x02, 0x0C, 0x02, 0x7F },   // M
    { 0x7F, 0x04, 0x08, 0x10, 0x7F },   // N
    { 0x3E, 0x41, 0x41, 0x41, 0x3E },   // O
    { 0x7F, 0x09, 0x09, 0x09, 0x06 },   // P
    { 0x3E, 0x41, 0x51, 0x21, 0x5E },   // Q
    { 0x7F, 0x09, 0x19, 0x29, 0x46 },   // R
    { 0x46, 0x49, 0x49, 0x49, 0x31 },   // S
    { 0x01, 0x01, 0x7F, 0x01, 0x01 },   // T
    { 0x3F, 0x40, 0x40, 0x40, 0x3F },   // U
    { 0x1F, 0x20, 0x40, 0x20, 0x1F },   // V
    { 0x3F, 0x40, 0x38, 0x40, 0x3F },   // W
    { 0x63, 0x14, 0x08, 0x14, 0x63 },   // X
    { 0x07, 0x08, 0x70, 0x08, 0x07 },   // Y
    { 0x61, 0x51, 0x49, 0x45, 0x43 },   // Z
    { 0x00, 0x7F, 0x41, 0x41, 0x00 },   // [
	{ 0x55, 0x2A, 0x55, 0x2A, 0x55 },   // 55
    { 0x00, 0x41, 0x41, 0x7F, 0x00 },   // ]
    { 0x04, 0x02, 0x01, 0x02, 0x04 },   // ^
    { 0x40, 0x40, 0x40, 0x40, 0x40 },   // _
    { 0x00, 0x01, 0x02, 0x04, 0x00 },   // '
    { 0x20, 0x54, 0x54, 0x54, 0x78 },   // a
    { 0x7F, 0x48, 0x44, 0x44, 0x38 },   // b
    { 0x38, 0x44, 0x44, 0x44, 0x20 },   // c
    { 0x38, 0x44, 0x44, 0x48, 0x7F },   // d
    { 0x38, 0x54, 0x54, 0x54, 0x18 },   // e
    { 0x08, 0x7E, 0x09, 0x01, 0x02 },   // f
    { 0x0C, 0x52, 0x52, 0x52, 0x3E },   // g
    { 0x7F, 0x08, 0x04, 0x04, 0x78 },   // h
    { 0x00, 0x44, 0x7D, 0x40, 0x00 },   // i
    { 0x20, 0x40, 0x44, 0x3D, 0x00 },   // j
    { 0x7F, 0x10, 0x28, 0x44, 0x00 },   // k
    { 0x00, 0x41, 0x7F, 0x40, 0x00 },   // l
    { 0x7C, 0x04, 0x18, 0x04, 0x78 },   // m
    { 0x7C, 0x08, 0x04, 0x04, 0x78 },   // n
    { 0x38, 0x44, 0x44, 0x44, 0x38 },   // o
    { 0x7C, 0x14, 0x14, 0x14, 0x08 },   // p
    { 0x08, 0x14, 0x14, 0x18, 0x7C },   // q
    { 0x7C, 0x08, 0x04, 0x04, 0x08 },   // r
    { 0x48, 0x54, 0x54, 0x54, 0x20 },   // s
    { 0x04, 0x3F, 0x44, 0x40, 0x20 },   // t
    { 0x3C, 0x40, 0x40, 0x20, 0x7C },   // u
    { 0x1C, 0x20, 0x40, 0x20, 0x1C },   // v
    { 0x3C, 0x40, 0x30, 0x40, 0x3C },   // w
    { 0x44, 0x28, 0x10, 0x28, 0x44 },   // x
    { 0x0C, 0x50, 0x50, 0x50, 0x3C },   // y
    { 0x44, 0x64, 0x54, 0x4C, 0x44 },   // z
    { 0x08, 0x6C, 0x6A, 0x19, 0x08 },   // { (gramotevichka)
    { 0x0C, 0x12, 0x24, 0x12, 0x0C },   // | (sarce)
    { 0x7E, 0x7E, 0x7E, 0x7E, 0x7E },    // kvadratche
};

void Delay(volatile unsigned long a) { a = 10*a; while (a!=0) a--; }

void LCDSend(unsigned char data, unsigned char cd) {

  GPIO_GPIO2CLR = LCD_CS;

  if(cd == SEND_CHR) {
    GPIO_GPIO2SET = LCD_DC;
  }
  else {
    GPIO_GPIO2CLR = LCD_DC;
  }

  sspSend(0, &data, 1);
  GPIO_GPIO2SET = LCD_CS;
}

void LCDClear(void) {
  memset(LcdMemory, 0, LCD_CACHE_SIZE);
}

void LCDUpdate ( void )
{
  int x,y;

  for(y=0; y<48 / 8; y++) {
    LCDSend(0x80, SEND_CMD );
    LCDSend(0x40 | y, SEND_CMD );
    for(x=0; x < 84; x++) {
      LCDSend( LcdMemory[y * 84 + x], SEND_CHR );
    }
  }
}

void lcd_reset()
{
  GPIO_GPIO2CLR |= LCD_RES;
  Delay(1000);
  GPIO_GPIO2SET |= LCD_RES;
  Delay(1000);
}

void lcd_init()
{
  // P2_13, 14 and 15 as output, not required, but initialization is nice
  IOCON_PIO2_13 = IOCON_COMMON_FUNC_GPIO | IOCON_COMMON_MODE_PULLUP;
  IOCON_PIO2_14 = IOCON_COMMON_FUNC_GPIO | IOCON_COMMON_MODE_PULLUP;
  IOCON_PIO2_15 = IOCON_COMMON_FUNC_GPIO | IOCON_COMMON_MODE_PULLUP;
  GPIO_GPIO2DIR |= (LCD_RES | LCD_CS | LCD_DC);
  GPIO_GPIO2SET |= (LCD_RES | LCD_CS | LCD_DC);

  lcd_reset();

  // Send sequence of command
  LCDSend( 0x21, SEND_CMD );  // LCD Extended Commands.
  LCDSend( 0xC8, SEND_CMD );  // Set LCD Vop (Contrast). 0xC8
  LCDSend( 0x04 | !!(LCD_START_LINE_ADDR&(1u<<6)), SEND_CMD );  // Set Temp S6 for start line
  LCDSend( 0x40 | (LCD_START_LINE_ADDR & ((1u<<6)-1)), SEND_CMD );  // Set Temp S[5:0] for start line
  //LCDSend( 0x13, SEND_CMD );  // LCD bias mode 1:48.
  LCDSend( 0x12, SEND_CMD );  // LCD bias mode 1:68.
  LCDSend( 0x20, SEND_CMD );  // LCD Standard Commands, Horizontal addressing mode.
  //LCDSend( 0x22, SEND_CMD );  // LCD Standard Commands, Vertical addressing mode.
  LCDSend( 0x08, SEND_CMD );  // LCD blank
  LCDSend( 0x0C, SEND_CMD );  // LCD in normal mode.


   // Clear and Update
  LCDClear();
  LCDUpdate();
}

void LCDContrast(unsigned char contrast) {

    //  LCD Extended Commands.
    LCDSend( 0x21, SEND_CMD );

    // Set LCD Vop (Contrast).
    LCDSend( 0x80 | contrast, SEND_CMD );

    //  LCD Standard Commands, horizontal addressing mode.
    LCDSend( 0x20, SEND_CMD );
}

void LCDChrXY (unsigned char x, unsigned char y, unsigned char ch )
{
  unsigned int    index   = 0;
  unsigned int    offset  = 0;
  unsigned int    i       = 0;

  // check for out off range
  if ( x > LCD_X_RES ) return;
  if ( y > LCD_Y_RES ) return;

  index=(unsigned int)x*5+(unsigned int)y*84;

  for ( i = 0; i < 5; i++ )
  {
    offset = FontLookup[ch - 32][i];
    LcdMemory[index] = offset;
    index++;
  }
}
void LCDChrXYInverse (unsigned char x, unsigned char y, unsigned char ch )
{
	unsigned int    index   = 0;
    unsigned int    i       = 0;

    // check for out off range
    if ( x > LCD_X_RES ) return;
    if ( y > LCD_Y_RES ) return;

	index=(unsigned int)x*5+(unsigned int)y*84;

    for ( i = 0; i < 5; i++ )
    {
      LcdMemory[index] = ~(FontLookup[ch - 32][i]);
      index++;
    }

}

void LCDStr(unsigned char row, const unsigned char *dataPtr, unsigned char inv ) {

  // variable for X coordinate
  unsigned char x = 0;

  // loop to the and of string
  while ( *dataPtr ) {
	if(inv) {
		LCDChrXYInverse(x, row, (*dataPtr));
	} else {
      LCDChrXY( x, row, (*dataPtr));
	}

    x++;
    dataPtr++;
  }

  LCDUpdate();

}


void lcd_test()
{
  LCDContrast(0x70);
  LCDStr(0, (unsigned char *)"**** RISSNER ****", 0);
  LCDStr(1, (unsigned char *)"++++ rissner ++++", 0);
  LCDStr(2, (unsigned char *)"//// RISSNER ////", 0);
  LCDStr(3, (unsigned char *)"**** rissner ****", 1);
  LCDStr(4, (unsigned char *)"++++ RISSNER ++++", 1);
  LCDStr(5, (unsigned char *)"//// rissner ////", 1);
}

/**************************************************************************/
void btnInit(void)
{
    GPIO_GPIO1DIR &= ~BTN_WKUP;
    GPIO_GPIO2DIR &= ~(BTN_USER1 | BTN_USER2 | BTN_USER3);
}

uint8_t btnState(void)
{
    uint8_t state = 0;
    state |= (GPIO_GPIO1PIN & BTN_WKUP)  ? 0 : 0x1;
    state |= (GPIO_GPIO2PIN & BTN_USER1) ? 0 : 0x2;
    state |= (GPIO_GPIO2PIN & BTN_USER2) ? 0 : 0x4;
    state |= (GPIO_GPIO2PIN & BTN_USER3) ? 0 : 0x8;
    return state;
}
/**************************************************************************/

void panic(void)
{
    char panic[] = "panic";
    LCDStr(0, (unsigned char *)panic, 1);
    while (1);
}

uint16_t crc4_update_remainder(uint16_t n_rem, uint8_t b)
{
    n_rem ^= b;
    for (uint8_t i=0; i<8; ++i) {
        if (n_rem & 0x8000)
            n_rem = (n_rem << 1) ^ 0x3000;
        else
            n_rem <<= 1;
    }
    return n_rem;
}

uint8_t verify_crc4(uint16_t *buf, uint16_t len)
{
    uint16_t n_rem = 0x0000;
    for (uint16_t i=0; i<len; ++i) {
        n_rem = crc4_update_remainder(n_rem, 0xFF & (buf[i] >> 8));
        n_rem = crc4_update_remainder(n_rem, 0xFF & buf[i]);
    }
    n_rem = 0x000F & (n_rem >> 12);
    return n_rem;
}

void assert_crc4(uint16_t *buf, uint16_t len)
{
    uint8_t crc = verify_crc4(buf, len);
    if (crc != (buf[len-1] & 0x000F)) {
        char buf[16];
        snprintf(buf, 16, "%i 0x%02x  --", len, crc);
        LCDStr(5, (unsigned char*)buf, 0);
        panic();
    }
}

/**************************************************************************/
/*!
    Main program entry point.  After reset, normal code execution will
    begin here.
*/
/**************************************************************************/

int main(void)
{
  // Configure cpu and mandatory peripherals
  systemInit();
  btnInit();

  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  // buzz a little bit :)
  GPIO_GPIO1DIR |= GPIO_IO_P6;
  GPIO_GPIO1MASK &= ~GPIO_IO_P6;
  uint32_t x = 0;
  while (x++ < 10)
  {
    systickDelay(1);
    gpioToggle(1, 6);
  }
  GPIO_GPIO1MASK |= GPIO_IO_P6;
  GPIO_GPIO1DIR &= ~GPIO_IO_P6;

  // MS5803 needs (sspClockPolarity_Low | sspClockPhase_RisingEdge) or
  // (sspClockPolarity_High | sspClockPhase_FallingEdge)
  // LCD can work with both those settings, too.
  sspInit(0, sspClockPolarity_High, sspClockPhase_FallingEdge);
  lcd_init();
  ms_init();

  lcd_test();
  Delay(1000000);
  LCDClear();

  ms_display_prom();
  Delay(1000000);
  LCDClear();

  int cnt = 0;
  unsigned char buf[20];
  memset(buf, 0, 20);

  while (1)
  {
    cnt++;

    // Toggle LED once per second
    currentSecond = systickGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;
      gpioToggle(CFG_LED_PORT, CFG_LED_PIN-1);
      gpioToggle(CFG_LED_PORT, CFG_LED_PIN);

      LCDClear();

      snprintf((char *)buf, 20, "C: %d", cnt);
      LCDStr(0, buf, 0);
      cnt = 0;

      // btn state
      snprintf((char *)buf, 20, "B: %02x", btnState());
      LCDStr(1, buf, 0);

      ms_update_screen();
    }
  }

  return 0;
}
