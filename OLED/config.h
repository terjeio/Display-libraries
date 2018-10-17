/*
 * config.h - for OLED library
 */

#define FRAMEBUFFER
#define SEVENSEGMENT
#define SPIMODE

#define SSD1306 // SPI displays uses this?
//#define SSD1331

#ifdef SSD1331
#define OLED_COLOR
#define OLED_NEWFONTS
#endif

#ifdef FRAMEBUFFER
#define OLED_NEWFONTS
#endif

#ifdef OLED_NEWFONTS
//#define FONTREAD_BYTEWISE // Uncomment if MCU cannot read 64bit pixeldata with 2 32-bit reads (MSP430? compiler bug?)
#endif
