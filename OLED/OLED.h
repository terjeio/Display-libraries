/******************************************************************************
 * File Name   :  OLED_SSD1306.h
 * Author      :  43oh - MSP430 News Projects and Forums
               :  (http://www.43oh.com)
 * Description :  Lowlevel driver for OLED SSD1306
 * Date        :  October 21, 2011.
 
               : Terje Io, 2018-03-03
               : Added optional framebuffer support and
               : support for Reddmann font format

               : Terje Io, 2018-08-26
               : Refactored and added support for 64x96 SSD1331 color display
 *****************************************************************************/

// NOTE: this file has been modified beyond recognition by Terje Io - not sure who to credit anymore
 
#ifndef __oled_h__
#define __oled_h__

#include <stdint.h>
#include <stdbool.h>

#include "config.h"

#ifdef OLED_NEWFONTS
#include "../fonts/font.h"
#endif

typedef enum {
    ThreeAndHAlfDigit = 0,
    FourDigit = 4,
} sevensegmode_t;

typedef enum {
    SPI,
    I2C
} interface_t;

typedef struct {
    uint32_t Width;
    uint32_t Height;
    uint8_t Orientation;
} screen_t;

typedef struct {
    screen_t screen;
    interface_t io;
    void (*writeData)(const uint8_t *data, uint16_t size);
    void (*writeCommand)(const uint8_t *data, uint16_t size);
    void (*delayms)(uint16_t ms);
} driver_t;

typedef struct {
    uint8_t width;          // width in pixels for fixed drawing
    uint8_t rows;           // height in rows, heigth in pixels is rows * 8
    uint8_t offset;         // offset to first character
    uint8_t charData[];     // bit field of all characters
} OLEDFont;

#ifdef OLED_COLOR

#define DPYCOLORTRANSLATE(c) ((((c) & 0x00f80000) >> 16) | (((c) & 0x00001C00) << 3) | (((c) & 0x0000E000) >> 13) | (((c) & 0x000000f8) << 5)) // RGB

typedef union {
    uint32_t value;
    struct {
        uint8_t B;
        uint8_t G;
        uint8_t R;
        uint8_t A;
    };
} RGBColor_t;

// NOTE: struct defines for colorRGB565 is not correct for little endian uint16_t, display format is big endian
typedef union {
    uint16_t value;
    struct {
        uint8_t firstByte;
        uint8_t secondByte;
    };
} colorRGB565;

typedef struct {
    uint8_t R;
    uint8_t G;
    uint8_t B;
} OLED_Color_t;

#else
#define BLACK 0
#define WHITE 1
#endif

#ifdef SSD1331 // SSD1331 color display

#define OLED_WIDTH 96
#define OLED_HEIGHT 64

#define OLED_SetReMap           0xA0
#define OLED_DisplayOff         0xAE
#define OLED_DisplayOn          0xAF
#define OLED_StartLine          0xA1
#define OLED_DisplayOffset      0xA2
#define OLED_NormalDisplay      0xA4
#define OLED_SetMuliplexRatio   0xA8

#define OLED_SetMaster      0xAD
#define OLED_PowerMode      0xB0
#define OLED_PreCharge      0xB1
#define OLED_ClockDiv       0xB3
#define OLED_PreChargeA     0x8A
#define OLED_PreChargeB     0x8B
#define OLED_PreChargeC     0x8C
#define OLED_PreChargeLevel 0xBB
#define OLED_VCOMH          0xBE
#define OLED_MasterCurrent  0x87
#define OLED_ContrastA      0x81
#define OLED_ContrastB      0x82
#define OLED_ContrastC      0x83
#define OLED_SetColumnStartAddress 0x15
#define OLED_SetRowStartAddress    0x75

#define OLED_SetPhasePeriodAdjustment 0xB1
#define OLED_DisplayClockDiv    0xB3

// Accelerated commands

#define OLED_CMD_DrawLine       0x21
#define OLED_CMD_DrawRectangle  0x22
#define OLED_CMD_Copy           0x23
#define OLED_CMD_DimWindow      0x24
#define OLED_CMD_ClearWindow    0x25
#define OLED_CMD_FillEnable     0x26

#define OLED_CMD_SetupScrolling         0x27
#define OLED_CMD_DeactivateScrolling    0x2E
#define OLED_CMD_ActivateScrolling      0x2F

#else

#ifdef SSD1306 // SSD1306 monochrome displays

#define OLED_SetSegmentRemap    0xA1
#define OLED_NormalDisplay      0xA6
#define OLED_InverseDisplay     0xA7
#define OLED_SetMuliplexRatio   0xA8

#define OLED_DisplayOff         0xAE
#define OLED_DisplayOn          0xAF

#define OLED_SetContrast        0x81

#define OLED_ActivateScroll     0x2F
#define OLED_DeactivateScroll   0x2E

#define OLED_SetComOutputScanDirectionNormal    0xC0
#define OLED_SetComOutputScanDirectionRemap     0xC8

#define OLED_SetDisplayOffset                   0xD3
#define OLED_SetComPins                         0xDA
#define OLED_SetVcomhDeselectLevel              0xDB
#define OLED_SetDisplayClockDiv                 0xD5
#define OLED_SetPrechargePeriod                 0xD9
#define OLED_SetLowerColumnStartAddress         0x00
#define OLED_SetHigherColumnStartAddress        0x10
#define OLED_SetStartLine                       0x40
#define OLED_SetPageStartAddress                0xB0


#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_MAXROWS 7

//#define OLED_Entire_Display_Resume 0xA4
//#define OLED_Entire_Display_On     0xA5

#define OLED_ExternalVcc            0x10
#define OLED_Internal_Vcc           0x02
#define OLED_SetPageAddress         0x22
#define OLED_SetColumnAddress       0x21
#define OLED_SetColumnAddressMSB    0x00
#define OLED_SetColumnAddressLSB    0x7F

#define OLED_SetMemoryMode                      0x20
#define OLED_ChargePumpSetting                  0x8D

// Scrolling #defines
#define OLED_SET_VERTICAL_SCROLL_AREA              0xA3
#define OLED_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL  0x29
#define OLED_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL   0x2A

#else // SH1106 monochrome displays

#define OLED_WIDTH 128
#define OLED_HEIGHT 64
#define OLED_MAXROWS 7

#define OLED_ChargePumpEnable   0xAD
#define OLED_ExternalVcc        0x8B

#endif
#endif

extern void OLED_DriverInit (driver_t *display);

void OLED_Init (void);
void OLED_OnOff (bool on);
void OLED_ClearScreen (void);
void OLED_DrawChar (OLEDFont *font, uint8_t row, uint8_t column, uint8_t data);
void OLED_DrawString (OLEDFont *font, uint8_t row, uint8_t column, const char *string);
void OLED_DrawPixel (uint16_t x, uint16_t y);
void OLED_DrawHLine (uint16_t xStart, uint16_t xStop, uint16_t y);
void OLED_DrawVLine (uint16_t x, uint16_t yStart, uint16_t yStop);
void OLED_DrawImage (const uint8_t* image, uint8_t row, uint8_t column);
void OLED_DrawCircle (uint16_t x,uint16_t y, uint16_t radius);

#ifdef OLED_COLOR
void OLED_SetColor (RGBColor_t color);
void OLED_SetBGColor (RGBColor_t color);
#else
void OLED_Contrast (uint8_t d);
void OLED_Invert (bool invert);
#endif

#if defined(OLED_COLOR) || defined(FRAMEBUFFER)
void OLED_DrawLine (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void OLED_DrawRectangle (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
void OLED_DrawFilledRectangle (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd);
#endif

#ifdef SEVENSEGMENT
void OLED_Set7SegMode (sevensegmode_t mode);
void OLED_Draw7Seg (int val, int dp);
#endif

#ifdef OLED_NEWFONTS
uint8_t OLEDF_GetFontWidth (Font *font);
uint8_t OLEDF_GetFontHeight (Font *font);
uint8_t OLEDF_GetCharWidth (Font *font, char c);
uint16_t OLEDF_GetStringWidth (Font *font, const char *string);
uint8_t OLEDF_DrawChar (Font *font, uint16_t x, uint16_t y, char c, bool opaque);
void OLEDF_DrawString (Font *font, uint16_t x, uint16_t y, const char *string, bool opaque);
#endif

#endif
