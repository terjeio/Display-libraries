/******************************************************************************
 * File Name   :  OLED_SSD1306.c
 * Author      :  43oh - MSP430 News Projects and Forums.
               :  (http://www.43oh.com)
 * Description :  Lowlevel driver for OLED SSD1306
 * Date        :  October 21, 2011.
 
               : Terje Io, 2018-03-03
               : Added optional framebuffer support and
               : support for Reddmann font format

               : Terje Io, 2018-08-26
               : Refactored and added support for 64x96 SSD1331 color display
 *****************************************************************************/

#include "OLED.h"

#define PARAM_SWAP(a,b) { register uint16_t t=(a);(a)=(b);(b)=t; }

#ifdef SEVENSEGMENT

#include "Font7Seg40x32.h"

static struct {
    uint8_t digit[4];
    bool negative;
    uint8_t dp;
    sevensegmode_t mode;
} sevenseg;

#endif

static driver_t driver;

#ifdef OLED_COLOR
static RGBColor_t fgColor, bgColor;
static colorRGB565 fgColor565, bgColor565;
#endif

#ifdef FRAMEBUFFER
static uint8_t frameBuffer[OLED_WIDTH * OLED_HEIGHT / 8];
#endif

#ifdef SSD1331

static const uint8_t OLED_init[] = {
        OLED_DisplayOff,
        OLED_SetReMap,
        0x72, // RGB
        OLED_StartLine,
        0x00,
        OLED_DisplayOffset,
        0x00,
        OLED_NormalDisplay,
        OLED_SetMuliplexRatio,
        0x3F,
        OLED_SetMaster,
        0x8E,
        OLED_PowerMode,
        0x0B,
        OLED_PreCharge,
        0x31,
        OLED_ClockDiv,
        0xF0,
        OLED_PreChargeA,
        0x64,
        OLED_PreChargeB,
        0x78,
        OLED_PreChargeC,
        0x64,
        OLED_PreChargeLevel,
        0x3A,
        OLED_VCOMH,
        0x3E,
        OLED_SetPhasePeriodAdjustment,
        0x31,
        OLED_DisplayClockDiv,
        0xF0,
        OLED_MasterCurrent,
        0x06,
        OLED_ContrastA,
        0x91,
        OLED_ContrastB,
        0x50,
        OLED_ContrastC,
        0x7D
};

#else

#ifdef SSD1306

static const uint8_t OLED_init[] = {
        OLED_DisplayOff,
        OLED_SetMuliplexRatio,
        0x3F,
        OLED_SetLowerColumnStartAddress,
        OLED_SetHigherColumnStartAddress,
        OLED_SetStartLine,
        OLED_SetSegmentRemap,
        OLED_NormalDisplay,
        OLED_SetDisplayOffset,
        0x00,
        OLED_SetDisplayClockDiv,
        0x80,
        OLED_SetPrechargePeriod,
        0xf1,
        OLED_SetComPins,
        0x12,
        OLED_SetVcomhDeselectLevel,
        0x40,
        OLED_ChargePumpSetting,
        0x14,
        OLED_SetContrast,
        0xCF
};

#else // SH1106

static const uint8_t OLED_init[] = {
    OLED_DisplayOff,
    OLED_SetMuliplexRatio,
    0x3F,
    OLED_SetLowerColumnStartAddress,        /*set lower column address*/
    OLED_SetHigherColumnStartAddress,       /*set higher column address*/
    OLED_SetStartLine,                      /*set display start line*/
    OLED_SetPageStartAddress,               /*set page address*/
    OLED_SetSegmentRemap,                   /*set segment remap*/
    OLED_NormalDisplay,                     /*normal / reverse*/
    OLED_ChargePumpEnable,                  /*set charge pump enable*/
    OLED_ExternalVcc,                       /*external VCC   */
    0x30,                                   /*0X30---0X33  set VPP   9V liangdu!!!!*/
    OLED_SetComOutputScanDirectionNormal,   /*Com scan direction*/
    OLED_SetDisplayOffset,                  /*set display offset*/
    0x00,                                   /*? 0x20 */
    OLED_SetDisplayClockDiv,                /*set osc division*/
    0x80,
    OLED_SetPrechargePeriod,                /*set pre-charge period*/
    0x1F,                                   /*? 0x22*/
    OLED_SetComPins,                        /*set COM pins*/
    0x12,
    OLED_SetVcomhDeselectLevel,             /*set vcomh*/
    0x40,
    OLED_SetContrast,
    0xCF
};

#endif
#endif

static inline void sendCommand (const uint8_t data)
{
    driver.writeCommand(&data, 1);
}

static inline void sendData (const uint8_t data)
{
    driver.writeData(&data, 1);
}

void OLED_Init (void)
{
    driver.screen.Height = OLED_HEIGHT;
    driver.screen.Width = OLED_WIDTH;
    driver.io = SPI;

    OLED_DriverInit(&driver);

    driver.delayms(300);

    driver.writeCommand(OLED_init, sizeof(OLED_init));

    OLED_ClearScreen();

#ifdef SEVENSEGMENT
    sevenseg.mode = ThreeAndHAlfDigit;
#endif

    driver.delayms(100);

    sendCommand(OLED_DisplayOn);

    driver.delayms(150);

#ifdef OLED_COLOR
    OLED_SetColor((RGBColor_t)0xFFFFFFFF);
    OLED_SetBGColor((RGBColor_t)0xFF000000);
#endif
}

#ifdef OLED_COLOR

void OLED_SetColor (RGBColor_t color)
{
    fgColor.R = (color.R >> 3) << 1;
    fgColor.G = color.G >> 2;
    fgColor.B = (color.B >> 3) << 1;
    fgColor565.value = DPYCOLORTRANSLATE(color.value);
}

void OLED_SetBGColor (RGBColor_t color)
{
    bgColor.R = (color.R >> 3) << 1;
    bgColor.G = color.G >> 2;
    bgColor.B = (color.B >> 3) << 1;
    bgColor565.value = DPYCOLORTRANSLATE(color.value);
}

static void setAddress (uint8_t page, uint8_t column)
{
    static uint8_t address[] = {OLED_SetColumnStartAddress, 0, OLED_WIDTH - 1, OLED_SetRowStartAddress, 0, OLED_HEIGHT - 1};

    address[1] = column;
    address[4] = page;

    driver.writeCommand(address, sizeof(address));
}

#ifdef OLED_CMD_DrawLine

void OLED_DrawLine (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    struct OLED_LineData {
        uint8_t cmd;
        uint8_t x0;
        uint8_t y0;
        uint8_t x1;
        uint8_t y1;
        OLED_Color_t color;
    } line;

    line.cmd = OLED_CMD_DrawLine;
    line.x0 = (uint8_t)xStart;
    line.y0 = (uint8_t)yStart;
    line.x1 = (uint8_t)xEnd;
    line.y1 = (uint8_t)yEnd;
    line.color.R = fgColor.R;
    line.color.G = fgColor.G;
    line.color.B = fgColor.B;

    driver.writeCommand((uint8_t *)&line, sizeof(line));
}

void OLED_DrawHLine(uint16_t xStart, uint16_t xStop, uint16_t y)
{
    OLED_DrawLine(xStart, y, xStop, y);
}

void OLED_DrawVLine(uint16_t x, uint16_t yStart, uint16_t yStop)
{
    OLED_DrawLine(x, yStart, x, yStop);
}

#endif

void OLED_DrawRectangle (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
#ifdef OLED_CMD_DrawRectangle

    struct OLED_RectangleData {
        uint8_t fillCmd;
        uint8_t fillMode;
        uint8_t cmd;
        uint8_t x0;
        uint8_t y0;
        uint8_t x1;
        uint8_t y1;
        OLED_Color_t color;
        OLED_Color_t fillColor;
    } rectangle;

    rectangle.fillCmd     = OLED_CMD_FillEnable;
    rectangle.fillMode    = 0;
    rectangle.cmd         = OLED_CMD_DrawRectangle;
    rectangle.x0          = (uint8_t)xStart;
    rectangle.y0          = (uint8_t)yStart;
    rectangle.x1          = (uint8_t)xEnd;
    rectangle.y1          = (uint8_t)yEnd;
    rectangle.color.R     = fgColor.R;
    rectangle.color.G     = fgColor.G;
    rectangle.color.B     = fgColor.B;

    driver.writeCommand((uint8_t *)&rectangle, sizeof(rectangle));

#else
    OLED_DrawHLine(xStart, xEnd, yStart);
    OLED_DrawHLine(xStart, xEnd, yEnd);
    OLED_DrawVLine(xStart, yStart, yEnd);
    OLED_DrawVLine(xEnd, yStart, yEnd);
#endif
}

void OLED_DrawFilledRectangle (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
#ifdef OLED_CMD_DrawRectangle

    struct OLED_RectangleData {
        uint8_t fillCmd;
        uint8_t fillMode;
        uint8_t cmd;
        uint8_t x0;
        uint8_t y0;
        uint8_t x1;
        uint8_t y1;
        OLED_Color_t color;
        OLED_Color_t fillColor;
    } rectangle;

    rectangle.fillCmd     = OLED_CMD_FillEnable;
    rectangle.fillMode    = 1;
    rectangle.cmd         = OLED_CMD_DrawRectangle;
    rectangle.x0          = (uint8_t)xStart;
    rectangle.y0          = (uint8_t)yStart;
    rectangle.x1          = (uint8_t)xEnd;
    rectangle.y1          = (uint8_t)yEnd;
    rectangle.color.R     = fgColor.R;
    rectangle.color.G     = fgColor.G;
    rectangle.color.B     = fgColor.B;
    rectangle.fillColor.R = bgColor.R;
    rectangle.fillColor.G = bgColor.G;
    rectangle.fillColor.B = bgColor.B;

    driver.writeCommand((uint8_t *)&rectangle, sizeof(rectangle));

    driver.delayms(1); // Wait for fill to complete

#else
    OLED_DrawHLine(xStart, xEnd, yStart);
    OLED_DrawHLine(xStart, xEnd, yEnd);
    OLED_DrawVLine(xStart, yStart, yEnd);
    OLED_DrawVLine(xEnd, yStart, yEnd);
#endif
}

#else // Monochrome

#ifdef FRAMEBUFFER
static uint8_t *setAddress (uint8_t page, uint8_t column)
#else
static void setAddress (uint8_t page, uint8_t column)
#endif
{
    if(page > OLED_MAXROWS)
        page = OLED_MAXROWS;

#ifdef SSD1306

    uint8_t columnAddress[] = { OLED_SetColumnAddress, OLED_SetColumnAddressMSB | column, OLED_SetColumnAddressLSB };

    sendCommand(OLED_SetPageStartAddress | (OLED_MAXROWS - page));
    driver.writeCommand(columnAddress, 3);

#else
    column += 2;

    if(column > OLED_WIDTH)
        column = OLED_WIDTH;

    sendCommand(OLED_SetPageStartAddress | (OLED_MAXROWS - page));
    sendCommand(OLED_SetLowerColumnStartAddress + (column & 0x0F));
    sendCommand(OLED_SetHigherColumnStartAddress + (column >> 4));
#endif

#ifdef FRAMEBUFFER
    return frameBuffer + (page * OLED_WIDTH) + column;
#endif
}

void OLED_Contrast (uint8_t contrast)
{
    uint8_t buf[2];

    buf[0] = OLED_SetContrast;
    buf[1] = contrast;
    driver.writeCommand(buf, 2);
}

void OLED_Invert (bool invert)
{
    // 0 = normal, 1 = inverted

    sendCommand(OLED_NormalDisplay + (invert ? 1 : 0));
}

void OLED_DrawLine (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    if (yStart == yEnd) // horizontal?
        OLED_DrawHLine(xStart, xEnd, yStart);

    else if (xStart == xEnd) // vertical?
        OLED_DrawVLine(xStart, yStart, yEnd);

    else { // angled

        int16_t dx, dy, sx, sy;

        if (xStart < xEnd) {
            sx = 1;
            dx = xEnd - xStart;
        } else {
            sx = -1;
            dx = xStart - xEnd;
        }

        if (yStart < yEnd) {
            sy = 1;
            dy = yEnd - yStart;
        } else {
            sy = -1;
            dy = yStart - yEnd;
        }

        int16_t e1 = dx - dy, e2;

        while (1) {
            OLED_DrawPixel(xStart, yStart);
            if (xStart == xEnd && yStart == yEnd)
                break;
            e2 = e1 << 1;
            if (e2 > -dy) {
                e1 = e1 - dy;
                xStart = xStart + sx;
            }
            if (e2 < dx) {
                e1 = e1 + dx;
                yStart = yStart + sy;
            }
        }
    }
}


void OLED_DrawHLine(uint16_t xStart, uint16_t xStop, uint16_t y)
{
    uint8_t temp;

    if(xStart > xStop)
        PARAM_SWAP(xStart, xStop);

    if((y > OLED_HEIGHT - 1) || (xStart > OLED_WIDTH - 1))
        return;

    if(xStop > OLED_WIDTH - 1)
        xStop = OLED_WIDTH - 1;

  #ifdef FRAMEBUFFER
    uint8_t *data = setAddress(y >> 3, xStart);
  #else
    setAddress(y >> 3, xStart);
  #endif

    temp = 0x80 >> (y & 0x07);

    while(xStart <= xStop) {
      #ifdef FRAMEBUFFER
        *data |= temp;
        sendData(*data++);
      #else
        sendData(temp);
      #endif
        xStart++;
    }
}

void OLED_DrawVLine(uint16_t x, uint16_t yStart, uint16_t yStop)
{
    uint8_t temp, page1, page2, pageStart;

    if((x > OLED_WIDTH - 1) || (yStart > OLED_HEIGHT - 1))
        return;

    if(yStop > OLED_HEIGHT - 1)
        yStop = OLED_HEIGHT - 1;

    if(yStart > yStop)
        PARAM_SWAP(yStart, yStop);

    page1 = yStart >> 3;
    page2 = yStop >> 3;
    pageStart = yStart & 0x07;
    temp = pageStart > 0 ? (0xFF00 >> pageStart) ^ 0xFF : 0xFF;

    if (page1 != page2) {
      #ifdef FRAMEBUFFER
        uint8_t *data = setAddress(page1, x);
        *data |= temp;
        sendData(*data);
      #else
        setAddress(page1, x);
        sendData(temp);
      #endif

        page1++;
        temp = 0xFF;

        while (page1 < page2) {
          #ifdef FRAMEBUFFER
            uint8_t *data = setAddress(page1, x);
            *data = temp;
          #else
            setAddress(page1, x);
          #endif
            sendData(temp);
            page1++;
        }
    }

    temp &= 0xFF << (7 - (yStop & 0x07));

  #ifdef FRAMEBUFFER
    uint8_t *data = setAddress(page2, x);
    *data |= temp;
    sendData(*data);
  #else
    setAddress(page2, x);
    sendData(temp);
  #endif
}

void OLED_DrawRectangle (uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd)
{
    OLED_DrawHLine(xStart, xEnd, yStart);
    OLED_DrawHLine(xStart, xEnd, yEnd);
    OLED_DrawVLine(xStart, yStart, yEnd);
    OLED_DrawVLine(xEnd, yStart, yEnd);
}


// from TI_DogsLCD_HAL
void OLED_DrawImage (const uint8_t* image, uint8_t row, uint8_t column)
{
    char a, height, width;

    width = image[0];
    height = image[1];

    for(a = 0; a < height; a++) {
        setAddress(row + a, column);
        driver.writeData(image + 2 + a * width, width);
    }
}


#endif

void OLED_OnOff (bool on)
{
    sendCommand(on ? OLED_DisplayOn : OLED_DisplayOff);
}

void OLED_ClearScreen (void)
{
#ifdef SSD1331
    uint8_t data[] = {OLED_CMD_ClearWindow, 0, 0, OLED_WIDTH - 1, OLED_HEIGHT - 1};
    driver.writeCommand(data, sizeof(data));
    driver.delayms(1);
#else
    int16_t row = OLED_MAXROWS + 1, col;

    while(row--) {
        setAddress(row, 0);
        col = OLED_WIDTH + 1;
        while(col--)
            sendData(0x00);
    }
#endif

#ifdef SEVENSEGMENT
    sevenseg.digit[0] = 0x0A;
    sevenseg.digit[1] = 0x0A;
    sevenseg.digit[2] = 0x0A;
    sevenseg.digit[3] = 0x0A;
    sevenseg.dp       = 0;
    sevenseg.negative = false;
#endif

  #ifdef FRAMEBUFFER
    row = sizeof(frameBuffer) - 1;
    frameBuffer[0] = 0x55;
    do {
        frameBuffer[row] = 0;
    } while(row--);
  #endif
}

void OLED_DrawPixel (uint16_t x, uint16_t y)
{
    if(x > OLED_WIDTH - 1 || y > OLED_HEIGHT - 1)
        return;

#ifdef SSD1331
    setAddress(y, x);
    driver.writeData((uint8_t *)&fgColor565.value, sizeof(fgColor565));
#else
  #ifdef FRAMEBUFFER
    uint8_t *data = setAddress(y >> 3, x);
    *data |= (0x80 >> (y & 0x07));
    sendData(*data);
  #else
    setAddress(y >> 3, x);
    sendData(0x80 >> (y & 0x07));
  #endif
#endif
}

/* Function     : OLED_DrawCircle (uint16_t x, uint16_t, int r)
 * Description  : draw circle at x,y of radius r
 * Input        : x,y, radius
 * Output       : display circle
 *
 * NOTE: requires framebuffer to work correctly
 */

void OLED_DrawCircle (uint16_t x, uint16_t y, uint16_t radius)
{
    register int xx = -radius;
    register int yy = 0;
    register int e = 2 - (2 * radius);
    do {
        OLED_DrawPixel(x - xx, y + yy);
        OLED_DrawPixel(x - yy, y - xx);
        OLED_DrawPixel(x + xx, y - yy);
        OLED_DrawPixel(x + yy, y + xx);
        if(e >  xx) e += ((++xx << 1) + 1);
        if(e <= yy) e += ((++yy << 1) + 1);
    } while (xx < 0);
}

#if defined(FRAMEBUFFER) || defined(OLED_NEWFONTS)
static void clearPixel (uint16_t x, uint16_t y)
{
    if(x > OLED_WIDTH - 1 || y > OLED_HEIGHT - 1)
        return;

#ifdef SSD1331
    setAddress(y, x);
    driver.writeData((uint8_t *)&bgColor565.value, sizeof(bgColor565));
#else
    uint8_t *data = setAddress(y >> 3, x);
    *data &= ~(0x80 >> (y & 0x07));
    sendData(*data);
#endif
}
#endif


void OLED_DrawChar (OLEDFont *font, uint8_t row, uint8_t column, uint8_t c)
{
    uint8_t *ptr, columns = font->width, rows = font->rows;
  #ifdef FRAMEBUFFER
    uint8_t i;
  #endif

    ptr = &font->charData[((c < 32 || c > 129 ? '.' : c) - font->offset) * rows * columns];
#ifdef SSD1331
    uint8_t x, y, i;
    if(row <= OLED_HEIGHT && column <= (OLED_WIDTH - columns)) {
        while(rows--) {
            y = row + rows;
            for(x = 0; x < columns; x++) {
                uint8_t pixels = *ptr++;
                for(i = 0; i < 8; i++) {
                    if(pixels & 0x01)
                        OLED_DrawPixel(column + x, y - i);
                    else
                        clearPixel(column + x, y - i);
                    pixels >>= 1;
                }
            }
        }
    }
#else
    if(row <= OLED_MAXROWS && column <= (OLED_WIDTH - columns)) {
        while(rows--) {
          #ifdef FRAMEBUFFER
            uint8_t *data = setAddress(row++, column);
            for(i = 0; i < columns; i++)
                data[i] = *ptr++;
            driver.writeData(data, columns);
          #else
            setAddress(row++, column);
            driver.writeData(ptr, columns);
            ptr += columns;
          #endif
        }
    }
#endif
}

void OLED_DrawString(OLEDFont *font, uint8_t row, uint8_t column, const char *string)
{
    uint8_t c;

    while ((c = *string++) != 0 && (column + font->width) < OLED_WIDTH) {
        OLED_DrawChar(font, row, column, c);
        column += font->width;
    }
}

#ifdef SEVENSEGMENT

void OLED_Set7SegMode (sevensegmode_t mode)
{
    sevenseg.mode = mode;
}

void OLED_Draw7Seg (int val, int dp) {

    uint8_t data[4], i, row = 3, col = 3;
    bool negative;

    if((negative = (val < 0)))
        val = - val;

    for(i = 0; i < 4; i++) {
        data[i] = val % 10;
        val = val / 10;
    }

    if(dp < 5) {
        i = 3;
        while(data[i] == 0 && (!dp || dp != 4 - i)) {
            data[i--] = 0x0A;
        }
    }

    if(negative != sevenseg.negative) {
        sevenseg.negative = negative;
        OLED_DrawChar((OLEDFont *)FONT7SEG8x40, row, col, negative ? '4' : '0');
        OLED_DrawChar((OLEDFont *)FONT7SEG8x40, row, col + 8, negative ? '3' : '0');
    }

    col += 96;

    for(i = 0; i < 4; i++) {
        if(data[i] != sevenseg.digit[i]) {
            if(i == 3 && sevenseg.mode == ThreeAndHAlfDigit)
                OLED_DrawChar((OLEDFont *)FONT7SEG8x40, row, col + 16, data[i] == 1 ? '1' : '0');
            else
                OLED_DrawChar((OLEDFont *)FONT7SEG26x40, row, col, data[i] + '0');
            sevenseg.digit[i] = data[i];
        }
        col -= 32;
    }

    if(dp != sevenseg.dp) {
        if(sevenseg.dp || dp == 6)
            OLED_DrawChar((OLEDFont *)FONT7SEG8x40, row, col + 25 + ((sevenseg.dp == 5 ? 2 : sevenseg.dp) << 5), '0');
        if(dp && dp != 6)
            OLED_DrawChar((OLEDFont *)FONT7SEG8x40, row, col + 25 + ((dp == 5 ? 2 : dp) << 5), dp == 5 ? '5' : '2');
        sevenseg.dp = dp;
    }
}

#endif

#ifdef OLED_NEWFONTS

/*
 * New font functions by Terje Io - 2015-08-02
 *
 * For fonts created by FontEditor written by H. Reddmann
 *
 * NOTE: Only available when using a framebuffer
 *
 */

static uint16_t getoffset (Font *font, unsigned char c)
{
    uint16_t offset = 0;

    if(c > font->firstChar) {
        c -= font->firstChar;
        while(c--)
            offset += font->charWidths[c];
    }

    return offset;
}

static uint8_t getSpaceWidth (Font *font)
{
    return ('0' < font->firstChar || '0' > font->lastChar ? font->width >> 2 : font->charWidths['0' - font->firstChar]) + 2;
}

uint8_t OLEDF_GetFontWidth (Font *font)
{
    return font->width;
}

uint8_t OLEDF_GetFontHeight (Font *font)
{
    return font->height;
}

uint8_t OLEDF_GetCharWidth (Font *font, char c)
{
    return c != ' ' && (c < font->firstChar || c > font->lastChar) ? 0 : (c == ' ' || font->charWidths[c - font->firstChar] == 0 ? getSpaceWidth(font) : font->charWidths[c - font->firstChar] + 2);
}

uint16_t OLEDF_GetStringWidth (Font *font, const char *string)
{

    char c;
    uint16_t width = 0;

    while((c = *string++))
        width += OLEDF_GetCharWidth(font, c);

    return width;
}

uint8_t OLEDF_DrawChar (Font *font, uint16_t x, uint16_t y, char c, bool opaque)
{
    uint_fast8_t width = OLEDF_GetCharWidth(font, c);
    uint8_t *fontData = font->charWidths + font->lastChar - font->firstChar + 1;
    uint_fast16_t fontRow, fontColumn, bitOffset, dataIndex, preShift;
    uint64_t pixels;
    bool paintSpace;

    if(width) {

        bitOffset = getoffset(font, c) * font->height;
        dataIndex = bitOffset >> 3;
        preShift = bitOffset - (dataIndex << 3);
        fontColumn = width;
        width -= 2;

        paintSpace = c == ' ' || !font->charWidths[c - font->firstChar];

        if(!paintSpace || opaque) {

            while(fontColumn--) {

                fontRow = font->height;

                if(!((fontColumn == 0) || (fontColumn > width)) && !paintSpace) {

#ifdef FONTREAD_BYTEWISE
                    pixels = (uint64_t)fontData[dataIndex];
                    pixels |= (uint64_t)fontData[dataIndex + 1] << 8;
                    pixels |= (uint64_t)fontData[dataIndex + 2] << 16;
                    pixels |= (uint64_t)fontData[dataIndex + 3] << 24;
                    pixels |= (uint64_t)fontData[dataIndex + 4] << 32;
                    pixels |= (uint64_t)fontData[dataIndex + 5] << 40;
                    pixels |= (uint64_t)fontData[dataIndex + 6] << 48;
                    pixels |= (uint64_t)fontData[dataIndex + 7] << 56;
#else
                    pixels = ((uint64_t)*((uint32_t *)&fontData[dataIndex + 4])) << 32;
                    pixels |= *((uint32_t *)&fontData[dataIndex]);
#endif

                    pixels >>= preShift;

                    while(fontRow--) {

                        if(pixels & 0x01) {
                           OLED_DrawPixel(x, y - fontRow);
                        } else if(opaque) {
                            clearPixel(x, y - fontRow);
                        }
                        pixels >>= 1;
                    }

                    bitOffset += font->height;
                    dataIndex = bitOffset >> 3;
                    preShift = bitOffset - (dataIndex << 3);

                } else if(opaque) {
                    while(fontRow--) {
                        clearPixel(x, y - fontRow);
                    }
                }
                x++;
            }
        }
    }

    return width == 0 ? 0 : width + 2;
}

void OLEDF_DrawString (Font *font, uint16_t x, uint16_t y, const char *string, bool opaque)
{
    char c;

    while((c = *string++)) {
        x += OLEDF_DrawChar(font, x, y, c, opaque);
    }
}

#endif
