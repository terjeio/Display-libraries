/*
 * lcd.c
 *
 *  Created on: Jul 12, 2013
 *      Author: RobG
 */

/*
 * Modified by Terje Io to use function pointers for driver access++
 */

#include "lcd.h"
#include "config.h"
#include "Touch/touch.h"

extern void LCD_DriverInit (lcd_driver_t *driver);

static lcd_driver_t *driver;

static void setArea(uint16_t xStart, uint16_t yStart, uint16_t xEnd, uint16_t yEnd) {

    driver->writeCommand(CASETP);

	driver->writeData(xStart >> 8);
	driver->writeData(xStart);

	driver->writeData(xEnd >> 8);
	driver->writeData(xEnd);

	driver->writeCommand(PASETP);

	driver->writeData(yStart >> 8);
	driver->writeData(yStart);

	driver->writeData(yEnd >> 8);
	driver->writeData(yEnd);

	driver->writeCommand(RAMWRP);
	// data to follow
}

////////////////////////////////////
// gamma, lut, and other inits
////////////////////////////////////

/////////////////////////////////////////////////
// ILI9340 based display
/////////////////////////////////////////////////
#ifdef ILI9340

static void displayState (bool on) {
    driver->writeCommand(on ? DISPON : DISPOFF);
}

static void changeOrientation (uint8_t orientation) {

    driver->writeCommand(ILIMAC);

	switch (orientation) {
        case Orientation_Horizontal:
            driver->writeData(0xE8);
            break;
        case Orientation_VerticalRotated:
            driver->writeData(0x88);
            break;
        case Orientation_HorizontalRotated:
            driver->writeData(0x28);
            break;
        default:
            driver->writeData(0x48);
	}
}

uint32_t getGolor (uint32_t RGBcolor)
{
	return ((((RGBcolor) & 0x00F80000) >> 8) | (((RGBcolor) & 0x0000FC00) >> 5) | (((RGBcolor) & 0x000000F8) >> 3));
}

void LCD_DisplayInit (lcd_driver_t *drv)
{
    driver = drv;
	driver->display.Width  = SHORT_EDGE_PIXELS;
	driver->display.Height = LONG_EDGE_PIXELS;
	driver->display.Orientation = Orientation_Vertical;
	driver->setArea = setArea;
	driver->displayOn = displayState;
	driver->changeOrientation = changeOrientation;
    driver->touchIRQHandler = 0;

    LCD_DriverInit(driver);
#ifdef TOUCH_MAXSAMPLES
    TOUCH_Init(driver);
#endif
	driver->delayms(20); // wait

	driver->writeCommand(PWRCTRLA);
	driver->writeData(0x39);
	driver->writeData(0x2C);
	driver->writeData(0x00);
	driver->writeData(0x34);
	driver->writeData(0x02);

	driver->writeCommand(PWRCTRLB);
	driver->writeData(0x00);
	driver->writeData(0XC1);
	driver->writeData(0X30);

	driver->writeCommand(DTCTRLA1);
	driver->writeData(0x85);
	driver->writeData(0x00);
	driver->writeData(0x78);

	driver->writeCommand(DTCTRLB);
	driver->writeData(0x00);
	driver->writeData(0x00);

	driver->writeCommand(POSC);
	driver->writeData(0x64);
	driver->writeData(0x03);
	driver->writeData(0X12);
	driver->writeData(0X81);

	driver->writeCommand(PRC);
	driver->writeData(0x20);

	driver->writeCommand(ILIPC1);
	driver->writeData(0x23);
	driver->writeCommand(ILIPC2);
	driver->writeData(0x10);
	driver->writeCommand(ILIVC1);
	driver->writeData(0x3e);
	driver->writeData(0x28);
	driver->writeCommand(ILIVC2);
	driver->writeData(0x86);

	setOrientation(driver->display.Orientation);

	driver->writeCommand(COLMOD);
	driver->writeData(0x55);

	driver->writeCommand(ILIFCNM);
	driver->writeData(0x00);
	driver->writeData(0x18);

	driver->writeCommand(ILIDFC);
	driver->writeData(0x08);
	driver->writeData(0x82);
	driver->writeData(0x27);

	driver->writeCommand(ILIGFD);
	driver->writeData(0x00);
	driver->writeCommand(ILIGS);
	driver->writeData(0x01);

	driver->writeCommand(ILIPGC);
	const unsigned char gamma1[] = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
	unsigned char c = 0;
	while (c < 16) {
	    driver->writeData(gamma1[c]);
		c++;
	}

	driver->writeCommand(ILINGC);
	const unsigned char gamma2[] = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
	c = 0;
	while (c < 16) {
	    driver->writeData(gamma2[c]);
		c++;
	}

	driver->writeCommand(SLEEPOUT);
	driver->delayms(120);
	driver->writeCommand(DISPON);
    driver->delayms(120);
/*
    driver->writeCommand(WRCABC);
    driver->writeData(0x01);

    driver->writeCommand(WRDISBV);
    driver->writeData(0x80);

    driver->writeCommand(WRCTRLD);
    driver->writeData(0x28);
*/
	driver->writeCommand(RAMWRP);
/*
	uint8_t id[5];

    driver->readDataBegin(0x0A);
    id[0] = driver->readData();
    id[1] = driver->readData();
//    id[2] = driver->readData();
//    id[3] = driver->readData();
//    id[4] = driver->readData();
    driver->readDataEnd();
*/
}

uint16_t readID()
{
    uint16_t id;

    driver->writeCommand(GER4SPI);
    driver->writeData(0x12);
    driver->readDataBegin(RDID4);
    id = driver->readData() << 8;
    driver->readDataEnd();

    driver->writeCommand(GER4SPI);
    driver->writeData(0x13);
    driver->readDataBegin(RDID4);
    id |= driver->readData();
    driver->readDataEnd();

    return id;
}

#endif
