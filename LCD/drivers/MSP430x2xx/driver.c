/*
 * driver.c - LCD panel driver for MSP430F5529
 *
 * v1.0.0 / 2017-12-18 / ©Io Engineering / Terje
 *
 */

 /* Copyright (c) 2017, Io Engineering
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * · Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * · Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * · Neither the name of the copyright holder nor the names of its contributors may
 *   be used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include "driver.h"
#include "lcd/lcd.h"
#include "lcd/touch/quickselect.h"

static volatile uint16_t ms_delay;
static lcd_driver_t *driver;

static void msdelay (uint16_t ms)
{
    ms_delay = ms;
    SYSTICK_TIMER_CTL |= TACLR|MC0;
    while (ms_delay);
}

typedef enum {
    SPI_Idle = 0,
    SPI_TX,
    SPI_TX_Deselect,
    SPI_TX_Repeat,
    SPI_RX,
} spi_mode_t;

typedef struct {
    uint32_t count;
    uint8_t *data;
    uint32_t txdata;
    uint8_t rxdata;
    volatile spi_mode_t state;
} spi_data_t;

static spi_data_t spi;
/*
static void writeWord (uint32_t word)
{
    while(spi.state);

    LCD_SELECT;

    spi.state = true;
    spi.txdata = color.highByte;
    LCD_SPI_IE |= UCTXIE;                     // Enable TX interrupt

    while(spi.state);
    spi.state = true;
    spi.txdata = color.lowByte;
    spi.deselect = LCD_CS_PIN|LCD_DC_PIN;
    LCD_SPI_IE |= UCTXIE;                     // Enable TX interrupt
}
*/

inline static void dpi_clk (uint16_t div)
{
    LCD_SPI_CTL1 |= UCSWRST;
    LCD_SPI_BR0 = div;
    LCD_SPI_CTL1 &= ~UCSWRST;
}

inline static uint8_t readByte (uint8_t cmd)
{
// TODO: change to not use interrupts?, have to block for result
    spi.state = SPI_RX;
    spi.txdata = cmd;
    spi.count = 1;
    spi.data = (uint8_t *)&spi.txdata;
    LCD_SPI_IFG = UCTXIFG;
    LCD_SPI_IE = UCRXIE|UCTXIE;                     // Enable TX interrupt
    while(spi.state);

    return spi.rxdata;
}

static void writePixels (uint16_t *pixels, uint32_t length)
{
#if SPI_MODE == WAIT
    uint8_t *data = (uint8_t *)pixels;
    length <<= 1;
    LCD_SELECT;
    while(length--) {
        LCD_SPI_TXBUF = *data++;
        while(LCD_SPI_STAT & UCBUSY);
    }
    LCD_DESELECT;
#else
    while(spi.state);

    LCD_SELECT;

    spi.state = SPI_TX_Deselect;
    spi.count = length << 1;
    spi.data = (uint8_t *)pixels;
    LCD_SPI_IE = UCTXIE;                     // Enable TX interrupt

    while(spi.state); // TODO: add output buffering in TJPGD (or output fn) to remove this wait?
#endif
}

static void writePixel (colorRGB565 color, uint32_t count)
{
#if SPI_MODE == WAIT
    LCD_SELECT;

    while(count--) {
        while(!(LCD_SPI_IFG & UCTXIFG)); // wait for buffer

        LCD_SPI_TXBUF = color.lowByte;
        while(LCD_SPI_STAT & UCBUSY);
        while(!(LCD_SPI_IFG & UCTXIFG)); // wait for buffer

        LCD_SPI_TXBUF = color.highByte;
        while(LCD_SPI_STAT & UCBUSY);
    }
    __delay_cycles(32);

    LCD_DESELECT;
#else
    while(spi.state);

    LCD_SELECT;

    spi.state = count > 1 ? SPI_TX_Repeat : SPI_TX_Deselect;
    spi.count = count * 2;
    spi.txdata = color.value;
    spi.data = (uint8_t *)&spi.txdata;
    LCD_SPI_IE = UCTXIE;                     // Enable TX interrupt
#endif
}

// code duplication, but we are saving clock cycles by not passing dataCommand
static void writeData(uint8_t data /*, uint16_t length*/)
{
#if SPI_MODE == WAIT

    LCD_SELECT;
    while(!(LCD_SPI_IFG & UCTXIFG)); // wait for buffer

    LCD_SPI_TXBUF = data;
    while(LCD_SPI_STAT & UCBUSY);
    __delay_cycles(32);

    LCD_DESELECT;
#else
    while(spi.state);

    LCD_SELECT;

    spi.state = SPI_TX_Deselect;
    spi.txdata = data;
    spi.count = 1;
    spi.data = (uint8_t *)&spi.txdata;
    LCD_SPI_IE = UCTXIE;                     // Enable TX interrupt
#endif
}

// code duplication, but we are saving clock cycles by not passing dataCommand
static void writeCommand(uint8_t command)
{
#if SPI_MODE == WAIT

    LCD_SELECT;
    LCD_DC_CMD;
    while(!(LCD_SPI_IFG & UCTXIFG)); // wait for buffer

    LCD_SPI_TXBUF = command;
    while(LCD_SPI_STAT & UCBUSY);

    LCD_DC_DATA;
    LCD_DESELECT;
    __delay_cycles(32);

#else
    while(spi.state);

    LCD_SELECT;
    LCD_DC_CMD;

    spi.state = SPI_TX_Deselect;
    spi.txdata = command;
    spi.data = (uint8_t *)&spi.txdata;
    spi.count = 1;
    LCD_SPI_IE = UCTXIE;                     // Enable TX interrupt
#endif
}

// code duplication, but we are saving clock cycles and stack by not passing params
static void readDataBegin(uint8_t command) {

    while(spi.state);

    LCD_SELECT;
    LCD_DC_CMD;

    spi.state = SPI_TX;
    spi.txdata = command;
    spi.data = (uint8_t *)&spi.txdata;
    spi.count = 1;
    dpi_clk(1);
    LCD_SPI_IE = UCTXIE;                     // Enable TX interrupt

    while(spi.state);
}

static uint8_t readData()
{
    return readByte(0);
}

static void readDataEnd() {

    LCD_DESELECT;
    dpi_clk(0);

#ifdef TOUCH_PANEL
	ENABLE_TOUCH;
#endif
}

#ifdef TOUCH_MAXSAMPLES

/* touch functions: ADS7843 */

static bool isPenDown (void)
{
    return (TOUCH_IRQ_PORT->IN & TOUCH_IRQ_PIN) == 0;
}

/* get position - uses 16 bit mode */

static uint16_t getPosition (bool xpos, uint8_t samples) {

    static uint32_t buffer[TOUCH_MAXSAMPLES];

    bool pendown;
    uint8_t cmd = xpos ? 0xD0 : 0x90;
    uint32_t sampling = samples, sample;
    uint32_t *data = &buffer[0];

    while(spi.state);

    dpi_clk(samples == TOUCH_MAXSAMPLES ? 20 : 2);

    if(xpos) {
        TOUCH_SELECT;
        _delay_cycles(500); // wait a  bit

        readByte(cmd);
    }

    readByte(0);
    readByte(cmd);

    while((pendown = (TOUCH_IRQ_PORT->IN & TOUCH_IRQ_PIN) == 0) && sampling) {

        sample = readByte(0) << 5;
        sample |= readByte(cmd) >> 3;

        *data++ = sample;
        sampling--;
    }

    if(!xpos || !pendown) {
        readByte(0);
        readByte(0);
        TOUCH_DESELECT;
        dpi_clk(0);
    }

    return sampling ? 0 : quick_select(buffer, samples);
}

#endif

/* MCU peripherals init */

void LCD_DriverInit (lcd_driver_t *drv)
{

    driver = drv;

    driver->writeData = writeData;
    driver->writeCommand = writeCommand;
    driver->writePixel = writePixel;
    driver->writePixels = writePixels;
    driver->readDataBegin = readDataBegin;
    driver->readData = readData;
    driver->readDataEnd = readDataEnd;
    driver->delayms = msdelay;

    spi.state = SPI_Idle;

    LCD_CS_PORT_DIR |= LCD_CS_PIN;
    LCD_DC_PORT_DIR |= LCD_DC_PIN;

    LCD_DESELECT;
    LCD_DC_DATA;

    LCD_SPI_PORT_SEL |= LCD_SCLK_PIN|LCD_MOSI_PIN|LCD_MISO_PIN;

    LCD_SPI_CTL1 |= UCSWRST|UCSSEL_2;
    LCD_SPI_CTL0 |= UCMST|UCSYNC|UCCKPL|UCMSB;
    LCD_SPI_BR0 = 2;
    LCD_SPI_BR1 = 0;
    LCD_SPI_CTL1 &= ~UCSWRST;

    SYSTICK_TIMER_EX0 |= 0;
    SYSTICK_TIMER_CTL = TASSEL__SMCLK;
    SYSTICK_TIMER_CCR0 = 25000; // for 1mS per count
    SYSTICK_TIMER_CCTL0 |= CCIE;

    _EINT();

#ifdef TOUCH_MAXSAMPLES

    driver->touchGetPosition = getPosition;
    driver->touchIsPenDown = isPenDown;

    NVIC->ISER[1] = 1 << ((PORT5_IRQn - 32) & 31); // Enable PORT5 interrupt in NVIC module

    TOUCH_CS_PORT_DIR |= TOUCH_CS_PIN;
    TOUCH_CS_PORT->OUT |= TOUCH_CS_PIN;
    TOUCH_IRQ_PORT_DIR &= ~TOUCH_IRQ_PIN;
    TOUCH_IRQ_PORT->OUT |= TOUCH_IRQ_PIN;
    TOUCH_IRQ_PORT->REN |= TOUCH_IRQ_PIN;
    TOUCH_IRQ_PORT->IES |= TOUCH_IRQ_PIN; //hi-lo
    TOUCH_IRQ_PORT->IFG &= ~TOUCH_IRQ_PIN;
    TOUCH_IRQ_PORT->IFG = 0;
    TOUCH_IRQ_PORT->IE |= TOUCH_IRQ_PIN;

    TOUCH_SELECT;

    _delay_cycles(500); // wait a  bit
    dpi_clk(2);

    /* dummy position read to put ADS7843 in a known state */
    readByte(0x90);
    readByte(0);
    readByte(0);

    TOUCH_DESELECT;

#endif

}

/* IRQ Handlers */

// Interrupt handler for 1 ms interval timer
#pragma vector=SYSTICK_TIMER0_VECTOR
__interrupt void systick_isr (void)
{
    if(ms_delay)
        ms_delay--;

    if(driver->systickCallback)
        driver->systickCallback();
    else if(!ms_delay)
        SYSTICK_TIMER_CTL &= ~(MC0|MC1);
}

#pragma vector=LCD_SPI_INTV
void EUSCIB0_IRQHandler(void)
{
    switch(LCD_SPI_IV) {

        case 0x02:
            LCD_SPI_IE = 0;
            if(spi.state == SPI_TX_Deselect || spi.state == SPI_TX_Repeat) {
                LCD_CS_PORT_OUT |= LCD_CS_PIN|LCD_DC_PIN;
              #ifdef TOUCH_PANEL
                ENABLE_TOUCH;
              #endif
            } else
                spi.rxdata = LCD_SPI_RXBUF;
            spi.state = SPI_Idle;
            break;

        case 0x04:
            if(!--spi.count) {
                if(spi.state == SPI_TX)
                    spi.state = SPI_Idle;
                LCD_SPI_IFG = 0;
                LCD_SPI_IE = spi.state == SPI_Idle ? 0 : UCRXIE;
            }
            if(spi.state == SPI_TX_Repeat && !(spi.count & 0x0001))
                LCD_SPI_TXBUF = *spi.data--;                  // Transmit characters
            else
                LCD_SPI_TXBUF = *spi.data++;                  // Transmit characters
            break;
    }
}

#ifdef TOUCH_MAXSAMPLES

void PORT5_IRQHandler(void)
{
    uint16_t ifg = TOUCH_IRQ_PORT->IFG;

    TOUCH_IRQ_PORT->IFG &= ~ifg;

    if(ifg & TOUCH_IRQ_PIN) {

        if(TOUCH_IRQ_PORT->IN & TOUCH_IRQ_PIN)
            TOUCH_IRQ_PORT->IES |= TOUCH_IRQ_PIN; //hi-lo
        else
            TOUCH_IRQ_PORT->IES &= ~TOUCH_IRQ_PIN; //lo-hi

        if(driver->touchIRQHandler)
            driver->touchIRQHandler();
    }
}
#endif
