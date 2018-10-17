/*
 * driver.c - OLED panel lowlevel driver for MSP430G2553
 *
 * v1.0.0 / 2018-03-03 / ©Io Engineering / Terje
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

#include <msp430.h>

#include "OLED/OLED.h"
#include "driver.h"

static void delay_ms (uint16_t ms)
{
    while(ms--)
        __delay_cycles(16000);
}

#ifdef SPIMODE

static void sendCommands (const uint8_t *data, uint16_t i)
{
	OLED_DC_CMD;	// D/C low
	OLED_SELECT;	// CS low

	while(i)
	{
		while(!(IFG2 & UCA0TXIFG));	// wait for buffer
		UCA0TXBUF = *data++;	    // load data, increment data pointer and
		i--;				        // decrement byte counter
	}

	while(UCA0STAT & UCBUSY); 		// wait for all to finish
	UCA0RXBUF;						// clear IFG and overruns

	__delay_cycles(32);

	OLED_DESELECT;	// CS high
    OLED_DC_DATA;
}

static void sendData (const uint8_t *data, uint16_t i)
{
	OLED_SELECT;	// CS low

	while(i)
	{
		// insert boundary check here
		while(!(IFG2 & UCA0TXIFG)); // wait for buffer
		UCA0TXBUF = *data++;	    // load data, increment data pointer and
		i--;				        // decrement byte counter
	}

	while(UCA0STAT & UCBUSY);       // wait for all to finish
	UCA0RXBUF;						// clear IFG and overruns

	__delay_cycles(32);

	OLED_DESELECT;	// CS high
}

void OLED_DriverInit (driver_t *display)
{
    display->delayms = delay_ms;
    display->writeCommand = sendCommands;
    display->writeData = sendData;

    OLED_SPI_PORT_SEL |= OLED_MOSI_PIN|OLED_SCLK_PIN;
    OLED_SPI_PORT_SEL2 |= OLED_MOSI_PIN|OLED_SCLK_PIN;

    OLED_DC_PORT_DIR |= OLED_DC_PIN;
    OLED_CS_PORT_DIR |= OLED_CS_PIN;

    OLED_DESELECT;
    OLED_DC_DATA;

    OLED_SPI_CTL1 |= UCSWRST|UCSSEL_2;
    OLED_SPI_CTL0 = UCCKPH|UCMSB|UCMST|UCMODE_1|UCSYNC;
    OLED_SPI_BR0 = 2;
    OLED_SPI_CTL1 &= ~UCSWRST;

#ifdef OLED_RESET_PIN
    OLED_RESET_PORT_DIR |= OLED_RESET_PIN;
    OLED_RESET_PORT_OUT &= ~OLED_RESET_PIN;
    delay_ms(15);
    OLED_RESET_PORT_OUT |= OLED_RESET_PIN;
#endif
}

#else

typedef enum {
    I2C_Idle,
    I2C_TXMode,
    I2C_TXData,
    I2C_TXStop
} i2c_state;

typedef struct {
    volatile i2c_state state;
    uint16_t count;
    uint8_t *data;
    uint8_t mode;
} i2c_trans_t;

static i2c_trans_t i2c;

inline static void StartI2Ctx()
{
   i2c.state = I2C_TXMode;

   while(OLED_I2C_STAT & UCBBUSY);

   OLED_I2C_SADDR = SH1106_I2C_ADDRESS;     // Set slave address,
   UC0IFG &= ~(UCB0TXIFG|UCB0RXIFG);        // Clear any pending interrupts
   UC0IE &= ~UCB0RXIE;                      // Disable RX interrupt
   UC0IE |= UCB0TXIE;
   OLED_I2C_CTL1 |= UCTR|UCTXSTT;

   while(i2c.state != I2C_Idle);

//   LPM0;
}

void sendCommands (const uint8_t *data, uint16_t i)
{
    i2c.count = i;
    i2c.data = (uint8_t *)data;
    i2c.mode = OLED_CommandMode;
    StartI2Ctx();
}

void sendData (const uint8_t *data, uint16_t i)
{
    i2c.count = i;
    i2c.data = (uint8_t *)data;
    i2c.mode = OLED_DataMode;
    StartI2Ctx();
}

void OLED_DriverInit (driver_t *display)
{
    display->delayms = delay_ms;
    display->writeCommand = sendCommands;
    display->writeData = sendData;

    i2c.state = I2C_Idle;

    OLED_I2C_PORT_SEL |= OLED_SDA_PIN|OLED_SDC_PIN;
    OLED_I2C_PORT_SEL2 |= OLED_SDA_PIN|OLED_SDC_PIN;

    IE2 &= ~(UCB0RXIE|UCB0TXIE);            // Disable RX/TX interrupt

    while (OLED_I2C_CTL1 & UCTXSTP);             // Ensure stop condition got sent

    OLED_I2C_CTL1 = UCSWRST|UCSSEL_2;       // Enable SW reset, use SMCLK
    OLED_I2C_CTL0 = UCMST|UCMODE_3|UCSYNC;  // I2C Master, synchronous mode
    OLED_I2C_BR0 = 40;                      // fSCL = SMCLK/12 = ~100kHz
    OLED_I2C_BR1 = 0;
    OLED_I2C_CTL1 &= ~UCSWRST;              // clear SW reset and resume operation
    UCB0I2CIE |= UCNACKIE;
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
	if(UC0IFG & UCB0TXIFG) {						// I2C TX

		switch(i2c.state) {

            case I2C_TXMode:
                OLED_I2C_TXBUF = i2c.mode;
                i2c.state = I2C_TXData;
                break;

            case I2C_TXData:
                OLED_I2C_TXBUF = *i2c.data++;   // Load TX buffer
                if(--i2c.count == 0)            // Decrement TX byte counter
                    i2c.state = I2C_TXStop;
                break;

            case I2C_TXStop:
                OLED_I2C_CTL1 |= UCTXSTP;	    // Send I2C stop condition,
                UC0IE &= ~UCB0TXIE;             // disable TXT interrupt and
                i2c.state = I2C_Idle;           // set mode to idle
    //			LPM0_EXIT;						// Exit LPM0
                break;
		}
	}
}

#pragma vector = USCIAB0RX_VECTOR
__interrupt void USCIAB0RX_ISR(void)
{
    if (OLED_I2C_STAT & UCNACKIFG) {
        OLED_I2C_STAT &= ~UCNACKIFG;                        // Clear NACK Flags
        i2c.state = I2C_Idle;
//        LPM0_EXIT;                                        // Exit LPM0
    }

    if (OLED_I2C_STAT & UCSTPIFG)                           //Stop or NACK Interrupt
        OLED_I2C_STAT &= ~(UCSTTIFG|UCSTPIFG|UCNACKIFG);    //Clear START/STOP/NACK Flags

    if (OLED_I2C_STAT & UCSTTIFG)
        OLED_I2C_STAT &= ~(UCSTTIFG);                       //Clear START Flags
}

#endif
