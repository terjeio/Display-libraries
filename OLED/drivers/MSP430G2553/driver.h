/*
 * driver.h - OLED panel lowlevel driver for MSP430G2553
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

#ifndef _OLEDDRIVER_H_
#define _OLEDDRIVER_H_

#include <stdint.h>

#include "portmacros.h"
#include "OLED/config.h"

#ifdef SPIMODE

#define SPI_MODE WAIT
//#define LEGACY_BOOSTERPACK

#define OLED_SPI A0
#define OLED_SPI_CTL0 usciCTL(OLED_SPI, 0)
#define OLED_SPI_CTL1 usciCTL(OLED_SPI, 1)
#define OLED_SPI_BR0 usciBR(OLED_SPI, 0)
#define OLED_SPI_BR1 usciBR(OLED_SPI, 1)
#define OLED_SPI_IE usciIE(OLED_SPI)
#define OLED_SPI_IFG usciIFG(OLED_SPI)
#define OLED_SPI_IV usciIV(OLED_SPI)
#define OLED_SPI_RXBUF usciRXBUF(OLED_SPI)
#define OLED_SPI_TXBUF usciTXBUF(OLED_SPI)
#define OLED_SPI_STAT usciSTAT(OLED_SPI)
#define OLED_SPI_INTV usciInt(OLED_SPI)

#define OLED_SPI_PORT 1
#define OLED_SCLK_PIN BIT4
#define OLED_MOSI_PIN BIT2
#define OLED_SPI_PORT_SEL portSel(OLED_SPI_PORT,)
#define OLED_SPI_PORT_SEL2 portSel(OLED_SPI_PORT, 2)

#ifdef LEGACY_BOOSTERPACK
#define OLED_CS_PORT 6
#define OLED_CS_PIN BIT6

#define OLED_DC_PORT 6
#define OLED_DC_PIN BIT5
#else
#define OLED_CS_PORT 1
#define OLED_CS_PIN BIT0

#define OLED_DC_PORT 1
#define OLED_DC_PIN BIT1
#endif

#define OLED_RESET_PORT 1
#define OLED_RESET_PIN BIT3
#define OLED_RESET_PORT_OUT   portOut(OLED_RESET_PORT)
#define OLED_RESET_PORT_DIR   portDir(OLED_RESET_PORT)


#define OLED_CS_PORT_OUT   portOut(OLED_CS_PORT)
#define OLED_CS_PORT_DIR   portDir(OLED_CS_PORT)
#define OLED_SELECT OLED_CS_PORT_OUT &= ~OLED_CS_PIN
#define OLED_DESELECT OLED_CS_PORT_OUT |= OLED_CS_PIN

#define OLED_DC_PORT_OUT   portOut(OLED_DC_PORT)
#define OLED_DC_PORT_DIR   portDir(OLED_DC_PORT)
#define OLED_DC_CMD OLED_DC_PORT_OUT &= ~OLED_DC_PIN
#define OLED_DC_DATA OLED_DC_PORT_OUT |= OLED_DC_PIN

#else // I2C MODE

#define SH1106_I2C_ADDRESS 0x3C
#define OLED_CommandMode   0x00  /* C0 and DC bit are 0         */
#define OLED_DataMode      0x40  /* C0 bit is 0 and DC bit is 1 */

#define OLED_I2C B0
#define OLED_I2C_CTL0 usciCTL(OLED_I2C, 0)
#define OLED_I2C_CTL1 usciCTL(OLED_I2C, 1)
#define OLED_I2C_BR0 usciBR(OLED_I2C, 0)
#define OLED_I2C_BR1 usciBR(OLED_I2C, 1)
#define OLED_I2C_IE usciIE(OLED_I2C)
#define OLED_I2C_IFG usciIFG(OLED_I2C)
#define OLED_I2C_IV usciIV(OLED_I2C)
#define OLED_I2C_RXBUF usciRXBUF(OLED_I2C)
#define OLED_I2C_TXBUF usciTXBUF(OLED_I2C)
#define OLED_I2C_STAT usciSTAT(OLED_I2C)
#define OLED_I2C_INTV usciInt(OLED_I2C)
#define OLED_I2C_SADDR usciSADDR(OLED_I2C)

#define OLED_I2C_PORT 1
#define OLED_SDA_PIN BIT7
#define OLED_SDC_PIN BIT6
#define OLED_I2C_PORT_SEL portSel(OLED_I2C_PORT,)
#define OLED_I2C_PORT_SEL2 portSel(OLED_I2C_PORT, 2)

#endif


#endif /* _OLEDDRIVER_H_ */
