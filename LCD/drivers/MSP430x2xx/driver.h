/*
 * driver.h - LCD panel driver for MSP430F5529
 *
 * v1.0.0 / 2017-12-22 / ©Io Engineering / Terje
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

#ifndef _LCDDRIVER_H_
#define _LCDDRIVER_H_

#include <stdint.h>

#include "portmacros.h"
#include "lcd/config.h"

#define SPI_MODE WAIT
//#define LEGACY_BOOSTERPACK

#define LCD_SPI B0
#define LCD_SPI_CTL0 usciCTL(LCD_SPI, 0)
#define LCD_SPI_CTL1 usciCTL(LCD_SPI, 1)
#define LCD_SPI_BR0 usciBR(LCD_SPI, 0)
#define LCD_SPI_BR1 usciBR(LCD_SPI, 1)
#define LCD_SPI_IE usciIE(LCD_SPI)
#define LCD_SPI_IFG usciIFG(LCD_SPI)
#define LCD_SPI_IV usciIV(LCD_SPI)
#define LCD_SPI_RXBUF usciRXBUF(LCD_SPI)
#define LCD_SPI_TXBUF usciTXBUF(LCD_SPI)
#define LCD_SPI_STAT usciSTAT(LCD_SPI)
#define LCD_SPI_INTV usciInt(LCD_SPI)

#define LCD_SPI_PORT 3
#define LCD_SCLK_PIN BIT2
#define LCD_MOSI_PIN BIT0
#define LCD_MISO_PIN BIT1
#define LCD_SPI_PORT_SEL portSel(LCD_SPI_PORT,)

#ifdef LEGACY_BOOSTERPACK
#define LCD_CS_PORT 6
#define LCD_CS_PIN BIT6

#define LCD_DC_PORT 6
#define LCD_DC_PIN BIT5
#else
#define LCD_CS_PORT 2
#define LCD_CS_PIN BIT6

#define LCD_DC_PORT 1
#define LCD_DC_PIN BIT6
#endif

#define LCD_CS_PORT_OUT   portOut(LCD_CS_PORT)
#define LCD_CS_PORT_DIR   portDir(LCD_CS_PORT)
#define LCD_SELECT LCD_CS_PORT_OUT &= ~LCD_CS_PIN
#define LCD_DESELECT LCD_CS_PORT_OUT |= LCD_CS_PIN

#define LCD_DC_PORT_OUT   portOut(LCD_DC_PORT)
#define LCD_DC_PORT_DIR   portDir(LCD_DC_PORT)
#define LCD_DC_CMD LCD_DC_PORT_OUT &= ~LCD_DC_PIN
#define LCD_DC_DATA LCD_DC_PORT_OUT |= LCD_DC_PIN

#define SYSTICK_TIMER B
#define SYSTICK_TIMER_INSTANCE  0
#define SYSTICK_TIMER_CTL       timerCtl(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE)
#define SYSTICK_TIMER_CCTL0     timerCCtl(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE, 0)
#define SYSTICK_TIMER_EX0       timerEx(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE)
#define SYSTICK_TIMER_CCR0      timerCcr(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE, 0)
#define SYSTICK_TIMER0_VECTOR   timerInt(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE, 0)

#ifdef TOUCH_MAXSAMPLES
#define TOUCH_IRQ_PORT P5
#define TOUCH_IRQ_PIN BIT4
#define TOUCH_CS_PORT P6
#define TOUCH_CS_PIN BIT5
#define TOUCH_SELECT TOUCH_CS_PORT->OUT &= ~TOUCH_CS_PIN
#define TOUCH_DESELECT TOUCH_CS_PORT->OUT |= TOUCH_CS_PIN
#endif

#endif /* _LCDDRIVER_H_ */
