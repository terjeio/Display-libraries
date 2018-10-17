/*
 * driver.c - LCD panel driver for MSP432, CMSIS version
 *
 * v1.0.3 / 2017-12-23 / ©Io Engineering / Terje
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

#ifndef _LCDDRIVER_H_
#define _LCDDRIVER_H_

#include <stdint.h>
#include "lcd/config.h"

//#define LEGACY_BOOSTERPACK

#define LCD_SPI_PORT P1
#define LCD_SCLK_PIN BIT5
#define LCD_MOSI_PIN BIT6
#define LCD_MISO_PIN BIT7

#ifdef LEGACY_BOOSTERPACK
#define LCD_CS_PORT P6
#define LCD_CS_PIN BIT0

#define LCD_DC_PORT P4
#define LCD_DC_PIN BIT3
#else
#define LCD_CS_PORT P5
#define LCD_CS_PIN BIT0

#define LCD_DC_PORT P4
#define LCD_DC_PIN BIT1
#endif

#define LCD_SELECT LCD_CS_PORT->OUT &= ~LCD_CS_PIN
#define LCD_DESELECT LCD_CS_PORT->OUT |= LCD_CS_PIN

#define LCD_DC_CMD LCD_DC_PORT->OUT &= ~LCD_DC_PIN
#define LCD_DC_DATA LCD_DC_PORT->OUT |= LCD_DC_PIN

#ifdef TOUCH_MAXSAMPLES
#define TOUCH_IRQ_PORT P5
#define TOUCH_IRQ_PIN BIT4
#define TOUCH_CS_PORT P6
#define TOUCH_CS_PIN BIT5
#define TOUCH_SELECT TOUCH_CS_PORT->OUT &= ~TOUCH_CS_PIN
#define TOUCH_DESELECT TOUCH_CS_PORT->OUT |= TOUCH_CS_PIN
#endif

#endif /* _LCDDRIVER_H_ */

