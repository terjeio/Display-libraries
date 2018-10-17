/*********************************************************************

*********************************************************************/

#define BLACK 0
#define WHITE 1

#define SPIMODE
#define SSD1306

#ifdef SPIMODE

// OLED GPIO CONFIG
#define OLED_RES BIT3	// P1.3

#define UCXxTXBUF UCB0TXBUF
#define UCXxRXBUF UCB0RXBUF
#define UCXxSTAT UCB0STAT
#define UCXxBR0 UCB0BR0

#define OLED_DC_PORT P4
#define OLED_DC_PIN	BIT1

#define OLED_CS_PORT P5
#define OLED_CS_PIN	BIT0

#define OLED_MOSI_PORT P1
#define OLED_MOSI_PIN BIT6

#define OLED_SCLK_PORT P1
#define OLED_SCLK_PIN BIT5

// OLED GPIO MACROS
#define OLED_SELECT		(OLED_CS_PORT->OUT &= ~OLED_CS_PIN)
#define OLED_DESELECT	(OLED_CS_PORT->OUT |= OLED_CS_PIN)
#define OLED_COMMAND	(OLED_DC_PORT->OUT &= ~OLED_DC_PIN)
#define OLED_DATA		(OLED_DC_PORT->OUT |= OLED_DC_PIN)

#else //I2C

#define SH1106_I2C_ADDRESS 0x3C
#define OLED_CommandMode   0x00  /* C0 and DC bit are 0         */
#define OLED_DataMode      0x40  /* C0 bit is 0 and DC bit is 1 */

#endif
