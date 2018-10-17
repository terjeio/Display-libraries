/******************************************************************************
 * File Name   :  OLED_SSD1306.c
 * Author      :  43oh - MSP430 News Projects and Forums.
               :  (http://www.43oh.com)
 * Description :  Lowlevel driver for the OLED SSD1306 
 * Date        :  October 21, 2011.
 *****************************************************************************/

#include <msp.h>
#include <stdint.h>

#include "driver.h"
#include "OLED/OLED.h"

static void delay_ms (uint16_t ms)
{
    //  SysCtlDelay((SysCtlClockGet() / (3 * 100)) * x10ms) ;

    while(ms--)
        __delay_cycles(48000);
}

#ifdef SPIMODE
void SPIInit (void) {

    OLED_SCLK_PORT->OUT |= OLED_SCLK_PIN;
    OLED_SCLK_PORT->DIR |= OLED_SCLK_PIN;
    OLED_SCLK_PORT->SEL0 |= OLED_SCLK_PIN | OLED_MOSI_PIN;
    OLED_CS_PORT->OUT |= OLED_CS_PIN;
    OLED_CS_PORT->DIR |= OLED_CS_PIN;
    OLED_MOSI_PORT->OUT |= OLED_MOSI_PIN;
    OLED_MOSI_PORT->DIR |= OLED_MOSI_PIN;
    OLED_DC_PORT->OUT |= OLED_DC_PIN;
    OLED_DC_PORT->DIR |= OLED_DC_PIN;


    EUSCI_B0->CTLW0 |= UCSWRST;
    EUSCI_B0->CTLW0 |= UCMST | UCSYNC | UCCKPL | UCMSB;
    EUSCI_B0->CTLW0 |= UCSSEL__SMCLK;
    EUSCI_B0->BRW = 4;
    EUSCI_B0->CTLW0 &= ~UCSWRST;

	OLED_DESELECT;

	OLED_COMMAND;

}

void SPI_WriteCommands (const uint8_t *data, uint16_t i)
{
	__disable_interrupt();

	OLED_DESELECT;	// cs high
	OLED_COMMAND;	// D/C low
	OLED_SELECT;	// cs low

	while(i)
	{

	    EUSCI_B0->TXBUF = (uint16_t)*data;
		while (EUSCI_B0->STATW & UCBUSY);

		data++;				// increment pointer

		i--;				// decrement byte counter
	}

	__delay_cycles(32);

	OLED_DESELECT;	// cs high

	__enable_interrupt();
}

void SPI_WriteData (const uint8_t *data, uint16_t i)
{
	__disable_interrupt();

	OLED_DESELECT;	// cs high
	OLED_DATA;		// D/C high
	OLED_SELECT;	// cs low

	while(i)
	{

		// insert boundary check here

	    EUSCI_B0->TXBUF = (uint16_t)*data;
		while (EUSCI_B0->STATW & UCBUSY);

		data++;				// increment pointer

		i--;				// decrement byte counter
	}

	__delay_cycles(32);

	OLED_DESELECT;	// cs high

	__enable_interrupt();
}

#else // I2C Mode

typedef struct {
//    volatile i2c_state_t state;
    volatile uint8_t count;
    uint8_t *data;
    bool getKeycode;
    uint8_t buffer[8];
    bool start;
} i2c_trans_t;

static i2c_trans_t i2c;

void I2CInit (const int i2caddress) {

    P1->SEL0 |= BIT6|BIT7;                     // Assign I2C pins to USCI_B0
/*
    while (EUSCI_B0->CTLW0 & UCTXSTP);              // Ensure stop condition got sent

    EUSCI_B0->CTLW1 |= UCSWRST;                 // Enable SW reset
    EUSCI_B0->CTLW1 = UCMST| UCMODE_3| UCSYNC;  // I2C Master, synchronous mode
    EUSCI_B0->CTLW1 = UCSSEL_2| UCSWRST;            // Use SMCLK, keep SW reset
    EUSCI_B0->BRW = 160;                            // fSCL = SMCLK/12 = ~100kHz
    EUSCI_B0->I2CSA = i2caddress;                   // Set slave address,
    EUSCI_B0->CTLW1 &= ~UCSWRST;                    // clear SW reset and resume operation
*/
    __enable_interrupt();
    NVIC->ISER[0] = 1 << ((EUSCIB0_IRQn) & 31); // Enable eUSCIB0 interrupt in NVIC module

    // Configure USCI_B0 for I2C mode
    EUSCI_B0->CTLW0 |= EUSCI_A_CTLW0_SWRST;                         // put eUSCI_B in reset state
    EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_MODE_3 | EUSCI_B_CTLW0_MST;    // I2C master mode, SMCLK
    EUSCI_B0->BRW = 30;                                             // baudrate 400 KHZ (SMCLK = 12MHz)
    EUSCI_B0->CTLW0 &=~ EUSCI_A_CTLW0_SWRST;                        // clear reset register
    EUSCI_B0->IE |= EUSCI_B_IE_TXIE0 | EUSCI_B_IE_NACKIE;           // transmit and NACK interrupt enable
}

static void StartI2Ctx()
{
   i2c.start = true;

//    SCB->SCR |= SCB_SCR_SLEEPONEXIT_Msk;                      // Don't wake up on exit from ISR
//    for (i = 1000; i > 0; i--);                               // Delay between transmissions
   EUSCI_B0->I2CSA = SH1106_I2C_ADDRESS;                       // configure slave address                                   // Load TX byte counter
   while (EUSCI_B0->CTLW0 & EUSCI_B_CTLW0_TXSTP);              // Ensure stop condition got sent
   EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TR | EUSCI_B_CTLW0_TXSTT;  // I2C TX, start condition
   while(i2c.count);
}

void I2C_WriteCommands (const uint8_t *data, uint16_t i)
{
    i2c.count = i;
    i2c.data = data;
    i2c.buffer[0] = OLED_CommandMode;
    StartI2Ctx();
    for (i = 5000; i > 0; i--);                       // Delay between transmissions
}

void I2C_WriteData (const uint8_t *data, uint16_t i)
{
    i2c.count = i;
    i2c.data = data;
    i2c.buffer[0] = OLED_DataMode;
    StartI2Ctx();
    for (i = 1000; i > 0; i--);                       // Delay between transmissions
}

// I2C interrupt service routine
void EUSCIB0_IRQHandler(void)
{
    if (EUSCI_B0->IFG & EUSCI_B_IFG_NACKIFG) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_NACKIFG;
        UCB0CTL1 |= EUSCI_B_CTLW0_TXSTT;                  // I2C start condition
    }
    if (EUSCI_B0->IFG & EUSCI_B_IFG_TXIFG0) {
        EUSCI_B0->IFG &= ~ EUSCI_B_IFG_TXIFG0;
        if(i2c.start) {
            EUSCI_B0->TXBUF = i2c.buffer[0];            // Load TX buffer
            i2c.start = false;
        } else if (i2c.count) {                               // Check TX byte counter
            EUSCI_B0->TXBUF = *i2c.data++;            // Load TX buffer
            i2c.count--;                        // Decrement TX byte counter
        //    tc++;
       } else {
            EUSCI_B0->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                     // I2C stop condition
            EUSCI_B0->IFG &= ~EUSCI_A_IFG_TXIFG;                      // Clear USCI_B0 TX int flag
            SCB->SCR &= ~SCB_SCR_SLEEPONEXIT_Msk;             // Wake up on exit from ISR
       }
    }
}
#endif

void OLED_DriverInit (driver_t *display)
{
    display->delayms = delay_ms;

#ifdef SPIMODE
	SPIInit();
    display->writeCommand = SPI_WriteCommands;
    display->writeData = SPI_WriteData;
#else
	I2CInit(SH1106_I2C_ADDRESS);
    display->writeCommand = I2C_WriteCommands;
    display->writeData = I2C_WriteData;
#endif
}

