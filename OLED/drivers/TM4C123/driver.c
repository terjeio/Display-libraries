/*
 * OLED lowlevel driver layer for Tiva C
 *
 *
 * 2017-09-06
 *
 */

#include <stdint.h>
#include <stdbool.h>

#include "inc/hw_gpio.h"
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"

#include "driverlib/sysctl.h"
#include "driverlib/pin_map.h"
#include "driverlib/gpio.h"
#include "driverlib/ssi.h"
#include "driverlib/i2c.h"

#include "OLED/OLED.h"

#define TXDLY 5
//#define SPIMODE

#ifdef SPIMODE

// OLED GPIO MACROS
#define OLED_SELECT   GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0)
#define OLED_DESELECT GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);
#define OLED_COMMAND  GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);
#define OLED_DATA     GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, GPIO_PIN_2);

static void SPIInit (void) {

    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA3_SSI0FSS);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE,GPIO_PIN_5|GPIO_PIN_4|GPIO_PIN_3|GPIO_PIN_2);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 4000000, 8);
    SSIEnable(SSI0_BASE);

    //CONFIG GPIO:
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); //output, ADS SPI ~RESET,
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2); //output, ADS SPI START,
    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4); //output, ADS SPI ~CS,
    GPIOPinTypeGPIOInput(GPIO_PORTE_BASE, GPIO_PIN_5);  //input,  ADS SPI ~DRDY (interrupt)

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, GPIO_PIN_1); //RESET=1
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0);          //START=0
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, GPIO_PIN_4);  //CS=1

    OLED_DESELECT;

    OLED_COMMAND;
}

static void SPI_WriteCommands (const uint8_t *data, int i)
{
    OLED_DESELECT;  // cs high
    OLED_COMMAND;   // D/C low
    OLED_SELECT;    // cs low

    while(i--)
    {
        SSIDataPut(SSI0_BASE, *data++);    // load data
    }

    while(SSIBusy(SSI0_BASE));          // wait for all to finish

    __delay_cycles(TXDLY);

    OLED_DESELECT;  // cs high
}

static void SPI_WriteData(const uint8_t *data, int i)
{
    OLED_DESELECT;  // cs high
    OLED_DATA;      // D/C high
    OLED_SELECT;    // cs low

    while(i--)
    {
        SSIDataPut(SSI0_BASE, *data++);    // load data
    }

    while(SSIBusy(SSI0_BASE));          // wait for all to finish

    __delay_cycles(TXDLY);

    OLED_DESELECT;  // cs high
}

#else // I2C Mode

#define SH1106_I2C_ADDRESS 0x3C
#define OLED_CommandMode        0x00  /* C0 and DC bit are 0         */
#define OLED_DataMode           0x40  /* C0 bit is 0 and DC bit is 1 */

typedef enum {
    I2CState_Idle = 0,
    I2CState_SendNext,
    I2CState_SendLast,
    I2CState_AwaitCompletion,
    I2CState_ReceiveNext,
    I2CState_ReceiveNextToLast,
    I2CState_ReceiveLast,
} i2c_state_t;

typedef struct {
    volatile i2c_state_t state;
    uint8_t count;
    uint8_t *data;
    bool getKeycode;
    uint8_t buffer[8];
    bool start;
} i2c_trans_t;

static i2c_trans_t i2c;
void I2C_interrupt_handler (void);

void I2CInit ()
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);
    GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), true);
    I2CIntRegister(I2C1_BASE, I2C_interrupt_handler);

    i2c.count = 0;
    i2c.state = I2CState_Idle;

    I2CMasterIntClear(I2C1_BASE);
    I2CMasterIntEnable(I2C1_BASE);

}

#define i2cIsBusy ((i2c.state != I2CState_Idle) || I2CMasterBusy(I2C1_BASE))

static void I2CSendMany (uint32_t i2cAddr, const uint8_t *data, uint16_t bytes)
{
    uint32_t i;
    if(i2c.start)
        bytes++;

    while(i2cIsBusy);

    i2c.count = bytes - 1;
    i2c.data  = data;
    i2c.state = bytes == 1 ? I2CState_AwaitCompletion : (bytes == 2 ? I2CState_SendLast : I2CState_SendNext);
    I2CMasterSlaveAddrSet(I2C1_BASE, i2cAddr, false);
    if(i2c.start)
        I2CMasterDataPut(I2C1_BASE, i2c.buffer[0]);
    else
        I2CMasterDataPut(I2C1_BASE, *i2c.data++);
    I2CMasterControl(I2C1_BASE, bytes == 1 ? I2C_MASTER_CMD_SINGLE_SEND : I2C_MASTER_CMD_BURST_SEND_START);
}

void I2C_WriteCommands (const uint8_t *data, uint16_t i)
{
    i2c.start = true;
    i2c.buffer[0] = OLED_CommandMode;

    I2CSendMany(SH1106_I2C_ADDRESS, data, i);
    while(i2cIsBusy);
}

void I2C_WriteData(const uint8_t *data, uint16_t i)
{
    i2c.start = true;
    i2c.buffer[0] = OLED_DataMode;

    I2CSendMany(SH1106_I2C_ADDRESS, data, i);
    while(i2cIsBusy);
}

void I2C_interrupt_handler (void)
{

    // based on code from https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/169882

    I2CMasterIntClear(I2C1_BASE);

//    if(I2CMasterErr(I2C1_BASE) == I2C_MASTER_ERR_NONE)


    switch(i2c.state)
    {

        case I2CState_Idle:
            break;

        case I2CState_SendNext:
        {
            I2CMasterDataPut(I2C1_BASE, *i2c.data++);
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_CONT);
            break;
        }

        case I2CState_SendLast:
        {
            I2CMasterDataPut(I2C1_BASE, *i2c.data);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

            i2c.state = I2CState_AwaitCompletion;
            break;
        }

        case I2CState_AwaitCompletion:
        {
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            break;
        }

        case I2CState_ReceiveNext:
        {

            *i2c.data++ = I2CMasterDataGet(I2C1_BASE);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);

            if(--i2c.count == 2)
                i2c.state = I2CState_ReceiveNextToLast;
            break;
        }

        case I2CState_ReceiveNextToLast:
        {
            *i2c.data++ = I2CMasterDataGet(I2C1_BASE);
            I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);

            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;
        }

        case I2CState_ReceiveLast:
        {
            *i2c.data = I2CMasterDataGet(I2C1_BASE);
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            break;
        }
    }
}
#endif

static void delay_ms (uint16_t ms)
{
    while(ms--)
        __delay_cycles(80000);
}

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



