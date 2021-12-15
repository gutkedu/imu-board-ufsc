#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"
#include "inc/hw_ints.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"
#include "driverlib/timer.h"
#include "driverlib/interrupt.h"

#include "utils/uartstdio.h"
#include "incs/spi_scc2130.h"
#include "incs/i2c_bno.h"

//Global Variables
bno055_gyro_raw bno1;
bno055_gyro_raw bno2;
bno055_gyro_raw bno3;
bno055_gyro_raw bno4;
Output_scc scc1;
Output_scc scc2;
Output_scc scc3;
Output_scc scc4;
Output_scc scc5;
Output_scc scc6;
Status_scc s_scc1;
Status_scc s_scc2;
Status_scc s_scc3;
Status_scc s_scc4;
Status_scc s_scc5;
Status_scc s_scc6;
uint8_t data_ready;
uint8_t data_error;
uint8_t RSdata;

void init_led(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);
}

void ConfigureUART0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    UARTStdioConfig(0, 921600, 16000000);
}

void ConfigureTIMER0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    //enable Timer to read sensor data at 2.3khz, TODO: revise the calculations.
    uint32_t ui32Period = (SysCtlClockGet() / 500);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    //enable timer0 int
    IntMasterEnable(); // enable processor interrupts
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
}

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear the timer interrupt

    bno1 = bno055_read_gyro_I2C0(BNO055_I2C_ADDR1);
    bno2 = bno055_read_gyro_I2C0(BNO055_I2C_ADDR2);
    bno3 = bno055_read_gyro_I2C1(BNO055_I2C_ADDR1);
    bno4 = bno055_read_gyro_I2C1(BNO055_I2C_ADDR2);
    //scc1 = read_and_process_gyro_SCC1();
    scc2 = read_and_process_gyro_SCC2();
    scc3 = read_and_process_gyro_SCC3();
    scc4 = read_and_process_gyro_SCC4();
    scc5 = read_and_process_gyro_SCC5();
    scc6 = read_and_process_gyro_SCC6();

    /*

     // SCC 2130 gyro prints..
     //UARTprintf("%d\t",scc1.gyro);
     UARTprintf("%d\t", scc2.gyro);
     UARTprintf("%d\t", scc3.gyro);
     UARTprintf("%d\t", scc4.gyro);
     UARTprintf("%d\t", scc5.gyro);
     UARTprintf("%d\t", scc6.gyro);

     //Gyro BNO055 print x-y-z \t
     UARTprintf("%d \t %d \t %d\t", bno1.x, bno1.y, bno1.z);
     UARTprintf("%d \t %d \t %d\t", bno2.x, bno2.y, bno2.z);
     UARTprintf("%d \t %d \t %d\t", bno3.x, bno3.y, bno3.z);
     UARTprintf("%d \t %d \t %d\t", bno4.x, bno4.y, bno4.z);

     //Temperature prints... SCC from top and bottom, Bno055 top and bottom...
     UARTprintf("%d \t", scc2.temp);
     UARTprintf("%d \t", scc6.temp);
     UARTprintf("%d \t", bno055_read_temp_I2C0(BNO055_I2C_ADDR1)); // temp bno1
     UARTprintf("%d \n", bno055_read_temp_I2C1(BNO055_I2C_ADDR2)); // temp bno4


     */

    /*
     s_scc1 = read_scc_status(1);
     s_scc2 = read_scc_status(2);
     s_scc3 = read_scc_status(3);
     s_scc4 = read_scc_status(4);
     s_scc5 = read_scc_status(5);
     s_scc6 = read_scc_status(6);

     UARTprintf(" %x\t %x\t %x\t %x\t %x\n", s_scc6.ComStat1,
     s_scc6.StatSum, s_scc6.RateStat1, s_scc6.RateStat2,
     s_scc6.AccStat);


     */

    data_ready = 1;

}

/**
 * main.c
 */
int main(void)
{

    SysCtlClockSet(
    SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ); //80mhz

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();
    FPUEnable();

    //Reset all sensors
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);
    delayMs(10);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0xff);

    init_led();
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08); //led on
    ConfigureUART0();
    ConfigureTIMER0();
    ConfigureSSI0();
    ConfigureSSI1();
    ConfigureSSI2();
    ConfigureI2C0();
    ConfigureI2C1();
    init_scc2130();
    init_bno055_I2C0(BNO055_I2C_ADDR1);
    init_bno055_I2C0(BNO055_I2C_ADDR2);
    init_bno055_I2C1(BNO055_I2C_ADDR1);
    init_bno055_I2C1(BNO055_I2C_ADDR2);
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00); //led off

    TimerEnable(TIMER0_BASE, TIMER_A); //enable timer0

    while (1)
    {
        //Timer interrupt running...

        if (data_error)
        {
            TimerDisable(TIMER0_BASE, TIMER_A);
            //stop
            // print status...

            //enable....
            TimerEnable(TIMER0_BASE, TIMER_A); // enable timer0 again
            data_error = false;
        }

        else if (data_ready)
        {
            //print
        }
    }

    return 0;
}

