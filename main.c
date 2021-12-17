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

void print_gyro_bno055();
void print_gyro_scc();
void print_temperature_scc();
void print_temperature_bno055();
void print_scc_data_error();
void print_scc_status();

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
    UARTStdioConfig(0, 115200, 16000000); //921600
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

    /*
     bno1 = read_bno055_gyro(BNO055_I2C_ADDR1, SELECT_I2C0);
     bno2 = read_bno055_gyro(BNO055_I2C_ADDR2, SELECT_I2C0);
     bno3 = read_bno055_gyro(BNO055_I2C_ADDR1, SELECT_I2C1);
     bno4 = read_bno055_gyro(BNO055_I2C_ADDR2, SELECT_I2C1);
     */

    //scc1 = read_process_gyro_temp_scc(SCC_1);
    scc2 = read_process_gyro_temp_scc(SCC_2);
    scc3 = read_process_gyro_temp_scc(SCC_3);
    scc4 = read_process_gyro_temp_scc(SCC_4);
    scc5 = read_process_gyro_temp_scc(SCC_5);
    scc6 = read_process_gyro_temp_scc(SCC_6);

    /*
     s_scc1 = read_scc_status(1);
     s_scc2 = read_scc_status(2);
     s_scc3 = read_scc_status(3);
     s_scc4 = read_scc_status(4);
     s_scc5 = read_scc_status(5);
     s_scc6 = read_scc_status(6);
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

    init_led();
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x08); //led on
    ConfigureUART0();
    ConfigureTIMER0();
    ConfigureSSI0();
    ConfigureSSI1();
    ConfigureSSI2();
    ConfigureI2C0();
    ConfigureI2C1();

    //Reset all sensors
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0x00);
    delayMs(10);
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_2, 0xff);

    init_scc2130();
    init_bno055(BNO055_I2C_ADDR1, SELECT_I2C0);
    init_bno055(BNO055_I2C_ADDR2, SELECT_I2C0);
    init_bno055(BNO055_I2C_ADDR1, SELECT_I2C1);
    init_bno055(BNO055_I2C_ADDR2, SELECT_I2C1);

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
            print_gyro_scc();
            UARTprintf("\n");
            data_ready = 0;
        }
    }

    return 0;
}

void print_gyro_bno055()
{
    //Gyro BNO055 print x-y-z \t
    UARTprintf("%d \t %d \t %d\t", bno1.x, bno1.y, bno1.z);
    UARTprintf("%d \t %d \t %d\t", bno2.x, bno2.y, bno2.z);
    UARTprintf("%d \t %d \t %d\t", bno3.x, bno3.y, bno3.z);
    UARTprintf("%d \t %d \t %d\t", bno4.x, bno4.y, bno4.z);
}

void print_gyro_scc()
{
    //UARTprintf("%d\t",scc1.gyro);
    UARTprintf("%d\t", scc2.gyro);
    UARTprintf("%d\t", scc3.gyro);
    UARTprintf("%d\t", scc4.gyro);
    UARTprintf("%d\t", scc5.gyro);
    UARTprintf("%d\t", scc6.gyro);
}

void print_temperature_scc()
{
    //UARTprintf("%d\t",scc1.temp);
    UARTprintf("%d\t", scc2.temp);
    UARTprintf("%d\t", scc3.temp);
    UARTprintf("%d\t", scc4.temp);
    UARTprintf("%d\t", scc5.temp);
    UARTprintf("%d\t", scc6.temp);
}

void print_temperature_bno055()
{
    UARTprintf("%d \t", read_bno055_temp(BNO055_I2C_ADDR1, SELECT_I2C0)); // temp bno1
    UARTprintf("%d \t", read_bno055_temp(BNO055_I2C_ADDR2, SELECT_I2C0)); // temp bno2
    UARTprintf("%d \t", read_bno055_temp(BNO055_I2C_ADDR1, SELECT_I2C1)); // temp bno3
    UARTprintf("%d \t", read_bno055_temp(BNO055_I2C_ADDR2, SELECT_I2C1)); // temp bno4
}

void print_scc_data_error()
{
    //UARTprintf("%d\t",scc1.data_error);
    UARTprintf("%d\t", scc2.data_error);
    UARTprintf("%d\t", scc3.data_error);
    UARTprintf("%d\t", scc4.data_error);
    UARTprintf("%d\t", scc5.data_error);
    UARTprintf("%d\t", scc6.data_error);
}

void print_scc_status()
{
    //SCC1
    //   UARTprintf("%x\t %x\t %x\t %x\t %x\n", s_scc1.ComStat1, s_scc1.StatSum,
    //         s_scc1.RateStat1, s_scc1.RateStat2, s_scc1.AccStat);

    //SCC2
    UARTprintf("%x\t %x\t %x\t %x\t %x\n", s_scc2.ComStat1, s_scc2.StatSum,
               s_scc2.RateStat1, s_scc2.RateStat2, s_scc2.AccStat);

    //SCC3
    UARTprintf("%x\t %x\t %x\t %x\t %x\n", s_scc3.ComStat1, s_scc3.StatSum,
               s_scc3.RateStat1, s_scc3.RateStat2, s_scc3.AccStat);

    //SCC4
    UARTprintf("%x\t %x\t %x\t %x\t %x\n", s_scc4.ComStat1, s_scc4.StatSum,
               s_scc4.RateStat1, s_scc4.RateStat2, s_scc4.AccStat);

    //SCC5
    UARTprintf("%x\t %x\t %x\t %x\t %x\n", s_scc5.ComStat1, s_scc5.StatSum,
               s_scc5.RateStat1, s_scc5.RateStat2, s_scc5.AccStat);

    //SCC6
    UARTprintf("%x\t %x\t %x\t %x\t %x\n", s_scc6.ComStat1, s_scc6.StatSum,
               s_scc6.RateStat1, s_scc6.RateStat2, s_scc6.AccStat);
}

