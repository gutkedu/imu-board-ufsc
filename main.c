#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ssi.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/ssi.h"

#include "utils/uartstdio.h"
#include "inc/spi_scc2130.h"
#include "inc/i2c_bno.h"

void Timer0IntHandler()
{
    // call sensor readings...
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear the timer interrupt

}

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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA); // Habilita a PORTA.
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0); // Habilita UART0
    GPIOPinConfigure(GPIO_PA0_U0RX); // Configura os pinos da GPIO para a UART0
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
    // Usa o oscilador interno preciso de 16MHz como fonte de clock da UART0
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
    // Inicializa a UART0 para 115200 bits/s com clock de 16MHz
    UARTStdioConfig(0, 115200, 16000000);
}

void ConfigureTIMER0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

    //enable Timer to read sensor data at 2.3khz, TODO: revise the calculations.
    uint32_t ui32Period = (SysCtlClockGet() / 2300);
    TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period - 1);
    //enable timer0 int

    IntMasterEnable(); // enable processor interrupts
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    IntEnable(INT_TIMER0A);
}

void Timer0IntHandler(void)
{
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT); // clear the timer interrupt
}

/**
 * main.c
 */
int main(void)
{

    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    FPULazyStackingEnable();

    SysCtlClockSet(
            SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN
                    | SYSCTL_XTAL_16MHZ); //80mhz

    ConfigureUART0(void);
    ConfigureTIMER0(void);
    ConfigureSSI0(void);
    ConfigureSSI1(void);
    ConfigureSSI2(void);

    //init_scc2130();

    //TimerEnable(TIMER0_BASE, TIMER_A); //enable timer0

    return 0;
}
