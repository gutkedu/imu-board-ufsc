#include <stdint.h>
#include <stdbool.h>
#include "inc/hw_memmap.h"
#include "inc/hw_ssi.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/ssi.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"

#include "spi_scc2130.h"

//Global variables

uint8_t LoopCounter;
uint8_t data_ready;
uint8_t data_error;

void delayMs(uint32_t ui32Ms)
{

    // 1 clock cycle = 1 / SysCtlClockGet() second
    // 1 SysCtlDelay = 3 clock cycle = 3 / SysCtlClockGet() second
    // 1 second = SysCtlClockGet() / 3
    // 0.001 second = 1 ms = SysCtlClockGet() / 3 / 1000
    SysCtlDelay(ui32Ms * (SysCtlClockGet() / 3 / 1000));
}

void ConfigureSSI0(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);

    GPIOPinConfigure(GPIO_PA2_SSI0CLK);
    GPIOPinConfigure(GPIO_PA4_SSI0RX);
    GPIOPinConfigure(GPIO_PA5_SSI0TX);
    GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_2 | GPIO_PIN_4);

    SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
    SSI_MODE_MASTER,
                       5000000, 16);
    SSIEnable(SSI0_BASE);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_2); // Reset_scc

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_0); // CS SCC 1 PE_1
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0xff);    // Chip select high

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_1); // CS SCC 2 PE_2
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_1, 0xff);    // Chip select high
}

void ConfigureSSI1(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI1);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //Unlock PF0
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinConfigure(GPIO_PF2_SSI1CLK);
    GPIOPinConfigure(GPIO_PF0_SSI1RX);
    GPIOPinConfigure(GPIO_PF1_SSI1TX);
    GPIOPinTypeSSI(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_0 | GPIO_PIN_1);

    SSIConfigSetExpClk(SSI1_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
    SSI_MODE_MASTER,
                       5000000, 16);
    SSIEnable(SSI1_BASE);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_3); // CS SCC 3 PE_3
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0xff);    // Chip select high

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4); // CS SCC 4 PE_4
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0xff);    // Chip select high
}

void ConfigureSSI2(void)
{
    SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PB4_SSI2CLK);
    GPIOPinConfigure(GPIO_PB6_SSI2RX);
    GPIOPinConfigure(GPIO_PB7_SSI2TX);
    GPIOPinTypeSSI(GPIO_PORTB_BASE, GPIO_PIN_4 | GPIO_PIN_6 | GPIO_PIN_7);

    SSIConfigSetExpClk(SSI2_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0,
    SSI_MODE_MASTER,
                       5000000, 16);
    SSIEnable(SSI2_BASE);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_5); // CS SCC 5 PE_5
    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0xff);    // Chip select high

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3); // CS SCC 6 PF_3
    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xff);    // Chip select high
}

// Send Request to SCC1
uint32_t send_request_SCC1(uint32_t request)
{
    uint32_t response;
    uint32_t temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0x00); // Chip select on
    SSIDataGetNonBlocking(SSI0_BASE, &temp); // read rx buffer to clear it.
    temp = 0x00;

    SSIDataPut(SSI0_BASE, request >> 16);
    while (SSIBusy(SSI0_BASE))
        ; // wait until the busy bit is cleared

    SSIDataGet(SSI0_BASE, &temp); // Read response high word
    response = temp <<= 16;
    temp = 0x00;

    SSIDataPut(SSI0_BASE, request & 0x000FFFF); //send request low word
    while (SSIBusy(SSI0_BASE))
        ;

    SSIDataGet(SSI0_BASE, &temp); // Read response low word
    response = response | temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_0, 0xff);

    return response;
}

// Send Request to SCC2
uint32_t send_request_SCC2(uint32_t request)
{
    uint32_t response;
    uint32_t temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0x00); // Chip select on
    SSIDataGetNonBlocking(SSI0_BASE, &temp); // read rx buffer to clear it.
    temp = 0x00;

    SSIDataPut(SSI0_BASE, request >> 16);
    while (SSIBusy(SSI0_BASE))
        ;
    // wait until the busy bit is cleared
    SSIDataGet(SSI0_BASE, &temp); // Read response high word
    response = temp <<= 16;
    temp = 0x00;

    SSIDataPut(SSI0_BASE, request & 0x000FFFF); //send request low word
    while (SSIBusy(SSI0_BASE))
        ;

    SSIDataGet(SSI0_BASE, &temp); // Read response low word
    response = response | temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_1, 0xff);

    return response;
}

// Send Request to SCC3
uint32_t send_request_SCC3(uint32_t request)
{
    uint32_t response;
    uint32_t temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0x00); // Chip select on
    SSIDataGetNonBlocking(SSI1_BASE, &temp); // read rx buffer to clear it.
    temp = 0x00;

    SSIDataPut(SSI1_BASE, request >> 16);
    while (SSIBusy(SSI1_BASE))
        ;
    // wait until the busy bit is cleared
    SSIDataGet(SSI1_BASE, &temp); // Read response high word
    response = temp <<= 16;
    temp = 0x00;

    SSIDataPut(SSI1_BASE, request & 0x000FFFF); //send request low word
    while (SSIBusy(SSI1_BASE))
        ;

    SSIDataGet(SSI1_BASE, &temp); // Read response low word
    response = response | temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_3, 0xff);

    return response;
}

// Send Request to SCC4
uint32_t send_request_SCC4(uint32_t request)
{
    uint32_t response;
    uint32_t temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0x00); // Chip select on
    SSIDataGetNonBlocking(SSI1_BASE, &temp); // read rx buffer to clear it.
    temp = 0x00;

    SSIDataPut(SSI1_BASE, request >> 16);
    while (SSIBusy(SSI1_BASE))
        ;
    // wait until the busy bit is cleared
    SSIDataGet(SSI1_BASE, &temp); // Read response high word
    response = temp <<= 16;
    temp = 0x00;

    SSIDataPut(SSI1_BASE, request & 0x000FFFF); //send request low word
    while (SSIBusy(SSI1_BASE))
        ;

    SSIDataGet(SSI1_BASE, &temp); // Read response low word
    response = response | temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_4, 0xff);

    return response;
}

// Send Request to SCC5
uint32_t send_request_SCC5(uint32_t request)
{
    uint32_t response;
    uint32_t temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0x00); // Chip select on
    SSIDataGetNonBlocking(SSI2_BASE, &temp); // read rx buffer to clear it.
    temp = 0x00;

    SSIDataPut(SSI2_BASE, request >> 16);
    while (SSIBusy(SSI2_BASE))
        ;
    // wait until the busy bit is cleared
    SSIDataGet(SSI2_BASE, &temp); // Read response high word
    response = temp <<= 16;
    temp = 0x00;

    SSIDataPut(SSI2_BASE, request & 0x000FFFF); //send request low word
    while (SSIBusy(SSI2_BASE))
        ;

    SSIDataGet(SSI2_BASE, &temp); // Read response low word
    response = response | temp;

    GPIOPinWrite(GPIO_PORTE_BASE, GPIO_PIN_5, 0xff);

    return response;
}

// Send Request to SCC6
uint32_t send_request_SCC6(uint32_t request)
{
    uint32_t response;
    uint32_t temp;

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00); // Chip select on
    SSIDataGetNonBlocking(SSI2_BASE, &temp); // read rx buffer to clear it.
    temp = 0x00;

    SSIDataPut(SSI2_BASE, request >> 16);
    while (SSIBusy(SSI2_BASE))
        ;
    // wait until the busy bit is cleared
    SSIDataGet(SSI2_BASE, &temp); // Read response high word
    response = temp <<= 16;
    temp = 0x00;

    SSIDataPut(SSI2_BASE, request & 0x000FFFF); //send request low word
    while (SSIBusy(SSI2_BASE))
        ;

    SSIDataGet(SSI2_BASE, &temp); // Read response low word
    response = response | temp;

    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0xff);

    return response;
}

Output_scc read_and_process_gyro_SCC1(void)
{
    uint16_t rate;
    uint16_t temp;
    uint32_t response_temp;
    uint32_t response_rate;
    uint32_t result_rate;
    uint32_t result_temp;
    uint8_t RSdata;
    Output_scc output;

    response_temp = send_request_SCC1(REQ_READ_RATE);
    response_rate = send_request_SCC1(REQ_READ_TEMP);

    //Handle rate data
    rate = (response_rate & DATA_FIELD_MASK) >> 8;
    RSdata = (response_rate & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    //Handle temperature data
    temp = (response_temp & DATA_FIELD_MASK) >> 8;
    RSdata = (response_temp & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    result_rate = rate;
    output.gyro = (short int) result_rate;
    result_temp = temp;
    output.temp = (short int) result_temp;

    return output;

}

Output_scc read_and_process_gyro_SCC2(void)
{
    uint16_t rate;
    uint16_t temp;
    uint32_t response_temp;
    uint32_t response_rate;
    uint32_t result_rate;
    uint32_t result_temp;
    uint8_t RSdata;
    Output_scc output;

    response_temp = send_request_SCC2(REQ_READ_RATE);
    response_rate = send_request_SCC2(REQ_READ_TEMP);

    //Handle rate data
    rate = (response_rate & DATA_FIELD_MASK) >> 8;
    RSdata = (response_rate & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    //Handle temperature data
    temp = (response_temp & DATA_FIELD_MASK) >> 8;
    RSdata = (response_temp & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    result_rate = rate;
    output.gyro = (short int) result_rate;
    result_temp = temp;
    output.temp = (short int) result_temp;

    return output;
}

Output_scc read_and_process_gyro_SCC3(void)
{
    uint16_t rate;
    uint16_t temp;
    uint32_t response_temp;
    uint32_t response_rate;
    uint32_t result_rate;
    uint32_t result_temp;
    uint8_t RSdata;
    Output_scc output;

    response_temp = send_request_SCC3(REQ_READ_RATE);
    response_rate = send_request_SCC3(REQ_READ_TEMP);

    //Handle rate data
    rate = (response_rate & DATA_FIELD_MASK) >> 8;
    RSdata = (response_rate & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    //Handle temperature data
    temp = (response_temp & DATA_FIELD_MASK) >> 8;
    RSdata = (response_temp & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;
    result_rate = rate;
    output.gyro = (short int) result_rate;
    result_temp = temp;
    output.temp = (short int) result_temp;

    return output;
}

Output_scc read_and_process_gyro_SCC4(void)
{
    uint16_t rate;
    uint16_t temp;
    uint32_t response_temp;
    uint32_t response_rate;
    uint32_t result_rate;
    uint32_t result_temp;
    uint8_t RSdata;
    Output_scc output;

    response_temp = send_request_SCC4(REQ_READ_RATE);
    response_rate = send_request_SCC4(REQ_READ_TEMP);

    //Handle rate data
    rate = (response_rate & DATA_FIELD_MASK) >> 8;
    RSdata = (response_rate & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    //Handle temperature data
    temp = (response_temp & DATA_FIELD_MASK) >> 8;
    RSdata = (response_temp & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    result_rate = rate;
    output.gyro = (short int) result_rate;
    result_temp = temp;
    output.temp = (short int) result_temp;

    return output;
}

Output_scc read_and_process_gyro_SCC5(void)
{
    uint16_t rate;
    uint16_t temp;
    uint32_t response_temp;
    uint32_t response_rate;
    uint32_t result_rate;
    uint32_t result_temp;
    uint8_t RSdata;
    Output_scc output;

    response_temp = send_request_SCC5(REQ_READ_RATE);
    response_rate = send_request_SCC5(REQ_READ_TEMP);

    //Handle rate data
    rate = (response_rate & DATA_FIELD_MASK) >> 8;
    RSdata = (response_rate & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    //Handle temperature data
    temp = (response_temp & DATA_FIELD_MASK) >> 8;
    RSdata = (response_temp & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    result_rate = rate;
    output.gyro = (short int) result_rate;
    result_temp = temp;
    output.temp = (short int) result_temp;

    return output;
}

Output_scc read_and_process_gyro_SCC6(void)
{
    uint16_t rate;
    uint16_t temp;
    uint32_t response_temp;
    uint32_t response_rate;
    uint32_t result_rate;
    uint32_t result_temp;
    uint8_t RSdata;
    Output_scc output;

    response_temp = send_request_SCC6(REQ_READ_RATE);
    response_rate = send_request_SCC6(REQ_READ_TEMP);

    //Handle rate data
    rate = (response_rate & DATA_FIELD_MASK) >> 8;
    RSdata = (response_rate & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    //Handle temperature data
    temp = (response_temp & DATA_FIELD_MASK) >> 8;
    RSdata = (response_temp & RS_FIELD_MASK) >> 24;
    if (RSdata != 1)
        data_error = true;

    result_rate = rate;
    output.gyro = (short int) result_rate;
    result_temp = temp;
    output.temp = (short int) result_temp;

    return output;
}

void init_scc2130(void)
{

    //sensor 1 power up...
    delayMs(25);               // 25ms
    //send_request_SCC1(REQ_WRITE_FLT_60); // set output filter to 60 hz
    send_request_SCC2(REQ_WRITE_FLT_60); // set output filter to 60 hz
    send_request_SCC3(REQ_WRITE_FLT_60); // set output filter to 60 hz
    send_request_SCC4(REQ_WRITE_FLT_60); // set output filter to 60 hz
    send_request_SCC5(REQ_WRITE_FLT_60); // set output filter to 60 hz
    send_request_SCC6(REQ_WRITE_FLT_60); // set output filter to 60 hz
    delayMs(595);               // 595 ms;

    //Clear status registers for sensor 1.
    //send_request_SCC1(REQ_READ_RATE_STAT1);
    //send_request_SCC1(REQ_READ_RATE_STAT2);
    //send_request_SCC1(REQ_READ_ACC_STAT);
    //send_request_SCC1(REQ_READ_COM_STAT1);
    //send_request_SCC1(REQ_READ_STAT_SUM);

    //Clear status registers for sensor 2.
    send_request_SCC2(REQ_READ_RATE_STAT1);
    send_request_SCC2(REQ_READ_RATE_STAT2);
    send_request_SCC2(REQ_READ_ACC_STAT);
    send_request_SCC2(REQ_READ_COM_STAT1);
    send_request_SCC2(REQ_READ_STAT_SUM);

    //Clear status registers for sensor 3.
    send_request_SCC3(REQ_READ_RATE_STAT1);
    send_request_SCC3(REQ_READ_RATE_STAT2);
    send_request_SCC3(REQ_READ_ACC_STAT);
    send_request_SCC3(REQ_READ_COM_STAT1);
    send_request_SCC3(REQ_READ_STAT_SUM);

    //Clear status registers for sensor 4.
    send_request_SCC4(REQ_READ_RATE_STAT1);
    send_request_SCC4(REQ_READ_RATE_STAT2);
    send_request_SCC4(REQ_READ_ACC_STAT);
    send_request_SCC4(REQ_READ_COM_STAT1);
    send_request_SCC4(REQ_READ_STAT_SUM);

    //Clear status registers for sensor 5.
    send_request_SCC5(REQ_READ_RATE_STAT1);
    send_request_SCC5(REQ_READ_RATE_STAT2);
    send_request_SCC5(REQ_READ_ACC_STAT);
    send_request_SCC5(REQ_READ_COM_STAT1);
    send_request_SCC5(REQ_READ_STAT_SUM);

    //Clear status registers for sensor 6.
    send_request_SCC6(REQ_READ_RATE_STAT1);
    send_request_SCC6(REQ_READ_RATE_STAT2);
    send_request_SCC6(REQ_READ_ACC_STAT);
    send_request_SCC6(REQ_READ_COM_STAT1);
    send_request_SCC6(REQ_READ_STAT_SUM);

}

Status_scc read_scc_status(int scc)
{
    Status_scc output;
    uint32_t response_comstat1;
    uint32_t response_statsum;
    uint32_t response_ratestat1;
    uint32_t response_ratestat2;
    uint32_t response_accstat;

    if (scc == 1)
    {
        response_comstat1 = send_request_SCC1(REQ_READ_STAT_SUM);
        response_statsum = send_request_SCC1(REQ_READ_RATE_STAT1);
        response_ratestat1 = send_request_SCC1(REQ_READ_RATE_STAT2);
        response_ratestat2 = send_request_SCC1(REQ_READ_ACC_STAT);
        response_accstat = send_request_SCC1(REQ_READ_COM_STAT1);

        output.ComStat1 = (response_comstat1 & DATA_FIELD_MASK) >> 8;
        output.StatSum = (response_statsum & DATA_FIELD_MASK) >> 8;
        output.RateStat1 = (response_ratestat1 & DATA_FIELD_MASK) >> 8;
        output.RateStat2 = (response_ratestat2 & DATA_FIELD_MASK) >> 8;
        output.AccStat = (response_accstat & DATA_FIELD_MASK) >> 8;
    }
    else if (scc == 2)
    {
        response_comstat1 = send_request_SCC2(REQ_READ_STAT_SUM);
        response_statsum = send_request_SCC2(REQ_READ_RATE_STAT1);
        response_ratestat1 = send_request_SCC2(REQ_READ_RATE_STAT2);
        response_ratestat2 = send_request_SCC2(REQ_READ_ACC_STAT);
        response_accstat = send_request_SCC2(REQ_READ_COM_STAT1);

        output.ComStat1 = (response_comstat1 & DATA_FIELD_MASK) >> 8;
        output.StatSum = (response_statsum & DATA_FIELD_MASK) >> 8;
        output.RateStat1 = (response_ratestat1 & DATA_FIELD_MASK) >> 8;
        output.RateStat2 = (response_ratestat2 & DATA_FIELD_MASK) >> 8;
        output.AccStat = (response_accstat & DATA_FIELD_MASK) >> 8;
    }
    else if (scc == 3)
    {
        response_comstat1 = send_request_SCC3(REQ_READ_STAT_SUM);
        response_statsum = send_request_SCC3(REQ_READ_RATE_STAT1);
        response_ratestat1 = send_request_SCC3(REQ_READ_RATE_STAT2);
        response_ratestat2 = send_request_SCC3(REQ_READ_ACC_STAT);
        response_accstat = send_request_SCC3(REQ_READ_COM_STAT1);

        output.ComStat1 = (response_comstat1 & DATA_FIELD_MASK) >> 8;
        output.StatSum = (response_statsum & DATA_FIELD_MASK) >> 8;
        output.RateStat1 = (response_ratestat1 & DATA_FIELD_MASK) >> 8;
        output.RateStat2 = (response_ratestat2 & DATA_FIELD_MASK) >> 8;
        output.AccStat = (response_accstat & DATA_FIELD_MASK) >> 8;
    }
    else if (scc == 4)
    {
        response_comstat1 = send_request_SCC4(REQ_READ_STAT_SUM);
        response_statsum = send_request_SCC4(REQ_READ_RATE_STAT1);
        response_ratestat1 = send_request_SCC4(REQ_READ_RATE_STAT2);
        response_ratestat2 = send_request_SCC4(REQ_READ_ACC_STAT);
        response_accstat = send_request_SCC4(REQ_READ_COM_STAT1);

        output.ComStat1 = (response_comstat1 & DATA_FIELD_MASK) >> 8;
        output.StatSum = (response_statsum & DATA_FIELD_MASK) >> 8;
        output.RateStat1 = (response_ratestat1 & DATA_FIELD_MASK) >> 8;
        output.RateStat2 = (response_ratestat2 & DATA_FIELD_MASK) >> 8;
        output.AccStat = (response_accstat & DATA_FIELD_MASK) >> 8;
    }
    else if (scc == 5)
    {
        response_comstat1 = send_request_SCC5(REQ_READ_STAT_SUM);
        response_statsum = send_request_SCC5(REQ_READ_RATE_STAT1);
        response_ratestat1 = send_request_SCC5(REQ_READ_RATE_STAT2);
        response_ratestat2 = send_request_SCC5(REQ_READ_ACC_STAT);
        response_accstat = send_request_SCC5(REQ_READ_COM_STAT1);

        output.ComStat1 = (response_comstat1 & DATA_FIELD_MASK) >> 8;
        output.StatSum = (response_statsum & DATA_FIELD_MASK) >> 8;
        output.RateStat1 = (response_ratestat1 & DATA_FIELD_MASK) >> 8;
        output.RateStat2 = (response_ratestat2 & DATA_FIELD_MASK) >> 8;
        output.AccStat = (response_accstat & DATA_FIELD_MASK) >> 8;
    }
    else if (scc == 6)
    {
        response_comstat1 = send_request_SCC6(REQ_READ_STAT_SUM);
        response_statsum = send_request_SCC6(REQ_READ_RATE_STAT1);
        response_ratestat1 = send_request_SCC6(REQ_READ_RATE_STAT2);
        response_ratestat2 = send_request_SCC6(REQ_READ_ACC_STAT);
        response_accstat = send_request_SCC6(REQ_READ_COM_STAT1);

        output.ComStat1 = (response_comstat1 & DATA_FIELD_MASK) >> 8;
        output.StatSum = (response_statsum & DATA_FIELD_MASK) >> 8;
        output.RateStat1 = (response_ratestat1 & DATA_FIELD_MASK) >> 8;
        output.RateStat2 = (response_ratestat2 & DATA_FIELD_MASK) >> 8;
        output.AccStat = (response_accstat & DATA_FIELD_MASK) >> 8;
    }
    else
    {
        output.ComStat1 = 0;
        output.StatSum = 0;
        output.RateStat1 = 0;
        output.RateStat2 = 0;
        output.AccStat = 0;
    }

    return output;
}

