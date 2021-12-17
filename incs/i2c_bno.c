#include <stdint.h>
#include <stdarg.h>
#include <stdbool.h>

#include "inc/tm4c123gh6pm.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"

#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"

#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "driverlib/qei.h"

//#include "incs/i2c_bno.h"
#include "i2c_bno.h"

uint8_t m_page = 0;
uint8_t m_operation_mode = BNO055_OPERATION_MODE_CONFIG;
uint8_t m_power_mode = BNO055_POWER_MODE_NORMAL;

void ConfigureI2C0()
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    // Wait for peripheral to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C0))
    {
    }

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}

void ConfigureI2C1()
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);

    // Wait for peripheral to be ready
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_I2C1))
    {
    }

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);

    //enable GPIO peripheral that contains I2C 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    // Configure the pin muxing for I2C0 functions
    GPIOPinConfigure(GPIO_PA6_I2C1SCL);
    GPIOPinConfigure(GPIO_PA7_I2C1SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C1_BASE + I2C_O_FIFOCTL) = 80008000;
}

void send_request_i2c(uint32_t t_addr, uint32_t t_register, uint32_t t_data,
                      int select_i2c)
{
    if (select_i2c == 0) // select i2c0
    {
        // Tell the master module what address it will place on the bus when communicating with the slave.
        I2CMasterSlaveAddrSet(I2C0_BASE, t_addr, false);
        //put data to be sent into FIFO
        I2CMasterDataPut(I2C0_BASE, t_register);
        //Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        // Wait until MCU is done transferring.
        while (I2CMasterBusy(I2C0_BASE))
            ;
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C0_BASE, t_data);
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while (I2CMasterBusy(I2C0_BASE))
            ;
    }

    else if (select_i2c == 1) // select i2c1
    {
        // Tell the master module what address it will place on the bus when communicating with the slave.
        I2CMasterSlaveAddrSet(I2C1_BASE, t_addr, false);
        //put data to be sent into FIFO
        I2CMasterDataPut(I2C1_BASE, t_register);
        //Initiate send of data from the MCU
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        // Wait until MCU is done transferring.
        while (I2CMasterBusy(I2C1_BASE))
            ;
        //put last piece of data into I2C FIFO
        I2CMasterDataPut(I2C1_BASE, t_data);
        //send next data that was just placed into FIFO
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);
        // Wait until MCU is done transferring.
        while (I2CMasterBusy(I2C1_BASE))
            ;
    }
    else
    {
        //nothing...
    }
}

uint32_t recieve_request_i2c(uint32_t t_addr, uint32_t t_register, int select_i2c)
{
    if (select_i2c == 0) // select i2c0
    {
        //specify that we are writing (a register address) to the slave device
        I2CMasterSlaveAddrSet(I2C0_BASE, t_addr, false);
        //specify register to be read
        I2CMasterDataPut(I2C0_BASE, t_register);
        //send control byte and register address byte to slave device
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        //wait for MCU to finish transaction
        while (I2CMasterBusy(I2C0_BASE))
            ;
        //specify that we are going to read from slave device
        I2CMasterSlaveAddrSet(I2C0_BASE, t_addr, true);
        //send control byte and read from the register we specified
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        //wait for MCU to finish transaction
        while (I2CMasterBusy(I2C0_BASE))
            ;
        //return data pulled from the specified register
        I2CMasterDataGet(I2C0_BASE);
        return I2CMasterDataGet(I2C0_BASE);

    }

    else if (select_i2c == 1) // select i2c1
    {
        //specify that we are writing (a register address) to the slave device
        I2CMasterSlaveAddrSet(I2C1_BASE, t_addr, false);
        //specify register to be read
        I2CMasterDataPut(I2C1_BASE, t_register);
        //send control byte and register address byte to slave device
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);
        //wait for MCU to finish transaction
        while (I2CMasterBusy(I2C1_BASE))
            ;
        //specify that we are going to read from slave device
        I2CMasterSlaveAddrSet(I2C1_BASE, t_addr, true);
        //send control byte and read from the register we specified
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        //wait for MCU to finish transaction
        while (I2CMasterBusy(I2C1_BASE))
            ;
        //return data pulled from the specified register
        I2CMasterDataGet(I2C1_BASE);
        return I2CMasterDataGet(I2C1_BASE);
    }

    else
    {
        return 0;
    }
}

void write_bno055(uint8_t t_page, uint8_t t_register, uint8_t t_data,
                  uint8_t addr, int select_i2c)
{
    if (m_page != t_page)
    {
        send_request_i2c(addr, BNO055_PAGE_ID_ADDR, t_page, select_i2c);
        m_page = t_page;
    }

    send_request_I2C0(addr, t_register, t_data);
}

uint8_t read_bno055(uint8_t t_page, uint8_t t_register, uint8_t addr,
                    int select_i2c)
{
    uint32_t ret_val;

    if (m_page != t_page)
    {
        send_request_i2c(addr, BNO055_PAGE_ID_ADDR, t_page, select_i2c);
        m_page = t_page;
    }

    ret_val = recieve_request_i2c(addr, t_register, select_i2c);
    return ret_val;
}

void set_op_mode_bno055(uint8_t t_operation_mode, uint8_t bno_addr,
                        int select_i2c)
{

    uint8_t prev_operation_mode = m_operation_mode;
    if (prev_operation_mode == BNO055_OPERATION_MODE_CONFIG)
    {
        write_bno055(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR, t_operation_mode,
                     bno_addr, select_i2c);
        SysCtlDelay(SysCtlClockGet() / 426);
    }
    else
    {
        write_bno055(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
        BNO055_OPERATION_MODE_CONFIG,
                     bno_addr, select_i2c);
        SysCtlDelay(SysCtlClockGet() / 159);
        if (t_operation_mode != BNO055_OPERATION_MODE_CONFIG)
        {
            write_bno055(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
            BNO055_OPERATION_MODE_CONFIG,
                         bno_addr, select_i2c);
            SysCtlDelay(SysCtlClockGet() / 426);
        }
    }
    m_operation_mode = t_operation_mode;
}

void set_power_mode_bno055(uint8_t t_power_mode, uint8_t bno_addr,
                           int select_i2c)
{

    uint8_t prev_operation_mode = m_operation_mode;
    if (prev_operation_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        set_op_mode_bno055(BNO055_OPERATION_MODE_CONFIG, bno_addr, select_i2c);
    }
    m_power_mode = t_power_mode;

    write_bno055(BNO055_PAGE_ZERO, BNO055_PWR_MODE_ADDR, t_power_mode, bno_addr,
                 select_i2c);
    SysCtlDelay(SysCtlClockGet() / 426);

    if (prev_operation_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        set_op_mode_bno055(prev_operation_mode, bno_addr, select_i2c);
    }
}

void init_bno055(uint8_t bno_addr, int select_i2c)
{
    // Select BNO055 config mode
    set_op_mode_bno055(BNO055_OPERATION_MODE_CONFIG, bno_addr, select_i2c);
    SysCtlDelay(SysCtlClockGet() / 200); //7ms
    uint8_t hold = 0x1B;
    // Configure GYR 230 hz and 250 dps
    write_bno055(BNO055_PAGE_ONE, BNO055_GYRO_CONFIG_ADDR, hold, bno_addr,
                 select_i2c);
    write_bno055(BNO055_PAGE_ONE, BNO055_GYRO_MODE_CONFIG_ADDR, 0x00, bno_addr,
                 select_i2c);

    // Select BNO055 gyro temperature source
    write_bno055(BNO055_PAGE_ZERO, BNO055_TEMP_SOURCE_ADDR, 0x01, bno_addr,
                 select_i2c);

    SysCtlDelay(SysCtlClockGet() / 200);
    set_op_mode_bno055(BNO055_OPERATION_MODE_GYRONLY, bno_addr, select_i2c);
    SysCtlDelay(SysCtlClockGet() / 200);

    write_bno055(BNO055_PAGE_ZERO, BNO055_UNIT_SEL_ADDR, 0x00, bno_addr,
                 select_i2c);

    // Select BNO055 system power mode
    set_power_mode_bno055(BNO055_POWER_MODE_NORMAL, bno_addr, select_i2c);
    SysCtlDelay(SysCtlClockGet() / 500);
}

bno055_gyro_raw bno055_read_gyro_I2C0(uint8_t bno_addr)
{
    bno055_gyro_raw output;
    int8_t lsb, msb;
    int16_t gyro_x, gyro_y, gyro_z;

    lsb = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_X_LSB_ADDR,
                           bno_addr);
    msb = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_X_MSB_ADDR,
                           bno_addr);
    gyro_x = (msb << 8) | lsb;
    output.x = (short int) gyro_x;

    lsb = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Y_LSB_ADDR,
                           bno_addr);
    msb = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Y_MSB_ADDR,
                           bno_addr);
    gyro_y = (msb << 8) | lsb;
    output.y = (short int) gyro_y;

    lsb = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Z_LSB_ADDR,
                           bno_addr);
    msb = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Z_MSB_ADDR,
                           bno_addr);
    gyro_z = (msb << 8) | lsb;
    output.z = (short int) gyro_z;

    return output;
}

bno055_gyro_raw bno055_read_gyro_I2C1(uint8_t bno_addr)
{
    bno055_gyro_raw output;
    int8_t lsb, msb;
    int16_t gyro_x, gyro_y, gyro_z;

    lsb = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_X_LSB_ADDR,
                           bno_addr);
    msb = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_X_MSB_ADDR,
                           bno_addr);
    gyro_x = (msb << 8) | lsb;
    output.x = (short int) gyro_x;
    lsb = 0, msb = 0;

    lsb = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Y_LSB_ADDR,
                           bno_addr);
    msb = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Y_MSB_ADDR,
                           bno_addr);
    gyro_y = (msb << 8) | lsb;
    output.y = (short int) gyro_y;
    lsb = 0, msb = 0;

    lsb = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Z_LSB_ADDR,
                           bno_addr);
    msb = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_GYRO_DATA_Z_MSB_ADDR,
                           bno_addr);
    gyro_z = (msb << 8) | lsb;
    output.z = (short int) gyro_z;

    return output;
}

short int bno055_read_temp_I2C0(uint8_t bno_addr)
{

    short int signed_degree;
    int16_t temp;

    temp = read_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_TEMP_ADDR, bno_addr);

    signed_degree = (short int) temp;
    return signed_degree;
}

short int bno055_read_temp_I2C1(uint8_t bno_addr)
{

    short int signed_degree;
    int16_t temp;

    temp = read_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_TEMP_ADDR, bno_addr);

    signed_degree = (short int) temp;
    return signed_degree;
}

