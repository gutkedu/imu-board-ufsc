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

static s8 error1;
static s8 error2;
static s8 error3;
static s8 error4;
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

s8 _imu_i2c_read_i2c0(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count)
{
    s8 comres = 0;
    // This function is used to select the device to read from
    // false == write to slave
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, false);

    // Set the I2C Bus to tell the device which first register is meant to be read
    I2CMasterDataPut(I2C0_BASE, reg_address);

    // send slave address, control bit, and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while (I2CMasterBusy(I2C0_BASE))
        ;

    // Read in 1 byte
    if (count == 1)
    {
        // specify that we are going to read from slave device
        // true == read from slave
        I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);

        //send slave address, control bit, and recieve the byte of data
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while (I2CMasterBusy(I2C0_BASE))
            ;

        // write byte to data variable
        arr_data[0] = I2CMasterDataGet(I2C0_BASE);

        // Check for errors
        if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // success
        }
        else
        {
            comres = -1; // error occured
        }
    }

    // Read in 2 bytes
    else if (count == 2)
    {
        // Get first byte
        I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while (I2CMasterBusy(I2C0_BASE))
            ;

        // put read data in data array
        arr_data[0] = I2CMasterDataGet(I2C0_BASE);

        // Check for errors
        if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // success
        }
        else
        {
            comres = -1; // error occured
        }

        // Read final byte from slave
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while (I2CMasterBusy(I2C0_BASE))
            ;
        // put read data in data array
        arr_data[1] = I2CMasterDataGet(I2C0_BASE);

        // Check for errors
        if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // success
        }
        else
        {
            comres = -1; // error occured
        }

    }

    // Read in more than 2 bytes
    else
    {
        u8 i;
        for (i = 0; i < count; i++)
        {
            if (i == 0)
            {
                // Start Communication
                I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, true);
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
            }
            else if (i == count - 1)
            {
                // Read the last byte
                I2CMasterControl(I2C0_BASE,
                I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            }
            else
            {
                // read middle byte
                I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            }

            while (I2CMasterBusy(I2C0_BASE))
                ;

            // put read data in data array
            arr_data[i] = I2CMasterDataGet(I2C0_BASE);

            if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
            {
                comres = 0; // success
            }
            else
            {
                comres = -1; // error occured, end comms and exit
                I2CMasterControl(I2C0_BASE,
                I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                while (I2CMasterBusy(I2C0_BASE))
                    ;
                i = count;
            }
        }
    }

    return comres;
}

s8 _imu_i2c_write_i2c0(u8 dev_address, u8 reg_address, u8 *var_data, u8 count)
{
    s8 comres = 0;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_address, false);

    //send the slave address, control bit, and registar address for where to write to
    I2CMasterDataPut(I2C0_BASE, reg_address);

    //Initiate send of data from the MCU
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until MCU is done transferring.
    while (I2CMasterBusy(I2C0_BASE))
        ;

    // the BNO055 only ever writes 1 byte of info so if count != 1, throw an error
    if (count == 1)
    {

        // send the information to write
        I2CMasterDataPut(I2C0_BASE, *var_data);

        // Initiate send of data from the MCU
        I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Wait until MCU is done transferring
        while (I2CMasterBusy(I2C0_BASE))
            ;

        if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // Success
        }
        else
        {
            comres = -1; // Error
        }
    }
    else
    {
        comres = -1; // Error
    }

    return comres;
}

s8 _imu_i2c_read_i2c1(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count)
{
    s8 comres = 0;
    // This function is used to select the device to read from
    // false == write to slave
    I2CMasterSlaveAddrSet(I2C1_BASE, dev_address, false);

    // Set the I2C Bus to tell the device which first register is meant to be read
    I2CMasterDataPut(I2C1_BASE, reg_address);

    // send slave address, control bit, and register address byte to slave device
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while (I2CMasterBusy(I2C1_BASE))
        ;

    // Read in 1 byte
    if (count == 1)
    {
        // specify that we are going to read from slave device
        // true == read from slave
        I2CMasterSlaveAddrSet(I2C1_BASE, dev_address, true);

        //send slave address, control bit, and recieve the byte of data
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
        while (I2CMasterBusy(I2C1_BASE))
            ;

        // write byte to data variable
        arr_data[0] = I2CMasterDataGet(I2C1_BASE);

        // Check for errors
        if (I2CMasterErr(I2C1_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // success
        }
        else
        {
            comres = -1; // error occured
        }
    }

    // Read in 2 bytes
    else if (count == 2)
    {
        // Get first byte
        I2CMasterSlaveAddrSet(I2C1_BASE, dev_address, true);
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
        while (I2CMasterBusy(I2C1_BASE))
            ;

        // put read data in data array
        arr_data[0] = I2CMasterDataGet(I2C1_BASE);

        // Check for errors
        if (I2CMasterErr(I2C1_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // success
        }
        else
        {
            comres = -1; // error occured
        }

        // Read final byte from slave
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
        while (I2CMasterBusy(I2C1_BASE))
            ;
        // put read data in data array
        arr_data[1] = I2CMasterDataGet(I2C1_BASE);

        // Check for errors
        if (I2CMasterErr(I2C1_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // success
        }
        else
        {
            comres = -1; // error occured
        }

    }

    // Read in more than 2 bytes
    else
    {
        u8 i;
        for (i = 0; i < count; i++)
        {
            if (i == 0)
            {
                // Start Communication
                I2CMasterSlaveAddrSet(I2C1_BASE, dev_address, true);
                I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_START);
            }
            else if (i == count - 1)
            {
                // Read the last byte
                I2CMasterControl(I2C1_BASE,
                I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
            }
            else
            {
                // read middle byte
                I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_RECEIVE_CONT);
            }

            while (I2CMasterBusy(I2C1_BASE))
                ;

            // put read data in data array
            arr_data[i] = I2CMasterDataGet(I2C1_BASE);

            if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
            {
                comres = 0; // success
            }
            else
            {
                comres = -1; // error occured, end comms and exit
                I2CMasterControl(I2C1_BASE,
                I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
                while (I2CMasterBusy(I2C1_BASE))
                    ;
                i = count;
            }
        }
    }

    return comres;
}

s8 _imu_i2c_write_i2c1(u8 dev_address, u8 reg_address, u8 *var_data, u8 count)
{
    s8 comres = 0;
    // Tell the master module what address it will place on the bus when
    // communicating with the slave.
    I2CMasterSlaveAddrSet(I2C1_BASE, dev_address, false);

    //send the slave address, control bit, and registar address for where to write to
    I2CMasterDataPut(I2C1_BASE, reg_address);

    //Initiate send of data from the MCU
    I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    // Wait until MCU is done transferring.
    while (I2CMasterBusy(I2C1_BASE))
        ;

    // the BNO055 only ever writes 1 byte of info so if count != 1, throw an error
    if (count == 1)
    {

        // send the information to write
        I2CMasterDataPut(I2C1_BASE, *var_data);

        // Initiate send of data from the MCU
        I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_BURST_SEND_FINISH);

        // Wait until MCU is done transferring
        while (I2CMasterBusy(I2C1_BASE))
            ;

        if (I2CMasterErr(I2C1_BASE) == I2C_MASTER_ERR_NONE)
        {
            comres = 0; // Success
        }
        else
        {
            comres = -1; // Error
        }
    }
    else
    {
        comres = -1; // Error
    }

    return comres;
}

void ms_delay(u32 ms)
{
    SysCtlDelay(ms * 5334); // 16000000MHz/3000 ~= 5334 assembly commands per ms
}

void init_bno_1()
{
    static struct bno055_t sensor_1;

    // initialize setup struct and populate the required information
    sensor_1.bus_write = _imu_i2c_write_i2c0;
    sensor_1.bus_read = _imu_i2c_read_i2c0;
    sensor_1.delay_msec = ms_delay;
    sensor_1.dev_addr = BNO055_I2C_ADDR1;

    // bno055 builtin initialization function
    error1 = bno055_init(&sensor_1);
}

void init_bno_2()
{
    static struct bno055_t sensor_2;
    sensor_2.bus_write = _imu_i2c_write_i2c0;
    sensor_2.bus_read = _imu_i2c_read_i2c0;
    sensor_2.delay_msec = ms_delay;
    sensor_2.dev_addr = BNO055_I2C_ADDR2;

    error2 = bno055_init(&sensor_2);
}

void init_bno_3()
{
    static struct bno055_t sensor_3;
    sensor_3.bus_write = _imu_i2c_write_i2c1;
    sensor_3.bus_read = _imu_i2c_read_i2c1;
    sensor_3.delay_msec = ms_delay;
    sensor_3.dev_addr = BNO055_I2C_ADDR1;

    error3 = bno055_init(&sensor_3);
}

void init_bno_4()
{
    static struct bno055_t sensor_4;
    sensor_4.bus_write = _imu_i2c_write_i2c1;
    sensor_4.bus_read = _imu_i2c_read_i2c1;
    sensor_4.delay_msec = ms_delay;
    sensor_4.dev_addr = BNO055_I2C_ADDR2;

    error4 = bno055_init(&sensor_4);
}

void set_bno_1_mode(u8 op_mode)
{
    error1 = bno055_set_operation_mode(op_mode);
}

Calibration calibrate_bno_1()
{
    u8 gyro_cal = 0, acc_cal = 0, mag_cal = 0, sys_cal = 0;

    error1 = bno055_get_gyro_calib_stat(&gyro_cal);
    error1 += bno055_get_mag_calib_stat(&mag_cal);
    error1 += bno055_get_accel_calib_stat(&acc_cal);
    error1 += bno055_get_sys_calib_stat(&sys_cal);

    Calibration output_1;
    output_1.gyro = gyro_cal;
    output_1.accl = acc_cal;
    output_1.magn = mag_cal;
    output_1.syst = sys_cal;

    return output_1;
}

s8 get_error()
{
    return error1;
}

//*************************************

void send_request_I2C0(uint32_t t_addr, uint32_t t_register, uint32_t t_data)
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

uint32_t recieve_request_I2C0(uint32_t t_addr, uint32_t t_register)
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

void send_request_I2C1(uint32_t t_addr, uint32_t t_register, uint32_t t_data)
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

uint32_t recieve_request_I2C1(uint32_t t_addr, uint32_t t_register)
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

void write_bno055_I2C0(uint8_t t_page, uint8_t t_register, uint8_t t_data,
                       uint8_t addr)
{
    if (m_page != t_page)
    {
        send_request_I2C0(addr, BNO055_PAGE_ID_ADDR, t_page);
        m_page = t_page;
    }

    send_request_I2C0(addr, t_register, t_data);
}

uint8_t read_bno055_I2C0(uint8_t t_page, uint8_t t_register, uint8_t addr)
{
    uint32_t ret_val;

    if (m_page != t_page)
    {
        send_request_I2C0(addr, BNO055_PAGE_ID_ADDR, t_page);
        m_page = t_page;
    }

    ret_val = recieve_request_I2C0(addr, t_register);
    //UARTprintf("ret_val: %d\n", ret_val);
    return ret_val;
}

void write_bno055_I2C1(uint8_t t_page, uint8_t t_register, uint8_t t_data,
                       uint8_t addr)
{
    if (m_page != t_page)
    {
        send_request_I2C1(addr, BNO055_PAGE_ID_ADDR, t_page);
        m_page = t_page;
    }

    send_request_I2C1(addr, t_register, t_data);
}

uint8_t read_bno055_I2C1(uint8_t t_page, uint8_t t_register, uint8_t addr)
{
    uint32_t ret_val;

    if (m_page != t_page)
    {
        send_request_I2C1(addr, BNO055_PAGE_ID_ADDR, t_page);
        m_page = t_page;
    }

    ret_val = recieve_request_I2C1(addr, t_register);
    //UARTprintf("ret_val: %d\n", ret_val);
    return ret_val;
}

void set_op_mode_bno055_I2C0(uint8_t t_operation_mode, uint8_t bno_addr)
{

    uint8_t prev_operation_mode = m_operation_mode;
    if (prev_operation_mode == BNO055_OPERATION_MODE_CONFIG)
    {
        write_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
                          t_operation_mode, bno_addr);
        SysCtlDelay(SysCtlClockGet() / 426);
    }
    else
    {
        write_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
        BNO055_OPERATION_MODE_CONFIG,
                          bno_addr);
        SysCtlDelay(SysCtlClockGet() / 159);
        if (t_operation_mode != BNO055_OPERATION_MODE_CONFIG)
        {
            write_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
            BNO055_OPERATION_MODE_CONFIG,
                              bno_addr);
            SysCtlDelay(SysCtlClockGet() / 426);
        }
    }
    m_operation_mode = t_operation_mode;
}

void set_op_mode_bno055_I2C1(uint8_t t_operation_mode, uint8_t bno_addr)
{

    uint8_t prev_operation_mode = m_operation_mode;
    if (prev_operation_mode == BNO055_OPERATION_MODE_CONFIG)
    {
        write_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
                          t_operation_mode, bno_addr);
        SysCtlDelay(SysCtlClockGet() / 426);
    }
    else
    {
        write_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
        BNO055_OPERATION_MODE_CONFIG,
                          bno_addr);
        SysCtlDelay(SysCtlClockGet() / 159);
        if (t_operation_mode != BNO055_OPERATION_MODE_CONFIG)
        {
            write_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_OPR_MODE_ADDR,
            BNO055_OPERATION_MODE_CONFIG,
                              bno_addr);
            SysCtlDelay(SysCtlClockGet() / 426);
        }
    }
    m_operation_mode = t_operation_mode;
}

void set_pow_mode_bno055_I2C0(uint8_t t_power_mode, uint8_t bno_addr)
{

    uint8_t prev_operation_mode = m_operation_mode;
    if (prev_operation_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        set_op_mode_bno055_I2C0(BNO055_OPERATION_MODE_CONFIG, bno_addr);
    }
    m_power_mode = t_power_mode;

    write_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_PWR_MODE_ADDR, t_power_mode,
                      bno_addr);
    SysCtlDelay(SysCtlClockGet() / 426);

    if (prev_operation_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        set_op_mode_bno055_I2C0(prev_operation_mode, bno_addr);
    }
}

void set_pow_mode_bno055_I2C1(uint8_t t_power_mode, uint8_t bno_addr)
{

    uint8_t prev_operation_mode = m_operation_mode;
    if (prev_operation_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        set_op_mode_bno055_I2C1(BNO055_OPERATION_MODE_CONFIG, bno_addr);
    }
    m_power_mode = t_power_mode;

    write_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_PWR_MODE_ADDR, t_power_mode,
                      bno_addr);
    SysCtlDelay(SysCtlClockGet() / 426);

    if (prev_operation_mode != BNO055_OPERATION_MODE_CONFIG)
    {
        set_op_mode_bno055_I2C1(prev_operation_mode, bno_addr);
    }
}

void init_bno055_I2C0(uint8_t bno_addr)
{
    // Select BNO055 config mode
    set_op_mode_bno055_I2C0(BNO055_OPERATION_MODE_CONFIG, bno_addr);
    SysCtlDelay(SysCtlClockGet() / 200); //7ms
    uint8_t hold = 0x1B;
    // Configure GYR 230 hz and 250 dps
    write_bno055_I2C0(BNO055_PAGE_ONE, BNO055_GYRO_CONFIG_ADDR, hold, bno_addr);
    write_bno055_I2C0(BNO055_PAGE_ONE, BNO055_GYRO_MODE_CONFIG_ADDR, 0x00,
                      bno_addr);

    // Select BNO055 gyro temperature source
    write_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_TEMP_SOURCE_ADDR, 0x01,
                      bno_addr);

    SysCtlDelay(SysCtlClockGet() / 200);
    set_op_mode_bno055_I2C0(BNO055_OPERATION_MODE_GYRONLY, bno_addr);
    SysCtlDelay(SysCtlClockGet() / 200);

    write_bno055_I2C0(BNO055_PAGE_ZERO, BNO055_UNIT_SEL_ADDR, 0x00, bno_addr);

    // Select BNO055 system power mode
    set_pow_mode_bno055_I2C0(BNO055_POWER_MODE_NORMAL, bno_addr);
    SysCtlDelay(SysCtlClockGet() / 500);
}

void init_bno055_I2C1(uint8_t bno_addr)
{
    // Select BNO055 config mode
    set_op_mode_bno055_I2C1(BNO055_OPERATION_MODE_CONFIG, bno_addr);
    SysCtlDelay(SysCtlClockGet() / 200); //7ms
    uint8_t hold = 0x1B;
    // Configure GYR 230 hz and 250 dps
    write_bno055_I2C1(BNO055_PAGE_ONE, BNO055_GYRO_CONFIG_ADDR, hold, bno_addr);
    write_bno055_I2C1(BNO055_PAGE_ONE, BNO055_GYRO_MODE_CONFIG_ADDR, 0x00,
                      bno_addr);

    // Select BNO055 gyro temperature source
    write_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_TEMP_SOURCE_ADDR, 0x01,
                      bno_addr);

    SysCtlDelay(SysCtlClockGet() / 200);
    set_op_mode_bno055_I2C1(BNO055_OPERATION_MODE_GYRONLY, bno_addr);
    SysCtlDelay(SysCtlClockGet() / 200);

    write_bno055_I2C1(BNO055_PAGE_ZERO, BNO055_UNIT_SEL_ADDR, 0x00, bno_addr);

    // Select BNO055 system power mode
    set_pow_mode_bno055_I2C1(BNO055_POWER_MODE_NORMAL, bno_addr);
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

