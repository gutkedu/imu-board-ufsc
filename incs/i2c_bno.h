/// \file i2c_bno.h
/// \brief Functions to interact with the IMU. These functions initialize and allow
/// for easy interaction with the BNO055 IMU.

#ifndef INIT_IMU_HG
#define INIT_IMU_HG

//#include "../incs/bno055.h"
#include "bno055.h"

typedef struct bno055_gyro_raw
{ // \brief store bno gyro signed values
    short int x;
    short int y;
    short int z;

} bno055_gyro_raw;

typedef struct Calibration
{
/// \brief store all calibration values
/// \returns a struct with the calibration data
    unsigned int gyro;
    unsigned int accl;
    unsigned int magn;
    unsigned int syst;
} Calibration;

/// \brief Configure I2C0. Sets the proper pins and other tiva settings to initialize i2c communication.
///
void ConfigureI2C0();

/// \brief Configure I2C1. Sets the proper pins and other tiva settings to initialize i2c communication.
void ConfigureI2C1();

/// \brief used read a value through i2c0 bus.
/// \param dev_address: the BNO055 address being used. This should be set in the BNO055_t struct.
/// \param reg_address: the registart to be read. Each function in the BNO055 driver lib has this preset.
/// \param *arr_data: an address to the array to store the data in. Each function in the BNO055 driver lib has this preset.
/// \param count: the number of bytes to read. Each function in the BNO055 driver lib has this preset.
/// \return comres: the result of the communication, (Per BNO055 documentation: 0 if success. -1 if failure)
s8 _imu_i2c_read_i2c0(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count);

/// \brief used to write a value through i2c0 bus.
/// \param dev_address: the BNO055 address being used. This should be set in the BNO055_t struct.
/// \param reg_address: the registart to be read. Each function in the BNO055 driver lib has this preset.
/// \param *var_data: an address to the variable to store the data in. Each function in the BNO055 driver lib has this preset.
/// \param count: the number of bytes to read. Each function in the BNO055 driver lib has this preset.
/// \return comres: the result of the communication, (Per BNO055 documentation: 0 if success. -1 if failure)
s8 _imu_i2c_write_i2c0(u8 dev_address, u8 reg_address, u8 *var_data, u8 count);

/// \brief used read a value through i2c1 bus.
/// \param dev_address: the BNO055 address being used. This should be set in the BNO055_t struct.
/// \param reg_address: the registart to be read. Each function in the BNO055 driver lib has this preset.
/// \param *arr_data: an address to the array to store the data in. Each function in the BNO055 driver lib has this preset.
/// \param count: the number of bytes to read. Each function in the BNO055 driver lib has this preset.
/// \return comres: the result of the communication, (Per BNO055 documentation: 0 if success. -1 if failure)
s8 _imu_i2c_read_i2c1(u8 dev_address, u8 reg_address, u8 *arr_data, u8 count);

/// \brief used to write a value through i2c1 bus.
/// \param dev_address: the BNO055 address being used. This should be set in the BNO055_t struct.
/// \param reg_address: the registart to be read. Each function in the BNO055 driver lib has this preset.
/// \param *var_data: an address to the variable to store the data in. Each function in the BNO055 driver lib has this preset.
/// \param count: the number of bytes to read. Each function in the BNO055 driver lib has this preset.
/// \return comres: the result of the communication, (Per BNO055 documentation: 0 if success. -1 if failure)
s8 _imu_i2c_write_i2c1(u8 dev_address, u8 reg_address, u8 *var_data, u8 count);

/// \brief initiate a blocking delay
/// \param ms: the duration (in milliseconds) to delay for
void ms_delay(u32 ms);

/// \brief used to set the initialization struct needed by the BNO055 driver
///
void init_bno_1();
void init_bno_2();
void init_bno_3();
void init_bno_4();

/// \brief used to get the error result returned by the BNO055 driver from the most recent interaction with the sensor. 0 == no error
///
s8 get_error();

/// \brief set the mode of the imu
///
void set_bno_1_mode(u8 op_mode);

/// \biref Get the absolute position by reading the quaternion
/// \returns the corresponding euler angles
//struct bno055_euler_float_t get_abs_position();

/// \brief Get the current calibration data from the IMU
/// \returns the calibration data
Calibration calibrate_imu();

///

void send_request_I2C0(uint32_t t_addr, uint32_t t_register, uint32_t t_data);
uint32_t recieve_request_I2C0(uint32_t t_addr, uint32_t t_register);
void send_request_I2C1(uint32_t t_addr, uint32_t t_register, uint32_t t_data);
uint32_t recieve_request_I2C1(uint32_t t_addr, uint32_t t_register);
void set_op_mode_bno055_I2C0(uint8_t t_operation_mode, uint8_t bno_addr);
void set_pow_mode_bno055_I2C0(uint8_t t_power_mode, uint8_t bno_addr);
void set_pow_mode_bno055_I2C1(uint8_t t_power_mode, uint8_t bno_addr);
void init_bno055_I2C0(uint8_t bno_addr);
void init_bno055_I2C1(uint8_t bno_addr);
uint8_t read_bno055_I2C1(uint8_t t_page, uint8_t t_register, uint8_t addr);
void write_bno055_I2C1(uint8_t t_page, uint8_t t_register, uint8_t t_data,
                       uint8_t addr);
uint8_t read_bno055_I2C0(uint8_t t_page, uint8_t t_register, uint8_t addr);
void write_bno055_I2C0(uint8_t t_page, uint8_t t_register, uint8_t t_data,
                       uint8_t addr);
bno055_gyro_raw bno055_read_gyro_I2C0(uint8_t bno_addr);
bno055_gyro_raw bno055_read_gyro_I2C1(uint8_t bno_addr);

#endif
