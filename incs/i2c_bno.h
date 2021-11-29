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

void ConfigureI2C0();
void ConfigureI2C1();
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
short int bno055_read_temp_I2C0(uint8_t bno_addr);
short int bno055_read_temp_I2C1(uint8_t bno_addr);

#endif
