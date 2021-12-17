/// \file i2c_bno.h
/// \brief Functions to interact with the IMU. These functions initialize and allow
/// for easy interaction with the BNO055 IMU.

#ifndef INIT_IMU_HG
#define INIT_IMU_HG

#include "bno055.h"

#define SELECT_I2C0 0
#define SELECT_I2C1 1

typedef struct bno055_gyro_raw
{ // \brief store bno gyro signed values
    short int x;
    short int y;
    short int z;

} bno055_gyro_raw;

void ConfigureI2C0();
void ConfigureI2C1();

void send_request_i2c(uint32_t t_addr, uint32_t t_register, uint32_t t_data,
                      int select_i2c);
uint32_t recieve_request_i2c(uint32_t t_addr, uint32_t t_register,
                             int select_i2c);

void write_bno055(uint8_t t_page, uint8_t t_register, uint8_t t_data,
                  uint8_t addr, int select_i2c);

uint8_t read_bno055(uint8_t t_page, uint8_t t_register, uint8_t addr,
                    int select_i2c);

void set_op_mode_bno055(uint8_t t_operation_mode, uint8_t bno_addr,
                        int select_i2c);
void set_power_mode_bno055(uint8_t t_power_mode, uint8_t bno_addr,
                           int select_i2c);
void init_bno055(uint8_t bno_addr, int select_i2c);

bno055_gyro_raw read_bno055_gyro(uint8_t bno_addr, int select_i2c);
short int read_bno055_temp(uint8_t bno_addr, int select_i2c);

#endif
