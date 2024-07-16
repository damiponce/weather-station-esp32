#ifndef _PL_I2C_H_
#define _PL_I2C_H_

#include "driver/gpio.h"
#include "driver/i2c_master.h"

i2c_master_bus_handle_t pl_i2c_init(gpio_num_t sda_pin,
                                    gpio_num_t scl_pin,
                                    uint32_t clock_speed);

esp_err_t pl_i2c_write(i2c_master_bus_handle_t bus_handle,
                       uint8_t slave_addr,
                       uint8_t *bytes,
                       int length);

uint8_t pl_i2c_read(i2c_master_bus_handle_t bus_handle, uint8_t slave_addr);

#endif // _PL_I2C_H_