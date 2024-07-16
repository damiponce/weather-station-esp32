// PROTOCOL LAYER
// Source file for i2c communication

#include "./pl_i2c.h"
#include "../general/general.h"

#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"

static const char *TAG = "pl_i2c";

i2c_master_bus_handle_t pl_i2c_init(gpio_num_t sda_pin,
                                    gpio_num_t scl_pin,
                                    uint32_t clock_speed)
{
    i2c_master_bus_config_t i2c_conf = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = 0,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .flags.enable_internal_pullup = false,
    };
    // i2c_conf.mode = I2C_MODE_MASTER;
    // i2c_conf.sda_io_num = sda_pin;
    // i2c_conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    // i2c_conf.scl_io_num = scl_pin;
    // i2c_conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    // i2c_conf.clk_source = clock_speed;

    // // apply configuration with port 0
    // log_status(TAG,
    //            i2c_param_config(I2C_NUM_0,
    //                             &i2c_conf),
    //            "i2c_param_config to I2C_NUM_0");

    // // install driver
    // log_status(TAG,
    //            i2c_driver_install(I2C_NUM_0,
    //                               I2C_MODE_MASTER,
    //                               0, 0, 0),
    //            "i2c_driver_install to I2C_NUM_0");

    i2c_master_bus_handle_t bus_handle;

    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_conf, &bus_handle));

    // log_status(TAG, i2c_new_master_bus(&i2c_conf, &bus_handle), "i2c_new_master_bus");

    ESP_LOGI(TAG, "finished init");
    return bus_handle;
}

esp_err_t pl_i2c_write(i2c_master_bus_handle_t bus_handle,
                       uint8_t slave_addr,
                       uint8_t *bytes,
                       int length)
{

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slave_addr,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    ESP_ERROR_CHECK(i2c_master_transmit(dev_handle, bytes, length, 1000 / portTICK_PERIOD_MS));
    return ESP_OK;
}

uint8_t pl_i2c_read(i2c_master_bus_handle_t bus_handle,
                    uint8_t slave_addr)
{
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = slave_addr,
        .scl_speed_hz = 100000,
    };

    i2c_master_dev_handle_t dev_handle;
    ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    uint8_t *byte = malloc(1);
    ESP_ERROR_CHECK(i2c_master_receive(dev_handle, byte, 1, 1000 / portTICK_PERIOD_MS));
    return *byte;
}

#if false
esp_err_t pl_i2c_write(uint8_t slave_addr,
                       uint8_t *bytes,
                       int length)
{
    // error from the command link exectution
    esp_err_t err = ESP_OK;

    // create a command link which holds the sequence of
    // i2c commands to execute.
    i2c_cmd_handle_t cmd_link = i2c_cmd_link_create();

    // populate the command link with start bit, slave
    // address (write bit set), payload of 'length' bytes
    // and stop bit.
    i2c_master_start(cmd_link);
    i2c_master_write_byte(cmd_link,
                          (slave_addr << 1),
                          true);

    for (int i = 0; i < length; i++)
    {
        i2c_master_write_byte(cmd_link, bytes[i], true);
    }

    i2c_master_stop(cmd_link);

    // execute commands
    err = i2c_master_cmd_begin(I2C_NUM_0,
                               cmd_link,
                               1000 / portTICK_PERIOD_MS);

    // delete command link
    i2c_cmd_link_delete(cmd_link);

    ESP_LOGV(TAG,
             "master write to 0x%02X: \n bytes:",
             slave_addr);
    for (int i = 0; i < length; i++)
    {
        ESP_LOGV(TAG, "0x%02X", bytes[i]);
    }

    // return the status of the execution of the command
    // link
    return err;
}

uint8_t pl_i2c_read(uint8_t slave_addr)
{
    // hold payload data to read
    uint8_t byte = 0xFF;

    // create a command link which holds the sequence of
    // i2c commands to execute.
    i2c_cmd_handle_t cmd_link = i2c_cmd_link_create();

    // populate the command link with start bit, write slave
    // address (write bit 0), read payload byte and stop
    // bit.
    i2c_master_start(cmd_link);
    i2c_master_write_byte(cmd_link,
                          ((slave_addr << 1) | 0x01),
                          true);
    i2c_master_read_byte(cmd_link, &byte, true);
    i2c_master_stop(cmd_link);

    // execute commands and the read value will be saved to
    // 'byte'
    i2c_master_cmd_begin(I2C_NUM_0,
                         cmd_link,
                         1000 / portTICK_PERIOD_MS);

    // delete command link
    i2c_cmd_link_delete(cmd_link);

    ESP_LOGV(TAG,
             "master read from 0x%02X: byte: 0x%02X",
             slave_addr,
             byte);

    // return the read value, invalid is 0xFF
    return byte;
}
#endif