// APPLICATION LAYER
// Source file of the BMP280 component.

// components
#include "./al_bmp280.h"

#include "../general/general.h"
#include "../pl_i2c/pl_i2c.h"

// esp-idf
#include "esp_log.h"
// #error "include FreeRTOS.h must appear in source files
// before include task.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>

// Type of calibration parameters
typedef struct calib_param_t
{
    uint16_t dig_T1;
    int16_t dig_T2;
    int16_t dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2;
    int16_t dig_P3;
    int16_t dig_P4;
    int16_t dig_P5;
    int16_t dig_P6;
    int16_t dig_P7;
    int16_t dig_P8;
    int16_t dig_P9;
} calib_param_t;

// slave address of bmp280
static uint8_t fd_BMP = 0x76;
// start of the eeprom
static uint8_t eeprom_start = 0x88;

// Calibration parameter object
static calib_param_t calib_param;

// Logging tag
static const char *TAG = "al_bmp280";

// function wrappers
static uint8_t read_byte(i2c_master_bus_handle_t bus_handle,
                         uint8_t slave_addr,
                         uint8_t addr)
{
    // set address on bmp280
    pl_i2c_write(bus_handle, slave_addr, &addr, 1);
    // read value from address
    return pl_i2c_read(bus_handle, slave_addr);
}

static esp_err_t write_byte(i2c_master_bus_handle_t bus_handle,
                            uint8_t slave_addr,
                            uint8_t addr,
                            uint8_t byte)
{
    // write eeprom address and content
    uint8_t bytes[2] = {addr, byte};
    return pl_i2c_write(bus_handle, slave_addr, bytes, 2);
}

static uint16_t get_uint_param(i2c_master_bus_handle_t bus_handle,
                               uint8_t slave_addr,
                               uint8_t eeprom_addr)
{
    // read msb and lsb
    uint16_t lsb = (uint16_t)read_byte(bus_handle,
                                       slave_addr,
                                       eeprom_addr);
    uint16_t msb = (uint16_t)read_byte(bus_handle,
                                       slave_addr,
                                       eeprom_addr + 1);
    return (msb << 8) + lsb;
}

static int16_t get_int_param(i2c_master_bus_handle_t bus_handle,
                             uint8_t slave_addr,
                             uint8_t eeprom_addr)
{
    // read msb and lsb
    int16_t lsb = (int16_t)read_byte(bus_handle,
                                     slave_addr,
                                     eeprom_addr);
    int16_t msb = (int16_t)read_byte(bus_handle,
                                     slave_addr,
                                     eeprom_addr + 1);
    return (msb << 8) + lsb;
}

static void clear_calib_param()
{
    calib_param.dig_T1 = 0;
    calib_param.dig_T2 = 0;
    calib_param.dig_T3 = 0;
    calib_param.dig_P1 = 0;
    calib_param.dig_P2 = 0;
    calib_param.dig_P3 = 0;
    calib_param.dig_P4 = 0;
    calib_param.dig_P5 = 0;
    calib_param.dig_P6 = 0;
    calib_param.dig_P7 = 0;
    calib_param.dig_P8 = 0;
    calib_param.dig_P9 = 0;
}

// Log the calibration parameter with verbose level.
void al_bmp280_log_calib_param()
{
    ESP_LOGV(TAG, "Calibration parameter");
    ESP_LOGV(TAG, "T1=0x%04X : %d", calib_param.dig_T1, calib_param.dig_T1);
    ESP_LOGV(TAG, "T2=0x%04X : %d", calib_param.dig_T2, calib_param.dig_T2);
    ESP_LOGV(TAG, "T3=0x%04X : %d", calib_param.dig_T3, calib_param.dig_T3);
    ESP_LOGV(TAG, "P1=0x%04X : %d", calib_param.dig_P1, calib_param.dig_P1);
    ESP_LOGV(TAG, "P2=0x%04X : %d", calib_param.dig_P2, calib_param.dig_P2);
    ESP_LOGV(TAG, "P3=0x%04X : %d", calib_param.dig_P3, calib_param.dig_P3);
    ESP_LOGV(TAG, "P4=0x%04X : %d", calib_param.dig_P4, calib_param.dig_P4);
    ESP_LOGV(TAG, "P5=0x%04X : %d", calib_param.dig_P5, calib_param.dig_P5);
    ESP_LOGV(TAG, "P6=0x%04X : %d", calib_param.dig_P6, calib_param.dig_P6);
    ESP_LOGV(TAG, "P7=0x%04X : %d", calib_param.dig_P7, calib_param.dig_P7);
    ESP_LOGV(TAG, "P8=0x%04X : %d", calib_param.dig_P8, calib_param.dig_P8);
    ESP_LOGV(TAG, "P9=0x%04X : %d", calib_param.dig_P9, calib_param.dig_P9);
}

/** Read the calibration parameter from the BMP280 via I2C.

**Requirement**
    Initialize the BMP280 component with `al_bmp280_init`.

**Parameters**
    - slave_addr:
        7bit I2C address of the bmp280 sensor
    - eeprom_start:
        8bit eeprom address where the calibration parameters
        start

**Description**
    Reading in the calibration parameters starting from the
    start address up to 20 register further.
*/
void al_bmp280_get_calib_param(i2c_master_bus_handle_t bus_handle,
                               uint8_t slave_addr,
                               uint8_t eeprom_start)
{
    ESP_LOGI(TAG, "Started getting calibration parameter");

    calib_param.dig_T1 = get_uint_param(bus_handle, slave_addr, eeprom_start);
    calib_param.dig_T2 = get_int_param(bus_handle, slave_addr, eeprom_start + 2);
    calib_param.dig_T3 = get_int_param(bus_handle, slave_addr, eeprom_start + 4);
    calib_param.dig_P1 = get_uint_param(bus_handle, slave_addr, eeprom_start + 6);
    calib_param.dig_P2 = get_int_param(bus_handle, slave_addr, eeprom_start + 8);
    calib_param.dig_P3 = get_int_param(bus_handle, slave_addr, eeprom_start + 10);
    calib_param.dig_P4 = get_int_param(bus_handle, slave_addr, eeprom_start + 12);
    calib_param.dig_P5 = get_int_param(bus_handle, slave_addr, eeprom_start + 14);
    calib_param.dig_P6 = get_int_param(bus_handle, slave_addr, eeprom_start + 16);
    calib_param.dig_P7 = get_int_param(bus_handle, slave_addr, eeprom_start + 18);
    calib_param.dig_P8 = get_int_param(bus_handle, slave_addr, eeprom_start + 20);
    calib_param.dig_P9 = get_int_param(bus_handle, slave_addr, eeprom_start + 22);

    ESP_LOGI(TAG, "Finshed getting calibration parameters");
}

esp_err_t al_bmp280_init(i2c_master_bus_handle_t bus_handle)
{
    esp_err_t err = ESP_OK;

    // set all calibration paramters to zero.
    clear_calib_param();

    // test if the communication to the BMP280 works with
    // the id 0x58 in register 0xD0
    if (0x85 != read_byte(bus_handle, fd_BMP, 0xD0))
    {
        ESP_LOGW(TAG, "Failed init");
        err = ESP_FAIL;
    }

    // get and print calibration parameter
    al_bmp280_get_calib_param(bus_handle, fd_BMP, eeprom_start);
    al_bmp280_log_calib_param();

    ESP_LOGI(TAG, "Finished init");
    return err;
}

/** Get the uncompensated temperature.

**Requirement**
    Initialize the BMP280 component with `al_bmp280_init`.

**Parameters**
    - slave_addr: 7bit I2C address of the bmp280 sensor

**Return**
    - ut: uncompensated temperature

**Description**
    Write 0x2E into reg 0xF4, wait 4.5ms and then read
    registers 0xF6 (MSB), 0xF7 (LSB). Log the value to the
    console.
*/
int32_t al_bmp280_get_ut(i2c_master_bus_handle_t bus_handle,
                         uint8_t slave_addr)
{
    // start measurement by writing value 0x2E into register
    // 0xF4
    write_byte(bus_handle, slave_addr, 0xF4, 0x5D);

    // delay time: 4.5ms = 4500µs
    // vTaskDelay(500 / portTICK_PERIOD_MS);

    while (read_byte(bus_handle, slave_addr, 0xF3) & 0x08)
    {
        vTaskDelay(1);
    }

    // read out uncompensated temperature
    int32_t ut_msb = ((int32_t)read_byte(bus_handle, slave_addr, 0xFA) << 12);
    int32_t ut_lsb = ((int32_t)read_byte(bus_handle, slave_addr, 0xFB) << 4);
    int32_t ut_xlsb = ((int32_t)read_byte(bus_handle, slave_addr, 0xFC) >> 4);
    int32_t ut = ut_msb | ut_lsb | ut_xlsb;

    ESP_LOGV(TAG, "ut=0x%08X : %d", ut, ut);

    return ut;
}

/** Get the uncompensated pressure.

**Requirement**
    Initialize the BMP280 component with `al_bmp280_init`.

**Parameters**
    - slave_addr:
        7bit I2C address of the bmp280 sensor
    - oss:
        oversampling setting, possible values from 0-3 see
        description

**Return**
    - up: uncompensated pressure

**Description**
    Write a different value to register 0xF4 depending on
    the value of `oss`, see the table below. Wait and then
    read 3 bytes and log the combined value.

    | mode                  | oss  | internal_number_of_samples | conversion_time |
    |-----------------------|------|----------------------------|-----------------|
    | ultra_low_power       | 0    | 1                          |  4.5ms          |
    | standard              | 1    | 2                          |  7.5ms          |
    | high_resolution       | 2    | 4                          | 13.5ms          |
    | ultra_high_resolution | 3    | 8                          | 25.5ms          |
*/

int32_t al_bmp280_get_up(i2c_master_bus_handle_t bus_handle,
                         uint8_t slave_addr)
{
    // start measurement by writing value 0x2E into register
    // 0xF4
    write_byte(bus_handle, slave_addr, 0xF4, 0x5D);

    // delay time: 4.5ms = 4500µs
    // vTaskDelay(500 / portTICK_PERIOD_MS);

    while (read_byte(bus_handle, slave_addr, 0xF3) & 0x08)
    {
        vTaskDelay(1);
    }

    int32_t up_msb = ((int32_t)read_byte(bus_handle, slave_addr, 0xF7) << 12);
    int32_t up_lsb = ((int32_t)read_byte(bus_handle, slave_addr, 0xF8) << 4);
    int32_t up_xlsb = ((int32_t)read_byte(bus_handle, slave_addr, 0xF9) >> 4);
    int32_t up = up_msb | up_lsb | up_xlsb;

    ESP_LOGV(TAG, "up=0x%08X : %d", up, up);

    return up;
}

int32_t fine_T;

float al_bmp280_get_temperature(i2c_master_bus_handle_t bus_handle)
{
    int32_t ut = al_bmp280_get_ut(bus_handle, fd_BMP);
#if false
    double var1, var2, T;
    var1 = (((double)ut) / 16384.0 - ((double)calib_param.dig_T1) / 1024.0) * ((double)calib_param.dig_T2);
    var2 = ((((double)ut) / 131072.0 - ((double)calib_param.dig_T1) / 8192.0) * (((double)ut) / 131072.0 - ((double)calib_param.dig_T1) / 8192.0)) * ((double)calib_param.dig_T3);
    T = (var1 + var2) / 5120.0;
#endif

#if true
    int32_t var1, var2, T;
    var1 = ((((ut >> 3) - ((int32_t)calib_param.dig_T1 << 1))) * ((int32_t)calib_param.dig_T2)) >> 11;
    var2 = (((((ut >> 4) - ((int32_t)calib_param.dig_T1)) * ((ut >> 4) - ((int32_t)calib_param.dig_T1))) >> 12) * ((int32_t)calib_param.dig_T3)) >> 14;
    fine_T = var1 + var2;
    T = (fine_T * 5 + 128) >> 8;
#endif

    ESP_LOGD(TAG,
             "Temperature in degrees Celsius: %.2f",
             (float)T / 100);

    return (float)T / 100;
}

float al_bmp280_get_pressure(i2c_master_bus_handle_t bus_handle)
{
    int32_t up = al_bmp280_get_up(bus_handle, fd_BMP);

    // fine_T = 128422;
    int32_t var1, var2, p;
    var1 = (((int32_t)fine_T) >> 1) - (int32_t)64000;
    var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)calib_param.dig_P6);
    var2 = var2 + ((var1 * ((int32_t)calib_param.dig_P5)) << 1);
    var2 = (var2 >> 2) + (((int32_t)calib_param.dig_P4) << 16);
    var1 = (((calib_param.dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)calib_param.dig_P2) * var1) >> 1)) >> 18;
    var1 = ((((32768 + var1)) * ((int32_t)calib_param.dig_P1)) >> 15);
    if (var1 == 0)
    {
        return 0; // avoid exception caused by division by zero
    }
    p = (((uint32_t)(((int32_t)1048576) - up) - (var2 >> 12))) * 3125;
    if (p < 0x80000000)
    {
        p = (p << 1) / ((uint32_t)var1);
    }
    else
    {
        p = (p / (uint32_t)var1) * 2;
    }
    var1 = (((int32_t)calib_param.dig_P9) * ((int32_t)(((p >> 3)) >> 13))) >> 12;
    var2 = (((int32_t)(p >> 2)) * ((int32_t)calib_param.dig_P8)) >> 13;
    p = (uint32_t)((int32_t)p + ((var1 + var2 + calib_param.dig_P7) >> 4));

    ESP_LOGD(TAG,
             "Pressure in hectopascals: %4.2f",
             (float)p);

    return (float)p;
}
