// / APPLICATION LAYER
// Headder file of the BMP280 component.

#ifndef _AL_BMP280_H_
#define _AL_BMP280_H_

#include "esp_err.h"
#include "driver/i2c_master.h"

/** Initialize the BMP280.

**Return**
    - err:
        success or fail if the sensor returns the correct id

**Description**
    Clear all calibration parameter. Calls
    `al_bmp280_get_calib_param`. Log the parameter to the
    console.
*/
esp_err_t al_bmp280_init(i2c_master_bus_handle_t bus_handle);

/** Get the real temperature.

**Requirement**
    Initialize the BMP280 component with `al_bmp280_init`.

**Return**
    - t: temperature in units of 0.1 celsius

**Description**
    Calls `al_bmp280_get_ut` and does the temperature
    conversion. Log the temperature to the console in debug
    mode.
*/
float al_bmp280_get_temperature(i2c_master_bus_handle_t bus_handle);

/** Get the real pressure.

**Requirement**
    Initialize the BMP280 component with `al_bmp280_init`.
    Call `al_bmp280_get_temperature` before to get the value
    of `b5`.

**Parameters**
    - oss:
        oversampling setting, possible values from 0-3 see
        `oss` in `al_bmp_280_get_ut`

**Return*
    - p: pressure in units of Pa

**Description**
    Calls `al_bmp280_get_up` and does the pressure
    conversion. Log the pressure to the console in debug
    mode.
*/
float al_bmp280_get_pressure(i2c_master_bus_handle_t bus_handle);

#endif // _AL_BMP280_H_
