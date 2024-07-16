cmake_minimum_required(VERSION 3.5)

set(COMPONENTS esptool_py main general dl_eth pl_i2c al_bmp180 al_bmp280)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(weather-station-esp32)