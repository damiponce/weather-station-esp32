#ifndef _GENERAL_H_
#define _GENERAL_H_

#include "esp_err.h"
#include "esp_log.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE

void log_status(const char *tag,
                esp_err_t status,
                const char *msg);

void get_time(char *buf);

long get_seed();

#endif // _GENERAL_H_