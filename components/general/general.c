// MISCELLANEOUS
// General functions

#include "./general.h"

#include <string.h>
#include <time.h>

void log_status(const char *tag,
                esp_err_t status,
                const char *msg)
{
    if (status != ESP_OK)
    {
        ESP_LOGW(tag,
                 "failed %s: %s - 0x%x",
                 msg,
                 esp_err_to_name(status),
                 status);
    }
    else
    {
        ESP_LOGV(tag, "success: %s", msg);
    }
}

void get_time(char *buf)
{
    time_t epoch;
    struct tm date;

    time(&epoch);
    setenv("TZ", "CET-1", 1);
    tzset();
    gmtime_r(&epoch, &date);
    sprintf(buf,
            "%04d-%02d-%02dT%02d:%02d:%02dZ",
            date.tm_year + 1900,
            date.tm_mon + 1,
            date.tm_mday,
            date.tm_hour,
            date.tm_min,
            date.tm_sec);
}

long get_seed()
{
    time_t epoch;
    time(&epoch);
    return (long)epoch;
}
