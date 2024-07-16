#include "../components/general/general.h"
#include "../components/pl_i2c/pl_i2c.h"
#include "../components/dl_eth/dl_eth.h"
#include "../components/al_bmp180/al_bmp180.h"
#include "../components/al_bmp280/al_bmp280.h"

#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "driver/i2c_master.h"

#include <inttypes.h>

static const char *TAG = "main";

// Entry point
void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Print on start
    ESP_LOGW(TAG, "System initialized!");

    esp_log_level_set("main", ESP_LOG_INFO);

    // Create an event loop
    log_status(TAG,
               esp_event_loop_create_default(),
               "esp_event_loop_create_default");

    // Initialize I2C
    esp_log_level_set("pl_i2c", ESP_LOG_INFO);
    i2c_master_bus_handle_t i2c_handle = pl_i2c_init(GPIO_NUM_5, GPIO_NUM_18, 100000);

#if true
    // Initialize BMP180
    esp_log_level_set("al_bmp180", ESP_LOG_INFO);
    al_bmp180_init(i2c_handle);

    esp_log_level_set("al_bmp280", ESP_LOG_INFO);
    al_bmp280_init(i2c_handle);

    // Main loop
    while (true)
    {
        float temperature = al_bmp180_get_temperature(i2c_handle);
        float pressure = al_bmp180_get_pressure(i2c_handle, 3);
        fprintf(stderr, "BMP180 - T[°C]: %.2f | P[Pa]: %.2f\n", temperature, pressure);

        temperature = al_bmp280_get_temperature(i2c_handle);
        pressure = al_bmp280_get_pressure(i2c_handle);
        fprintf(stderr, "BMP280 - T[°C]: %.2f | P[Pa]: %.2f\n\n", temperature, pressure);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
#endif

    while (true)
    {
        float temperature = al_bmp280_get_temperature(i2c_handle);
        float pressure = al_bmp280_get_pressure(i2c_handle);
        fprintf(stderr, "T[°C]: %.2f | P[Pa]: %.2f\n", temperature, pressure);

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }

#if false
    // Initialize Ethernet
    esp_log_level_set("dl_eth", ESP_LOG_INFO);

    log_status(TAG,
               esp_event_handler_instance_register(ETH_EVENT,
                                                   ESP_EVENT_ANY_ID,
                                                   &dl_eth_event_handler,
                                                   NULL,
                                                   NULL),
               "register ethernet events (any)");
    log_status(TAG,
               esp_event_handler_instance_register(IP_EVENT,
                                                   IP_EVENT_ETH_GOT_IP,
                                                   &dl_got_ip_event_handler,
                                                   NULL,
                                                   NULL),
               "register ip event IP_EVENT_STA_GOT_IP");

    dl_eth_init();
#endif
}