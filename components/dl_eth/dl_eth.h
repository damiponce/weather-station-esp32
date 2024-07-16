#ifndef _DL_ETH_H_
#define _DL_ETH_H_

#include "esp_eth_driver.h"

esp_err_t dl_eth_init();

void dl_eth_event_handler(void *arg, esp_event_base_t event_base,
                          int32_t event_id, void *event_data);

void dl_got_ip_event_handler(void *arg, esp_event_base_t event_base,
                             int32_t event_id, void *event_data);

#endif // _DL_ETH_H_