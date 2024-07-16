idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES esp_system esp_event nvs_flash driver esp_eth esp_netif 
)