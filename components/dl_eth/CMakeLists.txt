idf_component_register(
    SRCS "dl_eth.c"
    PRIV_REQUIRES driver esp_eth esp_netif
    INCLUDE_DIRS "."
)
