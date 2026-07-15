idf_component_register(
    SRCS
        "app_main.c"
        "wifi_ap.c"
        "web_server.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi nvs_flash esp_netif esp_event esp_http_server
)
