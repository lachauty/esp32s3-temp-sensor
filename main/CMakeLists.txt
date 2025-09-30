idf_component_register(
  SRCS
    "Temperature-Sensor.c"
    "wifi_mgr.c"
    "portal.c"
    "nvs_kv.c"
    "max31856.c"
  INCLUDE_DIRS
    "."
  REQUIRES               # headers/APIs used by your code
    driver
    esp_wifi
    esp_netif
    esp_event
    nvs_flash
    lwip
    esp_http_client
    esp_http_server
    esp_timer
  PRIV_REQUIRES          # TLS + WPA2-Enterprise bits
    mbedtls
    esp-tls
    wpa_supplicant
)
