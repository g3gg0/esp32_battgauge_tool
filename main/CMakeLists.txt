idf_component_register(
    SRCS
    "main.c"
    "cmd.c"
    "i2c.c"
    "bq.c"
    "wifi.c"
    "telnet.c"
    
    INCLUDE_DIRS 
    "."

    PRIV_REQUIRES driver esp_driver_gpio esp_hw_support esp_psram esp_wifi wpa_supplicant esp_event)
