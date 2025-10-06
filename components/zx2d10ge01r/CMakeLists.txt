idf_component_register(
    SRCS "zx2d10ge01r.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver spiffs esp_lvgl_port
    PRIV_REQUIRES fatfs esp_lcd esp_psram esp_lcd_st7701
)