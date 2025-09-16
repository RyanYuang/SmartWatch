idf_component_register(SRCS "hello_world_main.c"
                       "../managed_components/lvgl__lvgl/src/font/Apple_Font.c"
                       "../managed_components/lvgl__lvgl/demos/widgets/lv_demo_widgets.c"
                       PRIV_REQUIRES spi_flash
                       REQUIRES esp_driver_gpio
                       REQUIRES esp_driver_spi
                       REQUIRES esp_driver_i2c
                       REQUIRES esp_lcd
                       REQUIRES lvgl
                       INCLUDE_DIRS "."
                       "../managed_components/lvgl__lvgl"
                       "../managed_components/lvgl__lvgl/src")