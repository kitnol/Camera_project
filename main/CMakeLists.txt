idf_component_register(SRCS "camera_main.c" "i2cdev.c" "ds3231.c" "console_example_main.c" "to_bmp.c" "jpge.cpp" "esp_jpg_decode.c"
                       INCLUDE_DIRS "." "D:/Espressif/frameworks/esp-idf-v5.4.1/components/esp_driver_cam/isp_dvp/include"
                       REQUIRES esp_mm esp_driver_isp esp_driver_cam esp_driver_i2c sensor_init fatfs sdmmc vfs esp_adc
                       PRIV_REQUIRES esp32-camera esp_timer
                       )

