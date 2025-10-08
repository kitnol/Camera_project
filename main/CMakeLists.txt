idf_component_register(SRCS "ds3231.c" "i2cdev.c" "console_example_main.c" "to_bmp.c" "jpge.cpp" "esp_jpg_decode.c"
                       INCLUDE_DIRS "." "D:/Espressif/frameworks/esp-idf-v5.4.1/components/esp_driver_cam/isp_dvp/include"
                       REQUIRES esp_mm esp_driver_isp esp_driver_cam esp_driver_i2c sensor_init fatfs sdmmc vfs
                       PRIV_REQUIRES esp32-camera
                       )

