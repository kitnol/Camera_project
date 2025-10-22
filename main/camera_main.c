#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"


#include "esp_camera.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "cmd_system.h"
#include "cmd_nvs.h"
#include "argtable3/argtable3.h"

#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"
#include "esp_mac.h"
#include <portmacro.h>

#include "esp_sleep.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "hal/gpio_types.h"

#include "esp_partition.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"

#include "ds3231.h"

#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/adc_channel.h"
#include "esp_timer.h"

#define CAM_PIN_PWDN 9  // Power down pin (not used)
#define CAM_PIN_RESET 3 // Reset pin (not used)
#define CAM_PIN_XCLK 45 // External clock (safe pin) +
#define CAM_PIN_SIOD 1  // I2C SDA (safe pin) +
#define CAM_PIN_SIOC 2  // I2C SCL (safe pin) +

#define CAM_PIN_D7 48   // Data 7 (safe pin) +
#define CAM_PIN_D6 46   // Data 6 (safe pin) +
#define CAM_PIN_D5 8    // Data 5 (safe pin) +
#define CAM_PIN_D4 7    // Data 4 (safe pin) +
#define CAM_PIN_D3 4    // Data 3 (safe pin) +
#define CAM_PIN_D2 41   // Data 2 (safe pin) +
#define CAM_PIN_D1 40   // Data 1 (safe pin) +
#define CAM_PIN_D0 39   // Data 0 (safe pin) +
#define CAM_PIN_VSYNC 6 // Vertical sync (safe pin) +
#define CAM_PIN_HREF 42 // Horizontal reference (safe pin)
#define CAM_PIN_PCLK 5  // Pixel clock (safe pin) +

#define CAM_PWR CAM_PIN_PWDN
#define CAM_VREG_28V 

// SD card pin configuration (SDMMC)
#define SD_PIN_SCK 12
#define SD_PIN_CMD 13
#define SD_PIN_D0 11
#define SD_PIN_D1 10
#define SD_PIN_D2 38
#define SD_PIN_D3 14

#define PIN_NUM_MISO SD_PIN_D0  // D0
#define PIN_NUM_MOSI SD_PIN_CMD // CMD
#define PIN_NUM_CLK SD_PIN_SCK  // CLK
#define PIN_NUM_CS SD_PIN_D3    // D3 (not used in 4-line mode but defined)
#define PIN_NUM_D1 SD_PIN_D1    // D1
#define PIN_NUM_D2 SD_PIN_D2    // D2

// SD card pin configuration (SPI mode)
#define PIN_NUM_MISO SD_PIN_D0
#define PIN_NUM_MOSI SD_PIN_CMD
#define PIN_NUM_CLK SD_PIN_SCK
#define PIN_NUM_CS SD_PIN_D3

#define LED_PIN 15
#define USB_INT 21
#define BAT_VOLTAGE 16

#define RTC_SDA 17
#define RTC_SCL 18

#define MOUNT_POINT "/sdcard"
sdmmc_card_t *card_handle;
static bool sd_card_mounted = false;