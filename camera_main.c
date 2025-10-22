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

RTC_DATA_ATTR int running_mode = 0; // 0 = normal, 1 = motion detection
login_state_t login_state = LOGIN_NOT_ATEMPTED;
#define CORRECT_PASSWORD "hello"

typedef enum
{
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_CAPTURE,
    SYSTEM_STATE_SAVE,
    SYSTEM_STATE_CLEANUP,
    SYSTEM_STATE_SLEEP,
    SYSTEM_STATE_USB_CONNECTED,
    SYSTEM_STATE_FLASH_DRIVE
} system_state_t;

typedef enum
{
    LOGIN_NOT_ATEMPTED,
    LOGIN_SUCCESSFUL,
    LOGIN_UNSUCCESSFUL,
    LOGIN_TIME_OUT,
    LOGIN_TO_FLASH_DRIVE
} login_state_t;

/* ----------------------------------- FUNCTIONS --------------------------------------- */

/* Camera functions */
static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_5MP,       // Reduced from UXGA (800x600 instead of 1600x1200)
    .jpeg_quality = 10,                // Lower quality to reduce memory usage
    .fb_count = 2,                     // Single frame buffer without PSRAM
    .fb_location = CAMERA_FB_IN_PSRAM, // Use DRAM instead of PSRAM
    .grab_mode = CAMERA_GRAB_LATEST,
};

esp_err_t init_camera(void)
{

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }

    // Get camera sensor
    sensor_t *s = esp_camera_sensor_get();
    if (s == NULL)
    {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return ESP_FAIL;
    }

    // Configure camera settings for better quality
    s->set_brightness(s, 2);                 // -2 to 2
    s->set_contrast(s, 0);                   // -2 to 2
    s->set_saturation(s, 0);                 // -2 to 2
    s->set_special_effect(s, 0);             // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);                   // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);                   // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);                    // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);              // 0 = disable , 1 = enable
    s->set_aec2(s, 1);                       // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);                   // -2 to 2
    s->set_aec_value(s, 20);                 // 0 to 1200
    s->set_gain_ctrl(s, 1);                  // 0 = disable , 1 = enable
    s->set_agc_gain(s, 15);                  // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)4); // 0 to 6
    s->set_bpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);                    // 0 = disable , 1 = enable
    s->set_lenc(s, 1);                       // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);                    // 0 = disable , 1 = enable
    s->set_vflip(s, 0);                      // 0 = disable , 1 = enable
    s->set_dcw(s, 1);                        // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);                   // 0 = disable , 1 = enable

    sens = s;

    return ESP_OK;
}

esp_err_t deinit_camera(void)
{
    // Perform sensor software reset before deinitializing
    if (sens != NULL)
    {
        // Try to reset the sensor via I2C (OV5640 specific)
        // This helps ensure clean state for reinitialization
        sens->set_reg(sens, 0x3008, 0xff, 0x82); // Software reset
        vTaskDelay(pdMS_TO_TICKS(10));           // Wait for reset to complete

        // Clear the sensor reference
        sens = NULL;
    }

    // Deinitialize the camera driver
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Camera deinit failed with error 0x%x", err);
        return err;
    }

    return ESP_OK;
}