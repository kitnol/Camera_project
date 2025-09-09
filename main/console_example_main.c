#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_console.h"

#include "esp_camera.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

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

static const char *TAG = "camera_sd";
#define PROMPT_STR CONFIG_IDF_TARGET
#define VREG_ADDR 0x36

// BMP header structures
typedef struct __attribute__((packed)) {
    uint16_t type;          // BM
    uint32_t size;          // File size
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t offset;        // Offset to image data
} bmp_file_header_t;

typedef struct __attribute__((packed)) {
    uint32_t size;          // Header size
    int32_t width;
    int32_t height;
    uint16_t planes;
    uint16_t bits_per_pixel;
    uint32_t compression;
    uint32_t image_size;
    int32_t x_pixels_per_meter;
    int32_t y_pixels_per_meter;
    uint32_t colors_used;
    uint32_t colors_important;
} bmp_info_header_t;

// Camera pin configuration for ESP32-S3 with OV5640
// #define CAM_PIN_PWDN    -1   // Power down pin (not used)
// #define CAM_PIN_RESET   -1   // Reset pin (not used)
// #define CAM_PIN_XCLK    45   // External clock +
// #define CAM_PIN_SIOD    1    // I2C SDA +
// #define CAM_PIN_SIOC    2    // I2C SCL +

// #define CAM_PIN_D7      48   // Data 7 +
// #define CAM_PIN_D6      46   // Data 6 +
// #define CAM_PIN_D5      8    // Data 5 +
// #define CAM_PIN_D4      7    // Data 4 +
// #define CAM_PIN_D3      4    // Data 3 +
// #define CAM_PIN_D2      41   // Data 2 +
// #define CAM_PIN_D1      40   // Data 1 +
// #define CAM_PIN_D0      39   // Data 0 +
// #define CAM_PIN_VSYNC   6    // Vertical sync +
// #define CAM_PIN_HREF    42   // Horizontal reference +
// #define CAM_PIN_PCLK    5    // Pixel clock +

// ON BREAD-BOARD CONNECTION
// #define CAM_PIN_PWDN    38  // Power down pin (not used)
// #define CAM_PIN_RESET   -1  // Reset pin (not used)
// #define CAM_PIN_XCLK    15  // External clock (safe pin) +
// #define CAM_PIN_SIOD    1   // I2C SDA (safe pin) +
// #define CAM_PIN_SIOC    2   // I2C SCL (safe pin) +

// #define CAM_PIN_D7      16  // Data 7 (safe pin) +
// #define CAM_PIN_D6      17  // Data 6 (safe pin) +
// #define CAM_PIN_D5      8   // Data 5 (safe pin) +
// #define CAM_PIN_D4      7   // Data 4 (safe pin) +
// #define CAM_PIN_D3      4   // Data 3 (safe pin) +
// #define CAM_PIN_D2      10  // Data 2 (safe pin) +
// #define CAM_PIN_D1      11  // Data 1 (safe pin) +
// #define CAM_PIN_D0      12  // Data 0 (safe pin) +
// #define CAM_PIN_VSYNC   6   // Vertical sync (safe pin) +
// #define CAM_PIN_HREF    18  // Horizontal reference (safe pin) 
// #define CAM_PIN_PCLK    5   // Pixel clock (safe pin) +
// #define CAM_PWR         38

// ON PCB CONNECTION
#define CAM_PIN_PWDN    9  // Power down pin (not used)
#define CAM_PIN_RESET   3  // Reset pin (not used)
#define CAM_PIN_XCLK    45  // External clock (safe pin) +
#define CAM_PIN_SIOD    1   // I2C SDA (safe pin) +
#define CAM_PIN_SIOC    2   // I2C SCL (safe pin) +

#define CAM_PIN_D7      48  // Data 7 (safe pin) +
#define CAM_PIN_D6      46  // Data 6 (safe pin) +
#define CAM_PIN_D5      8   // Data 5 (safe pin) +
#define CAM_PIN_D4      7   // Data 4 (safe pin) +
#define CAM_PIN_D3      4   // Data 3 (safe pin) +
#define CAM_PIN_D2      41  // Data 2 (safe pin) +
#define CAM_PIN_D1      40  // Data 1 (safe pin) +
#define CAM_PIN_D0      39  // Data 0 (safe pin) +
#define CAM_PIN_VSYNC   6   // Vertical sync (safe pin) +
#define CAM_PIN_HREF    42  // Horizontal reference (safe pin) 
#define CAM_PIN_PCLK    5   // Pixel clock (safe pin) +
#define CAM_PWR         38

// SD card pin configuration (SPI mode)
// #define PIN_NUM_MISO    43
// #define PIN_NUM_MOSI    44
// #define PIN_NUM_CLK     0
// #define PIN_NUM_CS      13
//#define SD_PWR          9   // SD card power control

// SD card pin configuration (SDMMC)
#define SD_PIN_SCK  12
#define SD_PIN_CMD  13
#define SD_PIN_D0   11
#define SD_PIN_D1   10
#define SD_PIN_D2   38
#define SD_PIN_D3   14

// SD card pin configuration (SPI mode)
#define PIN_NUM_MISO    SD_PIN_D0
#define PIN_NUM_MOSI    SD_PIN_CMD
#define PIN_NUM_CLK     SD_PIN_SCK
#define PIN_NUM_CS      SD_PIN_D3
//#define SD_PWR          9   // SD card power control

#define MOUNT_POINT "/sdcard"
sdmmc_card_t *card_handle;

/* ----------------------------------- FUNCTIONS --------------------------------------- */

struct system_stats {   // Structure declaration
  char pwr;       // Camer power stats 
  char sd;        // SD initialisation stats
  char camera;    // Camera initialisation stats
};

sensor_t * sens;
camera_fb_t *fbsave;
bool taken = false;
struct system_stats sys;

RTC_DATA_ATTR int image_count = 0;
RTC_DATA_ATTR int time_set = 0;
RTC_DATA_ATTR


static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
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

    .xclk_freq_hz = 7000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_5MP,        // Reduced from UXGA (800x600 instead of 1600x1200)
    .jpeg_quality = 10,                 // Lower quality to reduce memory usage
    .fb_count = 1,                      // Single frame buffer without PSRAM
    .fb_location = CAMERA_FB_IN_PSRAM,  // Use DRAM instead of PSRAM
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
}; 


esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing camera");
    
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
        return err;
    }
    
    // Get camera sensor
    sensor_t * s = esp_camera_sensor_get();
    if (s == NULL) {
        ESP_LOGE(TAG, "Failed to get camera sensor");
        return ESP_FAIL;
    }

    // Configure camera settings for better quality
    s->set_brightness(s, 1);     // -2 to 2
    s->set_contrast(s, 0);       // -2 to 2
    s->set_saturation(s, 0);     // -2 to 2
    s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);       // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable
    s->set_aec2(s, 1);           // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);       // -2 to 2
    s->set_aec_value(s, 300);    // 0 to 1200
    s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);       // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)4);  // 0 to 6
    s->set_bpc(s, 1);            // 0 = disable , 1 = enable
    s->set_wpc(s, 1);            // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable
    s->set_lenc(s, 1);           // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);        // 0 = disable , 1 = enable
    s->set_vflip(s, 0);          // 0 = disable , 1 = enable
    s->set_dcw(s, 1);            // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
    
    ESP_LOGI(TAG, "Camera initialized successfully");
    
    sens = s;
    
    return ESP_OK;
}

esp_err_t deinit_camera(void)
{
    ESP_LOGI(TAG, "Deinitializing camera");
    
    // Perform sensor software reset before deinitializing
    if (sens != NULL) {
        ESP_LOGI(TAG, "Performing sensor reset");
        
        // Try to reset the sensor via I2C (OV5640 specific)
        // This helps ensure clean state for reinitialization
        sens->set_reg(sens, 0x3008, 0xff, 0x82); // Software reset
        vTaskDelay(pdMS_TO_TICKS(10)); // Wait for reset to complete
        
        // Clear the sensor reference
        sens = NULL;
    }
    
    // Deinitialize the camera driver
    esp_err_t err = esp_camera_deinit();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinit failed with error 0x%x", err);
        return err;
    }
    
    ESP_LOGI(TAG, "Camera deinitialized successfully");
    
    return ESP_OK;
}

esp_err_t init_sdcard(void)
{
    ESP_LOGI(TAG, "Initializing SD card");
    
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    sdmmc_card_t *card;
    const char mount_point[] = MOUNT_POINT;
    
    ESP_LOGI(TAG, "Initializing SD card using SPI peripheral");
    
    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = PIN_NUM_MOSI,
        .miso_io_num = PIN_NUM_MISO,
        .sclk_io_num = PIN_NUM_CLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };
    
    esp_err_t ret = spi_bus_initialize(host.slot, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bus.");
        return ret;
    }   
    
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = PIN_NUM_CS;
    slot_config.host_id = host.slot;
    
    ESP_LOGI(TAG, "Mounting filesystem");
    ret = esp_vfs_fat_sdspi_mount(mount_point, &host, &slot_config, &mount_config, &card);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set the CONFIG_EXAMPLE_FORMAT_IF_MOUNT_FAILED menuconfig option.");
                    } else {
                        ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                            "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }
    ESP_LOGI(TAG, "Filesystem mounted");
    
    // Card has been initialized, print its properties
    card_handle = card;
    sdmmc_card_print_info(stdout, card);
    
    return ESP_OK;
}

esp_err_t save_image_to_sd(camera_fb_t *fb, const char *filename)
{
    ESP_LOGI(TAG, "Saving image to SD card: %s", filename);

    char filepath[64];
    snprintf(filepath, sizeof(filepath), MOUNT_POINT"/%s", filename);

    FILE *file = fopen(filepath, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    if (fb == NULL) {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }

    size_t written = fwrite(fb->buf, 1, fb->len, file);
    fclose(file);

    if (written != fb->len) {
        ESP_LOGE(TAG, "Failed to write complete image. Written: %d, Expected: %d", written, fb->len);
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Image saved successfully: %s (%d bytes)", filepath, fb->len);
    return ESP_OK;
}

esp_err_t deinit_sdcard(void)
{
    esp_err_t ret = ESP_OK;
    
    // Unmount the filesystem
    if (card_handle != NULL) {
        ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to unmount filesystem: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "SD card unmounted successfully");
        }
        card_handle = NULL;
    }
    
    // Deinitialize SPI bus
    ret = spi_bus_free(SDSPI_DEFAULT_HOST);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
    }
    
    return ret;
}


void app_main(void)
{
    char filename[32];

    while (1) {        
        
        gpio_reset_pin(CAM_PWR);
        gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
        gpio_set_level(CAM_PWR, 0);

        // SD card init
        esp_err_t ret = init_sdcard();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SD card initialization failed");
            return;
        }

        // Camera init
        ret = init_camera();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Camera initialization failed");
            return;
        }


        // Taking picture
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb || fb == NULL) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(5000));
            ret = ESP_FAIL;
            continue;
        }
        else{
            ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);
            // Save to SD card
            ret = ESP_OK;
            snprintf(filename, sizeof(filename), "image%03d.jpg", image_count++);
            ret = save_image_to_sd(fb, filename);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save image");
            }
            esp_camera_fb_return(fb);
            
            // Camera deinit
            ret = deinit_camera();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Camera deinitialization failed");
                continue;
            }
            else{
                ESP_LOGI(TAG, "Camera deinitialized");
            }
        }

        
        //SD deinit
        ret = deinit_sdcard();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "SD deinitialization failed");
            continue;
        }
        else{
            ESP_LOGI(TAG, "SD deinitialized");
        }
        
        // Power down camera
        gpio_set_level(CAM_PWR, 1);
        gpio_hold_en(CAM_PWR);
        gpio_deep_sleep_hold_en();
        vTaskDelay(pdMS_TO_TICKS(50));

        // Deep sleep start
        ESP_LOGI(TAG, "Deep sleep start");
        esp_sleep_enable_timer_wakeup(5 * 1000 * 1000);
        ESP_LOGI(TAG, "timer wakeup source is ready");
        /* Enter sleep mode */
        esp_deep_sleep_start();
    }
}