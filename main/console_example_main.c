#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
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

#include "esp_wifi.h"
#include "img_converters.h"

static const char *TAG = "camera_sd";
#define PROMPT_STR CONFIG_IDF_TARGET

// ON PCB CONNECTION
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

typedef enum
{
    SD_MOUNTED,
    SD_NOT_MOUNTED,
    SD_MOUNT_ERROR
} sd_status_t;

sd_status_t sd_status = SD_NOT_MOUNTED;

typedef enum
{
    SYSTEM_STATE_INIT,
    SYSTEM_STATE_CAPTURE,
    SYSTEM_STATE_SAVE,
    SYSTEM_STATE_CLEANUP,
    SYSTEM_STATE_DAY_SLEEP,
    SYSTEM_STATE_NIGHT_SLEEP,
    SYSTEM_STATE_USB_CONNECTED,
    SYSTEM_STATE_FLASH_DRIVE,
    SYSTEM_STATE_OFF
} system_state_t;

typedef enum
{
    LOGIN_NOT_ATEMPTED,
    LOGIN_SUCCESSFUL,
    LOGIN_UNSUCCESSFUL,
    LOGIN_TIME_OUT,
    LOGIN_TO_FLASH_DRIVE
} login_state_t;

sensor_t *sens;
camera_fb_t *fbsave;

RTC_DATA_ATTR int running_mode = 0; // 0 = normal, 1 = motion detection
RTC_DATA_ATTR int onoff_mode = 0; // 0 = on, 1 = off
login_state_t login_state = LOGIN_NOT_ATEMPTED;
#define CORRECT_PASSWORD "hello"

/* ----------------------------------- FUNCTIONS --------------------------------------- */

/* Camera functions */
static camera_config_t camera_config = {
    .pin_pwdn = -1,
    .pin_reset = -1,
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

    .xclk_freq_hz = 40000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_QXGA,     // Reduced from UXGA (800x600 instead of 1600x1200)
    .jpeg_quality = 5,                // Lower quality to reduce memory usage
    .fb_count = 3,                     // Single frame buffer without PSRAM
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
    s->set_brightness(s, 0);                 // -2 to 2
    s->set_contrast(s, 0);                   // -2 to 2
    s->set_saturation(s, 1);                 // -2 to 2
    s->set_special_effect(s, 0);             // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);                   // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);                   // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);                    // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);              // 0 = disable , 1 = enable
    s->set_aec2(s, 1);                       // 0 = disable , 1 = enable
    s->set_ae_level(s, 2);                   // -2 to 2
    s->set_aec_value(s, 600);                 // 0 to 1200
    s->set_gain_ctrl(s, 0);                  // 0 = disable , 1 = enable
    s->set_agc_gain(s, 5);                  // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)4); // 0 to 6
    s->set_bpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);                    // 0 = disable , 1 = enable
    s->set_lenc(s, 1);                       // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);                    // 0 = disable , 1 = enable
    s->set_vflip(s, 0);                      // 0 = disable , 1 = enable
    s->set_dcw(s, 1);                        // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);                   // 0 = disable , 1 = enable

    //ESP_LOGI(TAG, "%x", s->set_raw_gma(s, 1));
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

/* SD card function */
static esp_err_t init_sd_card(void)
{
    esp_err_t ret;

    // Options for mounting the filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 3,
        .allocation_unit_size = 64 * 1024,
    };

    // Initialize SD card host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = 40*1000; // Use high speed

    // Initialize SD card slot configuration
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4; // 4-line mode
    slot_config.clk = PIN_NUM_CLK;
    slot_config.cmd = PIN_NUM_MOSI;
    slot_config.d0 = PIN_NUM_MISO;
    slot_config.d1 = PIN_NUM_D1;
    slot_config.d2 = PIN_NUM_D2;
    slot_config.d3 = PIN_NUM_CS;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    ret = sdmmc_host_init();
    ESP_ERROR_CHECK(ret);
    ret = sdmmc_host_init_slot(SDMMC_HOST_SLOT_1, &slot_config);
    ESP_ERROR_CHECK(ret);

    // Mount filesystem
    ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card_handle);

    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
        {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                          "If you want the card to be formatted, set format_if_mount_failed = true.");
        }
        else
        {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                          "Make sure SD card lines have pull-up resistors in place.",
                     esp_err_to_name(ret));
        }
        return ret;
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card_handle);

    sd_card_mounted = true;
    return ESP_OK;
}

esp_err_t save_image_to_sd(camera_fb_t *fb, const char *dir, const char *filename)
{
    char filepath[200];
    snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, dir);

    if(fb == NULL)
    {
        ESP_LOGE(TAG, "Frame buffer is NULL, cannot save image");
        return ESP_FAIL;
    }
    int64_t t_1 = esp_timer_get_time();

    struct stat st;
    if (stat(filepath, &st) != 0) {
        mkdir(filepath, 0700);  // Create directory if doesn't exist
    }

    snprintf(filepath, sizeof(filepath), "%s/%s/%s", MOUNT_POINT, dir, filename);

    const int flags = O_WRONLY | O_CREAT | O_TRUNC;
    int fd = open(filepath, flags, 0666);
    if (fd == -1)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    int64_t t_2 = esp_timer_get_time();
    ESP_LOGI(TAG, "Time taken to open file: %.lld ms", (t_2 - t_1) / 1000);
    
    // uint8_t *bmp_data;  // Will hold the BMP data
    // size_t bmp_len;     // Will hold the BMP data length
    // int64_t t_start_conv = esp_timer_get_time();
    // if (frame2bmp(fb, &bmp_data, &bmp_len) == false) {
    //     ESP_LOGW(TAG, "BMP conversion failed");
    //     fclose(file);
    //     return ESP_FAIL;
    // }
    // int64_t t_end_conv = esp_timer_get_time();
    // ESP_LOGI(TAG, "Time taken to convert to BMP: %.lld ms", (t_end_conv - t_start_conv) / 1000);

    int ret = write(fd, fb->buf, fb->len);
    if (ret != fb->len)
    {
        ESP_LOGE(TAG, "Failed to write complete image data to file: %s", filepath);
        close(fd);
        return ESP_FAIL;
    }

    int64_t t_3 = esp_timer_get_time();
    ESP_LOGI(TAG, "Time taken to write file: %.lld ms", (t_3 - t_2) / 1000);

    close(fd);

    int64_t t_4 = esp_timer_get_time();
    ESP_LOGI(TAG, "Time taken to close file: %.lld ms", (t_4 - t_3) / 1000);
    ESP_LOGI(TAG, "Image saved to %s (%u bytes)", filepath, (unsigned int)ret);
    return ESP_OK;
}

#define DMA_BUFFER_SIZE (64 * 1024)

esp_err_t write_psram_to_sd_fast(const char *dir, const char *filename, uint8_t *psram_data, size_t data_size)
{
    FILE *f = NULL;
    uint8_t *dma_buffer = NULL;
    esp_err_t ret = ESP_OK;
    int64_t start_time, end_time;

    char filepath[200];
    snprintf(filepath, sizeof(filepath), "%s/%s", MOUNT_POINT, dir);

    struct stat st;
    if (stat(filepath, &st) != 0) {
        mkdir(filepath, 0700);  // Create directory if doesn't exist
    }
    
    snprintf(filepath, sizeof(filepath), "%s/%s/%s", MOUNT_POINT, dir, filename);
    
    ESP_LOGI(TAG, "Writing %zu bytes from PSRAM to SD card: %s", data_size, filepath);
    
    // Open file with write buffering disabled for more predictable performance
    int64_t t_open1 = esp_timer_get_time();
    f = fopen(filepath, "wb");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing");
        ret = ESP_FAIL;
        goto cleanup;
    }
    int64_t t_open2 = esp_timer_get_time();
    ESP_LOGI(TAG, "Time taken to open file: %.2f ms", (t_open2 - t_open1) / 1000.0f);

    start_time = esp_timer_get_time();
    
    // Allocate DMA-capable buffer in internal RAM
    dma_buffer = (uint8_t*)heap_caps_malloc(DMA_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if (dma_buffer == NULL) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        return ESP_ERR_NO_MEM;
    }

    // Disable buffering for more direct control
    setvbuf(f, NULL, _IONBF, 0);
    
    // Write in chunks
    size_t bytes_written = 0;
    size_t bytes_remaining = data_size;
    
    while (bytes_remaining > 0) {
        size_t chunk_size = (bytes_remaining > DMA_BUFFER_SIZE) ? DMA_BUFFER_SIZE : bytes_remaining;
        
        // Copy from PSRAM to DMA buffer
        memcpy(dma_buffer, psram_data + bytes_written, chunk_size);
        
        // Write to SD card
        size_t written = fwrite(dma_buffer, 1, chunk_size, f);
        if (written != chunk_size) {
            ESP_LOGE(TAG, "Write error: expected %zu, wrote %zu", chunk_size, written);
            ret = ESP_FAIL;
            goto cleanup;
        }
        
        bytes_written += written;
        bytes_remaining -= written;
    }
    
    // Ensure data is flushed to SD card
    fflush(f);
    fsync(fileno(f));
    
    end_time = esp_timer_get_time();
    float duration_ms = (end_time - start_time) / 1000.0f;
    float speed_kbps = (data_size / 1024.0f) / (duration_ms / 1000.0f);
    
    ESP_LOGI(TAG, "Write complete: %zu bytes in %.2f ms (%.2f KB/s)", 
             data_size, duration_ms, speed_kbps);
    
cleanup:
    if (f) fclose(f);
    if (dma_buffer) free(dma_buffer);
    
    return ret;
}

static QueueHandle_t time_queue = NULL;
static QueueHandle_t file_queue = NULL;

typedef enum
{
    SD_STATE_TIME_RECEIVE,
    SD_STATE_OPEN,
    SD_STATE_SEND_FILE,
    SD_STATE_ERROR
} sd_state_t;

void sd_hadler(void* arg)
{
    FILE *f = NULL;
    struct tm timeinfo;
    char dir[64];
    char filepath[200];
    sd_state_t sd_state = SD_STATE_TIME_RECEIVE;
    int recieveAttempts = 0;
    while (1) {
        switch (sd_state) {
        case SD_STATE_TIME_RECEIVE:
            if (time_queue == NULL) {
                if(recieveAttempts > 100) {
                    ESP_LOGE(TAG, "Time queue not created, moving to error state");
                    vQueueDelete(time_queue);
                    time_queue = NULL;
                    sd_state = SD_STATE_ERROR;
                }
                recieveAttempts++;
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
            else if (xQueueReceive(time_queue, &timeinfo, pdMS_TO_TICKS(100))) {
                sd_state = SD_STATE_OPEN;
                vQueueDelete(time_queue);
                time_queue = NULL;
                recieveAttempts = 0;
                break;
            }
            else{
                if(recieveAttempts > 150) {
                    ESP_LOGE(TAG, "Time queue not created, moving to error state");
                    vQueueDelete(time_queue);
                    time_queue = NULL;
                    sd_state = SD_STATE_ERROR;
                }
                recieveAttempts++;
                vTaskDelay(pdMS_TO_TICKS(10));
                break;
            }
        case SD_STATE_OPEN:

            //Opening directory based on time
            snprintf(dir, sizeof(dir), "%s/%04dy%02dm%02dd", MOUNT_POINT, timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday);

            struct stat st;
            if (stat(dir, &st) != 0) {
                mkdir(dir, 0700);  // Create directory if doesn't exist
            }

            snprintf(filepath, sizeof(filepath), "%s/%02dx%02dx%02d.jpg", dir, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);

            f = fopen(filepath, "wb");
            if (f == NULL) {
                ESP_LOGE(TAG, "Failed to open file for writing");
                sd_state = SD_STATE_ERROR;
                break;
            }

            sd_state = SD_STATE_SEND_FILE;
            break;
        case SD_STATE_SEND_FILE:
            file_queue = xQueueCreate(3, sizeof(struct FILE*));
            if (file_queue == NULL) {
                ESP_LOGE(TAG, "Failed to create file queue");
                sd_state = SD_STATE_ERROR;
                break;
            }
            if (xQueueSend(file_queue, &f, 0) != pdPASS) {
                ESP_LOGE(TAG, "Failed to send time to queue");
                sd_state = SD_STATE_ERROR;
                break;
            }
            vTaskDelete(NULL);
            break;
        case SD_STATE_ERROR:
            if (f != NULL) {
                fclose(f);
                f = NULL;
            }
            if (time_queue != NULL) {
                vQueueDelete(time_queue);
                time_queue = NULL;
            }
            if (file_queue != NULL) {
                vQueueDelete(file_queue);
                file_queue = NULL;
            }
            vTaskDelete(NULL);
            sd_state = SD_STATE_TIME_RECEIVE;
            break;        
        }
    }

}

esp_err_t deinit_sdcard(void)
{
    esp_err_t ret = ESP_OK;

    // Unmount the filesystem
    if (card_handle != NULL)
    {
        ret = esp_vfs_fat_sdcard_unmount(MOUNT_POINT, card_handle);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "Failed to unmount filesystem: %s", esp_err_to_name(ret));
        }
        else
        {
            ESP_LOGI(TAG, "SD card unmounted successfully");
        }
        card_handle = NULL;
    }

    return ret;
}

/* Storage callbacks for TinyUSB MSC*/
static int32_t msc_read_cb(uint32_t lba, uint32_t offset, void *buffer, uint32_t bufsize)
{
    ESP_LOGD(TAG, "MSC Read - LBA: %lu, Offset: %lu, Size: %lu", lba, offset, bufsize);

    if (!sd_card_mounted || !card_handle)
    {
        ESP_LOGE(TAG, "SD card not mounted");
        return -1;
    }

    esp_err_t ret = sdmmc_read_sectors(card_handle, buffer, lba + (offset / 512), bufsize / 512);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to read sectors: %s", esp_err_to_name(ret));
        return -1;
    }

    return bufsize;
}

static int32_t msc_write_cb(uint32_t lba, uint32_t offset, uint8_t *buffer, uint32_t bufsize)
{
    ESP_LOGD(TAG, "MSC Write - LBA: %lu, Offset: %lu, Size: %lu", lba, offset, bufsize);

    if (!sd_card_mounted || !card_handle)
    {
        ESP_LOGE(TAG, "SD card not mounted");
        return -1;
    }

    esp_err_t ret = sdmmc_write_sectors(card_handle, buffer, lba + (offset / 512), bufsize / 512);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to write sectors: %s", esp_err_to_name(ret));
        return -1;
    }

    return bufsize;
}

static esp_err_t init_usb_msc(void)
{
    ESP_LOGI(TAG, "Initializing USB MSC");

    if (!sd_card_mounted || !card_handle)
    {
        ESP_LOGE(TAG, "SD card must be mounted before initializing USB MSC");
        return ESP_FAIL;
    }

    // Initialize TinyUSB
    tinyusb_config_t tusb_cfg = {
        .device_descriptor = NULL,
        .string_descriptor = NULL,
        .external_phy = false,
    };

    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    // Configure MSC storage
    const tinyusb_msc_sdmmc_config_t config_sdmmc = {
        .card = card_handle,
    };

    // Initialize MSC with callbacks
    tinyusb_msc_storage_init_sdmmc(&config_sdmmc);
    tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, NULL);

    ESP_LOGI(TAG, "USB MSC initialized successfully");
    return ESP_OK;
}

/* RTC functions */
static esp_err_t read_time(struct tm *timeinfo)
{
    static i2c_master_dev_handle_t ds3231_dev = NULL;

    struct tm timeinfo_rtc;
    esp_err_t ret = ds3231_init_desc(&ds3231_dev, I2C_NUM_0, RTC_SDA, RTC_SCL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DS3231: %s", esp_err_to_name(ret));
        return 1;
    }

    ret = ds3231_get_time(ds3231_dev, &timeinfo_rtc);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get current time");
        return ret;
    }

    *timeinfo = timeinfo_rtc;

    ret = ds3231_deinit(ds3231_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deinitialize DS3231: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

static esp_err_t set_alarm(void)
{
    static i2c_master_dev_handle_t ds3231_dev = NULL;

    struct tm timeinfo;

    esp_err_t ret = ds3231_init_desc(&ds3231_dev, I2C_NUM_0, RTC_SDA, RTC_SCL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DS3231: %s", esp_err_to_name(ret));
        return 1;
    }

    ret = ds3231_get_time(ds3231_dev, &timeinfo);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get current time");
        return ret;
    }

    // Set alarm for 30 seconds from now
    timeinfo.tm_sec = (timeinfo.tm_sec + 15) % 60;
    if (timeinfo.tm_sec < 30)
    {
        timeinfo.tm_min++; // Increment minute if seconds wrap around
    }

    ds3231_set_alarm(ds3231_dev, DS3231_ALARM_1, &timeinfo, DS3231_ALARM1_MATCH_SECMIN, NULL, 0);

    ret = ds3231_deinit(ds3231_dev);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to deinitialize DS3231: %s", esp_err_to_name(ret));
        return ret;
    }

    printf("Alarm set to trigger every 30 seconds\n");
    return ESP_OK;
}

esp_err_t append_text_to_file(const char *path, const char *text)
{
    ESP_LOGI(TAG, "Appending to file: %s", path);

    FILE *f = fopen(path, "a");
    if (f == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for appending");
        return ESP_FAIL;
    }

    fprintf(f, "%s", text);
    fclose(f);

    ESP_LOGI(TAG, "Text appended successfully");
    return ESP_OK;
}

/* ----------------------------------- CONSOLE FUNCTION --------------------------------------- */

/* Login command */
static struct
{
    struct arg_str *password;
    struct arg_end *end;
} login_args;

static int login_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&login_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, login_args.end, argv[0]);
        return 1;
    }

    const char *password = "hello";
    const char *pass = login_args.password->sval[0];
    printf("Inputed password: %s\n", pass);

    if (strcmp(pass, password) == 0)
    {
        printf("Login succesful!\n");
        login_state = LOGIN_SUCCESSFUL;
    }
    else
    {
        login_state = LOGIN_UNSUCCESSFUL;
        printf("Wrong password\n");
    }

    return 0;
}

static void register_login_command(void)
{
    login_args.password = arg_str1(NULL, NULL, "<password>", "Password");
    login_args.end = arg_end(1);

    const esp_console_cmd_t login_cmd = {
        .command = "login",
        .help = "Login",
        .hint = NULL,
        .func = &login_command_handler,
        .argtable = &login_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&login_cmd));
}

/* Turining camera off */
static int off_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: off\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    onoff_mode = 1;
    printf("Turning off device");
    printf("\n");

    return 0;
}

static void register_off_command(void)
{
    const esp_console_cmd_t off_cmd = {
        .command = "off",
        .help = "Turning off device",
        .hint = NULL,
        .func = &off_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&off_cmd));
}

/* Turining camera on */
static int on_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: off\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    onoff_mode = 0;
    printf("Turning on device");
    printf("\n");

    return 0;
}

static void register_on_command(void)
{
    const esp_console_cmd_t on_cmd = {
        .command = "on",
        .help = "Turning on device",
        .hint = NULL,
        .func = &on_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&on_cmd));
}

/* Turining camera into sd card */
static int sd_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: sd\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    login_state = LOGIN_TO_FLASH_DRIVE;
    printf("Initialising flash drive");
    printf("\n");

    return 0;
}

static void register_sd_command(void)
{
    const esp_console_cmd_t sd_cmd = {
        .command = "sd",
        .help = "Starting MSC mode",
        .hint = NULL,
        .func = &sd_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&sd_cmd));
}

/* RTC commands */
static i2c_master_dev_handle_t global_ds3231_dev = NULL;

static int init_rtc_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: initrtc\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    esp_err_t ret = ds3231_init_desc(&global_ds3231_dev, I2C_NUM_1, RTC_SDA, RTC_SCL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DS3231: %s", esp_err_to_name(ret));
        return 1;
    }

    printf("RTC initialized successfully\n");
    return 0;
}

static void register_init_rtc_command(void)
{
    const esp_console_cmd_t init_rtc_cmd = {
        .command = "init_rtc",
        .help = "Initialize RTC",
        .hint = NULL,
        .func = &init_rtc_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&init_rtc_cmd));
}

static int temp_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: temp\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    esp_err_t ret;
    i2c_master_dev_handle_t ds3231_dev;
    ret = ds3231_init_desc(&ds3231_dev, I2C_NUM_1, RTC_SDA, RTC_SCL);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to initialize DS3231: %s", esp_err_to_name(ret));
        return 1;
    }

    int16_t raw_temp;
    ret = ds3231_get_raw_temp(ds3231_dev, &raw_temp);
    if (ret == ESP_OK)
    {
        // Convert raw temperature to float
        float temp_celsius = raw_temp * 0.25;
        ESP_LOGI(TAG, "Raw Temperature: %d (0x%04X), Temperature: %.2f°C",
                 raw_temp, raw_temp, temp_celsius);
    }
    else
    {
        ESP_LOGE(TAG, "Failed to read temperature: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(TAG, "DS3231 initialized successfully!");

    return 0;
}

static void register_temp_command(void)
{
    const esp_console_cmd_t temp_cmd = {
        .command = "temp",
        .help = "Getting RTC temperature",
        .hint = NULL,
        .func = &temp_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&temp_cmd));
}

static struct
{
    struct arg_int *year;
    struct arg_int *month;
    struct arg_int *day;
    struct arg_int *hour;
    struct arg_int *minute;
    struct arg_end *end;
} time_args;

static int time_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **)&time_args);
    if (nerrors != 0)
    {
        arg_print_errors(stderr, time_args.end, argv[0]);
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    if (global_ds3231_dev == NULL)
    {
        printf("RTC not initialized. Please run 'init_rtc' command first.\n");
        return 1;
    }

    struct tm time = {
        .tm_year = time_args.year->ival[0] - 1900,
        .tm_mon = time_args.month->ival[0] - 1,
        .tm_mday = time_args.day->ival[0],
        .tm_hour = time_args.hour->ival[0],
        .tm_min = time_args.minute->ival[0],
        .tm_sec = 0};

    esp_err_t ret = ds3231_set_time(global_ds3231_dev, &time);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to set time");
        return 1;
    }

    printf("Time set to %02d:%02d:00\n", time.tm_hour, time.tm_min);
    return 0;
}

static void register_time_command(void)
{
    time_args.year = arg_int1(NULL, NULL, "<year>", "Year (e.g., 2024)");
    time_args.month = arg_int1(NULL, NULL, "<month>", "Month (1-12)");
    time_args.day = arg_int1(NULL, NULL, "<day>", "Day (1-31)");
    time_args.hour = arg_int1(NULL, NULL, "<hour>", "Hour (0-23)");
    time_args.minute = arg_int1(NULL, NULL, "<minute>", "Minute (0-59)");
    time_args.end = arg_end(2);

    const esp_console_cmd_t time_cmd = {
        .command = "settime",
        .help = "Set RTC time (hour minute)",
        .hint = NULL,
        .func = &time_command_handler,
        .argtable = &time_args};

    ESP_ERROR_CHECK(esp_console_cmd_register(&time_cmd));
}

static int get_time_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: gettime\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    if (global_ds3231_dev == NULL)
    {
        printf("RTC not initialized. Please run 'init_rtc' command first.\n");
        return 1;
    }

    struct tm timeinfo;
    esp_err_t ret = ds3231_get_time(global_ds3231_dev, &timeinfo);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "Failed to get time from RTC");
        return 1;
    }

    printf("Current RTC time: %04d.%02d.%02d_%02d:%02d:%02d\n", timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday, timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    return 0;
}

static void register_get_time_command(void)
{
    const esp_console_cmd_t get_time_cmd = {
        .command = "gettime",
        .help = "Get RTC time",
        .hint = NULL,
        .func = &get_time_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&get_time_cmd));
}

/*ADC functions*/

#define ADC_UNIT ADC_UNIT_2
#define ADC_CHANNEL ADC_CHANNEL_5 // GPIO16 for ADC2_CH5
#define ADC_ATTEN ADC_ATTEN_DB_12
#define ADC_BITWIDTH ADC_BITWIDTH_12

static adc_oneshot_unit_handle_t adc_handle;
static adc_cali_handle_t adc_cali_handle = NULL;
static bool do_calibration = false;

static bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "Calibration scheme: Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated)
    {
        ESP_LOGI(TAG, "Calibration scheme: Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK)
        {
            calibrated = true;
        }
    }
#endif

    adc_cali_handle = handle;
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC calibration success");
    }
    else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated)
    {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    }
    else
    {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}

// ADC Calibration Deinit
static void adc_calibration_deinit(void)
{
    if (do_calibration)
    {
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "Deregister curve fitting calibration scheme");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(adc_cali_handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
        ESP_LOGI(TAG, "Deregister line fitting calibration scheme");
        ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(adc_cali_handle));
#endif
    }
}

static int bat_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: bat\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    esp_err_t ret;
    int adc_raw;
    int voltage;
    char adc_read[200];

    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    // Configure ADC channel
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN,
        .bitwidth = ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &config));

    do_calibration = adc_calibration_init(ADC_UNIT, ADC_CHANNEL, ADC_ATTEN);

    ESP_LOGI(TAG, "Starting ADC readings...");

    ret = adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_raw);
    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "ADC Raw: %d", adc_raw);

        // Convert to voltage if calibration is available
        if (do_calibration)
        {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc_cali_handle, adc_raw, &voltage));
            ESP_LOGI(TAG, "ADC Voltage: %d mV", voltage);
            voltage = voltage * 1.4; // Assuming a voltage divider with equal resistors
            printf("Battery Voltage: %d mV\n", voltage);
            snprintf(adc_read, sizeof(adc_read), "Bat Voltage: %d mV;", voltage);
        }
    }
    else
    {
        ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(ret));
    }
    init_sd_card();
    append_text_to_file(MOUNT_POINT "/log.txt", adc_read);
    ESP_ERROR_CHECK(adc_oneshot_del_unit(adc_handle));
    adc_calibration_deinit();

    return 0;
}

static void register_bat_command(void)
{
    const esp_console_cmd_t bat_cmd = {
        .command = "bat",
        .help = "Read battery voltage",
        .hint = NULL,
        .func = &bat_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&bat_cmd));
}

/* Mode command */
static int mode_command_handler(int argc, char **argv)
{
    if (argc < 1)
    {
        printf("Usage: mode\n");
        return 1;
    }

    if (login_state != LOGIN_SUCCESSFUL)
    {
        printf("You are not logged in\n");
        return 1;
    }

    running_mode = 1;
    printf("Running mode changed to: %d\n", running_mode);
    printf("0 = normal mode, 1 = motion detection mode\n");
    return 0;
}

static void register_mode_command(void)
{
    const esp_console_cmd_t mode_cmd = {
        .command = "mode",
        .help = "Toggle running mode between normal (0) and motion detection (1)",
        .hint = NULL,
        .func = &mode_command_handler,
        .argtable = NULL};

    ESP_ERROR_CHECK(esp_console_cmd_register(&mode_cmd));
}

/* ---------------------------------------------------------------------------------------------------------------------------------------------- */

void app_main(void)
{
    //esp_log_level_set("*", ESP_LOG_ERROR);
    system_state_t state = SYSTEM_STATE_INIT;
    esp_err_t ret = ESP_OK;
    char filename[200];
    char dir[200];

    uint8_t *dma_buffer = NULL;
    FILE *f = NULL;
    int reciveAttempts = 0;
    camera_fb_t *fb = NULL;

    struct tm timeinfo;

    int64_t t_init = 0;
    int64_t t_pwr_up = 0;
    int64_t t_capture = 0;
    int64_t t_capture_end = 0;
    int64_t t_save = 0;
    int64_t t_cleanup = 0;
    int64_t t_end = 0;

    while (1)
    {
        switch (state)
        {
        case SYSTEM_STATE_INIT:
            esp_wifi_stop();
            t_init = esp_timer_get_time();

            gpio_set_direction(USB_INT, GPIO_MODE_INPUT);
            gpio_reset_pin(CAM_PWR);
            gpio_hold_dis(CAM_PWR);
            gpio_deep_sleep_hold_dis();
            gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
            gpio_set_level(CAM_PWR, 1);


            if (gpio_get_level(USB_INT) == 1 && login_state != LOGIN_TIME_OUT && running_mode == 0)
            {
                state = SYSTEM_STATE_USB_CONNECTED;
                break;
            }
            else
            {
                login_state = LOGIN_NOT_ATEMPTED;
            }

            if(onoff_mode == 1)
            {
                ESP_LOGI(TAG, "Device is turned off");
                state = SYSTEM_STATE_OFF;
                break;
            }

            ret = read_time(&timeinfo);
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to read time from RTC");
            }
            else
            {
                if (timeinfo.tm_hour >= 22 || timeinfo.tm_hour < 6) // Sleep between 10 PM and 6 AM
                {
                    state = SYSTEM_STATE_NIGHT_SLEEP;
                    break;
                }
            }

            if ((ret = init_sd_card()) != ESP_OK)
            {
                ESP_LOGE(TAG, "SD card init failed");
                state = SYSTEM_STATE_CLEANUP;
                break;
            }

            xTaskCreatePinnedToCore(sd_hadler, "sd_hadler", 4096, NULL, 2, NULL, 1);
            time_queue = xQueueCreate(3, sizeof(struct tm));
            if (time_queue == NULL) {
                ESP_LOGE(TAG, "Failed to create queue");
            } else {
                if (xQueueSend(time_queue, &timeinfo, 0) != pdPASS) {
                    ESP_LOGE(TAG, "Failed to send time to queue");
                }
            }
            
            state = SYSTEM_STATE_CAPTURE;
            break;
            
        case SYSTEM_STATE_CAPTURE:

            t_pwr_up = esp_timer_get_time();

            gpio_set_level(CAM_PWR, 0);
            vTaskDelay(pdMS_TO_TICKS(10)); // Wait for camera to power up
            if ((ret = init_camera()) != ESP_OK)
            {
                ESP_LOGE(TAG, "Camera init failed");
                state = SYSTEM_STATE_CLEANUP;
                gpio_set_level(CAM_PWR, 1);
                break;
            }

            t_capture = esp_timer_get_time();

            fb = esp_camera_fb_get();
            esp_camera_fb_return(fb);
            fb = esp_camera_fb_get();
            if (!fb)
            {
                ESP_LOGE(TAG, "Camera capture failed");
                gpio_set_level(CAM_PWR, 1);
                state = SYSTEM_STATE_CLEANUP;
                break;
            }
            ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);
            gpio_set_level(CAM_PWR, 1);
            
            t_capture_end = esp_timer_get_time();
            ESP_LOGI(TAG, "Capture time: %lld ms", (long long)(t_capture_end - t_capture) / 1000);
            state = SYSTEM_STATE_SAVE;
            break;

        case SYSTEM_STATE_SAVE:
                    t_save = esp_timer_get_time();
            // uint8_t *bmp_data;  // Will hold the BMP data
            // size_t bmp_len;     // Will hold the BMP data length
            // int64_t t_start_conv = esp_timer_get_time();
            // if (frame2bmp(fb, &bmp_data, &bmp_len) == false) {
            //     ESP_LOGW(TAG, "BMP conversion failed");
            //     state = SYSTEM_STATE_CLEANUP;
            //     break;
            // }
            // int64_t t_end_conv = esp_timer_get_time();
            // ESP_LOGI(TAG, "Time taken to convert to BMP: %.lld ms", (t_end_conv - t_start_conv) / 1000);
            if(file_queue == NULL) {
                vTaskDelay(pdMS_TO_TICKS(10));
                reciveAttempts++;
                if(reciveAttempts > 500) {
                    ESP_LOGE(TAG, "File queue not created, moving to cleanup state %d", reciveAttempts);
                    state = SYSTEM_STATE_CLEANUP;
                }
                break;
            }
            else if (xQueueReceive(file_queue, &f, pdMS_TO_TICKS(100))) {
                vQueueDelete(file_queue);
                file_queue = NULL;
                reciveAttempts = 0;
            }
            else{
                vTaskDelay(pdMS_TO_TICKS(10));
                reciveAttempts++;
                if(reciveAttempts > 600) {
                    ESP_LOGE(TAG, "File not received, moving to cleanup state");
                    vQueueDelete(file_queue);
                    file_queue = NULL;
                    state = SYSTEM_STATE_CLEANUP;
                }
                break;
            }

            if(fb == NULL || fb->len == 0 || fb->buf == NULL)
            {
                ESP_LOGE(TAG, "Frame buffer is NULL, cannot save image");
                state = SYSTEM_STATE_CLEANUP;
                break;
            }

            dma_buffer = (uint8_t*)heap_caps_malloc(DMA_BUFFER_SIZE, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
            if (dma_buffer == NULL) {
                ESP_LOGE(TAG, "Failed to allocate DMA buffer");
                state = SYSTEM_STATE_CLEANUP;
                break;
            }

            // Disable buffering for more direct control
            setvbuf(f, NULL, _IONBF, 0);
            
            // Write in chunks
            size_t bytes_written = 0;
            size_t bytes_remaining = fb->len;
            //size_t bytes_remaining = bmp_len;
            
            
            while (bytes_remaining > 0) {
                size_t chunk_size = (bytes_remaining > DMA_BUFFER_SIZE) ? DMA_BUFFER_SIZE : bytes_remaining;
                
                // Copy from PSRAM to DMA buffer
                memcpy(dma_buffer, fb->buf + bytes_written, chunk_size);
                
                // Write to SD card
                size_t written = fwrite(dma_buffer, 1, chunk_size, f);
                if (written != chunk_size) {
                    ESP_LOGE(TAG, "Write error: expected %zu, wrote %zu", chunk_size, written);
                    ret = ESP_FAIL;
                    goto cleanup;
                }
                
                bytes_written += written;
                bytes_remaining -= written;
            }
            
            // Ensure data is flushed to SD card
            fflush(f);
            fsync(fileno(f));
            
            cleanup:
                if (f) fclose(f);
                if (dma_buffer) free(dma_buffer);
            
            fb = NULL;
            state = SYSTEM_STATE_CLEANUP;
            break;

        case SYSTEM_STATE_CLEANUP:
            t_cleanup = esp_timer_get_time();
            
            if ((ret = deinit_sdcard()) != ESP_OK)
            {
                ESP_LOGE(TAG, "SD card deinit failed");
            }

            state = SYSTEM_STATE_DAY_SLEEP;
            break;

        case SYSTEM_STATE_DAY_SLEEP:
            gpio_set_level(CAM_PWR, 1);
            gpio_hold_en(CAM_PWR);
            gpio_deep_sleep_hold_en();
            vTaskDelay(pdMS_TO_TICKS(10));

            t_end = esp_timer_get_time();
            printf("Init time: %lld ms\n", (long long)(t_end - t_init) / 1000);
            printf("Power up time: %lld ms\n", (long long)(t_capture_end - t_pwr_up) / 1000);
            printf("Capture time: %lld ms\n", (long long)(t_capture_end - t_capture) / 1000);
            printf("Save time: %lld ms\n", (long long)(t_cleanup - t_save) / 1000);
            printf("Cleanup time: %lld ms\n", (long long)(t_end - t_cleanup) / 1000);

            // ESP_LOGI(TAG, "Day time detected, sleeping for 30 seconds"); 
            esp_sleep_enable_timer_wakeup(5 * 1000 * 1000); // 5 minutes
            if(running_mode == 0)
            {
                esp_sleep_enable_ext0_wakeup(21, 1);             // Wake up when GPIO 21 goes high
            }
        
            esp_deep_sleep_start();
            break;

        case SYSTEM_STATE_NIGHT_SLEEP:
            gpio_set_level(CAM_PWR, 1);
            gpio_hold_en(CAM_PWR);
            gpio_deep_sleep_hold_en();
            vTaskDelay(pdMS_TO_TICKS(50));

            gpio_reset_pin(GPIO_NUM_0);
            gpio_reset_pin(GPIO_NUM_4);
            gpio_reset_pin(GPIO_NUM_12);
            gpio_reset_pin(GPIO_NUM_13);
            gpio_reset_pin(GPIO_NUM_14);
            gpio_reset_pin(GPIO_NUM_15);
            gpio_reset_pin(GPIO_NUM_26);
            gpio_reset_pin(GPIO_NUM_27);
            gpio_reset_pin(GPIO_NUM_33);
            gpio_reset_pin(GPIO_NUM_34);
            gpio_reset_pin(GPIO_NUM_35);
            gpio_reset_pin(GPIO_NUM_36);
            gpio_reset_pin(GPIO_NUM_37);
            gpio_reset_pin(GPIO_NUM_38);
            gpio_reset_pin(GPIO_NUM_39);

            ESP_LOGI(TAG, "Entering deep sleep");
            t_end = esp_timer_get_time();
            printf("Init time: %lld ms\n", (long long)(t_end - t_init) / 1000);

            ESP_LOGI(TAG, "Day time detected, sleeping for 30 seconds"); 
            esp_sleep_enable_timer_wakeup(30 * 1000 * 1000); // 30 minutes
            if(gpio_get_level(USB_INT) == 1)
            {
                ESP_LOGI(TAG, "Charging, waking up when charger disconected");
                esp_sleep_enable_ext0_wakeup(21, 0);             // Wake up when GPIO 21 goes low
            }
            else{
                esp_sleep_enable_ext0_wakeup(21, 1);             // Wake up when GPIO 21 goes high
            }

            ESP_LOGI(TAG, "Night time detected, sleeping for 30 minutes");
            esp_sleep_enable_timer_wakeup(30 * 60 * 1000 * 1000); // 30 minutes

            esp_deep_sleep_start();
            break;

        case SYSTEM_STATE_OFF:
            gpio_set_level(CAM_PWR, 1);
            gpio_hold_en(CAM_PWR);
            gpio_deep_sleep_hold_en();
            vTaskDelay(pdMS_TO_TICKS(10));

            ESP_LOGI(TAG, "Turning off the device");
            esp_sleep_enable_ext0_wakeup(21, 1);             // Wake up when GPIO 21 goes high
            esp_deep_sleep_start();
            break;

        case SYSTEM_STATE_USB_CONNECTED:
            esp_console_repl_t *repl = NULL;
            esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();

            repl_config.prompt = PROMPT_STR ">";
            repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

            esp_console_register_help_command();
            register_system_common();
            register_login_command();
            register_sd_command();
            register_temp_command();
            register_init_rtc_command();
            register_time_command();
            register_get_time_command();
            register_mode_command();
            register_bat_command();
            register_off_command();
            register_on_command();

            ESP_LOGI(TAG, "Starting ESP32-S3 Camera with SD Card");
            ESP_LOGI(TAG, "Custom commands registered: 'login', 'sd', 'init_rtc', 'settime', 'gettime', 'on', 'off'");

            esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
            ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

            int login_attempts = 0;
            // ESP_ERROR_CHECK(esp_console_start_repl(repl));

            ret = esp_console_start_repl(repl);

            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to start console REPL: %s", esp_err_to_name(ret));
                state = SYSTEM_STATE_NIGHT_SLEEP;
                break;
            }

            while (gpio_get_level(USB_INT) && login_attempts < 30 && login_state != LOGIN_TO_FLASH_DRIVE && running_mode == 0)
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
                if (login_state == LOGIN_NOT_ATEMPTED)
                {
                    login_attempts++;
                }
            }

            if (login_state == LOGIN_TO_FLASH_DRIVE)
            {
                ESP_LOGI(TAG, "Camera goes to flash drive mode");
                state = SYSTEM_STATE_FLASH_DRIVE;
                break;
            }
            else
            {
                ESP_LOGE(TAG, "Login time out, closing comunication, to attempt login again unplug USB from camera, wait a minute and plug again");
                if(login_attempts >= 30)
                {
                    login_state = LOGIN_TIME_OUT;
                }
                login_attempts = 0;
                esp_console_deinit();
            }

            if(running_mode == 1)
            {
                state = SYSTEM_STATE_DAY_SLEEP;
                break;
            }

            if(login_state == LOGIN_TIME_OUT)
            {
                state = SYSTEM_STATE_NIGHT_SLEEP;
                break;
            }

            if(onoff_mode == 1)
            {
                ESP_LOGI(TAG, "Camera turning off command received");
                state = SYSTEM_STATE_OFF;
                break;
            }
            else{
                ESP_LOGI(TAG, "Camera turning on command received");
                state = SYSTEM_STATE_INIT;
            }


            break;
        case SYSTEM_STATE_FLASH_DRIVE:

            if (!sd_card_mounted)
            {
                ret = init_sd_card();
                if (ret != ESP_OK)
                {
                    ESP_LOGE(TAG, "Failed to initialize SD card");
                    login_state = LOGIN_NOT_ATEMPTED;
                    state = SYSTEM_STATE_CLEANUP;
                    break;
                }
            }

            // Initialize USB MSC
            ret = init_usb_msc();
            if (ret != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to initialize USB MSC");
                login_state = LOGIN_NOT_ATEMPTED;
                state = SYSTEM_STATE_CLEANUP;
                break;
            }

            while (gpio_get_level(USB_INT))
            {
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            state = SYSTEM_STATE_CLEANUP;
            login_state = LOGIN_NOT_ATEMPTED;
            break;
        }
    }
}