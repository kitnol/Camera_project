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

#include "esp_partition.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"

#include "ds3231.h"

static const char *TAG = "camera_sd";
#define PROMPT_STR CONFIG_IDF_TARGET
#define VREG_ADDR 0x36

// BMP header structures
typedef struct __attribute__((packed))
{
    uint16_t type; // BM
    uint32_t size; // File size
    uint16_t reserved1;
    uint16_t reserved2;
    uint32_t offset; // Offset to image data
} bmp_file_header_t;

typedef struct __attribute__((packed))
{
    uint32_t size; // Header size
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

#define PIN_NUM_MISO    SD_PIN_D0  // D0
#define PIN_NUM_MOSI    SD_PIN_CMD  // CMD
#define PIN_NUM_CLK     SD_PIN_SCK  // CLK
#define PIN_NUM_CS      SD_PIN_D3  // D3 (not used in 4-line mode but defined)
#define PIN_NUM_D1      SD_PIN_D1  // D1
#define PIN_NUM_D2      SD_PIN_D2  // D2

// SD card pin configuration (SPI mode)
#define PIN_NUM_MISO SD_PIN_D0
#define PIN_NUM_MOSI SD_PIN_CMD
#define PIN_NUM_CLK SD_PIN_SCK
#define PIN_NUM_CS SD_PIN_D3

#define LED_PIN 15
#define USB_INT 21

#define MOUNT_POINT "/sdcard"
sdmmc_card_t *card_handle;
static bool sd_card_mounted = false;

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

sensor_t *sens;
camera_fb_t *fbsave;

RTC_DATA_ATTR int image_count = 0;
RTC_DATA_ATTR int time_set = 0;
login_state_t login_state = LOGIN_NOT_ATEMPTED;
#define CORRECT_PASSWORD "hello"

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

    .xclk_freq_hz = 7000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_5MP,       // Reduced from UXGA (800x600 instead of 1600x1200)
    .jpeg_quality = 10,                // Lower quality to reduce memory usage
    .fb_count = 1,                     // Single frame buffer without PSRAM
    .fb_location = CAMERA_FB_IN_PSRAM, // Use DRAM instead of PSRAM
    .grab_mode = CAMERA_GRAB_WHEN_EMPTY,
};

esp_err_t init_camera(void)
{
    ESP_LOGI(TAG, "Initializing camera");

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
    s->set_brightness(s, 1);                 // -2 to 2
    s->set_contrast(s, 0);                   // -2 to 2
    s->set_saturation(s, 0);                 // -2 to 2
    s->set_special_effect(s, 0);             // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia)
    s->set_whitebal(s, 1);                   // 0 = disable , 1 = enable
    s->set_awb_gain(s, 1);                   // 0 = disable , 1 = enable
    s->set_wb_mode(s, 0);                    // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)
    s->set_exposure_ctrl(s, 1);              // 0 = disable , 1 = enable
    s->set_aec2(s, 1);                       // 0 = disable , 1 = enable
    s->set_ae_level(s, 0);                   // -2 to 2
    s->set_aec_value(s, 300);                // 0 to 1200
    s->set_gain_ctrl(s, 1);                  // 0 = disable , 1 = enable
    s->set_agc_gain(s, 0);                   // 0 to 30
    s->set_gainceiling(s, (gainceiling_t)4); // 0 to 6
    s->set_bpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_wpc(s, 1);                        // 0 = disable , 1 = enable
    s->set_raw_gma(s, 1);                    // 0 = disable , 1 = enable
    s->set_lenc(s, 1);                       // 0 = disable , 1 = enable
    s->set_hmirror(s, 0);                    // 0 = disable , 1 = enable
    s->set_vflip(s, 0);                      // 0 = disable , 1 = enable
    s->set_dcw(s, 1);                        // 0 = disable , 1 = enable
    s->set_colorbar(s, 0);                   // 0 = disable , 1 = enable

    ESP_LOGI(TAG, "Camera initialized successfully");

    sens = s;

    return ESP_OK;
}

esp_err_t deinit_camera(void)
{
    ESP_LOGI(TAG, "Deinitializing camera");

    // Perform sensor software reset before deinitializing
    if (sens != NULL)
    {
        ESP_LOGI(TAG, "Performing sensor reset");

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

    ESP_LOGI(TAG, "Camera deinitialized successfully");

    return ESP_OK;
}

/* SD card function */
static esp_err_t init_sd_card(void)
{
    esp_err_t ret;
    
    // Options for mounting the filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    
    ESP_LOGI(TAG, "Initializing SD card");
    
    // Initialize SD card host
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_52M;  // Use high speed
    
    // Initialize SD card slot configuration
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    slot_config.width = 4;  // 4-line mode
    slot_config.clk = PIN_NUM_CLK;
    slot_config.cmd = PIN_NUM_MOSI;
    slot_config.d0 = PIN_NUM_MISO;
    slot_config.d1 = PIN_NUM_D1;
    slot_config.d2 = PIN_NUM_D2;
    slot_config.d3 = PIN_NUM_CS;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;
    
    // Mount filesystem
    ret = esp_vfs_fat_sdmmc_mount(MOUNT_POINT, &host, &slot_config, &mount_config, &card_handle);
    
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem. "
                     "If you want the card to be formatted, set format_if_mount_failed = true.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). "
                     "Make sure SD card lines have pull-up resistors in place.", esp_err_to_name(ret));
        }
        return ret;
    }
    
    ESP_LOGI(TAG, "SD card mounted successfully");
    
    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, card_handle);
    
    sd_card_mounted = true;
    return ESP_OK;
}

esp_err_t save_image_to_sd(camera_fb_t *fb, const char *filename)
{
    ESP_LOGI(TAG, "Saving image to SD card: %s", filename);

    char filepath[64];
    snprintf(filepath, sizeof(filepath), MOUNT_POINT "/%s", filename);

    FILE *file = fopen(filepath, "wb");
    if (file == NULL)
    {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    if (fb == NULL)
    {
        ESP_LOGE(TAG, "Camera capture failed");
        return ESP_FAIL;
    }

    size_t written = fwrite(fb->buf, 1, fb->len, file);
    fclose(file);

    if (written != fb->len)
    {
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

    // Deinitialize SPI bus
    ret = spi_bus_free(SDSPI_DEFAULT_HOST);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to free SPI bus: %s", esp_err_to_name(ret));
    }

    return ret;
}

/* RTC functions */

/* Storage callbacks for TinyUSB MSC*/
static int32_t msc_read_cb(uint32_t lba, uint32_t offset, void* buffer, uint32_t bufsize)
{
    ESP_LOGD(TAG, "MSC Read - LBA: %lu, Offset: %lu, Size: %lu", lba, offset, bufsize);
    
    if (!sd_card_mounted || !card_handle) {
        ESP_LOGE(TAG, "SD card not mounted");
        return -1;
    }
    
    esp_err_t ret = sdmmc_read_sectors(card_handle, buffer, lba + (offset / 512), bufsize / 512);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read sectors: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return bufsize;
}

static int32_t msc_write_cb(uint32_t lba, uint32_t offset, uint8_t* buffer, uint32_t bufsize)
{
    ESP_LOGD(TAG, "MSC Write - LBA: %lu, Offset: %lu, Size: %lu", lba, offset, bufsize);
    
    if (!sd_card_mounted || !card_handle) {
        ESP_LOGE(TAG, "SD card not mounted");
        return -1;
    }
    
    esp_err_t ret = sdmmc_write_sectors(card_handle, buffer, lba + (offset / 512), bufsize / 512);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to write sectors: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return bufsize;
}

static esp_err_t init_usb_msc(void)
{
    ESP_LOGI(TAG, "Initializing USB MSC");
    
    if (!sd_card_mounted || !card_handle) {
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
    
    if(strcmp(pass, password) == 0){
        printf("Login succesful!\n");
        login_state = LOGIN_SUCCESSFUL;
    }
    else{
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

/* Turining camera into sd card */
static int sd_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: sd\n");
        return 1;
    }
    if(login_state == LOGIN_SUCCESSFUL){
        login_state = LOGIN_TO_FLASH_DRIVE;
        printf("Initialising flash drive");
        printf("\n");
    }
    else{
        printf("You are not logged in\n");
    }

    return 0;
}

static void register_sd_command(void)
{
    const esp_console_cmd_t sd_cmd = {
        .command = "sd",
        .help = "Starting MSC mode",
        .hint = NULL,
        .func = &sd_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&sd_cmd));
}

/* RTC temperature */
static int temp_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: temp\n");
        return 1;
    }
    if(login_state == LOGIN_SUCCESSFUL){
        i2c_dev_t dev;
        ds3231_init_desc(&dev, 1, 17, 18);
        int16_t temp;
        esp_err_t res = ds3231_get_raw_temp(&dev, &temp);
        if(res == ESP_OK){
            printf("Raw temperature: %d (0.25C units)\n", temp);
            float temp_c = temp * 0.25;
            printf("Temperature: %.2f C\n", temp_c);
        }
        else{
            printf("Failed to read temperature: %s\n", esp_err_to_name(res));
        }
    }
    else{
        printf("You are not logged in\n");
    }

    return 0;
}

static void register_temp_command(void)
{
    const esp_console_cmd_t temp_cmd = {
        .command = "temp",
        .help = "Getting RTC temperature",
        .hint = NULL,
        .func = &temp_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&temp_cmd));
}



void app_main(void)
{   

    system_state_t state = SYSTEM_STATE_INIT;
    esp_err_t ret = ESP_OK;
    char filename[32];
    camera_fb_t *fb = NULL;

    while (1)
    {
        switch (state)
        {
        case SYSTEM_STATE_INIT:
            gpio_set_direction(USB_INT, GPIO_MODE_INPUT);
            if (gpio_get_level(USB_INT) == 0)
            {
                login_state = LOGIN_NOT_ATEMPTED;
            }
            if (gpio_get_level(USB_INT) == 1 && login_state != LOGIN_TIME_OUT)
            {
                ESP_LOGE(TAG, "+ USB connected");
                state = SYSTEM_STATE_USB_CONNECTED;
                gpio_reset_pin(CAM_PWR);
                gpio_hold_dis(CAM_PWR);
                gpio_deep_sleep_hold_dis();
                gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
                gpio_set_level(CAM_PWR, 1);
                break;
            }
            else
            {
                ESP_LOGE(TAG, "- USB disconnected");
            }

            gpio_reset_pin(CAM_PWR);
            gpio_hold_dis(CAM_PWR);
            gpio_deep_sleep_hold_dis();
            gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
            gpio_set_level(CAM_PWR, 0);

            if ((ret = init_sd_card()) != ESP_OK)
            {
                ESP_LOGE(TAG, "SD card init failed");
                state = SYSTEM_STATE_CLEANUP;
                break;
            }

            if ((ret = init_camera()) != ESP_OK)
            {
                ESP_LOGE(TAG, "Camera init failed");
                state = SYSTEM_STATE_CLEANUP;
                break;
            }
            state = SYSTEM_STATE_CAPTURE;
            break;

        case SYSTEM_STATE_CAPTURE:
            fb = esp_camera_fb_get();
            if (!fb)
            {
                ESP_LOGE(TAG, "Camera capture failed");
                state = SYSTEM_STATE_CLEANUP;
                break;
            }
            ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);
            state = SYSTEM_STATE_SAVE;
            break;

        case SYSTEM_STATE_SAVE:
            snprintf(filename, sizeof(filename), "image%03d.jpg", image_count++);
            if ((ret = save_image_to_sd(fb, filename)) != ESP_OK)
            {
                ESP_LOGE(TAG, "Failed to save image");
            }
            esp_camera_fb_return(fb);
            fb = NULL;
            state = SYSTEM_STATE_CLEANUP;
            break;

        case SYSTEM_STATE_CLEANUP:
            if ((ret = deinit_camera()) != ESP_OK)
            {
                ESP_LOGE(TAG, "Camera deinit failed");
            }
            if ((ret = deinit_sdcard()) != ESP_OK)
            {
                ESP_LOGE(TAG, "SD card deinit failed");
            }
            state = SYSTEM_STATE_SLEEP;
            break;

        case SYSTEM_STATE_SLEEP:
            gpio_set_level(CAM_PWR, 1);
            gpio_hold_en(CAM_PWR);
            gpio_deep_sleep_hold_en();
            vTaskDelay(pdMS_TO_TICKS(50));

            ESP_LOGI(TAG, "Entering deep sleep");
            esp_sleep_enable_timer_wakeup(5 * 1000 * 1000);
            
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

            ESP_LOGI(TAG, "Starting ESP32-S3 Camera with SD Card");
            ESP_LOGI(TAG, "Custom commands registered: 'login', 'sdcon'");

            esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
            ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

            int login_attempts = 0;
            ESP_ERROR_CHECK(esp_console_start_repl(repl));
            while (gpio_get_level(USB_INT) && login_attempts < 30 && login_state != LOGIN_TO_FLASH_DRIVE){
                vTaskDelay(pdMS_TO_TICKS(1000));
                if(login_state == LOGIN_NOT_ATEMPTED)
                {
                    login_attempts++;
                }
            }

            if(login_state == LOGIN_TO_FLASH_DRIVE)
            {
                ESP_LOGI(TAG, "Camera goes to flash drive mode");
                state = SYSTEM_STATE_FLASH_DRIVE;
                break;
            }
            else{
                ESP_LOGE(TAG, "Login time out, closing comunication, to attempt login again unplug USB from camera, wait a minute and plug again");
                login_state = LOGIN_TIME_OUT;
                login_attempts = 0;
                esp_console_deinit();
                state = SYSTEM_STATE_INIT;
                break;
            }

            
            state = SYSTEM_STATE_INIT;
            break;
        case SYSTEM_STATE_FLASH_DRIVE:
            if(!sd_card_mounted)
            {
                ret = init_sd_card();
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "Failed to initialize SD card");
                    login_state = LOGIN_NOT_ATEMPTED;
                    state = SYSTEM_STATE_CLEANUP;
                    break;
                }
            }
            
            // Initialize USB MSC
            ret = init_usb_msc();
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to initialize USB MSC");
                login_state = LOGIN_NOT_ATEMPTED;
                state = SYSTEM_STATE_CLEANUP;
                break;
            }

            while (gpio_get_level(USB_INT)){
                vTaskDelay(pdMS_TO_TICKS(1000));
            }

            state = SYSTEM_STATE_CLEANUP;
            login_state = LOGIN_NOT_ATEMPTED;
            break;
        }
    }
}