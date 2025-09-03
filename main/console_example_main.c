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

// SD card pin configuration (SPI mode)
#define PIN_NUM_MISO    11
#define PIN_NUM_MOSI    13
#define PIN_NUM_CLK     12
#define PIN_NUM_CS      14
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

    .xclk_freq_hz = 5000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,

    .pixel_format = PIXFORMAT_JPEG,
    .frame_size = FRAMESIZE_VGA,      // Reduced from UXGA (800x600 instead of 1600x1200)
    .jpeg_quality = 10,                // Lower quality to reduce memory usage
    .fb_count = 1,                     // Single frame buffer without PSRAM
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
    s->set_brightness(s, 0);     // -2 to 2
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

void rgb_to_bgr(uint8_t *rgb_data, uint8_t *bgr_data, int width, int height)
{
    // Convert RGB to BGR format (BMP uses BGR order)
    for (int i = 0; i < width * height; i++) {
        bgr_data[i * 3 + 0] = rgb_data[i * 3 + 2]; // B
        bgr_data[i * 3 + 1] = rgb_data[i * 3 + 1]; // G
        bgr_data[i * 3 + 2] = rgb_data[i * 3 + 0]; // R
    }
}

esp_err_t save_bmp_to_sd(camera_fb_t *fb, const char *filename)
{
    FILE *file = fopen(filename, "wb");
    if (file == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filename);
        return ESP_FAIL;
    }
    
    int width = 0, height = 0;
    
    // Get image dimensions based on frame size
    switch (camera_config.frame_size) {
        case FRAMESIZE_QQVGA:    width = 160;  height = 120;  break;
        case FRAMESIZE_QCIF:     width = 176;  height = 144;  break;
        case FRAMESIZE_HQVGA:    width = 240;  height = 176;  break;
        case FRAMESIZE_240X240:  width = 240;  height = 240;  break;
        case FRAMESIZE_QVGA:     width = 320;  height = 240;  break;
        case FRAMESIZE_CIF:      width = 400;  height = 296;  break;
        case FRAMESIZE_HVGA:     width = 480;  height = 320;  break;
        case FRAMESIZE_VGA:      width = 640;  height = 480;  break;
        case FRAMESIZE_SVGA:     width = 800;  height = 600;  break;
        case FRAMESIZE_XGA:      width = 1024; height = 768;  break;
        case FRAMESIZE_HD:       width = 1280; height = 720;  break;
        case FRAMESIZE_SXGA:     width = 1280; height = 1024; break;
        case FRAMESIZE_UXGA:     width = 1600; height = 1200; break;
        default:                 width = 800;  height = 600;  break;
    }
    
    // Convert RGB to BGR (BMP format requirement)
    uint8_t *bgr_data = malloc(width * height * 3);
    if (bgr_data == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory for BGR conversion");
        fclose(file);
        return ESP_FAIL;
    }
    
    rgb_to_bgr(fb->buf, bgr_data, width, height);
    
    // Create BMP file header
    
    // Calculate padding for 4-byte alignment
    int row_padding = (4 - ((width * 3) % 4)) % 4;
    uint32_t image_size = (width * 3 + row_padding) * height;
    uint32_t file_size = sizeof(bmp_file_header_t) + sizeof(bmp_info_header_t) + image_size;
    
    bmp_file_header_t file_header = {
        .type = 0x4D42,  // 'BM'
        .size = file_size,
        .reserved1 = 0,
        .reserved2 = 0,
        .offset = sizeof(bmp_file_header_t) + sizeof(bmp_info_header_t)
    };
    
    // Create BMP info header
    bmp_info_header_t info_header = {
        .size = sizeof(bmp_info_header_t),
        .width = width,
        .height = height,
        .planes = 1,
        .bits_per_pixel = 24,
        .compression = 0,
        .image_size = image_size,
        .x_pixels_per_meter = 2835,  // 72 DPI
        .y_pixels_per_meter = 2835,  // 72 DPI
        .colors_used = 0,
        .colors_important = 0
    };
    
    // Write headers
    fwrite(&file_header, sizeof(bmp_file_header_t), 1, file);
    fwrite(&info_header, sizeof(bmp_info_header_t), 1, file);
    
    // Write image data (BMP is stored bottom-to-top)
    uint8_t padding[3] = {0, 0, 0};
    for (int y = height - 1; y >= 0; y--) {
        fwrite(&bgr_data[y * width * 3], width * 3, 1, file);
        if (row_padding > 0) {
            fwrite(padding, row_padding, 1, file);
        }
    }
    
    free(bgr_data);
    fclose(file);
    
    ESP_LOGI(TAG, "BMP file saved: %s (%dx%d)", filename, width, height);
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

static struct {
    struct arg_str *message;
    struct arg_int *count;
    struct arg_end *end;
} print_args;

static int print_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &print_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, print_args.end, argv[0]);
        return 1;
    }

    const char* message = print_args.message->sval[0];
    int count = 1;
    
    if (print_args.count->count > 0) {
        count = print_args.count->ival[0];
        if (count < 1) {
            printf("Count must be at least 1\n");
            return 1;
        }
        if (count > 100) {
            printf("Count cannot exceed 100\n");
            return 1;
        }
    }

    printf("Printing message %d time(s):\n", count);
    for (int i = 0; i < count; i++) {
        printf("[%d] %s\n", i + 1, message);
    }

    return 0;
}

static void register_print_command(void)
{
    print_args.message = arg_str1(NULL, NULL, "<message>", "Message to print");
    print_args.count = arg_int0("c", "count", "<n>", "Number of times to print (default: 1, max: 100)");
    print_args.end = arg_end(2);

    const esp_console_cmd_t print_cmd = {
        .command = "print",
        .help = "Print a message with optional repeat count",
        .hint = NULL,
        .func = &print_command_handler,
        .argtable = &print_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&print_cmd));
}

/* Math command for float calculations */
static struct {
    struct arg_dbl *num1;
    struct arg_dbl *num2;
    struct arg_str *operation;
    struct arg_end *end;
} math_args;

static int math_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &math_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, math_args.end, argv[0]);
        return 1;
    }

    double num1 = math_args.num1->dval[0];
    double num2 = math_args.num2->dval[0];
    const char* op = math_args.operation->sval[0];
    double result = 0.0;
    bool valid_op = true;

    printf("Input values:\n");
    printf("  Number 1: %.6f (as float: %.6f)\n", num1, (float)num1);
    printf("  Number 2: %.6f (as float: %.6f)\n", num2, (float)num2);
    printf("  Operation: %s\n", op);

    if (strcmp(op, "add") == 0 || strcmp(op, "+") == 0) {
        result = num1 + num2;
        printf("Result (double): %.6f + %.6f = %.6f\n", num1, num2, result);
        printf("Result (float):  %.6f + %.6f = %.6f\n", (float)num1, (float)num2, (float)result);
    } else if (strcmp(op, "sub") == 0 || strcmp(op, "-") == 0) {
        result = num1 - num2;
        printf("Result (double): %.6f - %.6f = %.6f\n", num1, num2, result);
        printf("Result (float):  %.6f - %.6f = %.6f\n", (float)num1, (float)num2, (float)result);
    } else if (strcmp(op, "mul") == 0 || strcmp(op, "*") == 0) {
        result = num1 * num2;
        printf("Result (double): %.6f * %.6f = %.6f\n", num1, num2, result);
        printf("Result (float):  %.6f * %.6f = %.6f\n", (float)num1, (float)num2, (float)result);
    } else if (strcmp(op, "div") == 0 || strcmp(op, "/") == 0) {
        if (num2 == 0.0) {
            printf("Error: Division by zero!\n");
            return 1;
        }
        result = num1 / num2;
        printf("Result (double): %.6f / %.6f = %.6f\n", num1, num2, result);
        printf("Result (float):  %.6f / %.6f = %.6f\n", (float)num1, (float)num2, (float)result);
    } else {
        printf("Error: Unknown operation '%s'. Use: add, sub, mul, div (or +, -, *, /)\n", op);
        valid_op = false;
    }

    if (valid_op) {
        printf("\nPrecision comparison:\n");
        printf("  Double precision: %.15f\n", result);
        printf("  Float precision:  %.15f\n", (double)(float)result);
        printf("  Difference:       %.15e\n", result - (double)(float)result);
    }

    return 0;
}

static void register_math_command(void)
{
    math_args.num1 = arg_dbl1(NULL, NULL, "<num1>", "First number (double/float)");
    math_args.num2 = arg_dbl1(NULL, NULL, "<num2>", "Second number (double/float)");
    math_args.operation = arg_str1(NULL, NULL, "<op>", "Operation: add/+, sub/-, mul/*, div//");
    math_args.end = arg_end(3);

    const esp_console_cmd_t math_cmd = {
        .command = "math",
        .help = "Perform math operations on floating point numbers",
        .hint = NULL,
        .func = &math_command_handler,
        .argtable = &math_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&math_cmd));
}

/* Echo command - simpler version that just echoes all arguments */
static int echo_command_handler(int argc, char **argv)
{
    if (argc < 2) {
        printf("Usage: echo <message>\n");
        return 1;
    }

    printf("Echo: ");
    for (int i = 1; i < argc; i++) {
        printf("%s", argv[i]);
        if (i < argc - 1) {
            printf(" ");
        }
    }
    printf("\n");

    return 0;
}

static void register_echo_command(void)
{
    const esp_console_cmd_t echo_cmd = {
        .command = "echo",
        .help = "Echo the given message",
        .hint = NULL,
        .func = &echo_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&echo_cmd));
}

/* Take BMP pictures command handler + */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} takebmp_args;


static int takebmp_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &takebmp_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, takebmp_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = takebmp_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);

    if (taken != false)
    {
        esp_camera_fb_return(fbsave);
    }
    else{
        taken = true;
    }

    char filename[32];
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    
    int len = sizeof(fb);
    // printf(fb->buf);
    // for(int i = 0; i < len; i++)
    // {
    //     printf("%d\n", fb->buf[i]);
    // }

    ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);

    snprintf(filename, sizeof(filename), "image%03d.bmp", val);

    esp_err_t ret = save_bmp_to_sd(fb, filename);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save image");
    }

    fbsave = fb;

    return 0;
}

static void register_takebmp_command(void)
{
    takebmp_args.val = arg_str1(NULL, NULL, "<num>", "take value");
    takebmp_args.end = arg_end(3);

    const esp_console_cmd_t takebmp_cmd = {
        .command = "takebmp",
        .help = "Take picture",
        .hint = NULL,
        .func = &takebmp_command_handler,
        .argtable = &takebmp_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&takebmp_cmd));
}


/* Take pictures command handler + */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} take_args;


static int take_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &take_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, take_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = take_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);

    if (taken != false)
    {
        esp_camera_fb_return(fbsave);
    }
    else{
        taken = true;
    }

    char filename[32];
    camera_fb_t *fb = esp_camera_fb_get();
    if (fb == NULL) {
        ESP_LOGE(TAG, "Camera capture failed 2");
        return 0;
    }
    if (!fb) {
        ESP_LOGE(TAG, "Camera capture failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        return 0;
    }
    
    int len = sizeof(fb);
    // printf(fb->buf);
    // for(int i = 0; i < len; i++)
    // {
    //     printf("%d\n", fb->buf[i]);
    // }

    ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);

    snprintf(filename, sizeof(filename), "image%03d.jpg", val);

    esp_err_t ret = save_image_to_sd(fb, filename);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to save image");
    }

    fbsave = fb;

    return 0;
}

static void register_take_command(void)
{
    take_args.val = arg_str1(NULL, NULL, "<num>", "take value");
    take_args.end = arg_end(3);

    const esp_console_cmd_t take_cmd = {
        .command = "take",
        .help = "Take picture",
        .hint = NULL,
        .func = &take_command_handler,
        .argtable = &take_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&take_cmd));
}

/* Camera init command + */
static int cinit_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: cinit\n");
        return 1;
    }

    esp_err_t ret = init_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed");
        return 1;
    }

    printf("Camera initialised");
    printf("\n");

    return 0;
}

static void register_cinit_command(void)
{
    const esp_console_cmd_t cinit_cmd = {
        .command = "cinit",
        .help = "Camera initialising command",
        .hint = NULL,
        .func = &cinit_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cinit_cmd));
}

/* Camera deinit command + */
static int cdeinit_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: cdeinit\n");
        return 1;
    }

    esp_err_t ret = deinit_camera();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Camera deinitialization failed");
        return 1;
    }

    printf("Camera deinitialised");
    printf("\n");

    return 0;
}

static void register_cdeinit_command(void)
{
    const esp_console_cmd_t cdeinit_cmd = {
        .command = "cdeinit",
        .help = "Camera deinitialising command",
        .hint = NULL,
        .func = &cdeinit_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&cdeinit_cmd));
}

/* Camera power up + */
static int pwrup_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: pwrup\n");
        return 1;
    }

    gpio_reset_pin(CAM_PWR);
    gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PWR, 0);
    printf("Camera power up");
    printf("\n");

    return 0;
}

static void register_pwrup_command(void)
{
    const esp_console_cmd_t pwrup_cmd = {
        .command = "pwrup",
        .help = "Camera power up command",
        .hint = NULL,
        .func = &pwrup_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&pwrup_cmd));
}

/* Camera power down + */
static int pwrdown_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: pwrdown\n");
        return 1;
    }

    gpio_reset_pin(CAM_PWR);
    gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PWR, 1);
    printf("Camera power down");
    printf("\n");

    return 0;
}

static void register_pwrdown_command(void)
{
    const esp_console_cmd_t pwrdown_cmd = {
        .command = "pwrdown",
        .help = "Camera power down command",
        .hint = NULL,
        .func = &pwrdown_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&pwrdown_cmd));
}

/* SD init command + */
static int sdinit_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: sdinit\n");
        return 1;
    }

    esp_err_t ret = init_sdcard();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card initialization failed");
        return 1;
    }

    printf("SD card initialised");
    printf("\n");

    return 0;
}

static void register_sdinit_command(void)
{
    const esp_console_cmd_t sdinit_cmd = {
        .command = "sdinit",
        .help = "SD card initialising command",
        .hint = NULL,
        .func = &sdinit_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&sdinit_cmd));
}

/* SD deinit command + */
static int sddeinit_command_handler(int argc, char **argv)
{
    if (argc < 1) {
        printf("Usage: sddeinit\n");
        return 1;
    }

    esp_err_t ret = deinit_sdcard();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD card initialization failed");
        return 1;
    }

    printf("SD card deinitialised");
    printf("\n");

    return 0;
}

static void register_sddeinit_command(void)
{
    const esp_console_cmd_t sddeinit_cmd = {
        .command = "sddeinit",
        .help = "SD card deinitialising command",
        .hint = NULL,
        .func = &sddeinit_command_handler,
        .argtable = NULL
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&sddeinit_cmd));
}

/* AE command + */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} ae_args;

static int ae_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &ae_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, ae_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = ae_args.val->sval[0];
    char *endptr;
    double val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("AE value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }

    if(val > 2.0 || val < -2.0)
    {
        printf("Incorrect argument\n AE value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }
    else{
        sens->set_ae_level(sens, (float)val);
        printf("AE level set to %.3f\n", (float)val);
    }

    return 0;
}

static void register_ae_command(void)
{
    ae_args.val = arg_str1(NULL, NULL, "<num>", "AE value (default: 0, limit: <-2; 2>)");
    ae_args.end = arg_end(3);

    const esp_console_cmd_t ae_cmd = {
        .command = "ae",
        .help = "Set auto exposure value",
        .hint = NULL,
        .func = &ae_command_handler,
        .argtable = &ae_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&ae_cmd));
}

/* AE command + */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} aec_args;

static int aec_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &aec_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, aec_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = aec_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("AEC value (default: 300, limit: <0; 1200>)\n");
        return 1;
    }

    if(val > 2.0 || val < -2.0)
    {
        printf("Incorrect argument\n AEC value (default: 300, limit: <0; 1200>)\n");
        return 1;
    }
    else{
        sens->set_aec_value(sens, val);
        printf("AEC level set to %d\n", val);
    }

    return 0;
}

static void register_aec_command(void)
{
    aec_args.val = arg_str1(NULL, NULL, "<num>", "AEC value (default: 300, limit: <0; 1200>)");
    aec_args.end = arg_end(3);

    const esp_console_cmd_t aec_cmd = {
        .command = "aec",
        .help = "Set auto exposure value",
        .hint = NULL,
        .func = &aec_command_handler,
        .argtable = &aec_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&aec_cmd));
}


/* Brightness command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} br_args;

static int br_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &br_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, br_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = br_args.val->sval[0];
    char *endptr;
    double val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("Brightness value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }

    if(val > 2.0 || val < -2.0)
    {
        printf("Incorrect argument\n Brightness value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }
    else{
        sens->set_brightness(sens, (float)val);
        printf("Brightness level set to %.3f\n", (float)val);
    }

    return 0;
}

static void register_br_command(void)
{
    br_args.val = arg_str1(NULL, NULL, "<num>", "Brightness value (default: 0, limit: <-2; 2>)");
    br_args.end = arg_end(3);

    const esp_console_cmd_t br_cmd = {
        .command = "br",
        .help = "Set brightness value",
        .hint = NULL,
        .func = &br_command_handler,
        .argtable = &br_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&br_cmd));
}

/* Contrast command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} con_args;

static int con_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &con_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, con_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = con_args.val->sval[0];
    char *endptr;
    double val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("Contrast value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }

    if(val > 2.0 || val < -2.0)
    {
        printf("Incorrect argument\n Contrast value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }
    else{
        sens->set_contrast(sens, (float)val);
        printf("Contrast level set to %.3f\n", (float)val);
    }

    return 0;
}

static void register_con_command(void)
{
    con_args.val = arg_str1(NULL, NULL, "<num>", "Contrast value (default: 0, limit: <-2; 2>)");
    con_args.end = arg_end(3);

    const esp_console_cmd_t con_cmd = {
        .command = "con",
        .help = "Set contrast value",
        .hint = NULL,
        .func = &con_command_handler,
        .argtable = &con_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&con_cmd));
}

/* Saturation command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} sat_args;

static int sat_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &sat_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, sat_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = sat_args.val->sval[0];
    char *endptr;
    double val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("Saturation value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }

    if(val > 2.0 || val < -2.0)
    {
        printf("Incorrect argument\n Saturation value (default: 0, limit: <-2; 2>)\n");
        return 1;
    }
    else{
        sens->set_saturation(sens, (float)val);
        printf("Saturation level set to %.3f\n", (float)val);
    }

    return 0;
}

static void register_sat_command(void)
{
    sat_args.val = arg_str1(NULL, NULL, "<num>", "Saturation value (default: 0, limit: <-2; 2>)");
    sat_args.end = arg_end(3);

    const esp_console_cmd_t sat_cmd = {
        .command = "sat",
        .help = "Set saturation value",
        .hint = NULL,
        .func = &sat_command_handler,
        .argtable = &sat_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&sat_cmd));
}

/* AEV command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} aev_args;

static int aev_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &aev_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, aev_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = aev_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("AEV value (default: 300, limit: <0; 1200>)\n");
        return 1;
    }

    if(val > 1200 || val < 0)
    {
        printf("Incorrect argument\n AEV value (default: 300, limit: <0; 1200>)\n");
        return 1;
    }
    else{
        sens->set_aec_value(sens, (int)val);
        printf("AEV level set to %.3f\n", (float)val);
    }

    return 0;
}

static void register_aev_command(void)
{
    aev_args.val = arg_str1(NULL, NULL, "<num>", "AEV value (default: 0, limit: <0; 30>)");
    aev_args.end = arg_end(3);

    const esp_console_cmd_t aev_cmd = {
        .command = "aev",
        .help = "Set aev value",
        .hint = NULL,
        .func = &aev_command_handler,
        .argtable = &aev_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&aev_cmd));
}

/* Auto gain command*/
static struct {
    struct arg_str *val;
    struct arg_end *end;
} ag_args;

static int ag_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &ag_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, ag_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = ag_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("Auto gain value (default: 0, limit: <0; 30>)\n");
        return 1;
    }

    if(val > 30 || val < 0)
    {
        printf("Incorrect argument\n Auto gain value (default: 0, limit: <0; 30>)\n");
        return 1;
    }
    else{
        sens->set_agc_gain(sens, (int)val);
        printf("Auto gain level set to %.3f\n", (float)val);
    }

    return 0;
}

static void register_ag_command(void)
{
    ag_args.val = arg_str1(NULL, NULL, "<num>", "Auto gain value (default: 0, limit: <0; 30>)");
    ag_args.end = arg_end(3);

    const esp_console_cmd_t ag_cmd = {
        .command = "ag",
        .help = "Set ag value",
        .hint = NULL,
        .func = &ag_command_handler,
        .argtable = &ag_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&ag_cmd));
}

/* Balck pixel correction command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} bpc_args;

static int bpc_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &bpc_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, bpc_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = bpc_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("Black pixel correction (default: 1, 0 - disable, 1 - enable)\n");
        return 1;
    }

    if(val == 0)
    {
        sens->set_bpc(sens, 0);
        printf("Black pixel correction: disable\n");
    }
    else if(val == 1){
        sens->set_bpc(sens, 1);
        printf("Black pixel correction: enable\n");
    }
    else{
        printf("Incorrect argument\n Black pixel correction (default: 1, 0 - disable, 1 - enable)\n");
        return 1;
    }

    return 0;
}

static void register_bpc_command(void)
{
    bpc_args.val = arg_str1(NULL, NULL, "<num>", "Black pixel correction (default: 1, 0 - disable, 1 - enable)");
    bpc_args.end = arg_end(3);

    const esp_console_cmd_t bpc_cmd = {
        .command = "bpc",
        .help = "Set bpc value",
        .hint = NULL,
        .func = &bpc_command_handler,
        .argtable = &bpc_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&bpc_cmd));
}

/* White pixel correction command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} wpc_args;

static int wpc_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &wpc_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, wpc_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = wpc_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("White pixel correction (default: 1, 0 - disable, 1 - enable)\n");
        return 1;
    }

    if(val == 0)
    {
        sens->set_wpc(sens, 0);
        printf("White pixel correction: disable\n");
    }
    else if(val == 1){
        sens->set_wpc(sens, 1);
        printf("White pixel correction: enable\n");
    }
    else{
        printf("Incorrect argument\n White pixel correction (default: 1, 0 - disable, 1 - enable)\n");
        return 1;
    }

    return 0;
}

static void register_wpc_command(void)
{
    wpc_args.val = arg_str1(NULL, NULL, "<num>", "White pixel correction (default: 1, 0 - disable, 1 - enable)");
    wpc_args.end = arg_end(3);

    const esp_console_cmd_t wpc_cmd = {
        .command = "wpc",
        .help = "Set wpc value",
        .hint = NULL,
        .func = &wpc_command_handler,
        .argtable = &wpc_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&wpc_cmd));
}

/* DCW correction command */
static struct {
    struct arg_str *val;
    struct arg_end *end;
} dcw_args;

static int dcw_command_handler(int argc, char **argv)
{
    int nerrors = arg_parse(argc, argv, (void **) &dcw_args);
    if (nerrors != 0) {
        arg_print_errors(stderr, dcw_args.end, argv[0]);
        return 1;
    }

    // Custom parsing to handle negative numbers
    const char *val_str = dcw_args.val->sval[0];
    char *endptr;
    int val = strtod(val_str, &endptr);
    
    // Check if conversion was successful
    if (endptr == val_str || *endptr != '\0') {
        printf("Invalid number format: %s\n", val_str);
        printf("DCW correction (default: 1, 0 - disable, 1 - enable)\n");
        return 1;
    }

    if(val == 0)
    {
        sens->set_dcw(sens, 0);
        printf("DCW correction: disable\n");
    }
    else if(val == 1){
        sens->set_dcw(sens, 1);
        printf("DCW correction: enable\n");
    }
    else{
        printf("Incorrect argument\n DCW correction (default: 1, 0 - disable, 1 - enable)\n");
        return 1;
    }

    return 0;
}

static void register_dcw_command(void)
{
    dcw_args.val = arg_str1(NULL, NULL, "<num>", "DCW correction (default: 0, 0 - disable, 1 - enable)");
    dcw_args.end = arg_end(3);

    const esp_console_cmd_t dcw_cmd = {
        .command = "dcw",
        .help = "Set dcw value",
        .hint = NULL,
        .func = &dcw_command_handler,
        .argtable = &dcw_args
    };

    ESP_ERROR_CHECK(esp_console_cmd_register(&dcw_cmd));
}

void app_main(void)
{

    int image_count = 0;
    char filename[32];
    
    // Power pin init
    gpio_reset_pin(CAM_PWR);
    gpio_set_direction(CAM_PWR, GPIO_MODE_OUTPUT);
    gpio_set_level(CAM_PWR, 0);
    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_console_repl_t *repl = NULL;
    esp_console_repl_config_t repl_config = ESP_CONSOLE_REPL_CONFIG_DEFAULT();
    
    repl_config.prompt = PROMPT_STR ">";
    repl_config.max_cmdline_length = CONFIG_CONSOLE_MAX_COMMAND_LINE_LENGTH;

    esp_console_register_help_command();
    register_system_common();

    register_nvs();
    
    /* Register our custom commands */
    register_print_command();
    register_echo_command();
    register_math_command();
    register_ae_command();
    register_take_command();
    register_takebmp_command();
    register_cinit_command();
    register_cdeinit_command();
    register_pwrup_command();
    register_pwrdown_command();
    register_bpc_command();
    register_sdinit_command();
    register_sddeinit_command();
    register_br_command();
    register_con_command();
    register_sat_command();
    register_aev_command();
    register_ag_command();
    register_bpc_command();
    register_wpc_command();
    register_dcw_command();
    
    ESP_LOGI(TAG, "Starting ESP32-S3 Camera with SD Card");
    ESP_LOGI(TAG, "Custom commands registered: 'print', 'echo', 'math', 'ae', 'take', 'cinit', 'pwrup', 'pwrdown', 'bpc', 'sdinit', 'sddeinit'");
    
#if defined(CONFIG_ESP_CONSOLE_UART_DEFAULT) || defined(CONFIG_ESP_CONSOLE_UART_CUSTOM)
    esp_console_dev_uart_config_t hw_config = ESP_CONSOLE_DEV_UART_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_uart(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_CDC)
    esp_console_dev_usb_cdc_config_t hw_config = ESP_CONSOLE_DEV_CDC_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_cdc(&hw_config, &repl_config, &repl));

#elif defined(CONFIG_ESP_CONSOLE_USB_SERIAL_JTAG)
    esp_console_dev_usb_serial_jtag_config_t hw_config = ESP_CONSOLE_DEV_USB_SERIAL_JTAG_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_console_new_repl_usb_serial_jtag(&hw_config, &repl_config, &repl));

#else
#error Unsupported console type
#endif
    
    ESP_ERROR_CHECK(esp_console_start_repl(repl));

    // SD card init
    

    //gpio_set_level(CAM_PWR, 1);

    // while (1) {        
        
    //     camera_fb_t *fb = esp_camera_fb_get();
    //     if (!fb) {
    //         ESP_LOGE(TAG, "Camera capture failed");
    //         vTaskDelay(pdMS_TO_TICKS(5000));
    //         continue;
    //     }

    //     ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);
        
    //     // Generate filename with timestamp
    //     snprintf(filename, sizeof(filename), "image%03d.jpg", image_count++);
        
    //     // Save to SD card
    //     esp_err_t ret = save_image_to_sd(fb, filename);
    //     if (ret != ESP_OK) {
    //         ESP_LOGE(TAG, "Failed to save image");
    //     }
        
    //     // Return the frame buffer back to the driver for reuse
    //     //esp_camera_deinit();
    //     gpio_set_level(CAM_PWR, 1);
    //     gpio_hold_en(CAM_PWR);
    //     gpio_deep_sleep_hold_en();
    //     vTaskDelay(pdMS_TO_TICKS(50));
        
    //     // Wait 10 seconds before taking next picture
    //     ESP_LOGE(TAG, "Deep sleep start");
    //     esp_sleep_enable_timer_wakeup(30 * 1000 * 1000);
    //     ESP_LOGI(TAG, "timer wakeup source is ready");
    //     /* Enter sleep mode */
    //     ESP_LOGI(TAG, "Woke up");
    //     //vTaskDelay(pdMS_TO_TICKS(30000));
    //     gpio_hold_dis(CAM_PWR);
    //     gpio_set_level(CAM_PWR, 0);
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     esp_camera_fb_return(fb);
    // }
}