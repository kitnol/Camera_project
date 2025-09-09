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

#include <errno.h>
#include <dirent.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_partition.h"
#include "tinyusb.h"
#include "tusb_msc_storage.h"
#include "diskio_impl.h"
#include "diskio_sdmmc.h"

static const char *TAG = "camera_sd";
#define PROMPT_STR CONFIG_IDF_TARGET
#define VREG_ADDR 0x36

#define TUSB_DESC_TOTAL_LEN (TUD_CONFIG_DESC_LEN + TUD_MSC_DESC_LEN)

enum {
    ITF_NUM_MSC = 0,
    ITF_NUM_TOTAL
};

enum {
    EDPT_CTRL_OUT = 0x00,
    EDPT_CTRL_IN  = 0x80,

    EDPT_MSC_OUT  = 0x01,
    EDPT_MSC_IN   = 0x81,
};

// BMP header structures
// typedef struct attribute((packed)) {
//     uint16_t type;          // BM
//     uint32_t size;          // File size
//     uint16_t reserved1;
//     uint16_t reserved2;
//     uint32_t offset;        // Offset to image data
// } bmp_file_header_t;

// typedef struct attribute((packed)) {
//     uint32_t size;          // Header size
//     int32_t width;
//     int32_t height;
//     uint16_t planes;
//     uint16_t bits_per_pixel;
//     uint32_t compression;
//     uint32_t image_size;
//     int32_t x_pixels_per_meter;
//     int32_t y_pixels_per_meter;
//     uint32_t colors_used;
//     uint32_t colors_important;
// } bmp_info_header_t;

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

// USB detection pin
#define USB_INT 21

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
    .fb_count = 3,                      // Single frame buffer without PSRAM
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

// mount the partition and show all the files in BASE_PATH
static void _mount(void)
{
    ESP_LOGI(TAG, "Mount storage...");
    ESP_ERROR_CHECK(tinyusb_msc_storage_mount(MOUNT_POINT));

    // List all the files in this directory
    ESP_LOGI(TAG, "\nls command output:");
    struct dirent *d;
    DIR *dh = opendir(MOUNT_POINT);
    if (!dh) {
        if (errno == ENOENT) {
            //If the directory is not found
            ESP_LOGE(TAG, "Directory doesn't exist %s", MOUNT_POINT);
        } else {
            //If the directory is not readable then throw error and exit
            ESP_LOGE(TAG, "Unable to read directory %s", MOUNT_POINT);
        }
        return;
    }
    //While the next entry is not readable we will print directory files
    while ((d = readdir(dh)) != NULL) {
        printf("%s\n", d->d_name);
    }
    return;
}

static esp_err_t storage_init_sdmmc(sdmmc_card_t **card)
{
    esp_err_t ret = ESP_OK;
    bool host_init = false;
    sdmmc_card_t *sd_card;

    ESP_LOGI(TAG, "Initializing SDCard");

    // By default, SD card frequency is initialized to SDMMC_FREQ_DEFAULT (20MHz)
    // For setting a specific frequency, use host.max_freq_khz (range 400kHz - 40MHz for SDMMC)
    // Example: for fixed frequency of 10MHz, use host.max_freq_khz = 10000;
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();

    // For SoCs where the SD power can be supplied both via an internal or external (e.g. on-board LDO) power supply.
    // When using specific IO pins (which can be used for ultra high-speed SDMMC) to connect to the SD card
    // and the internal LDO power supply, we need to initialize the power supply first.
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    sd_pwr_ctrl_ldo_config_t ldo_config = {
        .ldo_chan_id = CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_IO_ID,
    };
    sd_pwr_ctrl_handle_t pwr_ctrl_handle = NULL;

    ret = sd_pwr_ctrl_new_on_chip_ldo(&ldo_config, &pwr_ctrl_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create a new on-chip LDO power control driver");
        return ret;
    }
    host.pwr_ctrl_handle = pwr_ctrl_handle;
#endif

    // This initializes the slot without card detect (CD) and write protect (WP) signals.
    // Modify slot_config.gpio_cd and slot_config.gpio_wp if your board has these signals.
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();

    // For SD Card, set bus width to use
    slot_config.width = 4;

    // On chips where the GPIOs used for SD card can be configured, set the user defined values
    slot_config.clk = SD_PIN_SCK;
    slot_config.cmd = SD_PIN_CMD;
    slot_config.d0 = SD_PIN_CMD;
    slot_config.d1 = SD_PIN_D1;
    slot_config.d2 = SD_PIN_D2;
    slot_config.d3 = SD_PIN_D3;


    // Enable internal pullups on enabled pins. The internal pullups
    // are insufficient however, please make sure 10k external pullups are
    // connected on the bus. This is for debug / example purpose only.
    // slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    // not using ff_memalloc here, as allocation in internal RAM is preferred
    sd_card = (sdmmc_card_t *)malloc(sizeof(sdmmc_card_t));
    ESP_GOTO_ON_FALSE(sd_card, ESP_ERR_NO_MEM, clean, TAG, "could not allocate new sdmmc_card_t");

    ESP_GOTO_ON_ERROR((*host.init)(), clean, TAG, "Host Config Init fail");
    host_init = true;

    ESP_GOTO_ON_ERROR(sdmmc_host_init_slot(host.slot, (const sdmmc_slot_config_t *) &slot_config),
                      clean, TAG, "Host init slot fail");

    while (sdmmc_card_init(&host, sd_card)) {
        ESP_LOGE(TAG, "The detection pin of the slot is disconnected(Insert uSD card). Retrying...");
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    // Card has been initialized, print its properties
    sdmmc_card_print_info(stdout, sd_card);
    *card = sd_card;

    return ESP_OK;

clean:
    if (host_init) {
        if (host.flags & SDMMC_HOST_FLAG_DEINIT_ARG) {
            host.deinit_p(host.slot);
        } else {
            (*host.deinit)();
        }
    }
    if (sd_card) {
        free(sd_card);
        sd_card = NULL;
    }
#if CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    // We don't need to duplicate error here as all error messages are handled via sd_pwr_* call
    sd_pwr_ctrl_del_on_chip_ldo(pwr_ctrl_handle);
#endif // CONFIG_EXAMPLE_SD_PWR_CTRL_LDO_INTERNAL_IO
    return ret;
}

// callback that is delivered when storage is mounted/unmounted by application.
static void storage_mount_changed_cb(tinyusb_msc_event_t *event)
{
    ESP_LOGI(TAG, "Storage mounted to application: %s", event->mount_changed_data.is_mounted ? "Yes" : "No");
}

static tusb_desc_device_t descriptor_config = {
    .bLength = sizeof(descriptor_config),
    .bDescriptorType = TUSB_DESC_DEVICE,
    .bcdUSB = 0x0200,
    .bDeviceClass = TUSB_CLASS_MISC,
    .bDeviceSubClass = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0 = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor = 0x303A, // This is Espressif VID. This needs to be changed according to Users / Customers
    .idProduct = 0x4002,
    .bcdDevice = 0x100,
    .iManufacturer = 0x01,
    .iProduct = 0x02,
    .iSerialNumber = 0x03,
    .bNumConfigurations = 0x01
};

static char const *string_desc_arr[] = {
    (const char[]) { 0x09, 0x04 },  // 0: is supported language is English (0x0409)
    "TinyUSB",                      // 1: Manufacturer
    "TinyUSB Device",               // 2: Product
    "123456",                       // 3: Serials
    "Example MSC",                  // 4. MSC
};

static uint8_t const msc_fs_configuration_desc[] = {
    // Config number, interface count, string index, total length, attribute, power in mA
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, TUSB_DESC_TOTAL_LEN, TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    // Interface number, string index, EP Out & EP In address, EP size
    TUD_MSC_DESCRIPTOR(ITF_NUM_MSC, 0, EDPT_MSC_OUT, EDPT_MSC_IN, 64),
};

void app_main(void)
{
    char filename[32];

    while (1) {        

        gpio_reset_pin(USB_INT);
        gpio_set_direction(USB_INT, GPIO_MODE_INPUT);

        if(gpio_get_level(USB_INT) == 1)
        {
            static sdmmc_card_t *card = NULL;
            ESP_ERROR_CHECK(storage_init_sdmmc(&card));

            const tinyusb_msc_sdmmc_config_t config_sdmmc = {
                .card = card,
                .callback_mount_changed = storage_mount_changed_cb,  /* First way to register the callback. This is while initializing the storage. */
                .mount_config.max_files = 5,
            };
            ESP_ERROR_CHECK(tinyusb_msc_storage_init_sdmmc(&config_sdmmc));
            ESP_ERROR_CHECK(tinyusb_msc_register_callback(TINYUSB_MSC_EVENT_MOUNT_CHANGED, storage_mount_changed_cb)); /* Other way to register the callback i.e. registering using separate API. If the callback had been already registered, it will be overwritten. */

            //mounted in the app by default
            _mount();

            ESP_LOGI(TAG, "USB MSC initialization");
            const tinyusb_config_t tusb_cfg = {
                .device_descriptor = &descriptor_config,
                .string_descriptor = string_desc_arr,
                .string_descriptor_count = sizeof(string_desc_arr) / sizeof(string_desc_arr[0]),
                .external_phy = false,
        #if (TUD_OPT_HIGH_SPEED)
                .fs_configuration_descriptor = msc_fs_configuration_desc,
                .hs_configuration_descriptor = msc_hs_configuration_desc,
                .qualifier_descriptor = &device_qualifier,
        #else
                .configuration_descriptor = msc_fs_configuration_desc,
        #endif // TUD_OPT_HIGH_SPEED
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));
    ESP_LOGI(TAG, "USB MSC initialization DONE");
        }
        
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
        esp_camera_fb_return(fb);
        fb = esp_camera_fb_get();
        esp_camera_fb_return(fb);
        fb = esp_camera_fb_get();
        if (!fb || fb == NULL) {
            ESP_LOGE(TAG, "Camera capture failed");
            vTaskDelay(pdMS_TO_TICKS(5000));
            ret = ESP_FAIL;
            continue;
        }
        else{
            ESP_LOGI(TAG, "Picture taken! Size: %d bytes", fb->len);
            ret = ESP_OK;
            // Save to SD card
            snprintf(filename, sizeof(filename), "image%03d.jpg", image_count++);
            ret = save_image_to_sd(fb, filename);
            if (ret != ESP_OK) {
                ESP_LOGE(TAG, "Failed to save image");
            }
        }

// Camera deinit
        ret = deinit_camera();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Camera deinitialization failed");
            continue;
        }
        else{
            ESP_LOGI(TAG, "Camera deinitialized");
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