#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"

#include "driver/i2c_master.h"
#include "driver/gpio.h"

#include "ds3231.h"

#define I2C_FREQ_HZ 400000

#define DS3231_STAT_OSCILLATOR 0x80
#define DS3231_STAT_32KHZ      0x08
#define DS3231_STAT_ALARM_2    0x02
#define DS3231_STAT_ALARM_1    0x01

#define DS3231_CTRL_OSCILLATOR    0x80
#define DS3231_CTRL_TEMPCONV      0x20
#define DS3231_CTRL_ALARM_INTS    0x04
#define DS3231_CTRL_ALARM2_INT    0x02
#define DS3231_CTRL_ALARM1_INT    0x01

#define DS3231_ALARM_WDAY   0x40
#define DS3231_ALARM_NOTSET 0x80

#define DS3231_ADDR_TIME    0x00
#define DS3231_ADDR_ALARM1  0x07
#define DS3231_ADDR_ALARM2  0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS  0x0f
#define DS3231_ADDR_AGING   0x10
#define DS3231_ADDR_TEMP    0x11

#define DS3231_12HOUR_FLAG  0x40
#define DS3231_12HOUR_MASK  0x1f
#define DS3231_PM_FLAG      0x20
#define DS3231_MONTH_MASK   0x1f

static const char *TAG = "DS3231";

#define CHECK_ARG(ARG) do { if (!ARG) return ESP_ERR_INVALID_ARG; } while (0)

static i2c_master_bus_handle_t bus_handle = NULL;

uint8_t bcd2dec(uint8_t val)
{
    return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
    return ((val / 10) << 4) + (val % 10);
}

esp_err_t ds3231_init_desc(i2c_master_dev_handle_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
    CHECK_ARG(dev);
    
    esp_err_t ret = ESP_OK;
    
    // Initialize I2C master bus only if not already initialized
    if (bus_handle == NULL) {
        i2c_master_bus_config_t bus_cfg = {
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .i2c_port = port,
            .scl_io_num = scl_gpio,
            .sda_io_num = sda_gpio,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        
        ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &bus_handle), TAG, "Failed to create I2C master bus");
    }
    
    // Add DS3231 device to the bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = DS3231_I2C_ADDRESS,
        .scl_speed_hz = 100000,
    };
    
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(bus_handle, &dev_cfg, dev), TAG, "Failed to add DS3231 device");
    
    return ESP_OK;
}

esp_err_t ds3231_free_desc(i2c_master_dev_handle_t *dev)
{
    CHECK_ARG(dev);
    
    esp_err_t ret = i2c_master_bus_rm_device(*dev);
    *dev = NULL;
    
    return ret;
}

static esp_err_t ds3231_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t write_buf[len + 1];
    write_buf[0] = reg;
    memcpy(&write_buf[1], data, len);
    
    return i2c_master_transmit(dev, write_buf, len + 1, -1);
}

static esp_err_t ds3231_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, -1);
}

esp_err_t ds3231_set_time(i2c_master_dev_handle_t dev, struct tm *time)
{
    CHECK_ARG(time);
    
    uint8_t data[7];
    
    /* time/date data */
    data[0] = dec2bcd(time->tm_sec);
    data[1] = dec2bcd(time->tm_min);
    data[2] = dec2bcd(time->tm_hour);
    /* The week data must be in the range 1 to 7, and to keep the start on the
     * same day as for tm_wday have it start at 1 on Sunday. */
    data[3] = dec2bcd(time->tm_wday + 1);
    data[4] = dec2bcd(time->tm_mday);
    data[5] = dec2bcd(time->tm_mon + 1);
    data[6] = dec2bcd(time->tm_year - 100);  // tm_year is years since 1900
    
    return ds3231_write_reg(dev, DS3231_ADDR_TIME, data, 7);
}

esp_err_t ds3231_get_time(i2c_master_dev_handle_t dev, struct tm *time)
{
    CHECK_ARG(time);
    
    uint8_t data[7];
    
    /* read time */
    esp_err_t res = ds3231_read_reg(dev, DS3231_ADDR_TIME, data, 7);
    if (res != ESP_OK) return res;
    
    /* convert to unix time structure */
    time->tm_sec = bcd2dec(data[0]);
    time->tm_min = bcd2dec(data[1]);
    if (data[2] & DS3231_12HOUR_FLAG) {
        /* 12H */
        time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
        /* AM/PM? */
        if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
    } else {
        time->tm_hour = bcd2dec(data[2]); /* 24H */
    }
    time->tm_wday = bcd2dec(data[3]) - 1;
    time->tm_mday = bcd2dec(data[4]);
    time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
    time->tm_year = bcd2dec(data[6]) + 100;  // tm_year is years since 1900
    time->tm_isdst = 0;
    
    return ESP_OK;
}

esp_err_t ds3231_set_alarm(i2c_master_dev_handle_t dev, ds3231_alarm_t alarms, 
                           struct tm *time1, ds3231_alarm1_rate_t option1, 
                           struct tm *time2, ds3231_alarm2_rate_t option2)
{
    uint8_t ctrl_reg;
    esp_err_t res;
    
    // Read current control register
    res = ds3231_read_reg(dev, DS3231_ADDR_CONTROL, &ctrl_reg, 1);
    if (res != ESP_OK) return res;
    
    // Configure Alarm 1
    if (alarms == DS3231_ALARM_1 || alarms == DS3231_ALARM_BOTH) {
        CHECK_ARG(time1);
        
        uint8_t data[4];
        
        data[0] = dec2bcd(time1->tm_sec);
        data[1] = dec2bcd(time1->tm_min);
        data[2] = dec2bcd(time1->tm_hour);
        data[3] = dec2bcd(time1->tm_wday + 1);
        
        // Set alarm rate bits (A1M1-A1M4 and DY/DT)
        switch (option1) {
            case DS3231_ALARM1_EVERY_SECOND:
                data[0] |= DS3231_ALARM_NOTSET;
                data[1] |= DS3231_ALARM_NOTSET;
                data[2] |= DS3231_ALARM_NOTSET;
                data[3] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM1_MATCH_SEC:
                data[1] |= DS3231_ALARM_NOTSET;
                data[2] |= DS3231_ALARM_NOTSET;
                data[3] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM1_MATCH_SECMIN:
                data[2] |= DS3231_ALARM_NOTSET;
                data[3] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM1_MATCH_SECMINHOUR:
                data[3] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM1_MATCH_SECMINHOURDAY:
                data[3] |= DS3231_ALARM_WDAY;
                break;
            case DS3231_ALARM1_MATCH_SECMINHOURDATE:
                data[3] = dec2bcd(time1->tm_mday);
                break;
        }
        
        res = ds3231_write_reg(dev, DS3231_ADDR_ALARM1, data, 4);
        if (res != ESP_OK) return res;
        
        // Enable Alarm 1 interrupt
        ctrl_reg |= DS3231_CTRL_ALARM1_INT | DS3231_CTRL_ALARM_INTS;
        
        // Clear Alarm 1 flag
        uint8_t status_reg;
        res = ds3231_read_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
        if (res != ESP_OK) return res;
        status_reg &= ~DS3231_STAT_ALARM_1;
        res = ds3231_write_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
        if (res != ESP_OK) return res;
    }
    
    // Configure Alarm 2
    if (alarms == DS3231_ALARM_2 || alarms == DS3231_ALARM_BOTH) {
        CHECK_ARG(time2);
        
        uint8_t data[3];
        
        data[0] = dec2bcd(time2->tm_min);
        data[1] = dec2bcd(time2->tm_hour);
        data[2] = dec2bcd(time2->tm_wday + 1);
        
        // Set alarm rate bits (A2M2-A2M4 and DY/DT)
        switch (option2) {
            case DS3231_ALARM2_EVERY_MIN:
                data[0] |= DS3231_ALARM_NOTSET;
                data[1] |= DS3231_ALARM_NOTSET;
                data[2] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM2_MATCH_MIN:
                data[1] |= DS3231_ALARM_NOTSET;
                data[2] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM2_MATCH_MINHOUR:
                data[2] |= DS3231_ALARM_NOTSET;
                break;
            case DS3231_ALARM2_MATCH_MINHOURDAY:
                data[2] |= DS3231_ALARM_WDAY;
                break;
            case DS3231_ALARM2_MATCH_MINHOURDATE:
                data[2] = dec2bcd(time2->tm_mday);
                break;
        }
        
        res = ds3231_write_reg(dev, DS3231_ADDR_ALARM2, data, 3);
        if (res != ESP_OK) return res;
        
        // Enable Alarm 2 interrupt
        ctrl_reg |= DS3231_CTRL_ALARM2_INT | DS3231_CTRL_ALARM_INTS;
        
        // Clear Alarm 2 flag
        uint8_t status_reg;
        res = ds3231_read_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
        if (res != ESP_OK) return res;
        status_reg &= ~DS3231_STAT_ALARM_2;
        res = ds3231_write_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
        if (res != ESP_OK) return res;
    }
    
    // Disable alarms if DS3231_ALARM_NONE
    if (alarms == DS3231_ALARM_NONE) {
        ctrl_reg &= ~(DS3231_CTRL_ALARM1_INT | DS3231_CTRL_ALARM2_INT);
    }
    
    // Write control register
    return ds3231_write_reg(dev, DS3231_ADDR_CONTROL, &ctrl_reg, 1);
}

esp_err_t ds3231_clear_alarm_flags(i2c_master_dev_handle_t dev, ds3231_alarm_t alarms)
{
    uint8_t status_reg;
    esp_err_t res;
    
    res = ds3231_read_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
    if (res != ESP_OK) return res;
    
    if (alarms == DS3231_ALARM_1 || alarms == DS3231_ALARM_BOTH) {
        status_reg &= ~DS3231_STAT_ALARM_1;
    }
    
    if (alarms == DS3231_ALARM_2 || alarms == DS3231_ALARM_BOTH) {
        status_reg &= ~DS3231_STAT_ALARM_2;
    }
    
    return ds3231_write_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
}

esp_err_t ds3231_check_alarm_flags(i2c_master_dev_handle_t dev, ds3231_alarm_t *alarms)
{
    CHECK_ARG(alarms);
    
    uint8_t status_reg;
    esp_err_t res;
    
    res = ds3231_read_reg(dev, DS3231_ADDR_STATUS, &status_reg, 1);
    if (res != ESP_OK) return res;
    
    *alarms = DS3231_ALARM_NONE;
    
    if (status_reg & DS3231_STAT_ALARM_1) {
        *alarms = DS3231_ALARM_1;
    }
    
    if (status_reg & DS3231_STAT_ALARM_2) {
        if (*alarms == DS3231_ALARM_1) {
            *alarms = DS3231_ALARM_BOTH;
        } else {
            *alarms = DS3231_ALARM_2;
        }
    }
    
    return ESP_OK;
}

esp_err_t ds3231_get_raw_temp(i2c_master_dev_handle_t dev, int16_t *temp)
{
    CHECK_ARG(temp);
    
    uint8_t data[2];
    
    esp_err_t res = ds3231_read_reg(dev, DS3231_ADDR_TEMP, data, sizeof(data));
    if (res == ESP_OK)
        *temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;
    
    return res;
}

esp_err_t ds3231_get_temp_integer(i2c_master_dev_handle_t dev, int8_t *temp)
{
    CHECK_ARG(temp);
    
    int16_t t_int;
    
    esp_err_t res = ds3231_get_raw_temp(dev, &t_int);
    if (res == ESP_OK)
        *temp = t_int >> 2;
    
    return res;
}

esp_err_t ds3231_get_temp_float(i2c_master_dev_handle_t dev, float *temp)
{
    CHECK_ARG(temp);
    
    int16_t t_int;
    
    esp_err_t res = ds3231_get_raw_temp(dev, &t_int);
    if (res == ESP_OK)
        *temp = t_int * 0.25;
    
    return res;
}

esp_err_t ds3231_deinit(i2c_master_dev_handle_t dev)
{
    CHECK_ARG(dev);

    esp_err_t ret;

    ret = i2c_master_bus_rm_device(dev);
    
    return ret;
}