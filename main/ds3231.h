#ifndef MAIN_DS3231_H_
#define MAIN_DS3231_H_

#include <time.h>
#include <stdbool.h>
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include <i2cdev.h>


#define DS3231_I2C_ADDRESS 0x68 //!< I2C address

#define DS3231_STAT_OSCILLATOR 0x80
#define DS3231_STAT_32KHZ      0x08
#define DS3231_STAT_BUSY       0x04
#define DS3231_STAT_ALARM_2    0x02
#define DS3231_STAT_ALARM_1    0x01

#define DS3231_CTRL_OSCILLATOR    0x80
#define DS3231_CTRL_SQUAREWAVE_BB 0x40
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

uint8_t bcd2dec(uint8_t val);
uint8_t dec2bcd(uint8_t val);

/**
 * Alarms
 */
typedef enum
{
    DS3231_ALARM_NONE = 0,//!< No alarms
    DS3231_ALARM_1,       //!< First alarm
    DS3231_ALARM_2,       //!< Second alarm
    DS3231_ALARM_BOTH     //!< Both alarms
} ds3231_alarm_t;

/**
 * First alarm rate
 */
typedef enum
{
    DS3231_ALARM1_EVERY_SECOND = 0,
    DS3231_ALARM1_MATCH_SEC,
    DS3231_ALARM1_MATCH_SECMIN,
    DS3231_ALARM1_MATCH_SECMINHOUR,
    DS3231_ALARM1_MATCH_SECMINHOURDAY,
    DS3231_ALARM1_MATCH_SECMINHOURDATE
} ds3231_alarm1_rate_t;

/**
 * Second alarm rate
 */
typedef enum
{
    DS3231_ALARM2_EVERY_MIN = 0,
    DS3231_ALARM2_MATCH_MIN,
    DS3231_ALARM2_MATCH_MINHOUR,
    DS3231_ALARM2_MATCH_MINHOURDAY,
    DS3231_ALARM2_MATCH_MINHOURDATE
} ds3231_alarm2_rate_t;

/**
 * Squarewave frequency
 */
typedef enum
{
    DS3231_SQWAVE_1HZ    = 0x00,
    DS3231_SQWAVE_1024HZ = 0x08,
    DS3231_SQWAVE_4096HZ = 0x10,
    DS3231_SQWAVE_8192HZ = 0x18
} ds3231_sqwave_freq_t;

esp_err_t ds3231_init_desc(i2c_master_dev_handle_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio);
esp_err_t ds3231_free_desc(i2c_master_dev_handle_t *dev);

esp_err_t ds3231_set_time(i2c_master_dev_handle_t dev, struct tm *time);
esp_err_t ds3231_get_time(i2c_master_dev_handle_t dev, struct tm *time);

esp_err_t ds3231_get_raw_temp(i2c_master_dev_handle_t dev, int16_t *temp);
esp_err_t ds3231_get_temp_integer(i2c_master_dev_handle_t dev, int8_t *temp);
esp_err_t ds3231_get_temp_float(i2c_master_dev_handle_t dev, float *temp);
esp_err_t ds3231_set_alarm(i2c_master_dev_handle_t dev, ds3231_alarm_t alarms, struct tm *time1, ds3231_alarm1_rate_t option1, struct tm *time2, ds3231_alarm2_rate_t option2);

esp_err_t ds3231_deinit(i2c_master_dev_handle_t dev);

#endif /* MAIN_DS3231_H_ */