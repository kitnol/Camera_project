#include <string.h>
#include <time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "driver/i2c_master.h"
#include "driver/i2c.h"
#include "driver/i2c_master.h"
#include "driver/i2c_slave.h"

#include "ds3231.h"

#define TEST_I2C_PORT I2C_NUM_1 /*!< I2C port number for master dev */
#define RTC_SDA_GPIO 17 /*!< GPIO number for I2C master data  */
#define RTC_SCL_GPIO 18 /*!< GPIO number for I2C master clock */

#define CHECK_ARG(ARG) do { if (!ARG) return ESP_ERR_INVALID_ARG; } while (0)

uint8_t bcd2dec(uint8_t val)
{
	return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t dec2bcd(uint8_t val)
{
	return ((val / 10) << 4) + (val % 10);
}

i2c_master_bus_handle_t bus_general;

esp_err_t ds3231_init_desc(i2c_master_dev_handle_t *dev, i2c_port_t port, gpio_num_t sda_gpio, gpio_num_t scl_gpio)
{
	i2c_master_bus_config_t bus_cfg = {
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.i2c_port = port,
		.scl_io_num = scl_gpio,
		.sda_io_num = sda_gpio,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};

	i2c_master_bus_handle_t bus_handle;
	ESP_ERROR_CHECK(i2c_master_bus_init(port, &bus_cfg, &bus_handle));

	bus_general = bus_handle;

	i2c_device_config_t dev_cfg = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = DS3231_I2C_ADDRESS,
		.scl_speed_hz = 100000,
	};

	i2c_master_dev_handle_t dev_handle;
	ESP_ERROR_CHECK(i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));

	*dev = dev_handle;

	return ESP_OK;
}

esp_err_t ds3231_set_time(i2c_master_dev_handle_t *dev, struct tm *time)
{
	CHECK_ARG(dev);
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
	data[6] = dec2bcd(time->tm_year - 2000);

	return i2c_dev_write_reg(dev, DS3231_ADDR_TIME, data, 7);
}

esp_err_t ds3231_get_raw_temp(i2c_master_dev_handle_t *dev, int16_t *temp)
{
	CHECK_ARG(dev);
	CHECK_ARG(temp);

	uint8_t data[2];

	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_TEMP, data, sizeof(data));
	if (res == ESP_OK)
		*temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;

	return res;
}

esp_err_t ds3231_get_temp_integer(i2c_master_dev_handle_t *dev, int8_t *temp)
{
	CHECK_ARG(temp);

	int16_t t_int;

	esp_err_t res = ds3231_get_raw_temp(dev, &t_int);
	if (res == ESP_OK)
		*temp = t_int >> 2;

	return res;
}

esp_err_t ds3231_get_temp_float(i2c_master_dev_handle_t *dev, float *temp)
{
	CHECK_ARG(temp);

	int16_t t_int;

	esp_err_t res = ds3231_get_raw_temp(dev, &t_int);
	if (res == ESP_OK)
		*temp = t_int * 0.25;

	return res;
}

esp_err_t ds3231_get_time(i2c_master_dev_handle_t *dev, struct tm *time)
{
	CHECK_ARG(dev);
	CHECK_ARG(time);

	uint8_t data[7];

	/* read time */
	esp_err_t res = i2c_dev_read_reg(dev, DS3231_ADDR_TIME, data, 7);
		if (res != ESP_OK) return res;

	/* convert to unix time structure */
	time->tm_sec = bcd2dec(data[0]);
	time->tm_min = bcd2dec(data[1]);
	if (data[2] & DS3231_12HOUR_FLAG)
	{
		/* 12H */
		time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
		/* AM/PM? */
		if (data[2] & DS3231_PM_FLAG) time->tm_hour += 12;
	}
	else time->tm_hour = bcd2dec(data[2]); /* 24H */
	time->tm_wday = bcd2dec(data[3]) - 1;
	time->tm_mday = bcd2dec(data[4]);
	time->tm_mon  = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
	time->tm_year = bcd2dec(data[6]) + 2000;
	time->tm_isdst = 0;

	// apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
	//applyTZ(time);

	return ESP_OK;
}
