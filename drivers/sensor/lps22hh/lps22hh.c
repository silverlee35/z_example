/* ST Microelectronics LPS22HH pressure and temperature sensor
 *
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/lps22hh.pdf
 */

#define DT_DRV_COMPAT st_lps22hh

#include <drivers/sensor.h>
#include <kernel.h>
#include <device.h>
#include <init.h>
#include <sys/byteorder.h>
#include <sys/__assert.h>
#include <logging/log.h>

#include "lps22hh.h"

LOG_MODULE_REGISTER(LPS22HH, CONFIG_SENSOR_LOG_LEVEL);

static inline int lps22hh_set_odr_raw(const struct device *dev, uint8_t odr)
{
	const struct lps22hh_config * const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;

	return lps22hh_data_rate_set(ctx, odr);
}

static int lps22hh_sample_fetch(const struct device *dev,
				enum sensor_channel chan)
{
	struct lps22hh_data *data = dev->data;
	const struct lps22hh_config * const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	uint32_t raw_press;
	int16_t raw_temp;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);

	if (lps22hh_pressure_raw_get(ctx, &raw_press) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}
	if (lps22hh_temperature_raw_get(ctx, &raw_temp) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->sample_press = raw_press;
	data->sample_temp = raw_temp;

	return 0;
}

static inline void lps22hh_press_convert(struct sensor_value *val,
					 int32_t raw_val)
{
	int32_t press_tmp = raw_val >> 8; /* raw value is left aligned (24 msb) */

	/* Pressure sensitivity is 4096 LSB/hPa */
	/* Also convert hPa into kPa */

	val->val1 = press_tmp / 40960;

	/* For the decimal part use (3125 / 128) as a factor instead of
	 * (1000000 / 40960) to avoid int32 overflow
	 */
	val->val2 = (press_tmp % 40960) * 3125 / 128;
}

static inline void lps22hh_temp_convert(struct sensor_value *val,
					int16_t raw_val)
{
	/* Temperature sensitivity is 100 LSB/deg C */
	val->val1 = raw_val / 100;
	val->val2 = ((int32_t)raw_val % 100) * 10000;
}

static int lps22hh_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct lps22hh_data *data = dev->data;

	if (chan == SENSOR_CHAN_PRESS) {
		lps22hh_press_convert(val, data->sample_press);
	} else if (chan == SENSOR_CHAN_AMBIENT_TEMP) {
		lps22hh_temp_convert(val, data->sample_temp);
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const uint16_t lps22hh_map[] = {0, 1, 10, 25, 50, 75, 100, 200};

static int lps22hh_odr_set(const struct device *dev, uint16_t freq)
{
	int odr;

	for (odr = 0; odr < ARRAY_SIZE(lps22hh_map); odr++) {
		if (freq == lps22hh_map[odr]) {
			break;
		}
	}

	if (odr == ARRAY_SIZE(lps22hh_map)) {
		LOG_DBG("bad frequency");
		return -EINVAL;
	}

	if (lps22hh_set_odr_raw(dev, odr) < 0) {
		LOG_DBG("failed to set sampling rate");
		return -EIO;
	}

	return 0;
}

static int lps22hh_attr_set(const struct device *dev,
			    enum sensor_channel chan,
			    enum sensor_attribute attr,
			    const struct sensor_value *val)
{
	if (chan != SENSOR_CHAN_ALL) {
		LOG_WRN("attr_set() not supported on this channel.");
		return -ENOTSUP;
	}

	switch (attr) {
	case SENSOR_ATTR_SAMPLING_FREQUENCY:
		return lps22hh_odr_set(dev, val->val1);
	default:
		LOG_DBG("operation not supported.");
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api lps22hh_driver_api = {
	.attr_set = lps22hh_attr_set,
	.sample_fetch = lps22hh_sample_fetch,
	.channel_get = lps22hh_channel_get,
#if CONFIG_LPS22HH_TRIGGER
	.trigger_set = lps22hh_trigger_set,
#endif
};

static int lps22hh_init_chip(const struct device *dev)
{
	const struct lps22hh_config * const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	uint8_t chip_id;
	int ret;

	if (lps22hh_device_id_get(ctx, &chip_id) < 0) {
		LOG_ERR("%s: Not able to read dev id", dev->name);
		return -EIO;
	}

	if (chip_id != LPS22HH_ID) {
		LOG_ERR("%s: Invalid chip ID 0x%02x", dev->name, chip_id);
		return -EIO;
	}

	LOG_DBG("%s: chip id 0x%x", dev->name, chip_id);

	ret = lps22hh_set_odr_raw(dev, CONFIG_LPS22HH_SAMPLING_RATE);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set sampling rate", dev->name);
		return ret;
	}

	if (lps22hh_block_data_update_set(ctx, PROPERTY_ENABLE) < 0) {
		LOG_ERR("%s: Failed to set BDU", dev->name);
		return ret;
	}

	return 0;
}

static int lps22hh_init(const struct device *dev)
{
	if (lps22hh_init_chip(dev) < 0) {
		LOG_DBG("Failed to initialize chip");
		return -EIO;
	}

#ifdef CONFIG_LPS22HH_TRIGGER
	if (lps22hh_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupt.");
		return -EIO;
	}
#endif

	return 0;
}

#if DT_NUM_INST_STATUS_OKAY(DT_DRV_COMPAT) == 0
#warning "LPS22HH driver enabled without any devices"
#endif

/*
 * Instantiation macros used when a device is on a SPI bus.
 */

#ifdef CONFIG_LPS22HH_TRIGGER
#define LPS22HH_CFG_IRQ(inst) \
	.gpio_int = GPIO_DT_SPEC_INST_GET(inst, drdy_gpios),
#else
#define LPS22HH_CFG_IRQ(inst)
#endif /* CONFIG_LPS22HH_TRIGGER */

#define LPS22HH_SPI_OPERATION (SPI_WORD_SET(8) |			\
				SPI_OP_MODE_MASTER |			\
				SPI_MODE_CPOL |				\
				SPI_MODE_CPHA)				\

#define LPS22HH_CONFIG_SPI(inst)					\
	{								\
		.ctx = {						\
			.read_reg =					\
			   (stmdev_read_ptr) stmemsc_spi_read,		\
			.write_reg =					\
			   (stmdev_write_ptr) stmemsc_spi_write,	\
			.handle =					\
			   (void *)&lps22hh_config_##inst.stmemsc_cfg,	\
		},							\
		.stmemsc_cfg = {					\
			.spi = SPI_DT_SPEC_INST_GET(inst,		\
					   LPS22HH_SPI_OPERATION,	\
					   0),				\
		},							\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, drdy_gpios),	\
			(LPS22HH_CFG_IRQ(inst)), ())			\
	}

/*
 * Instantiation macros used when a device is on an I2C bus.
 */

#define LPS22HH_CONFIG_I2C(inst)					\
	{								\
		.ctx = {						\
			.read_reg =					\
			   (stmdev_read_ptr) stmemsc_i2c_read,		\
			.write_reg =					\
			   (stmdev_write_ptr) stmemsc_i2c_write,	\
			.handle =					\
			   (void *)&lps22hh_config_##inst.stmemsc_cfg,	\
		},							\
		.stmemsc_cfg = {					\
			.i2c = I2C_DT_SPEC_INST_GET(inst),		\
		},							\
		COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, drdy_gpios),	\
			(LPS22HH_CFG_IRQ(inst)), ())			\
	}

/*
 * Main instantiation macro. Use of COND_CODE_1() selects the right
 * bus-specific macro at preprocessor time.
 */

#define LPS22HH_DEFINE(inst)							\
	static struct lps22hh_data lps22hh_data_##inst;				\
	static const struct lps22hh_config lps22hh_config_##inst =		\
	COND_CODE_1(DT_INST_ON_BUS(inst, spi),					\
		    (LPS22HH_CONFIG_SPI(inst)),					\
		    (LPS22HH_CONFIG_I2C(inst)));				\
	DEVICE_DT_INST_DEFINE(inst, lps22hh_init, NULL, &lps22hh_data_##inst,	\
			      &lps22hh_config_##inst, POST_KERNEL,		\
			      CONFIG_SENSOR_INIT_PRIORITY, &lps22hh_driver_api);

DT_INST_FOREACH_STATUS_OKAY(LPS22HH_DEFINE)
