/*
 * Copyright (c) 2018 Philémon Jaermann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <i2c.h>
#include <init.h>
#include <sensor.h>
#include <logging/sys_log.h>

#include "lsm303dlhc_magn.h"

static int lsm303dlhc_sample_fetch(struct device *dev,
				   enum sensor_channel chan)
{
	struct lsm303dlhc_magn_data *drv_data = dev->driver_data;
	u8_t tries = 11;
	u8_t magn_buf[6];
	u8_t status;

	/* Check data ready flag */
	while (tries-- > 0) {
		if (i2c_reg_read_byte(drv_data->i2c,
				      LSM303DLHC_MAGN_I2C_ADDR,
				      LSM303DLHC_SR_REG_M,
				      &status) < 0) {
			SYS_LOG_ERR("Failed to read status register.");
			return -EIO;
		}

		if ((status & LSM303DLHC_MAGN_DRDY) || tries == 0) {
			break;
		}

		k_sleep(LSM303DLHC_MAGN_DRDY_WAIT_TIME);
	}

	if (!(status & LSM303DLHC_MAGN_DRDY)) {
		SYS_LOG_ERR("Sensor data not available.");
		return -EIO;
	}

	if (i2c_burst_read(drv_data->i2c,
			   LSM303DLHC_MAGN_I2C_ADDR,
			   LSM303DLHC_REG_MAGN_X_LSB,
			   magn_buf, 6) < 0) {
		SYS_LOG_ERR("Could not read magn axis data.");
		return -EIO;
	}

	drv_data->magn_x = (magn_buf[0] << 8) | magn_buf[1];
	drv_data->magn_y = (magn_buf[4] << 8) | magn_buf[5];
	drv_data->magn_z = (magn_buf[2] << 8) | magn_buf[3];

	return 0;
}

static void lsm303dlhc_convert(struct sensor_value *val,
			       s64_t raw_val)
{
	val->val1 = raw_val / LSM303DLHC_MAGN_LSB_GAUSS;
	val->val2 = (1000000 * raw_val / LSM303DLHC_MAGN_LSB_GAUSS) % 1000000;
}

static int lsm303dlhc_channel_get(struct device *dev,
				  enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct lsm303dlhc_magn_data *drv_data = dev->driver_data;

	if (chan == SENSOR_CHAN_MAGN_X) {
		lsm303dlhc_convert(val, drv_data->magn_x);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		lsm303dlhc_convert(val, drv_data->magn_y);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		lsm303dlhc_convert(val, drv_data->magn_z);
	} else if (chan == SENSOR_CHAN_MAGN_XYZ) {
		lsm303dlhc_convert(val, drv_data->magn_x);
		lsm303dlhc_convert(val + 1, drv_data->magn_y);
		lsm303dlhc_convert(val + 2, drv_data->magn_z);
	} else {
		return -ENOTSUP;
	}
	return 0;
}

static const struct sensor_driver_api lsm303dlhc_magn_driver_api = {
	.sample_fetch = lsm303dlhc_sample_fetch,
	.channel_get = lsm303dlhc_channel_get,
};

static int lsm303dlhc_magn_init(struct device *dev)
{
	struct lsm303dlhc_magn_data *drv_data = dev->driver_data;

	drv_data->i2c = device_get_binding(LSM303DLHC_MAGN_I2C_DEV);
	if (drv_data->i2c == NULL) {
		SYS_LOG_ERR("Could not get pointer to %s device",
				LSM303DLHC_MAGN_I2C_DEV);
		return -ENODEV;
	}

	k_sleep(1);

	/* Set magnmeter output data rate */
	if (i2c_reg_write_byte(drv_data->i2c,
			       LSM303DLHC_MAGN_I2C_ADDR,
			       LSM303DLHC_CRA_REG_M,
			       LSM303DLHC_MAGN_ODR_BITS) < 0) {
		SYS_LOG_ERR("Failed to configure chip.");
		return -EIO;
	}

	/* Set magnmeter full scale range */
	if (i2c_reg_write_byte(drv_data->i2c,
			       LSM303DLHC_MAGN_I2C_ADDR,
			       LSM303DLHC_CRB_REG_M,
			       LSM303DLHC_MAGN_FS_BITS) < 0) {
		SYS_LOG_ERR("Failed to set magnmeter full scale range.");
		return -EIO;
	}

	/* Continuous update */
	if (i2c_reg_write_byte(drv_data->i2c,
			       LSM303DLHC_MAGN_I2C_ADDR,
			       LSM303DLHC_MR_REG_M,
			       LSM303DLHC_MAGN_CONT_UPDATE) < 0) {
		SYS_LOG_ERR("Failed to enable continuous data update.");
		return -EIO;
	}
	return 0;
}

static struct lsm303dlhc_magn_data lsm303dlhc_magn_driver;

DEVICE_AND_API_INIT(lsm303dlhc_magn, CONFIG_LSM303DLHC_MAGN_NAME,
		    lsm303dlhc_magn_init, &lsm303dlhc_magn_driver,
		    NULL, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,
		    &lsm303dlhc_magn_driver_api);
