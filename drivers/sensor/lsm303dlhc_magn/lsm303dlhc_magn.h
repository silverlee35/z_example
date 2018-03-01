/*
 * Copyright (c) 2018 Philémon Jaermann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_LSM303DLHC_MAGN_
#define __SENSOR_LSM303DLHC_MAGN_

#define LSM303DLHC_MAGN_I2C_ADDRESS	CONFIG_LSM303DLHC_MAGN_I2C_ADDR

#define LSM303_DLHC_MAGN_X_EN_BIT	BIT(0)
#define LSM303DLHC_MAGN_Y_EN_BIT	BIT(1)
#define LSM303DLHC_MAGN_Z_EN_BIT	BIT(2)
#define LSM303DLHC_MAGN_EN_BITS		(LSM303_DLHC_MAGN_X_EN_BIT | \
					LSM303DLHC_MAGN_Y_EN_BIT | \
					LSM303DLHC_MAGN_Z_EN_BIT)

#if	(CONFIG_LSM303DLHC_MAGN_ODR == 0)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	134
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 1)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	67
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 2)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	34
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 3)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	14
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 4)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	7
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 5)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	4
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 6)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	2
#elif	(CONFIG_LSM303DLHC_MAGN_ODR == 7)
	#define LSM303DLHC_MAGN_DRDY_WAIT_TIME	1
#endif

#define LSM303DLHC_MAGN_ODR_SHIFT	2
#define LSM303DLHC_MAGN_ODR_BITS	(CONFIG_LSM303DLHC_MAGN_ODR << \
					LSM303DLHC_MAGN_ODR_SHIFT)

#if	(CONFIG_LSM303DLHC_MAGN_RANGE == 1)
	#define LSM303DLHC_MAGN_LSB_GAUSS	1100
#elif	(CONFIG_LSM303DLHC_MAGN_RANGE == 2)
	#define LSM303DLHC_MAGN_LSB_GAUSS	760
#elif	(CONFIG_LSM303DLHC_MAGN_RANGE == 3)
	#define LSM303DLHC_MAGN_LSB_GAUSS	600
#elif	(CONFIG_LSM303DLHC_MAGN_RANGE == 4)
	#define LSM303DLHC_MAGN_LSB_GAUSS	400
#elif	(CONFIG_LSM303DLHC_MAGN_RANGE == 5)
	#define LSM303DLHC_MAGN_LSB_GAUSS	355
#elif	(CONFIG_LSM303DLHC_MAGN_RANGE == 6)
	#define LSM303DLHC_MAGN_LSB_GAUSS	295
#elif	(CONFIG_LSM303DLHC_MAGN_RANGE == 7)
	#define LSM303DLHC_MAGN_LSB_GAUSS	205
#endif

#define LSM303DLHC_MAGN_FS_SHIFT	5
#define LSM303DLHC_MAGN_FS_BITS		(CONFIG_LSM303DLHC_MAGN_RANGE << \
					LSM303DLHC_MAGN_FS_SHIFT)
#define LSM303DLHC_MAGN_CONT_UPDATE	0x00
#define LSM303DLHC_MAGN_DRDY		BIT(0)

#define LSM303DLHC_CRA_REG_M		0x00
#define LSM303DLHC_CRB_REG_M		0x01
#define LSM303DLHC_MR_REG_M		0x02
#define LSM303DLHC_REG_MAGN_X_LSB	0x03
#define LSM303DLHC_SR_REG_M		0x09

#define LSM303DLHC_MAGN_I2C_ADDR	CONFIG_LSM303DLHC_MAGN_I2C_ADDR
#define LSM303DLHC_MAGN_I2C_DEV		CONFIG_LSM303DLHC_MAGN_I2C_MASTER_DEV
struct lsm303dlhc_magn_data {
	struct device *i2c;

	s16_t magn_x;
	s16_t magn_y;
	s16_t magn_z;
};
#endif /* _SENSOR_LSM303DLHC_MAGN_ */
