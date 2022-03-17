/*
 * Copyright (c) 2017 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LIS2DH_LIS2DH_H_
#define ZEPHYR_DRIVERS_SENSOR_LIS2DH_LIS2DH_H_

#include <kernel.h>
#include <device.h>
#include <sys/util.h>
#include <stdint.h>
#include <drivers/gpio.h>
#include <drivers/sensor.h>
#include <string.h>
#include <sys/ring_buffer.h>

#define LIS2DH_FIFO_SZ			(32 * 6)
#define LIS2DH_FIFO_RB_SZ		(LIS2DH_FIFO_SZ * 2)
#define LIS2DH_REG_WAI			0x0f
#define LIS2DH_CHIP_ID			0x33

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
#include <drivers/spi.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
#include <drivers/i2c.h>
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c) */

#define LIS2DH_AUTOINCREMENT_ADDR	BIT(7)

#define LIS2DH_REG_CTRL0		0x1e
#define LIS2DH_SDO_PU_DISC_SHIFT	7
#define LIS2DH_SDO_PU_DISC_MASK		BIT(LIS2DH_SDO_PU_DISC_SHIFT)

#define LIS2DH_REG_CTRL1		0x20
#define LIS2DH_ACCEL_XYZ_SHIFT		0
#define LIS2DH_ACCEL_X_EN_BIT		BIT(0)
#define LIS2DH_ACCEL_Y_EN_BIT		BIT(1)
#define LIS2DH_ACCEL_Z_EN_BIT		BIT(2)
#define LIS2DH_ACCEL_EN_BITS		(LIS2DH_ACCEL_X_EN_BIT | \
					LIS2DH_ACCEL_Y_EN_BIT | \
					LIS2DH_ACCEL_Z_EN_BIT)
#define LIS2DH_ACCEL_XYZ_MASK		BIT_MASK(3)

#define LIS2DH_LP_EN_BIT_MASK		BIT(3)
#if defined(CONFIG_LIS2DH_OPER_MODE_LOW_POWER)
	#define LIS2DH_LP_EN_BIT	BIT(3)
#else
	#define LIS2DH_LP_EN_BIT	0
#endif

#define LIS2DH_SUSPEND			0

#define LIS2DH_ODR_1			1
#define LIS2DH_ODR_2			2
#define LIS2DH_ODR_3			3
#define LIS2DH_ODR_4			4
#define LIS2DH_ODR_5			5
#define LIS2DH_ODR_6			6
#define LIS2DH_ODR_7			7
#define LIS2DH_ODR_8			8
#define LIS2DH_ODR_9			9

#if defined(CONFIG_LIS2DH_ODR_1)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_1
#elif defined(CONFIG_LIS2DH_ODR_2)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_2
#elif defined(CONFIG_LIS2DH_ODR_3)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_3
#elif defined(CONFIG_LIS2DH_ODR_4) || defined(CONFIG_LIS2DH_ODR_RUNTIME)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_4
#elif defined(CONFIG_LIS2DH_ODR_5)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_5
#elif defined(CONFIG_LIS2DH_ODR_6)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_6
#elif defined(CONFIG_LIS2DH_ODR_7)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_7
#elif defined(CONFIG_LIS2DH_ODR_8)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_8
#elif defined(CONFIG_LIS2DH_ODR_9_NORMAL) || defined(CONFIG_LIS2DH_ODR_9_LOW)
	#define LIS2DH_ODR_IDX		LIS2DH_ODR_9
#endif

#define LIS2DH_ODR_SHIFT		4
#define LIS2DH_ODR_RATE(r)		((r) << LIS2DH_ODR_SHIFT)
#define LIS2DH_ODR_BITS			(LIS2DH_ODR_RATE(LIS2DH_ODR_IDX))
#define LIS2DH_ODR_MASK			(BIT_MASK(4) << LIS2DH_ODR_SHIFT)

#define LIS2DH_REG_CTRL2		0x21
#define LIS2DH_HPIS2_EN_BIT		BIT(1)
#define LIS2DH_FDS_EN_BIT		BIT(3)

#define LIS2DH_REG_CTRL3		0x22
#define LIS2DH_EN_DRDY1_INT1_SHIFT	4
#define LIS2DH_EN_DRDY1_INT1		BIT(LIS2DH_EN_DRDY1_INT1_SHIFT)
#define LIS2DH_EN_WTM_INT1		BIT(2)
#define LIS2DH_EN_OVRUN_INT1		BIT(1)

#define LIS2DH_REG_CTRL4		0x23
#define LIS2DH_FS_SHIFT			4
#define LIS2DH_FS_MASK			(BIT_MASK(2) << LIS2DH_FS_SHIFT)

#if defined(CONFIG_LIS2DH_ACCEL_RANGE_2G) ||\
	defined(CONFIG_LIS2DH_ACCEL_RANGE_RUNTIME)
	#define LIS2DH_FS_IDX		0
#elif defined(CONFIG_LIS2DH_ACCEL_RANGE_4G)
	#define LIS2DH_FS_IDX		1
#elif defined(CONFIG_LIS2DH_ACCEL_RANGE_8G)
	#define LIS2DH_FS_IDX		2
#elif defined(CONFIG_LIS2DH_ACCEL_RANGE_16G)
	#define LIS2DH_FS_IDX		3
#endif

#define LIS2DH_FS_SELECT(fs)		((fs) << LIS2DH_FS_SHIFT)
#define LIS2DH_FS_BITS			(LIS2DH_FS_SELECT(LIS2DH_FS_IDX))
#if defined(CONFIG_LIS2DH_OPER_MODE_HIGH_RES)
	#define LIS2DH_HR_BIT		BIT(3)
#else
	#define LIS2DH_HR_BIT		0
#endif

#define LIS2DH_REG_CTRL5		0x24
#define LIS2DH_LIR_INT2_SHIFT		1
#define LIS2DH_LIR_INT1_SHIFT		3
#define LIS2DH_FIFO_EN_SHIFT		6
#define LIS2DH_EN_LIR_INT2		BIT(LIS2DH_LIR_INT2_SHIFT)
#define LIS2DH_EN_LIR_INT1		BIT(LIS2DH_LIR_INT1_SHIFT)
#define LIS2DH_FIFO_EN			BIT(LIS2DH_FIFO_EN_SHIFT)

#define LIS2DH_REG_CTRL6		0x25
#define LIS2DH_EN_INT2_INT2_SHIFT	5
#define LIS2DH_EN_INT2_INT2		BIT(LIS2DH_EN_INT2_INT2_SHIFT)
#define LIS2DH_EN_INT1_INT1_SHIFT	6
#define LIS2DH_EN_INT1_INT1		BIT(LIS2DH_EN_INT1_INT1_SHIFT)

#define LIS2DH_REG_REFERENCE		0x26

#define LIS2DH_REG_STATUS		0x27
#define LIS2DH_STATUS_ZYZ_OVR		BIT(7)
#define LIS2DH_STATUS_Z_OVR		BIT(6)
#define LIS2DH_STATUS_Y_OVR		BIT(5)
#define LIS2DH_STATUS_X_OVR		BIT(4)
#define LIS2DH_STATUS_OVR_MASK		(BIT_MASK(4) << 4)
#define LIS2DH_STATUS_ZYX_DRDY		BIT(3)
#define LIS2DH_STATUS_Z_DRDY		BIT(2)
#define LIS2DH_STATUS_Y_DRDY		BIT(1)
#define LIS2DH_STATUS_X_DRDY		BIT(0)
#define LIS2DH_STATUS_DRDY_MASK		BIT_MASK(4)

#define LIS2DH_REG_ACCEL_X_LSB		0x28
#define LIS2DH_REG_ACCEL_Y_LSB		0x2A
#define LIS2DH_REG_ACCEL_Z_LSB		0x2C
#define LIS2DH_REG_ACCEL_X_MSB		0x29
#define LIS2DH_REG_ACCEL_Y_MSB		0x2B
#define LIS2DH_REG_ACCEL_Z_MSB		0x2D

#define LIS2DH_FIFO_CTRL_REG		0x2E
#define LIS2DH_FIFO_CTRL_FTH_MASK	BIT_MASK(5)
#define LIS2DH_FIFO_CTRL_FTH(n)		n
#define LIS2DH_FIFO_CTRL_TR_MASK	BIT(5)
#define LIS2DH_FIFO_CTRL_TR_INT1	0
#define LIS2DH_FIFO_CTRL_TR_INT2	BIT(5)
#define LIS2DH_FIFO_CTRL_MODE_MASK	(BIT_MASK(2) << 6)
#define LIS2DH_FIFO_CTRL_MODE_BYPASS	0
#define LIS2DH_FIFO_CTRL_MODE_FIFO	BIT(6)
#define LIS2DH_FIFO_CTRL_MODE_STREAM	BIT(7)
#define LIS2DH_FIFO_CTRL_MODE_TRIGGER	(BIT_MASK(2) << 6)

#define LIS2DH_FIFO_SRC_REG		0x2F
#define LIS2DH_FIFO_SRC_WTM		BIT(7)
#define LIS2DH_FIFO_SRC_OVRN_FIFO	BIT(6)
#define LIS2DH_FIFO_SRC_EMPTY		BIT(5)
#define LIS2DH_FIFO_SRC_FSS4		BIT(4)
#define LIS2DH_FIFO_SRC_FSS3		BIT(3)
#define LIS2DH_FIFO_SRC_FSS2		BIT(2)
#define LIS2DH_FIFO_SRC_FSS1		BIT(1)
#define LIS2DH_FIFO_SRC_FSS0		BIT(0)
#define LIS2DH_FIFO_SRC_N_MASK		BIT_MASK(5)

#define LIS2DH_REG_INT1_CFG		0x30
#define LIS2DH_REG_INT1_SRC		0x31
#define LIS2DH_REG_INT1_THS		0x32
#define LIS2DH_REG_INT1_DUR		0x33
#define LIS2DH_REG_INT2_CFG		0x34
#define LIS2DH_REG_INT2_SRC		0x35
#define LIS2DH_REG_INT2_THS		0x36
#define LIS2DH_REG_INT2_DUR		0x37

#define LIS2DH_AOI_CFG			BIT(7)
#define LIS2DH_INT_CFG_ZHIE_ZUPE	BIT(5)
#define LIS2DH_INT_CFG_ZLIE_ZDOWNE	BIT(4)
#define LIS2DH_INT_CFG_YHIE_YUPE	BIT(3)
#define LIS2DH_INT_CFG_YLIE_YDOWNE	BIT(2)
#define LIS2DH_INT_CFG_XHIE_XUPE	BIT(1)
#define LIS2DH_INT_CFG_XLIE_XDOWNE	BIT(0)

/* sample buffer size includes status register */
#define LIS2DH_BUF_SZ			7

#define LIS2DH_CTRL4_BDU_BIT    BIT(7)

union lis2dh_sample {
	uint8_t raw[LIS2DH_BUF_SZ];
	struct {
		uint8_t status;
		int16_t xyz[3];
	} __packed;
};

union lis2dh_bus_cfg {
#if DT_ANY_INST_ON_BUS_STATUS_OKAY(i2c)
	uint16_t i2c_slv_addr;
#endif

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_dt_spec spi;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

struct temperature {
	uint8_t cfg_addr;
	uint8_t enable_mask;
	uint8_t dout_addr;
	uint8_t fractional_bits;
};

struct lis2dh_config {
	const char *bus_name;
	int (*bus_init)(const struct device *dev);
	const union lis2dh_bus_cfg bus_cfg;
#ifdef CONFIG_LIS2DH_TRIGGER
	const struct gpio_dt_spec gpio_drdy;
	const struct gpio_dt_spec gpio_int;
#endif /* CONFIG_LIS2DH_TRIGGER */
	struct {
		bool is_lsm303agr_dev : 1;
		bool disc_pull_up : 1;
		bool anym_on_int1 : 1;
	} hw;
#ifdef CONFIG_LIS2DH_MEASURE_TEMPERATURE
	const struct temperature temperature;
#endif
};

struct lis2dh_transfer_function {
	int (*read_data)(const struct device *dev, uint8_t reg_addr,
			 uint8_t *value, uint8_t len);
	int (*write_data)(const struct device *dev, uint8_t reg_addr,
			  uint8_t *value, uint8_t len);
	int (*read_reg)(const struct device *dev, uint8_t reg_addr,
			uint8_t *value);
	int (*write_reg)(const struct device *dev, uint8_t reg_addr,
			 uint8_t value);
	int (*update_reg)(const struct device *dev, uint8_t reg_addr,
			  uint8_t mask, uint8_t value);
};

struct lis2dh_data {
	const struct device *bus;
	const struct lis2dh_transfer_function *hw_tf;

	union lis2dh_sample sample;
	/* current scaling factor, in micro m/s^2 / lsb */
	uint32_t scale;

#ifdef CONFIG_LIS2DH_MEASURE_TEMPERATURE
	struct sensor_value temperature;
#endif

#ifdef CONFIG_PM_DEVICE
	uint8_t reg_ctrl1_active_val;
#endif

#ifdef CONFIG_LIS2DH_TRIGGER
	const struct device *dev;
	struct gpio_callback gpio_int1_cb;
	struct gpio_callback gpio_int2_cb;

	sensor_trigger_handler_t handler_drdy;
	sensor_trigger_handler_t handler_anymotion;
	atomic_t trig_flags;
	enum sensor_channel chan_drdy;

#if defined(CONFIG_LIS2DH_TRIGGER_OWN_THREAD)
	K_KERNEL_STACK_MEMBER(thread_stack, CONFIG_LIS2DH_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_LIS2DH_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
#endif

#endif /* CONFIG_LIS2DH_TRIGGER */

#if defined(CONFIG_LIS2DH_FIFO_MODE)
	struct ring_buf fifo_rb;
	uint8_t rb_storage[LIS2DH_FIFO_RB_SZ];
#endif /* CONFIG_LIS2DH_FIFO_MODE */

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
	struct spi_cs_control cs_ctrl;
#endif /* DT_ANY_INST_ON_BUS_STATUS_OKAY(spi) */
};

#if DT_ANY_INST_ON_BUS_STATUS_OKAY(spi)
int lis2dh_spi_access(struct lis2dh_data *ctx, uint8_t cmd,
		      void *data, size_t length);
#endif

#ifdef CONFIG_LIS2DH_TRIGGER
int lis2dh_trigger_set(const struct device *dev,
		       const struct sensor_trigger *trig,
		       sensor_trigger_handler_t handler);

int lis2dh_init_interrupt(const struct device *dev);

int lis2dh_acc_slope_config(const struct device *dev,
			    enum sensor_attribute attr,
			    const struct sensor_value *val);
#endif

int lis2dh_spi_init(const struct device *dev);
int lis2dh_i2c_init(const struct device *dev);


#endif /* __SENSOR_LIS2DH__ */
