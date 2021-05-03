/*
 * Copyright (c) 2020 arithmetics.io
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <sys/util.h>
#include <drivers/sensor.h>
#include "drivers/sensor/fdc2x1x.h"
#include <stdio.h>

#define CH_BUF_INIT(m)          {},

K_SEM_DEFINE(sem, 0, 1);

#ifdef CONFIG_FDC2X1X_TRIGGER
static void trigger_handler(const struct device *dev,
			    struct sensor_trigger *trigger)
{
	switch (trigger->type) {
	case SENSOR_TRIG_DATA_READY:
		if (sensor_sample_fetch(dev)) {
			printk("Sample fetch error\n");
			return;
		}
		k_sem_give(&sem);
		break;
	default:
		printk("Unknown trigger\n");
	}
}
#endif

#ifdef CONFIG_PM_DEVICE
static void pm_cb(const struct device *dev,
		  int status,
		  void *context,
		  void *arg)
{
	ARG_UNUSED(dev);
	ARG_UNUSED(arg);

	switch (*(uint32_t *)context) {
	case PM_DEVICE_ACTIVE_STATE:
		printk("Enter ACTIVE_STATE ");
		break;
	case PM_DEVICE_LOW_POWER_STATE:
		printk("Enter LOW_POWER_STATE ");
		break;
	case PM_DEVICE_OFF_STATE:
		printk("Enter OFF_STATE ");
		break;
	}

	if (status) {
		printk("Fail\n");
	} else {
		printk("Success\n");
	}
}
#endif

void main(void)
{
	struct sensor_value ch_buf[] = {
		DT_FOREACH_CHILD(DT_INST(0, ti_fdc2x1x), CH_BUF_INIT)
	};

	uint8_t num_ch = ARRAY_SIZE(ch_buf);
	enum sensor_channel base;
	int i;

	const struct device *dev =
		device_get_binding(DT_LABEL(DT_INST(0, ti_fdc2x1x)));

	if (dev == NULL) {
		printk("Could not get %s device\n",
		       DT_LABEL(DT_INST(0, ti_fdc2x1x)));
		return;
	}

#ifdef CONFIG_FDC2X1X_TRIGGER
	struct sensor_trigger trig = {
		.type = SENSOR_TRIG_DATA_READY,
		.chan = SENSOR_CHAN_ALL,
	};

	if (sensor_trigger_set(dev, &trig, trigger_handler)) {
		printk("Could not set trigger\n");
		return;
	}
#endif

#ifdef CONFIG_PM_DEVICE
	/* Testing the power modes */
	uint32_t p_state;

	p_state = PM_DEVICE_LOW_POWER_STATE;
	device_set_power_state(dev, p_state, pm_cb, NULL);

	p_state = PM_DEVICE_OFF_STATE;
	device_set_power_state(dev, p_state, pm_cb, NULL);

	p_state = PM_DEVICE_ACTIVE_STATE;
	device_set_power_state(dev, p_state, pm_cb, NULL);
#endif

	while (1) {
#ifdef CONFIG_FDC2X1X_TRIGGER
		k_sem_take(&sem, K_FOREVER);
#else
		if (sensor_sample_fetch(dev) < 0) {
			printk("Sample fetch failed\n");
			return;
		}
#endif
		base = SENSOR_CHAN_FDC2X1X_FREQ_CH0;
		for (i = 0; i < num_ch; i++) {
			sensor_channel_get(dev, base++, &ch_buf[i]);
			printf("ch%d: %f MHz ", i, sensor_value_to_double(&ch_buf[i]));
		}
		printf("\n");

		base = SENSOR_CHAN_FDC2X1X_CAPACITANCE_CH0;
		for (i = 0; i < num_ch; i++) {
			sensor_channel_get(dev, base++, &ch_buf[i]);
			printf("ch%d: %f pF ", i, sensor_value_to_double(&ch_buf[i]));
		}
		printf("\n\n");


#ifdef CONFIG_PM_DEVICE
		p_state = PM_DEVICE_OFF_STATE;
		device_set_power_state(dev, p_state, pm_cb, NULL);
		k_sleep(K_MSEC(2000));
		p_state = PM_DEVICE_ACTIVE_STATE;
		device_set_power_state(dev, p_state, pm_cb, NULL);
#elif CONFIG_FDC2X1X_TRIGGER_NONE
		k_sleep(K_MSEC(100));
#endif
	}
}
