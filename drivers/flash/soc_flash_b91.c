/*
 * Copyright (c) 2021 Telink Semiconductor
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT telink_b91_flash_controller
#define FLASH_SIZE DT_REG_SIZE(DT_INST(0, soc_nv_flash))

#include "flash.h"
#include <device.h>
#include <drivers/flash.h>


/* driver data structure */
struct flash_b91_data {
	struct k_sem write_lock;
};

/* driver parameters structure */
static const struct flash_parameters flash_b91_parameters =
{
	.write_block_size = DT_PROP(DT_INST(0, soc_nv_flash), write_block_size),
	.erase_value = 0xff,
};


/* Check for correct offset and length */
static bool flash_b91_is_range_valid(off_t offset, size_t len)
{
	/* check for min value */
	if ((offset < 0) || (len < 1)) {
		return false;
	}

	/* check for max value */
	if ((offset + len) > FLASH_SIZE) {
		return false;
	}

	return true;
}

/* API implementation: driver initialization */
static int flash_b91_init(const struct device *dev)
{
	struct flash_b91_data *dev_data = dev->data;

	k_sem_init(&dev_data->write_lock, 1, 1);

	return 0;
}

/* API implementation: erase */
static int flash_b91_erase(const struct device *dev, off_t offset, size_t len)
{
	int page_nums = len / PAGE_SIZE;
	struct flash_b91_data *dev_data = dev->data;

	/* check for valid length */
	if (!len) {
		return 0;
	}

	/* check for valid range */
	if (!flash_b91_is_range_valid(offset, len)) {
		return -EINVAL;
	}

	/* Erase can be done only by pages */
	if (((offset % PAGE_SIZE) != 0) || ((len % PAGE_SIZE) != 0)) {
		return -EINVAL;
	}

	/* take semaphore */
	if (k_sem_take(&dev_data->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	/* erase flash page by page */
	for (int i = 0; i < page_nums; i++) {
		flash_erase_page(offset);
		offset += PAGE_SIZE;
	}

	/* release semaphore */
	k_sem_give(&dev_data->write_lock);

	return 0;
}

/* API implementation: write */
static int flash_b91_write(const struct device *dev, off_t offset,
			   const void *data, size_t len)
{
	struct flash_b91_data *dev_data = dev->data;

	/* check for valid length */
	if (!len) {
		return 0;
	}

	/* check for valid range */
	if (!flash_b91_is_range_valid(offset, len)) {
		return -EINVAL;
	}

	/* take semaphore */
	if (k_sem_take(&dev_data->write_lock, K_NO_WAIT)) {
		return -EACCES;
	}

	/* write flash */
	flash_write_page(offset, len, (unsigned char *)data);

	/* release semaphore */
	k_sem_give(&dev_data->write_lock);

	return 0;
}

/* API implementation: read */
static int flash_b91_read(const struct device *dev, off_t offset,
			  void *data, size_t len)
{
	ARG_UNUSED(dev);

	/* check for valid length */
	if (!len) {
		return 0;
	}

	/* check for valid range */
	if (!flash_b91_is_range_valid(offset, len)) {
		return -EINVAL;
	}

	/* read flash */
	flash_read_page(offset, len, (unsigned char *)data);

	return 0;
}

/* API implementation: write_protection */
static int flash_b91_write_protection(const struct device *dev, bool enable)
{
	int status = 0;
	struct flash_b91_data *dev_data = dev->data;

	if (enable) {
		status = k_sem_take(&dev_data->write_lock, K_FOREVER);
	} else {
		k_sem_give(&dev_data->write_lock);
	}

	return status;
}

/* API implementation: get_parameters */
static const struct flash_parameters *
flash_b91_get_parameters(const struct device *dev)
{
	ARG_UNUSED(dev);

	return &flash_b91_parameters;
}

/* API implementation: page_layout */
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
static const struct flash_pages_layout dev_layout = {
	.pages_count = FLASH_SIZE / PAGE_SIZE,
	.pages_size = PAGE_SIZE,
};

static void flash_b91_pages_layout(const struct device *dev,
				   const struct flash_pages_layout **layout,
				   size_t *layout_size)
{
	*layout = &dev_layout;
	*layout_size = 1;
}
#endif /* CONFIG_FLASH_PAGE_LAYOUT */

static struct flash_b91_data flash_data;

static const struct flash_driver_api flash_b91_api = {
	.erase = flash_b91_erase,
	.write = flash_b91_write,
	.read = flash_b91_read,
	.write_protection = flash_b91_write_protection,
	.get_parameters = flash_b91_get_parameters,
#if defined(CONFIG_FLASH_PAGE_LAYOUT)
	.page_layout = flash_b91_pages_layout,
#endif
};

/* Driver registration */
DEVICE_DT_INST_DEFINE(0, flash_b91_init,
		      NULL, &flash_data, NULL, POST_KERNEL,
		      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &flash_b91_api);
