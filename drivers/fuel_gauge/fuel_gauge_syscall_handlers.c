/*
 * Copyright 2023 Google LLC
 * Copyright (C) Microsoft Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/syscall_handler.h>
#include <zephyr/drivers/fuel_gauge.h>

static inline int z_vrfy_fuel_gauge_get_prop(const struct device *dev,
					     struct fuel_gauge_get_property *props,
					     size_t props_len)
{
	struct fuel_gauge_get_property k_props[props_len];

	Z_OOPS(Z_SYSCALL_DRIVER_FUEL_GAUGE(dev, get_property));

	Z_OOPS(z_user_from_copy(k_props, props,
				props_len * sizeof(struct fuel_gauge_get_property)));

	int ret = z_impl_fuel_gauge_get_prop(dev, k_props, props_len);

	Z_OOPS(z_user_to_copy(props, k_props, props_len * sizeof(struct fuel_gauge_get_property)));

	return ret;
}

#include <syscalls/fuel_gauge_get_prop_mrsh.c>

static inline int z_vrfy_fuel_gauge_get_block_prop(const struct device *dev,
						    struct fuel_gauge_get_block_property *prop,
						    void *dst, size_t dst_len)
{
	char k_buf[dst_len];
	struct fuel_gauge_get_block_property k_prop;

	Z_OOPS(Z_SYSCALL_DRIVER_FUEL_GAUGE(dev, get_block_property));

	Z_OOPS(z_user_from_copy(&k_prop, prop, 
				sizeof(struct fuel_gauge_get_block_property)));

	int ret = z_impl_fuel_gauge_get_block_prop(dev, &k_prop, k_buf, dst_len);

	Z_OOPS(z_user_to_copy(dst, k_buf, dst_len));

	Z_OOPS(z_user_to_copy(prop, &k_prop,
			      sizeof(struct fuel_gauge_get_block_property)));

	return ret;
}

#include <syscalls/fuel_gauge_get_block_prop_mrsh.c>

static inline int z_vrfy_fuel_gauge_set_prop(const struct device *dev,
					     struct fuel_gauge_set_property *props,
					     size_t props_len)
{
	struct fuel_gauge_set_property k_props[props_len];

	Z_OOPS(Z_SYSCALL_DRIVER_FUEL_GAUGE(dev, set_property));

	Z_OOPS(z_user_from_copy(k_props, props,
				props_len * sizeof(struct fuel_gauge_set_property)));

	int ret = z_impl_fuel_gauge_set_prop(dev, k_props, props_len);

	Z_OOPS(z_user_to_copy(props, k_props, props_len * sizeof(struct fuel_gauge_set_property)));

	return ret;
}

#include <syscalls/fuel_gauge_set_prop_mrsh.c>
