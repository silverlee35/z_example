/*
 * Copyright 2022 Google LLC
 * Copyright (C) Microsoft Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/fuel_gauge.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>
#include <zephyr/ztest.h>
#include <zephyr/ztest_assert.h>

struct sbs_gauge_new_api_fixture {
	const struct device *dev;
	const struct fuel_gauge_driver_api *api;
};

static void *sbs_gauge_new_api_setup(void)
{
	static ZTEST_DMEM struct sbs_gauge_new_api_fixture fixture;

	fixture.dev = DEVICE_DT_GET_ANY(sbs_sbs_gauge_new_api);

	k_object_access_all_grant(fixture.dev);

	zassert_true(device_is_ready(fixture.dev), "Fuel Gauge not found");

	return &fixture;
}

ZTEST_USER_F(sbs_gauge_new_api, test_get_all_props_failed_returns_negative)
{
	struct fuel_gauge_get_property props[] = {
		{
			/* Invalid property */
			.property_type = FUEL_GAUGE_PROP_MAX,
		},
	};

	int ret = fuel_gauge_get_prop(fixture->dev, props, ARRAY_SIZE(props));

	zassert_equal(props[0].status, -ENOTSUP, "Getting bad property %d has a good status.",
		      props[0].property_type);

	zassert_true(ret < 0);
}

ZTEST_USER_F(sbs_gauge_new_api, test_get_some_props_failed_returns_failed_prop_count)
{
	struct fuel_gauge_get_property props[] = {
		{
			/* First invalid property */
			.property_type = FUEL_GAUGE_PROP_MAX,
		},
		{
			/* Second invalid property */
			.property_type = FUEL_GAUGE_PROP_MAX,
		},
		{
			/* Valid property */
			.property_type = FUEL_GAUGE_VOLTAGE,
		},

	};

	int ret = fuel_gauge_get_prop(fixture->dev, props, ARRAY_SIZE(props));

	zassert_equal(props[0].status, -ENOTSUP, "Getting bad property %d has a good status.",
		      props[0].property_type);

	zassert_equal(props[1].status, -ENOTSUP, "Getting bad property %d has a good status.",
		      props[1].property_type);

	zassert_ok(props[2].status, "Property %d getting %d has a bad status.", 2,
		   props[2].property_type);

	zassert_equal(ret, 2);
}

ZTEST_USER_F(sbs_gauge_new_api, test_set_all_props_failed_returns_negative)
{
	struct fuel_gauge_set_property props[] = {
		{
			/* Invalid property */
			.property_type = FUEL_GAUGE_PROP_MAX,
		},
	};

	int ret = fuel_gauge_set_prop(fixture->dev, props, ARRAY_SIZE(props));

	zassert_equal(props[0].status, -ENOTSUP, "Setting bad property %d has a good status.",
		      props[0].property_type);

	zassert_true(ret < 0);
}

ZTEST_USER_F(sbs_gauge_new_api, test_set_some_props_failed_returns_failed_prop_count)
{
	struct fuel_gauge_set_property props[] = {
		{
			/* First invalid property */
			.property_type = FUEL_GAUGE_PROP_MAX,
		},
		{
			/* Second invalid property */
			.property_type = FUEL_GAUGE_PROP_MAX,
		},
		{
			/* Valid property */
			.property_type = FUEL_GAUGE_SBS_MFR_ACCESS,
			/* Set Manufacturer's Access to arbitrary word */
			.value.sbs_mfr_access_word = 1,
		},

	};

	int ret = fuel_gauge_set_prop(fixture->dev, props, ARRAY_SIZE(props));

	zassert_equal(props[0].status, -ENOTSUP, "Setting bad property %d has a good status.",
		      props[0].property_type);

	zassert_equal(props[1].status, -ENOTSUP, "Setting bad property %d has a good status.",
		      props[1].property_type);

	zassert_ok(props[2].status, "Property %d setting %d has a bad status.", 2,
		   props[2].property_type);

	zassert_equal(ret, 2);
}

ZTEST_USER_F(sbs_gauge_new_api, test_set_prop_can_be_get)
{
	uint16_t word = BIT(15) | BIT(0);
	struct fuel_gauge_set_property set_prop = {
		/* Valid property */
		.property_type = FUEL_GAUGE_SBS_MFR_ACCESS,
		/* Set Manufacturer's Access to 16 bit word*/
		.value.sbs_mfr_access_word = word,
	};

	zassert_ok(fuel_gauge_set_prop(fixture->dev, &set_prop, 1));
	zassert_ok(set_prop.status);

	struct fuel_gauge_get_property get_prop = {
		.property_type = FUEL_GAUGE_SBS_MFR_ACCESS,
	};

	zassert_ok(fuel_gauge_get_prop(fixture->dev, &get_prop, 1));
	zassert_ok(get_prop.status);
	zassert_equal(get_prop.value.sbs_mfr_access_word, word);
}

ZTEST_USER_F(sbs_gauge_new_api, test_get_props__returns_ok)
{
	/* Validate what props are supported by the driver */
	struct fuel_gauge_get_property props[] = {
		{
			.property_type = FUEL_GAUGE_VOLTAGE,
		},
		{
			.property_type = FUEL_GAUGE_CURRENT,
		},
		{
			.property_type = FUEL_GAUGE_AVG_CURRENT,
		},
		{
			.property_type = FUEL_GAUGE_TEMPERATURE,
		},
		{
			.property_type = FUEL_GAUGE_STATE_OF_CHARGE,
		},
		{
			.property_type = FUEL_GAUGE_RUNTIME_TO_FULL,
		},
		{
			.property_type = FUEL_GAUGE_RUNTIME_TO_EMPTY,
		},
		{
			.property_type = FUEL_GAUGE_REMAINING_CAPACITY,
		},
		{
			.property_type = FUEL_GAUGE_FULL_CHARGE_CAPACITY,
		},
		{
			.property_type = FUEL_GAUGE_CYCLE_COUNT,
		},
		{
			.property_type = FUEL_GAUGE_SBS_MFR_ACCESS,
		},
		{
			.property_type = FUEL_GAUGE_SBS_MODE,
		},
		{
			.property_type = FUEL_GAUGE_CHARGE_CURRENT,
		},
		{
			.property_type = FUEL_GAUGE_CHARGE_VOLTAGE,
		},
		{
			.property_type = FUEL_GAUGE_STATUS,
		},
		{
			.property_type = FUEL_GAUGE_DESIGN_CAPACITY,
		},
		{
			.property_type = FUEL_GAUGE_DESIGN_VOLTAGE,
		},
		{
			.property_type = FUEL_GAUGE_SBS_ATRATE,
		},
		{
			.property_type = FUEL_GAUGE_SBS_ATRATE_TIME_TO_FULL,
		},
		{
			.property_type = FUEL_GAUGE_SBS_ATRATE_TIME_TO_EMPTY,
		},
		{
			.property_type = FUEL_GAUGE_SBS_ATRATE_OK,
		},
	};

	int ret = fuel_gauge_get_prop(fixture->dev, props, ARRAY_SIZE(props));

	for (int i = 0; i < ARRAY_SIZE(props); i++) {
		zassert_ok(props[i].status, "Property %d getting %d has a bad status.", i,
			   props[i].property_type);
	}
	zassert_ok(ret);
}

ZTEST_USER_F(sbs_gauge_new_api, test_get_block_props__returns_ok)
{
	/* Validate what properties are supported by the driver */
	struct fuel_gauge_get_block_property prop;
	struct sbs_gauge_manufacturer_name mfg_name;
	struct sbs_gauge_device_name dev_name;
	struct sbs_gauge_device_chemistry chem;

	prop.property_type = FUEL_GAUGE_MANUFACTURER_NAME;
	int ret = fuel_gauge_get_block_prop(fixture->dev, &prop, &mfg_name, sizeof(mfg_name));
	zassert_ok(prop.status, "Property %d has a bad status.", prop.property_type);
	zassert_ok(ret);

	prop.property_type = FUEL_GAUGE_DEVICE_NAME;
	ret = fuel_gauge_get_block_prop(fixture->dev, &prop, &dev_name, sizeof(dev_name));
	zassert_ok(prop.status, "Property %d has a bad status.", prop.property_type);
	zassert_ok(ret);

	prop.property_type = FUEL_GAUGE_DEVICE_CHEMISTRY;
	ret = fuel_gauge_get_block_prop(fixture->dev, &prop, &chem, sizeof(chem));
	zassert_ok(prop.status, "Property %d has a bad status.", prop.property_type);
	zassert_ok(ret);
}

ZTEST_USER_F(sbs_gauge_new_api, test_set_props__returns_ok)
{
	/* Validate what props are supported by the driver */

	struct fuel_gauge_set_property props[] = {
		{
			.property_type = FUEL_GAUGE_SBS_MFR_ACCESS,
		},
		{
			.property_type = FUEL_GAUGE_SBS_MODE,
		},
		{
			.property_type = FUEL_GAUGE_SBS_ATRATE,
		},
	};

	int ret = fuel_gauge_set_prop(fixture->dev, props, ARRAY_SIZE(props));

	for (int i = 0; i < ARRAY_SIZE(props); i++) {
		zassert_ok(props[i].status, "Property %d writing %d has a bad status.", i,
			   props[i].property_type);
	}

	zassert_ok(ret);
}

ZTEST_SUITE(sbs_gauge_new_api, NULL, sbs_gauge_new_api_setup, NULL, NULL, NULL);
