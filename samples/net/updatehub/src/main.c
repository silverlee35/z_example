/*
 * Copyright (c) 2018 O.S.Systems
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <updatehub.h>
#include <dfu/mcuboot.h>
#include <sys/printk.h>
#include <logging/log.h>

#if defined(CONFIG_UPDATEHUB_DTLS)
#include <net/tls_credentials.h>
#include "c_certificates.h"
#endif


#include <device.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

#ifndef DT_ALIAS_LED0_GPIOS_FLAGS
#define FLAGS 0
#else
#define FLAGS DT_ALIAS_LED0_GPIOS_FLAGS
#endif

/* Make sure the board's devicetree declares led0 in its /aliases. */
#ifdef DT_ALIAS_LED0_GPIOS_CONTROLLER
#define LED0	DT_ALIAS_LED0_GPIOS_CONTROLLER
#define PIN	DT_ALIAS_LED0_GPIOS_PIN
#else
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#endif

LOG_MODULE_REGISTER(main);

void main(void)
{
	int ret = -1;

	LOG_INF("UpdateHub sample app started");


#if defined(CONFIG_UPDATEHUB_DTLS)
	if (tls_credential_add(CA_CERTIFICATE_TAG,
			       TLS_CREDENTIAL_SERVER_CERTIFICATE,
			       server_certificate,
			       sizeof(server_certificate)) < 0) {
		LOG_ERR("Failed to register server certificate");
		return;
	}

	if (tls_credential_add(CA_CERTIFICATE_TAG,
			       TLS_CREDENTIAL_PRIVATE_KEY,
			       private_key,
			       sizeof(private_key)) < 0) {
		LOG_ERR("Failed to register private key");
		return;
	} 
#endif

	/* The image of application needed be confirmed */
	LOG_INF("Confirming the boot image");
	ret = boot_write_img_confirmed();
	if (ret < 0) {
		LOG_ERR("Error to confirm the image");
	}

#if defined(CONFIG_UPDATEHUB_POLLING)
	LOG_INF("Starting UpdateHub polling mode");
	updatehub_autohandler();
#endif 

#if defined(CONFIG_UPDATEHUB_MANUAL)
	LOG_INF("Starting UpdateHub manual mode");

	enum updatehub_response resp;

	switch (updatehub_probe()) {
	case UPDATEHUB_HAS_UPDATE:
		switch (updatehub_update()) {
		case UPDATEHUB_OK:
			ret = 0;
			sys_reboot(SYS_REBOOT_WARM);
			break;

		default:
			LOG_ERR("Error installing update.");
			break;
		}

	case UPDATEHUB_NO_UPDATE:
		LOG_INF("No update found");
		ret = 0;
		break;

	default:
		LOG_ERR("Invalid response");
		break;
	} 
#endif


/*
	struct device *dev;
	bool led_is_on = true;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		LOG_INF("No LED device found");
		return;
	}
	LOG_INF("LED device found");
	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev, PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_sleep(SLEEP_TIME_MS);
	}*/
}

