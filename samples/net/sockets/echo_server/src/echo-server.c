/* echo-server.c - Networking echo server */

/*
 * Copyright (c) 2016 Intel Corporation.
 * Copyright (c) 2018 Nordic Semiconductor ASA.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(net_echo_server_sample, LOG_LEVEL_DBG);

#include <zephyr/kernel.h>
#include <zephyr/linker/sections.h>
#include <errno.h>
#include <zephyr/shell/shell.h>

#include <zephyr/net/net_core.h>
#include <zephyr/net/tls_credentials.h>

#include <zephyr/net/net_mgmt.h>
#include <zephyr/net/net_event.h>
#include <zephyr/net/dhcpv4.h>
#include <zephyr/net/dhcpv6.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/conn_mgr_monitor.h>

#include "common.h"
#include "certificate.h"

#define APP_BANNER "Run echo server"

static struct k_sem quit_lock;
static struct net_mgmt_event_callback mgmt_cb;
static bool connected;
K_SEM_DEFINE(run_app, 0, 1);
static bool want_to_quit;

#if defined(CONFIG_USERSPACE)
K_APPMEM_PARTITION_DEFINE(app_partition);
struct k_mem_domain app_domain;
#endif

#define EVENT_MASK (NET_EVENT_L4_CONNECTED | \
		    NET_EVENT_L4_DISCONNECTED)

APP_DMEM struct configs conf = {
	.ipv4 = {
		.proto = "IPv4",
	},
	.ipv6 = {
		.proto = "IPv6",
	},
};

void quit(void)
{
	k_sem_give(&quit_lock);
}

static void start_udp_and_tcp(void)
{
	LOG_INF("Starting...");

	if (IS_ENABLED(CONFIG_NET_TCP)) {
		start_tcp();
	}

	if (IS_ENABLED(CONFIG_NET_UDP)) {
		start_udp();
	}
}

static void stop_udp_and_tcp(void)
{
	LOG_INF("Stopping...");

	if (IS_ENABLED(CONFIG_NET_UDP)) {
		stop_udp();
	}

	if (IS_ENABLED(CONFIG_NET_TCP)) {
		stop_tcp();
	}
}

static void event_handler(struct net_mgmt_event_callback *cb,
			  uint32_t mgmt_event, struct net_if *iface)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(cb);

	if ((mgmt_event & EVENT_MASK) != mgmt_event) {
		return;
	}

	if (want_to_quit) {
		k_sem_give(&run_app);
		want_to_quit = false;
	}

	if (mgmt_event == NET_EVENT_L4_CONNECTED) {
		LOG_INF("Network connected");

		connected = true;
		k_sem_give(&run_app);

		return;
	}

	if (mgmt_event == NET_EVENT_L4_DISCONNECTED) {
		if (connected == false) {
			LOG_INF("Waiting network to be connected");
		} else {
			LOG_INF("Network disconnected");
			connected = false;
		}

		k_sem_reset(&run_app);

		return;
	}
}

static void dhcp_init_cb(struct net_if *iface, void *user_data)
{
	if (IS_ENABLED(CONFIG_NET_DHCPV4)) {
		net_dhcpv4_start(iface);
	}

	if (IS_ENABLED(CONFIG_NET_DHCPV6)) {
		struct net_dhcpv6_params params = {
			.request_addr = IS_ENABLED(CONFIG_NET_CONFIG_DHCPV6_REQUEST_ADDR),
			.request_prefix = IS_ENABLED(CONFIG_NET_CONFIG_DHCPV6_REQUEST_PREFIX),
		};

		net_dhcpv6_start(iface, &params);
	}
}

static int online_checker_setup_tls(struct net_if *iface,
				    const sec_tag_t **sec_tag_list,
				    size_t *sec_tag_size,
				    const char **tls_hostname,
				    const char *url,
				    const char *host,
				    const char *port,
				    struct sockaddr *dst,
				    void *user_data)
{
	ARG_UNUSED(iface);
	ARG_UNUSED(url);
	ARG_UNUSED(host);
	ARG_UNUSED(port);
	ARG_UNUSED(dst);
	ARG_UNUSED(user_data);

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
#define CA_CERTIFICATE_TAG 3

	/* We can re-use the echo-server certificates */
	static const sec_tag_t sec_tags[] = {
		CA_CERTIFICATE_TAG,
	};
	static bool setup_done;
	int ret;

	if (setup_done == false) {
		ret = tls_credential_add(CA_CERTIFICATE_TAG,
					 TLS_CREDENTIAL_CA_CERTIFICATE,
					 server_certificate,
					 sizeof(server_certificate));
		if (ret < 0) {
			LOG_ERR("Failed to register public certificate: %d", ret);
		}

		setup_done = true;
	}

	LOG_DBG("Returning %d security tag%s", ARRAY_SIZE(sec_tags),
		ARRAY_SIZE(sec_tags) > 1 ? "s" : "");

	*sec_tag_list = sec_tags;
	*sec_tag_size = sizeof(sec_tags);

	/* This works only for this sample and not to be used in real life
	 * environment.
	 */
	*tls_hostname = "localhost";

	return 0;
#else
	ARG_UNUSED(sec_tag_list);
	ARG_UNUSED(sec_tag_size);
	ARG_UNUSED(tls_hostname);

	return -ENOTSUP;
#endif
}

static void init_app(void)
{
	int ret;

#if defined(CONFIG_USERSPACE)
	struct k_mem_partition *parts[] = {
#if Z_LIBC_PARTITION_EXISTS
		&z_libc_partition,
#endif
		&app_partition
	};

	ret = k_mem_domain_init(&app_domain, ARRAY_SIZE(parts), parts);

	__ASSERT(ret == 0, "k_mem_domain_init() failed %d", ret);
	ARG_UNUSED(ret);
#endif

	k_sem_init(&quit_lock, 0, K_SEM_MAX_LIMIT);

	LOG_INF(APP_BANNER);

#if defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS)
	int err;

#if defined(CONFIG_NET_SAMPLE_CERTS_WITH_SC)
	err = tls_credential_add(SERVER_CERTIFICATE_TAG,
				 TLS_CREDENTIAL_CA_CERTIFICATE,
				 ca_certificate,
				 sizeof(ca_certificate));
	if (err < 0) {
		LOG_ERR("Failed to register CA certificate: %d", err);
	}
#endif /* defined(CONFIG_NET_SAMPLE_CERTS_WITH_SC) */

	err = tls_credential_add(SERVER_CERTIFICATE_TAG,
				 TLS_CREDENTIAL_SERVER_CERTIFICATE,
				 server_certificate,
				 sizeof(server_certificate));
	if (err < 0) {
		LOG_ERR("Failed to register public certificate: %d", err);
	}


	err = tls_credential_add(SERVER_CERTIFICATE_TAG,
				 TLS_CREDENTIAL_PRIVATE_KEY,
				 private_key, sizeof(private_key));
	if (err < 0) {
		LOG_ERR("Failed to register private key: %d", err);
	}

#if defined(CONFIG_MBEDTLS_KEY_EXCHANGE_PSK_ENABLED)
	err = tls_credential_add(PSK_TAG,
				TLS_CREDENTIAL_PSK,
				psk,
				sizeof(psk));
	if (err < 0) {
		LOG_ERR("Failed to register PSK: %d", err);
	}
	err = tls_credential_add(PSK_TAG,
				TLS_CREDENTIAL_PSK_ID,
				psk_id,
				sizeof(psk_id) - 1);
	if (err < 0) {
		LOG_ERR("Failed to register PSK ID: %d", err);
	}
#endif /* defined(CONFIG_MBEDTLS_KEY_EXCHANGE_PSK_ENABLED) */
#endif /* defined(CONFIG_NET_SOCKETS_SOCKOPT_TLS) */

	if (IS_ENABLED(CONFIG_NET_CONNECTION_MANAGER)) {
		net_mgmt_init_event_callback(&mgmt_cb,
					     event_handler, EVENT_MASK);
		net_mgmt_add_event_callback(&mgmt_cb);

		conn_mgr_mon_resend_status();
	}

	init_vlan();
	init_tunnel();
	init_ws();
	init_usb();

	/* If network settings library is disabled, then start DHCP
	 * for all available network interfaces.
	 */
	if (!IS_ENABLED(CONFIG_NET_CONFIG_SETTINGS)) {
		net_if_foreach(dhcp_init_cb, NULL);
	}

	if (IS_ENABLED(CONFIG_NET_CONNECTION_MANAGER_ONLINE_CONNECTIVITY_CHECK)) {
		ret = conn_mgr_register_online_checker_cb(online_checker_setup_tls,
							  NULL);
		if (ret < 0) {
			LOG_WRN("Cannot setup online checker TLS support (%d)", ret);
		}
	}
}

static int cmd_sample_quit(const struct shell *sh,
			  size_t argc, char *argv[])
{
	want_to_quit = true;

	conn_mgr_mon_resend_status();

	quit();

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sample_commands,
	SHELL_CMD(quit, NULL,
		  "Quit the sample application\n",
		  cmd_sample_quit),
	SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(sample, &sample_commands,
		   "Sample application commands", NULL);

int main(void)
{
	init_app();

	if (!IS_ENABLED(CONFIG_NET_CONNECTION_MANAGER)) {
		/* If the config library has not been configured to start the
		 * app only after we have a connection, then we can start
		 * it right away.
		 */
		k_sem_give(&run_app);
	}

	/* Wait for the connection. */
	k_sem_take(&run_app, K_FOREVER);

	start_udp_and_tcp();

	k_sem_take(&quit_lock, K_FOREVER);

	if (connected) {
		stop_udp_and_tcp();
	}
	return 0;
}
