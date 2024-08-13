/*
 * Copyright 2024 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/kernel.h>

#include <zephyr/settings/settings.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/classic/spp.h>

#include <zephyr/shell/shell.h>

#include "bt.h"

#define HELP_NONE "[none]"

static int spp_inited;
static uint8_t spp_id;

static const uint8_t spp_uuid[16] = {0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80,
				     0x00, 0x10, 0x00, 0x00, 0x66, 0x66, 0x00, 0x00};

void spp_connected_cb(struct bt_conn *conn, uint8_t sid)
{
	shell_print(ctx_shell, "spp connected conn:%p, spp_id: %d", conn, sid);
	spp_id = sid;
}

void spp_disconnected_cb(struct bt_conn *conn, uint8_t sid)
{
	shell_print(ctx_shell, "spp disconnected conn:%p, spp_id: %d", conn, sid);
	spp_id = 0;
}

void spp_recv_cb(struct bt_conn *conn, uint8_t sid, uint8_t *data, uint16_t len)
{
	shell_print(ctx_shell, "spp recv spp_id %d, data len:%d", sid, len);
}

static struct bt_spp_cb spp_cb = {
	.connected = spp_connected_cb,
	.disconnected = spp_disconnected_cb,
	.recv = spp_recv_cb,
};

static int cmd_register_cb(const struct shell *sh, int32_t argc, char *argv[])
{
	int ret = -1;

	if (spp_inited) {
		shell_print(sh, "already registered");
		return ret;
	}

	spp_inited = 1;
	ret = bt_spp_register_cb(&spp_cb);
	if (ret != 0) {
		shell_print(sh, "fail");
		return ret;
	}

	shell_print(sh, "success");
	return ret;
}

static int cmd_register_srv(const struct shell *sh, int32_t argc, char *argv[])
{
	int ret;

	if (!spp_inited) {
		shell_print(sh, "need to register spp cb");
		return -ENOEXEC;
	}

	ret = bt_spp_register_service(spp_uuid);
	if (ret != 0) {
		shell_print(sh, "fail");
		return ret;
	}

	shell_print(sh, "success");
	return ret;
}

static int cmd_spp_connect(const struct shell *sh, size_t argc, char *argv[])
{
	int ret, i;
	uint8_t uuid[16];

	if (!spp_inited) {
		shell_print(sh, "need to register hid connection callbacks");
		return -ENOEXEC;
	}

	if (!default_conn) {
		shell_error(sh, "please connect bt first");
		return -ENOEXEC;
	}

	for (i = 0; i < 16; i++) {
		uuid[i] = spp_uuid[15 - i];
	}

	ret = bt_spp_connect(default_conn, uuid);
	if (!ret) {
		shell_error(sh, "fail to connect spp device, ret:%d", ret);
	}

	return 0;
}

static int cmd_spp_send(const struct shell *sh, size_t argc, char *argv[])
{
	uint8_t send_buf[] = {0, 1, 2, 3, 4, 5};

	if (!spp_inited) {
		shell_print(sh, "need to register hid connection callbacks");
		return -ENOEXEC;
	}

	if (!default_conn) {
		shell_error(sh, "Not connected");
		return -ENOEXEC;
	}

	bt_spp_send(spp_id, send_buf, ARRAY_SIZE(send_buf));

	return 0;
}

static int cmd_spp_disconnect(const struct shell *sh, size_t argc, char *argv[])
{
	if (!spp_inited) {
		shell_print(sh, "need to register hid connection callbacks");
		return -ENOEXEC;
	}

	if (!default_conn) {
		shell_error(sh, "spp device is not connected");
	}

	bt_spp_disconnect(spp_id);

	return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(
	spp_cmds, SHELL_CMD_ARG(register_cb, NULL, "register SPP callbacks", cmd_register_cb, 1, 0),
	SHELL_CMD_ARG(register_srv, NULL, "register SPP service", cmd_register_srv, 1, 0),
	SHELL_CMD_ARG(connect, NULL, HELP_NONE, cmd_spp_connect, 1, 0),
	SHELL_CMD_ARG(send, NULL, HELP_NONE, cmd_spp_send, 1, 0),
	SHELL_CMD_ARG(disconnect, NULL, HELP_NONE, cmd_spp_disconnect, 1, 0), SHELL_SUBCMD_SET_END);

static int cmd_spp(const struct shell *sh, size_t argc, char **argv)
{
	if (argc == 1) {
		shell_help(sh);
		return SHELL_CMD_HELP_PRINTED;
	}

	shell_error(sh, "%s unknown parameter: %s", argv[0], argv[1]);

	return -ENOEXEC;
}

SHELL_CMD_ARG_REGISTER(spp, &spp_cmds, "Bluetooth SPP sh commands", cmd_spp, 1, 1);
