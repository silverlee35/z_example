/*
 * Copyright 2024 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <string.h>
#include <strings.h>
#include <errno.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/util.h>

#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/l2cap.h>
#include <zephyr/bluetooth/classic/rfcomm.h>
#include <zephyr/bluetooth/classic/sdp.h>
#include <zephyr/bluetooth/classic/spp.h>

#include "host/hci_core.h"
#include "host/conn_internal.h"

#include "l2cap_br_internal.h"
#include "rfcomm_internal.h"

#define LOG_LEVEL CONFIG_BT_SPP_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(bt_spp);

#define MAX_SPP_CHANNEL 4
#define SPP_UUID128_LEN 16
#define SPP_RFCOMM_MTU  CONFIG_BT_L2CAP_TX_MTU

#define SPP_INDEX_TO_ID(x) ((x) | 0x80)
#define SPP_ID_TO_INDEX(x) ((x) & (~0x80))

#define SDP_CLIENT_BUF_LEN 512

enum {
	SPP_STATE_IDLE,
	SPP_STATE_DISCOVERING,
	SPP_STATE_USED,
};

struct bt_spp_server {
	uint8_t spp_id;
	uint8_t state: 4;
	uint8_t active_connect: 1;
	uint8_t spp_uuid[SPP_UUID128_LEN];

	struct bt_conn *conn;
	struct bt_rfcomm_server rfcomm_server;
	struct bt_rfcomm_dlc rfcomm_dlc;
};

static K_MUTEX_DEFINE(spp_lock);
static struct bt_spp_cb *spp_cb;
static struct bt_spp_server spp_server[MAX_SPP_CHANNEL];
static uint8_t spp_state;
static struct bt_uuid_128 sdp_uuid;

NET_BUF_POOL_FIXED_DEFINE(sdp_client_pool, CONFIG_BT_MAX_CONN, SDP_CLIENT_BUF_LEN, 8, NULL);
NET_BUF_POOL_FIXED_DEFINE(spp_pool, CONFIG_BT_MAX_CONN, BT_L2CAP_BUF_SIZE(CONFIG_BT_L2CAP_TX_MTU),
			  16, NULL);

#define BT_SPP_ATTRS(i)                                                                            \
	static struct bt_sdp_attribute spp_attrs##i[] = {                                          \
		BT_SDP_NEW_SERVICE,                                                                \
		BT_SDP_LIST(BT_SDP_ATTR_SVCLASS_ID_LIST, BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 17),    \
			    BT_SDP_DATA_ELEM_LIST({BT_SDP_TYPE_SIZE(BT_SDP_UUID128),               \
						   spp_server[i].spp_uuid}, )),                    \
		BT_SDP_LIST(                                                                       \
			BT_SDP_ATTR_PROTO_DESC_LIST, BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 12),        \
			BT_SDP_DATA_ELEM_LIST(                                                     \
				{BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 3),                             \
				 BT_SDP_DATA_ELEM_LIST({BT_SDP_TYPE_SIZE(BT_SDP_UUID16),           \
							BT_SDP_ARRAY_16(BT_SDP_PROTO_L2CAP)}, )},  \
				{BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 5),                             \
				 BT_SDP_DATA_ELEM_LIST(                                            \
					 {BT_SDP_TYPE_SIZE(BT_SDP_UUID16),                         \
					  BT_SDP_ARRAY_16(BT_SDP_PROTO_RFCOMM)},                   \
					 {BT_SDP_TYPE_SIZE(BT_SDP_UINT8),                          \
					  &spp_server[i].rfcomm_server.channel}, )}, )),           \
		BT_SDP_LIST(BT_SDP_ATTR_PROFILE_DESC_LIST, BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 8),   \
			    BT_SDP_DATA_ELEM_LIST(                                                 \
				    {BT_SDP_TYPE_SIZE_VAR(BT_SDP_SEQ8, 6),                         \
				     BT_SDP_DATA_ELEM_LIST(                                        \
					     {BT_SDP_TYPE_SIZE(BT_SDP_UUID16),                     \
					      BT_SDP_ARRAY_16(BT_SDP_SERIAL_PORT_SVCLASS)},        \
					     {BT_SDP_TYPE_SIZE(BT_SDP_UINT16),                     \
					      BT_SDP_ARRAY_16(0x0102)}, )}, )),                    \
	};

BT_SPP_ATTRS(0);
BT_SPP_ATTRS(1);
BT_SPP_ATTRS(2);
BT_SPP_ATTRS(3);

static struct bt_sdp_record spp_rec[MAX_SPP_CHANNEL] = {
	BT_SDP_RECORD(spp_attrs0),
	BT_SDP_RECORD(spp_attrs1),
	BT_SDP_RECORD(spp_attrs2),
	BT_SDP_RECORD(spp_attrs3),
};

static void *bt_spp_alloc_server(void)
{
	uint8_t i;

	for (i = 0; i < MAX_SPP_CHANNEL; i++) {
		if (spp_server[i].spp_id == 0) {
			spp_server[i].spp_id = SPP_INDEX_TO_ID(i);
			spp_server[i].active_connect = 0;
			return &spp_server[i];
		}
	}

	return NULL;
}

static void *bt_spp_find_server_by_uuid(const uint8_t *uuid)
{
	uint8_t i;

	for (i = 0; i < MAX_SPP_CHANNEL; i++) {
		if (spp_server[i].spp_id && spp_server[i].active_connect == 0 &&
		    memcmp(spp_server[i].spp_uuid, uuid, SPP_UUID128_LEN) == 0) {
			return &spp_server[i];
		}
	}

	return NULL;
}

static struct bt_spp_server *bt_spp_find_server_by_sip(uint8_t spp_id)
{
	uint8_t i;

	for (i = 0; i < MAX_SPP_CHANNEL; i++) {
		if (spp_server[i].spp_id == spp_id) {
			return &spp_server[i];
		}
	}

	return NULL;
}

static struct bt_spp_server *bt_spp_find_server_by_conn(struct bt_conn *conn)
{
	uint8_t i;

	for (i = 0; i < MAX_SPP_CHANNEL; i++) {
		if (spp_server[i].conn == conn && spp_server[i].spp_id &&
		    spp_server[i].state == SPP_STATE_DISCOVERING) {
			return &spp_server[i];
		}
	}

	return NULL;
}

static struct bt_spp_server *bt_spp_find_server_by_dlci(struct bt_rfcomm_dlc *dlci)
{
	uint8_t i;

	for (i = 0; i < MAX_SPP_CHANNEL; i++) {
		if (spp_server[i].spp_id && (&spp_server[i].rfcomm_dlc == dlci)) {
			return &spp_server[i];
		}
	}

	return NULL;
}

static void bt_spp_free_server(struct bt_spp_server *server)
{
	memset(server, 0, sizeof(struct bt_spp_server));
}

static void bt_spp_recv(struct bt_rfcomm_dlc *dlci, struct net_buf *buf)
{
	struct bt_spp_server *server;

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_dlci(dlci);
	if (!server || !server->conn) {
		LOG_ERR("can't find server for dlci:%p, server:%p", dlci, server);
		k_mutex_unlock(&spp_lock);
		return;
	}

	if (spp_cb && spp_cb->recv) {
		spp_cb->recv(server->conn, server->spp_id, buf->data, buf->len);
	}

	k_mutex_unlock(&spp_lock);
}

static void bt_spp_connected(struct bt_rfcomm_dlc *dlci)
{
	struct bt_spp_server *server;

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_dlci(dlci);
	if (!server || !server->conn) {
		LOG_ERR("Can't find server for dlci:%p, server:%p", dlci, server);
		k_mutex_unlock(&spp_lock);
		return;
	}

	if (spp_cb && spp_cb->connected) {
		spp_cb->connected(server->conn, server->spp_id);
	}

	k_mutex_unlock(&spp_lock);
}

static void bt_spp_disconnected(struct bt_rfcomm_dlc *dlci)
{
	struct bt_spp_server *server;

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_dlci(dlci);
	if (!server || !server->conn) {
		LOG_ERR("Can't find server for dlci:%p, server:%p", dlci, server);
		k_mutex_unlock(&spp_lock);
		return;
	}

	if (spp_cb && spp_cb->disconnected) {
		spp_cb->disconnected(server->conn, server->spp_id);
	}

	if (server->active_connect) {
		bt_spp_free_server(server);
	} else {
		server->conn = NULL;
	}

	k_mutex_unlock(&spp_lock);
}

static struct bt_rfcomm_dlc_ops spp_rfcomm_ops = {
	.recv = bt_spp_recv,
	.connected = bt_spp_connected,
	.disconnected = bt_spp_disconnected,
};

static int bt_spp_accept(struct bt_conn *conn, struct bt_rfcomm_dlc **dlc)
{
	struct bt_spp_server *server;

	LOG_DBG("%s conn %p", __func__, conn);

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_conn(conn);
	if (!server || server->conn) {
		k_mutex_unlock(&spp_lock);
		LOG_ERR("can't find server for conn:%p", conn);
		return -ENOMEM;
	}

	server->conn = conn;
	*dlc = &server->rfcomm_dlc;

	k_mutex_unlock(&spp_lock);
	return 0;
}

int bt_spp_send(uint8_t spp_id, uint8_t *data, uint16_t len)
{
	struct bt_spp_server *server;
	struct net_buf *buf;
	int ret;

	if (len > SPP_RFCOMM_MTU) {
		LOG_ERR("rfcomm send data len:%d over max:%d", len, SPP_RFCOMM_MTU);
		return -ENOMEM;
	}

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_sip(spp_id);
	if (!server || !server->conn) {
		k_mutex_unlock(&spp_lock);
		return -EIO;
	}

	buf = bt_rfcomm_create_pdu(&spp_pool);
	net_buf_add_mem(buf, data, len);

	ret = bt_rfcomm_dlc_send(&server->rfcomm_dlc, buf);
	if (ret < 0) {
		LOG_ERR("rfcomm unable to send: %d", ret);
		net_buf_unref(buf);
	} else {
		ret = len;
	}

	k_mutex_unlock(&spp_lock);
	return ret;
}

static uint8_t bt_spp_sdp_recv(struct bt_conn *conn, struct bt_sdp_client_result *response)
{
	int ret;
	uint16_t param;
	struct bt_spp_server *server;

	if (!response) {
		LOG_WRN("spp sdp is  null");
		return BT_SDP_DISCOVER_UUID_CONTINUE;
	}

	k_mutex_lock(&spp_lock, K_FOREVER);
	server = bt_spp_find_server_by_conn(conn);
	if (!server) {
		spp_state = SPP_STATE_IDLE;
		k_mutex_unlock(&spp_lock);
		return BT_SDP_DISCOVER_UUID_STOP;
	}

	if (!response->resp_buf) {
		LOG_ERR("spp sdp resp_buf is null");
		goto done;
	}

	ret = bt_sdp_get_proto_param(response->resp_buf, BT_SDP_PROTO_RFCOMM, &param);
	if (ret < 0) {
		LOG_ERR("spp sdp  get proto fail");
		goto done;
	}

	spp_state = SPP_STATE_IDLE;
	server->state = SPP_STATE_USED;
	server->rfcomm_server.accept = NULL;
	server->rfcomm_dlc.ops = &spp_rfcomm_ops;
	server->rfcomm_dlc.mtu = SPP_RFCOMM_MTU;

	ret = bt_rfcomm_dlc_connect(conn, &server->rfcomm_dlc, param);
	if (ret) {
		k_mutex_unlock(&spp_lock);
		LOG_ERR("spp rdconn connect fail, err:%d", ret);
		goto done;
	}

	k_mutex_unlock(&spp_lock);
	return BT_SDP_DISCOVER_UUID_STOP;

done:
	spp_state = SPP_STATE_IDLE;
	if (spp_cb && spp_cb->disconnected) {
		spp_cb->disconnected(server->conn, server->spp_id);
	}

	bt_spp_free_server(server);
	k_mutex_unlock(&spp_lock);
	return BT_SDP_DISCOVER_UUID_STOP;
}

static struct bt_sdp_discover_params spp_sdp = {
	.uuid = (struct bt_uuid *)&sdp_uuid,
	.func = bt_spp_sdp_recv,
	.pool = &sdp_client_pool,
};

uint8_t bt_spp_connect(struct bt_conn *conn, const uint8_t *uuid)
{
	int ret;
	struct bt_spp_server *server = NULL;

	if (spp_state == SPP_STATE_DISCOVERING) {
		LOG_WRN("conn:%p is discovering", conn);
		return 0;
	}

	server = bt_spp_alloc_server();
	if (!server) {
		LOG_WRN("spp alloc server fail");
		return 0;
	}

	spp_state = SPP_STATE_DISCOVERING;
	sdp_uuid.uuid.type = BT_UUID_TYPE_128;
	memcpy(sdp_uuid.val, uuid, 16);

	server->active_connect = 1;
	server->state = SPP_STATE_DISCOVERING;
	ret = bt_sdp_discover(conn, &spp_sdp);
	if (ret) {
		LOG_ERR("spp sdp discover fail");
		spp_state = SPP_STATE_IDLE;
		bt_spp_free_server(server);
		return 0;
	}

	server->conn = conn;
	return server->spp_id;
}

int bt_spp_disconnect(uint8_t spp_id)
{
	struct bt_spp_server *server;
	int ret;

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_sip(spp_id);
	if (!server || !server->conn) {
		LOG_ERR("spp server null or conn not exist");
		k_mutex_unlock(&spp_lock);
		return -EIO;
	}

	ret = bt_rfcomm_dlc_disconnect(&server->rfcomm_dlc);
	if (ret < 0) {
		LOG_ERR("bt rfcomm disconnect err:%d", ret);
	}

	k_mutex_unlock(&spp_lock);
	return ret;
}

uint8_t bt_spp_register_service(const uint8_t *uuid)
{
	struct bt_spp_server *server;
	uint8_t channel;
	int ret = 0;

	k_mutex_lock(&spp_lock, K_FOREVER);

	server = bt_spp_find_server_by_uuid(uuid);
	if (server) {
		ret = server->spp_id;
		LOG_ERR("exist spp id:%d", ret);
		goto out_with_unlock;
	}

	server = bt_spp_alloc_server();
	if (!server) {
		LOG_ERR("find new server fail");
		goto out_with_unlock;
	}

	channel = bt_rfcomm_alloc_channel();
	if (!channel) {
		LOG_ERR("alloc spp channal fail");
		goto out_with_unlock;
	}

	server->state = SPP_STATE_USED;
	memcpy(server->spp_uuid, uuid, SPP_UUID128_LEN);
	server->rfcomm_server.channel = channel;
	server->rfcomm_server.accept = &bt_spp_accept;
	server->rfcomm_dlc.ops = &spp_rfcomm_ops;
	server->rfcomm_dlc.mtu = SPP_RFCOMM_MTU;

	ret = bt_rfcomm_server_register(&server->rfcomm_server);
	if (ret < 0) {
		LOG_ERR("register rfcomm server failed");
		goto out_with_unregister;
	}

	ret = bt_sdp_register_service(&spp_rec[SPP_ID_TO_INDEX(server->spp_id)]);
	if (ret < 0) {
		LOG_ERR("register sdp server failed");
		goto out_with_unregister;
	}

	return ret;

out_with_unregister:
	bt_rfcomm_free_channel(channel);
	bt_spp_free_server(server);
out_with_unlock:
	k_mutex_unlock(&spp_lock);
	return ret;
}

int bt_spp_register_cb(struct bt_spp_cb *cb)
{
	spp_cb = cb;
	return 0;
}
