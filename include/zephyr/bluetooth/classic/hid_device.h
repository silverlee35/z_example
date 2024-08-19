/** @file
 *  @brief HID Device Protocol handling.
 */

/*
 * Copyright 2024 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_INCLUDE_BLUETOOTH_HID_DEVICE_H_
#define ZEPHYR_INCLUDE_BLUETOOTH_HID_DEVICE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/classic/sdp.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/l2cap.h>

#define BT_HID_MAX_MTU         64
#define BT_HID_REPORT_DATA_LEN 64

/** @brief HID report types in get, set, data */
enum {
	BT_HID_REPORT_TYPE_OTHER = 0,
	BT_HID_REPORT_TYPE_INPUT,
	BT_HID_REPORT_TYPE_OUTPUT,
	BT_HID_REPORT_TYPE_FEATURE,
};

/** @brief HID handshake response code */
enum {
	BT_HID_HANDSHAKE_RSP_SUCCESS = 0,
	BT_HID_HANDSHAKE_RSP_NOT_READY,
	BT_HID_HANDSHAKE_RSP_ERR_INVALID_REP_ID,
	BT_HID_HANDSHAKE_RSP_ERR_UNSUPPORTED_REQ,
	BT_HID_HANDSHAKE_RSP_ERR_INVALID_PARAM,
	BT_HID_HANDSHAKE_RSP_ERR_UNKNOWN = 14,
	BT_HID_HANDSHAKE_RSP_ERR_FATAL,
};

/** @brief HID protocol parameters for set protocal */
enum {
	BT_HID_PROTOCOL_BOOT_MODE = 0,
	BT_HID_PROTOCOL_REPORT_MODE,
};

/** @brief HID event type */
typedef enum bt_hid_event {
	BT_HID_EVENT_GET_REPORT = 0,
	BT_HID_EVENT_SET_REPORT,
	BT_HID_EVENT_GET_PROTOCOL,
	BT_HID_EVENT_SET_PROTOCOL,
	BT_HID_EVENT_INTR_DATA,
	BT_HID_EVENT_UNPLUG,
	BT_HID_EVENT_SUSPEND,
	BT_HID_EVENT_EXIT_SUSPEND,
} bt_hid_event_t;

/** @brief HID role */
typedef enum bt_hid_role {
	BT_HID_ROLE_ACCEPTOR,
	BT_HID_ROLE_INITIATOR
} bt_hid_role_t;

/** @brief HID seesion role */
typedef enum bt_hid_session_role {
	BT_HID_SESSION_ROLE_CTRL,
	BT_HID_SESSION_ROLE_INTR
} bt_hid_session_role_t;

/** @brief HID session structure. */
struct bt_hid_session {
	struct bt_l2cap_br_chan br_chan;
	bt_hid_session_role_t role; /* As ctrl or intr */
};

/** @brief HID Device structure. */
struct bt_hid_device {
	struct bt_conn *conn;
	struct net_buf *buf;

	bt_hid_role_t role; /* As initial or accept */
	struct bt_hid_session ctrl_session;
	struct bt_hid_session intr_session;

	uint8_t state;
	uint8_t unplug;
};

/** @brief HID Device report structure. */
struct bt_hid_report {
	struct bt_conn *conn;
	uint8_t type;
	uint8_t data[BT_HID_REPORT_DATA_LEN];
	int len;
};

/** @brief HID Device callbacks. */
struct bt_hid_device_cb {
	void (*connected)(struct bt_hid_device *hid);
	void (*disconnected)(struct bt_hid_device *hid);
	void (*event_cb)(struct bt_hid_device *hid, bt_hid_event_t event, uint8_t *data,
			 uint16_t len);
};

/** @brief HID Device Register Callback.
 *
 *  The cb is called when bt_hid_device is called or it is operated by remote device.
 *
 *  @param cb The callback function.
 *
 *  @return 0 in case of success and error code in case of error.
 */
int bt_hid_device_register_cb(struct bt_hid_device_cb *cb);

/** @brief HID Device Connect.
 *
 *  This function is to be called after the conn parameter is obtained by
 *  performing a GAP procedure. The API is to be used to establish HID
 *  Device connection between devices.
 *
 *  @param conn Pointer to bt_conn structure.
 *
 *  @return pointer to struct bt_hid_device in case of success or NULL in case
 *  of error.
 */
struct bt_hid_device *bt_hid_device_connect(struct bt_conn *conn);

/** @brief HID Device Disconnect
 *
 * This function close HID Device connection.
 *
 *  @param hid The bt_hid_device instance.
 *
 *  @return 0 in case of success and error code in case of error.
 */
int bt_hid_devie_disconnect(struct bt_hid_device *hid);

/** @brief HID Device Send Ctrl Data.
 *
 *  Send hid data by ctrl channel.
 *
 *  @param hid The bt_hid_device instance.
 *  @param type HID report type.
 *  @param data send buffer.
 *  @param len buffer size.
 *
 *  @return size in case of success and error code in case of error.
 */
int bt_hid_device_send_ctrl_data(struct bt_hid_device *hid, uint8_t type, uint8_t *data,
				 uint16_t len);

/** @brief HID Device Send Intr Data.
 *
 *  Send hid data by Intr channel.
 *
 *  @param hid The bt_hid_device instance.
 *  @param type HID report type.
 *  @param data send buffer.
 *  @param len buffer size.
 *
 *  @return size in case of success and error code in case of error.
 */
int bt_hid_device_send_intr_data(struct bt_hid_device *hid, uint8_t type, uint8_t *data,
				 uint16_t len);

/** @brief HID Device Send Response Data.
 *
 *  Send hid status by ctrl channel.
 *
 *  @param hid The bt_hid_device instance.
 *  @param status response status.
 *
 *  @return size in case of success and error code in case of error.
 */
int bt_hid_device_send_response(struct bt_hid_device *hid, uint8_t status);

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_BLUETOOTH_HID_DEVICE_H_ */
