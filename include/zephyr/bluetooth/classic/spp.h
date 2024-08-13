/** @file
 *  @brief Bluetooth SPP Protocol handling.
 */

/*
 * Copyright 2024 Xiaomi Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __BT_SPP_H
#define __BT_SPP_H

/**
 * @brief SPP
 * @defgroup SPP
 * @ingroup bluetooth
 * @{
 */

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/bluetooth/buf.h>
#include <zephyr/bluetooth/conn.h>

/** @brief spp app call back structure. */
struct bt_spp_cb {
	void (*connected)(struct bt_conn *conn, uint8_t spp_id);
	void (*disconnected)(struct bt_conn *conn, uint8_t spp_id);
	void (*recv)(struct bt_conn *conn, uint8_t spp_id, uint8_t *data, uint16_t len);
};

/** @brief Register SPP Callback.
 *
 *  Register SPP Callback
 *
 *  @param cb The bt_spp_cb attribute.
 *
 *  @return 0 in case of success or negative value in case of error.
 */
int bt_spp_register_cb(struct bt_spp_cb *cb);

/** @brief Register SPP Service.
 *
 *  Register SPP service.
 *
 *  @param uuid  spp uuid.
 *
 *  @return 0 in case of success or negative value in case of error.
 */
uint8_t bt_spp_register_service(const uint8_t *uuid);

/** @brief SPP Connect
 *
 * This function establish spp connection.
 *
 *  @param conn The conn instance.
 *  @param uuid spp uuid.
 *
 *  @return 0 in case of success and error code in case of error.
 */
uint8_t bt_spp_connect(struct bt_conn *conn, const uint8_t *uuid);

/** @brief SPP Send
 *
 * This function send spp buffer.
 *
 *  @param spp_id The SPP id.
 *  @param data Send buffer.
 *  @param len  Buffer size.
 *
 *  @return size in case of success and error code in case of error.
 */
int bt_spp_send(uint8_t spp_id, uint8_t *data, uint16_t len);

/** @brief SPP Disconnect
 *
 * This function disconnect spp connection.
 *
 *  @param spp_id The SPP id.
 *
 *  @return 0 in case of success and error code in case of error.
 */
int bt_spp_disconnect(uint8_t spp_id);

#ifdef __cplusplus
}
#endif

/**
 * @}
 */
#endif /* __BT_SPP_H */
