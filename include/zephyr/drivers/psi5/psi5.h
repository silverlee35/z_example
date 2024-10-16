/*
 * Copyright 2024 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Peripheral Sensor Interface (PSI5) driver API.
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_PSI5_H_
#define ZEPHYR_INCLUDE_DRIVERS_PSI5_H_

#include <zephyr/kernel.h>
#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PSI5 frame structure
 */
struct psi5_frame {
	union {
		/* PSI5 recevice data frame */
		struct {
			uint32_t data;
			uint32_t timestamp;
			uint8_t crc;
		} msg;

		/* PSI5 serial message channel (SMC) data frame */
		struct {
			uint16_t data;
			uint8_t id;
			uint8_t crc;
		} smc_msg;
	};
};

/**
 * @brief PSI5 state
 */
enum psi5_state {
	PSI5_STATE_TX_READY,          /* Driver is ready to transmit new data */
	PSI5_STATE_TX_OVERWRITE,      /* Data register overwrite */
	PSI5_STATE_MSG_RECEIVED,      /* PSI5 Message Received Event */
	PSI5_STATE_MSG_OVERWRITE,     /* PSI5 Message Overwrite Event */
	PSI5_STATE_MSG_ERR,           /* PSI5 Message Errors Present Event */
	PSI5_STATE_SMC_MSG_RECEIVED,  /* PSI5 SMC Message Received Event */
	PSI5_STATE_SMC_MSG_OVERWRITE, /* PSI5 SMC Message Overwrite Event */
	PSI5_STATE_SMC_MSG_ERR,       /* PSI5 SMC Message Errors Present Event */
};

/** @cond INTERNAL_HIDDEN */

/**
 * @brief Defines the application callback handler function signature
 *
 * @param dev        Pointer to the device structure for the driver instance.
 * @param channel_id Pointer to the channel id for the driver instance.
 * @param psi5_state State of PSI5.
 * @param user_data  User data provided when the frame was sent.
 */
typedef void (*psi5_tx_callback_t)(const struct device *dev, uint8_t channel_id, enum psi5_state,
				   void *user_data);

/**
 * @brief Defines the application callback handler function signature for receiving.
 *
 * @param dev        Pointer to the device structure for the driver instance.
 * @param channel_id Pointer to the channel id for the driver instance.
 * @param frame      Received frame.
 * @param psi5_state State of PSI5.
 * @param user_data  User data provided when the filter was added.
 */
typedef void (*psi5_rx_callback_t)(const struct device *dev, uint8_t channel_id,
				   struct psi5_frame *frame, enum psi5_state, void *user_data);

/**
 * @brief Callback API upon starting PSI5
 * See @a psi5_start() for argument description
 */
typedef int (*psi5_start_t)(const struct device *dev, uint8_t channel_id);

/**
 * @brief Callback API upon stopping PSI5
 * See @a psi5_stop() for argument description
 */
typedef int (*psi5_stop_t)(const struct device *dev, uint8_t channel_id);

/**
 * @brief Callback API upon sending PSI5 frame
 * See @a psi5_transmit() for argument description
 */
typedef int (*psi5_send_t)(const struct device *dev, uint8_t channel_id, const uint64_t psi5_data,
			   k_timeout_t timeout, psi5_tx_callback_t callback, void *user_data);

/**
 * @brief Callback API upon adding RX callback
 * See @a psi5_add_rx_callback() for argument description
 */
typedef void (*psi5_add_rx_callback_t)(const struct device *dev, uint8_t channel_id,
				       psi5_rx_callback_t callback, void *user_data);

__subsystem struct psi5_driver_api {
	psi5_start_t start;
	psi5_stop_t stop;
	psi5_send_t send;
	psi5_add_rx_callback_t add_rx_callback;
};

/** @endcond */

/**
 * @brief Start PSI5
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param channel_id Pointer to the channel id for the driver instance.
 * @retval 0 if successful.
 * @retval -EALREADY if the device is already started.
 * @retval -EIO General input/output error, failed to start device.
 */

static inline int psi5_start(const struct device *dev, uint8_t channel_id)
{
	const struct psi5_driver_api *api = (const struct psi5_driver_api *)dev->api;

	if (api->start) {
		return api->start(dev, channel_id);
	}

	return -ENOSYS;
}

/**
 * @brief Stop PSI5
 *
 * @param dev Pointer to the device structure for the driver instance.
 * @param channel_id Pointer to the channel id for the driver instance.
 * @retval 0 if successful.
 * @retval -EALREADY if the device is already stopped.
 * @retval -EIO General input/output error, failed to stop device.
 */

static inline int psi5_stop(const struct device *dev, uint8_t channel_id)
{
	const struct psi5_driver_api *api = (const struct psi5_driver_api *)dev->api;

	if (api->stop) {
		return api->stop(dev, channel_id);
	}

	return -ENOSYS;
}

/**
 * @brief Transmitting PSI5 frames
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param channel_id Pointer to the channel id for the driver instance.
 * @param psi5_data PSI5 data to transmit.
 * @param timeout   Timeout waiting for ready to transmit new data.
 * @param callback  Optional callback for when the frame was sent or a
 *                  transmission error occurred. If ``NULL``, this function is
 *                  blocking until frame is sent.
 * @param user_data User data to pass to callback function.
 *
 * @retval 0 if successful.
 * @retval -ENOTSUP if an unsupported parameter was passed to the function.
 * @retval -ENETDOWN if PSI5 is in stopped state.
 * @retval -EIO if a general transmit error occurred.
 * @retval -EAGAIN on timeout.
 */

static inline int psi5_send(const struct device *dev, uint8_t channel_id, const uint64_t psi5_data,
			    k_timeout_t timeout, psi5_tx_callback_t callback, void *user_data)
{
	const struct psi5_driver_api *api = (const struct psi5_driver_api *)dev->api;

	if (api->send) {
		return api->send(dev, channel_id, psi5_data, timeout, callback, user_data);
	}

	return -ENOSYS;
}

/**
 * @name Receiving PSI5 frames
 *
 * @{
 */

/**
 * @brief Add a callback function for receiving PSI5 frames
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param channel_id Pointer to the channel id for the driver instance.
 * @param callback  This function is called by PSI5 driver whenever a frame is received.
 * @param user_data User data to pass to callback function.
 *
 * @param dev       Pointer to the device structure for the driver instance.
 * @param callback  Callback function.
 * @param user_data User data to pass to callback function.
 */

static inline int psi5_add_rx_callback(const struct device *dev, uint8_t channel_id,
				       psi5_rx_callback_t callback, void *user_data)
{
	const struct psi5_driver_api *api = (const struct psi5_driver_api *)dev->api;

	if (api->add_rx_callback) {
		api->add_rx_callback(dev, channel_id, callback, user_data);

		return 0;
	}

	return -ENOSYS;
}

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_PSI5_H_ */
