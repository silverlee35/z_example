/*
 * Copyright (c) 2018, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#include <zephyr/drivers/i2c.h>
#include <zephyr/dt-bindings/i2c/i2c.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <nrfx_twim.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(i2c_nrfx_twim, CONFIG_I2C_LOG_LEVEL);

#define I2C_TRANSFER_TIMEOUT_MSEC		K_MSEC(500)

struct i2c_nrfx_twim_data {
	struct k_sem transfer_sync;
	struct k_sem completion_sync;
	volatile nrfx_err_t res;
	uint8_t *msg_buf;
};

struct i2c_nrfx_twim_config {
	nrfx_twim_t twim;
	nrfx_twim_config_t twim_config;
	uint16_t concat_buf_size;
	uint16_t flash_buf_max_size;
	void (*irq_connect)(void);
#ifdef CONFIG_PINCTRL
	const struct pinctrl_dev_config *pcfg;
#endif
};

static int i2c_nrfx_twim_recover_bus(const struct device *dev);

static int i2c_nrfx_twim_transfer(const struct device *dev,
				  struct i2c_msg *msgs,
				  uint8_t num_msgs, uint16_t addr)
{
	struct i2c_nrfx_twim_data *dev_data = dev->data;
	const struct i2c_nrfx_twim_config *dev_config = dev->config;
	int ret = 0;
	uint8_t *msg_buf = dev_data->msg_buf;
	uint16_t msg_buf_used = 0;
	uint16_t concat_buf_size = dev_config->concat_buf_size;
	nrfx_twim_xfer_desc_t cur_xfer = {
		.address = addr
	};

	k_sem_take(&dev_data->transfer_sync, K_FOREVER);

	/* Dummy take on completion_sync sem to be sure that it is empty */
	k_sem_take(&dev_data->completion_sync, K_NO_WAIT);

	(void)pm_device_runtime_get(dev);

	for (size_t i = 0; i < num_msgs; i++) {
		if (I2C_MSG_ADDR_10_BITS & msgs[i].flags) {
			ret = -ENOTSUP;
			break;
		}

		/* This fragment needs to be merged with the next one if:
		 * - it is not the last fragment
		 * - it does not end a bus transaction
		 * - the next fragment does not start a bus transaction
		 * - the direction of the next fragment is the same as this one
		 */
		bool concat_next = ((i + 1) < num_msgs)
				&& !(msgs[i].flags & I2C_MSG_STOP)
				&& !(msgs[i + 1].flags & I2C_MSG_RESTART)
				&& ((msgs[i].flags & I2C_MSG_READ)
				    == (msgs[i + 1].flags & I2C_MSG_READ));

		/* If we need to concatenate the next message, or we've
		 * already committed to concatenate this message, add it to
		 * the buffer after verifying there's room.
		 */
		if (concat_next || (msg_buf_used != 0)) {
			if ((msg_buf_used + msgs[i].len) > concat_buf_size) {
				LOG_ERR("Need to use concatenation buffer and "
					"provided size is insufficient "
					"(%u + %u > %u). "
					"Adjust the zephyr,concat-buf-size "
					"property in the \"%s\" node.",
					msg_buf_used, msgs[i].len,
					concat_buf_size, device_name_get(dev));
				ret = -ENOSPC;
				break;
			}
			if (!(msgs[i].flags & I2C_MSG_READ)) {
				memcpy(msg_buf + msg_buf_used,
				       msgs[i].buf,
				       msgs[i].len);
			}
			msg_buf_used += msgs[i].len;

		/* TWIM peripherals cannot transfer data directly from
		 * flash. If a buffer located in flash is provided for
		 * a write transaction, its content must be copied to
		 * RAM before the transfer can be requested.
		 */
		} else if (!(msgs[i].flags & I2C_MSG_READ) &&
			   !nrfx_is_in_ram(msgs[i].buf)) {
			if (msgs[i].len > dev_config->flash_buf_max_size) {
				LOG_ERR("Cannot copy flash buffer of size: %u. "
					"Adjust the zephyr,flash-buf-max-size "
					"property in the \"%s\" node.",
					msgs[i].len, device_name_get(dev));
				ret = -EINVAL;
				break;
			}

			memcpy(msg_buf, msgs[i].buf, msgs[i].len);
			msg_buf_used = msgs[i].len;
		}

		if (concat_next) {
			continue;
		}

		if (msg_buf_used == 0) {
			cur_xfer.p_primary_buf = msgs[i].buf;
			cur_xfer.primary_length = msgs[i].len;
		} else {
			cur_xfer.p_primary_buf = msg_buf;
			cur_xfer.primary_length = msg_buf_used;
		}
		cur_xfer.type = (msgs[i].flags & I2C_MSG_READ) ?
			NRFX_TWIM_XFER_RX : NRFX_TWIM_XFER_TX;

		nrfx_err_t res = nrfx_twim_xfer(&dev_config->twim,
						&cur_xfer,
						(msgs[i].flags & I2C_MSG_STOP) ?
						 0 : NRFX_TWIM_FLAG_TX_NO_STOP);
		if (res != NRFX_SUCCESS) {
			if (res == NRFX_ERROR_BUSY) {
				ret = -EBUSY;
				break;
			} else {
				ret = -EIO;
				break;
			}
		}

		ret = k_sem_take(&dev_data->completion_sync,
				 I2C_TRANSFER_TIMEOUT_MSEC);
		if (ret != 0) {
			/* Whatever the frequency, completion_sync should have
			 * been given by the event handler.
			 *
			 * If it hasn't, it's probably due to an hardware issue
			 * on the I2C line, for example a short between SDA and
			 * GND.
			 * This is issue has also been when trying to use the
			 * I2C bus during MCU internal flash erase.
			 *
			 * In many situation, a retry is sufficient.
			 * However, some time the I2C device get stuck and need
			 * help to recover.
			 * Therefore we always call i2c_nrfx_twim_recover_bus()
			 * to make sure everything has been done to restore the
			 * bus from this error.
			 */
			LOG_ERR("Error on I2C line occurred for message %d", i);
			(void)i2c_nrfx_twim_recover_bus(dev);
			ret = -EIO;
			break;
		}

		res = dev_data->res;

		if (res != NRFX_SUCCESS) {
			LOG_ERR("Error 0x%08X occurred for message %d", res, i);
			ret = -EIO;
			break;
		}

		/* If concatenated messages were I2C_MSG_READ type, then
		 * content of concatenation buffer has to be copied back into
		 * buffers provided by user. */
		if ((msgs[i].flags & I2C_MSG_READ)
		    && cur_xfer.p_primary_buf == msg_buf) {
			int j = i;

			while (msg_buf_used >= msgs[j].len) {
				msg_buf_used -= msgs[j].len;
				memcpy(msgs[j].buf,
				       msg_buf + msg_buf_used,
				       msgs[j].len);
				j--;
			}

		}

		msg_buf_used = 0;
	}

	(void)pm_device_runtime_put(dev);

	k_sem_give(&dev_data->transfer_sync);

	return ret;
}

static void event_handler(nrfx_twim_evt_t const *p_event, void *p_context)
{
	struct i2c_nrfx_twim_data *dev_data = p_context;

	switch (p_event->type) {
	case NRFX_TWIM_EVT_DONE:
		dev_data->res = NRFX_SUCCESS;
		break;
	case NRFX_TWIM_EVT_ADDRESS_NACK:
		dev_data->res = NRFX_ERROR_DRV_TWI_ERR_ANACK;
		break;
	case NRFX_TWIM_EVT_DATA_NACK:
		dev_data->res = NRFX_ERROR_DRV_TWI_ERR_DNACK;
		break;
	default:
		dev_data->res = NRFX_ERROR_INTERNAL;
		break;
	}

	k_sem_give(&dev_data->completion_sync);
}

static int i2c_nrfx_twim_configure(const struct device *dev,
				   uint32_t i2c_config)
{
	const struct i2c_nrfx_twim_config *dev_config = dev->config;

	if (I2C_ADDR_10_BITS & i2c_config) {
		return -EINVAL;
	}

	switch (I2C_SPEED_GET(i2c_config)) {
	case I2C_SPEED_STANDARD:
		nrf_twim_frequency_set(dev_config->twim.p_twim,
				       NRF_TWIM_FREQ_100K);
		break;
	case I2C_SPEED_FAST:
		nrf_twim_frequency_set(dev_config->twim.p_twim,
				       NRF_TWIM_FREQ_400K);
		break;
#if NRF_TWIM_HAS_1000_KHZ_FREQ
	case I2C_SPEED_FAST_PLUS:
		nrf_twim_frequency_set(dev_config->twim.p_twim,
				       NRF_TWIM_FREQ_1000K);
		break;
#endif
	default:
		LOG_ERR("unsupported speed");
		return -EINVAL;
	}

	return 0;
}

static int i2c_nrfx_twim_recover_bus(const struct device *dev)
{
	const struct i2c_nrfx_twim_config *dev_config = dev->config;
	enum pm_device_state state;
	uint32_t scl_pin;
	uint32_t sda_pin;
	nrfx_err_t err;

#ifdef CONFIG_PINCTRL
	scl_pin = nrf_twim_scl_pin_get(dev_config->twim.p_twim);
	sda_pin = nrf_twim_sda_pin_get(dev_config->twim.p_twim);
#else
	scl_pin = dev_config->twim_config.scl;
	sda_pin = dev_config->twim_config.sda;
#endif

	/* disable peripheral if active (required to release SCL/SDA lines) */
	(void)pm_device_state_get(dev, &state);
	if (state == PM_DEVICE_STATE_ACTIVE) {
		nrfx_twim_disable(&dev_config->twim);
	}

	err = nrfx_twim_bus_recover(scl_pin, sda_pin);

	/* restore peripheral if it was active before */
	if (state == PM_DEVICE_STATE_ACTIVE) {
#ifdef CONFIG_PINCTRL
		(void)pinctrl_apply_state(dev_config->pcfg,
					  PINCTRL_STATE_DEFAULT);
#endif
		nrfx_twim_enable(&dev_config->twim);
	}

	return (err == NRFX_SUCCESS ? 0 : -EBUSY);
}

static const struct i2c_driver_api i2c_nrfx_twim_driver_api = {
	.configure   = i2c_nrfx_twim_configure,
	.transfer    = i2c_nrfx_twim_transfer,
	.recover_bus = i2c_nrfx_twim_recover_bus,
};

#ifdef CONFIG_PM_DEVICE
static int twim_nrfx_pm_action(const struct device *dev,
			       enum pm_device_action action)
{
	const struct i2c_nrfx_twim_config *dev_config = dev->config;
	int ret = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
#ifdef CONFIG_PINCTRL
		ret = pinctrl_apply_state(dev_config->pcfg,
					  PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			return ret;
		}
#endif
		nrfx_twim_enable(&dev_config->twim);
		break;

	case PM_DEVICE_ACTION_SUSPEND:
		nrfx_twim_disable(&dev_config->twim);

#ifdef CONFIG_PINCTRL
		ret = pinctrl_apply_state(dev_config->pcfg,
					  PINCTRL_STATE_SLEEP);
		if (ret < 0) {
			return ret;
		}
#endif
		break;

	default:
		ret = -ENOTSUP;
	}

	return ret;
}
#endif /* CONFIG_PM_DEVICE */

static int i2c_nrfx_twim_init(const struct device *dev)
{
	const struct i2c_nrfx_twim_config *dev_config = dev->config;
	struct i2c_nrfx_twim_data *dev_data = dev->data;

	dev_config->irq_connect();

#ifdef CONFIG_PINCTRL
	int err = pinctrl_apply_state(dev_config->pcfg,
				      COND_CODE_1(CONFIG_PM_DEVICE_RUNTIME,
						  (PINCTRL_STATE_SLEEP),
						  (PINCTRL_STATE_DEFAULT)));
	if (err < 0) {
		return err;
	}
#endif

	if (nrfx_twim_init(&dev_config->twim, &dev_config->twim_config,
			   event_handler, dev_data) != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize device: %s", device_name_get(dev));
		return -EIO;
	}

#ifdef CONFIG_PM_DEVICE_RUNTIME
	pm_device_init_suspended(dev);
	pm_device_runtime_enable(dev);
#else
	nrfx_twim_enable(&dev_config->twim);
#endif

	return 0;
}

#define I2C_NRFX_TWIM_INVALID_FREQUENCY  ((nrf_twim_frequency_t)-1)
#define I2C_NRFX_TWIM_FREQUENCY(bitrate)				       \
	(bitrate == I2C_BITRATE_STANDARD  ? NRF_TWIM_FREQ_100K :	       \
	 bitrate == 250000                ? NRF_TWIM_FREQ_250K :	       \
	 bitrate == I2C_BITRATE_FAST      ? NRF_TWIM_FREQ_400K :	       \
	IF_ENABLED(NRF_TWIM_HAS_1000_KHZ_FREQ,				       \
	(bitrate == I2C_BITRATE_FAST_PLUS ? NRF_TWIM_FREQ_1000K :))	       \
					    I2C_NRFX_TWIM_INVALID_FREQUENCY)

#define I2C(idx) DT_NODELABEL(i2c##idx)
#define I2C_FREQUENCY(idx)						       \
	I2C_NRFX_TWIM_FREQUENCY(DT_PROP(I2C(idx), clock_frequency))

#define CONCAT_BUF_SIZE(idx)						       \
	COND_CODE_1(DT_NODE_HAS_PROP(I2C(idx), zephyr_concat_buf_size),	       \
		    (DT_PROP(I2C(idx), zephyr_concat_buf_size)), (0))
#define FLASH_BUF_MAX_SIZE(idx)						       \
	COND_CODE_1(DT_NODE_HAS_PROP(I2C(idx), zephyr_flash_buf_max_size),     \
		    (DT_PROP(I2C(idx), zephyr_flash_buf_max_size)), (0))

#define USES_MSG_BUF(idx)						       \
	COND_CODE_0(CONCAT_BUF_SIZE(idx),				       \
		(COND_CODE_0(FLASH_BUF_MAX_SIZE(idx), (0), (1))),	       \
		(1))
#define MSG_BUF_SIZE(idx)  MAX(CONCAT_BUF_SIZE(idx), FLASH_BUF_MAX_SIZE(idx))

#define I2C_NRFX_TWIM_PIN_CFG(idx)			\
	COND_CODE_1(CONFIG_PINCTRL,			\
		(.skip_gpio_cfg = true,			\
		 .skip_psel_cfg = true,),		\
		(.scl = DT_PROP(I2C(idx), scl_pin),	\
		 .sda = DT_PROP(I2C(idx), sda_pin),))

#define I2C_NRFX_TWIM_DEVICE(idx)					       \
	NRF_DT_CHECK_PIN_ASSIGNMENTS(I2C(idx), 1, scl_pin, sda_pin);	       \
	BUILD_ASSERT(I2C_FREQUENCY(idx) !=				       \
		     I2C_NRFX_TWIM_INVALID_FREQUENCY,			       \
		     "Wrong I2C " #idx " frequency setting in dts");	       \
	static void irq_connect##idx(void)				       \
	{								       \
		IRQ_CONNECT(DT_IRQN(I2C(idx)), DT_IRQ(I2C(idx), priority),     \
			    nrfx_isr, nrfx_twim_##idx##_irq_handler, 0);       \
	}								       \
	IF_ENABLED(USES_MSG_BUF(idx),					       \
		(static uint8_t twim_##idx##_msg_buf[MSG_BUF_SIZE(idx)];))     \
	static struct i2c_nrfx_twim_data twim_##idx##_data = {		       \
		.transfer_sync = Z_SEM_INITIALIZER(			       \
			twim_##idx##_data.transfer_sync, 1, 1),		       \
		.completion_sync = Z_SEM_INITIALIZER(			       \
			twim_##idx##_data.completion_sync, 0, 1),	       \
		IF_ENABLED(USES_MSG_BUF(idx),				       \
			(.msg_buf = twim_##idx##_msg_buf,))		       \
	};								       \
	IF_ENABLED(CONFIG_PINCTRL, (PINCTRL_DT_DEFINE(I2C(idx))));	       \
	static const struct i2c_nrfx_twim_config twim_##idx##z_config = {      \
		.twim = NRFX_TWIM_INSTANCE(idx),			       \
		.twim_config = {					       \
			I2C_NRFX_TWIM_PIN_CFG(idx)			       \
			.frequency = I2C_FREQUENCY(idx),		       \
		},							       \
		.concat_buf_size = CONCAT_BUF_SIZE(idx),		       \
		.flash_buf_max_size = FLASH_BUF_MAX_SIZE(idx),		       \
		.irq_connect = irq_connect##idx,			       \
		IF_ENABLED(CONFIG_PINCTRL,				       \
			(.pcfg = PINCTRL_DT_DEV_CONFIG_GET(I2C(idx)),))	       \
	};								       \
	PM_DEVICE_DT_DEFINE(I2C(idx), twim_nrfx_pm_action);		       \
	I2C_DEVICE_DT_DEFINE(I2C(idx),					       \
		      i2c_nrfx_twim_init,				       \
		      PM_DEVICE_DT_GET(I2C(idx)),			       \
		      &twim_##idx##_data,				       \
		      &twim_##idx##z_config,				       \
		      POST_KERNEL,					       \
		      CONFIG_I2C_INIT_PRIORITY,				       \
		      &i2c_nrfx_twim_driver_api)

#ifdef CONFIG_I2C_0_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(0);
#endif

#ifdef CONFIG_I2C_1_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(1);
#endif

#ifdef CONFIG_I2C_2_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(2);
#endif

#ifdef CONFIG_I2C_3_NRF_TWIM
I2C_NRFX_TWIM_DEVICE(3);
#endif
