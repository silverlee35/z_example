/*
 * Copyright (c) 2020 Lucian Copeland for Adafruit Industries
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for Feather STM32F405 board */
static const struct pin_config pinconf[] = {
#ifdef CONFIG_UART_3
	{STM32_PIN_PB10, STM32F4_PINMUX_FUNC_PB10_USART3_TX},
	{STM32_PIN_PB11, STM32F4_PINMUX_FUNC_PB11_USART3_RX},
#endif  /* CONFIG_UART_3 */
#ifdef CONFIG_I2C_1
	{STM32_PIN_PB6, STM32F4_PINMUX_FUNC_PB6_I2C1_SCL},
	{STM32_PIN_PB7, STM32F4_PINMUX_FUNC_PB7_I2C1_SDA},
#endif /* CONFIG_I2C_1 */
#ifdef CONFIG_SPI_2
	{STM32_PIN_PB13, STM32F4_PINMUX_FUNC_PB13_SPI2_MASTER_SCK},
	{STM32_PIN_PB14, STM32F4_PINMUX_FUNC_PB14_SPI2_MASTER_MISO},
	{STM32_PIN_PB15, STM32F4_PINMUX_FUNC_PB15_SPI2_MASTER_MOSI},
#endif /* CONFIG_SPI_2 */
#ifdef CONFIG_USB_DC_STM32
	{STM32_PIN_PA11, STM32F4_PINMUX_FUNC_PA11_OTG_FS_DM},
	{STM32_PIN_PA12, STM32F4_PINMUX_FUNC_PA12_OTG_FS_DP},
#endif	/* CONFIG_USB_DC_STM32 */
};

static int pinmux_stm32_init(struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
		CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
