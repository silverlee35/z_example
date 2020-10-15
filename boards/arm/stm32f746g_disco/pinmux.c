/*
 * Copyright (c) 2018 Yurii Hamann
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <device.h>
#include <init.h>
#include <drivers/pinmux.h>
#include <sys/sys_io.h>

#include <pinmux/stm32/pinmux_stm32.h>

/* pin assignments for STM32F746G-DISCO board */
static const struct pin_config pinconf[] = {
#ifdef CONFIG_USB_DC_STM32
	{STM32_PIN_PA11, STM32F7_PINMUX_FUNC_PA11_OTG_FS_DM},
	{STM32_PIN_PA12, STM32F7_PINMUX_FUNC_PA12_OTG_FS_DP},
#endif	/* CONFIG_USB_DC_STM32 */
#if DT_NODE_HAS_STATUS(DT_NODELABEL(sdmmc1), okay) && \
	CONFIG_DISK_ACCESS_STM32_SDMMC
	{STM32_PIN_PC8, STM32F7_PINMUX_FUNC_PC8_SDMMC1_D0},
	{STM32_PIN_PC9, STM32F7_PINMUX_FUNC_PC9_SDMMC1_D1},
	{STM32_PIN_PC10, STM32F7_PINMUX_FUNC_PC10_SDMMC1_D2},
	{STM32_PIN_PC11, STM32F7_PINMUX_FUNC_PC11_SDMMC1_D3},
	{STM32_PIN_PC12, STM32F7_PINMUX_FUNC_PC12_SDMMC1_CK},
	{STM32_PIN_PC13, STM32_MODER_INPUT_MODE},
	{STM32_PIN_PD2, STM32F7_PINMUX_FUNC_PD2_SDMMC1_CMD},
#endif
};

static int pinmux_stm32_init(const struct device *port)
{
	ARG_UNUSED(port);

	stm32_setup_pins(pinconf, ARRAY_SIZE(pinconf));

	return 0;
}

SYS_INIT(pinmux_stm32_init, PRE_KERNEL_1,
		CONFIG_PINMUX_STM32_DEVICE_INITIALIZATION_PRIORITY);
