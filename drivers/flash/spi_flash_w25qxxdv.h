/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 *
 * @brief This file defines the private data structures for spi flash driver
 */

#ifndef ZEPHYR_DRIVERS_FLASH_SPI_FLASH_W25QXXDV_H_
#define ZEPHYR_DRIVERS_FLASH_SPI_FLASH_W25QXXDV_H_


struct spi_flash_data {
	struct device *spi;
#if defined(DT_INST_0_WINBOND_W25QXX_CS_GPIOS_CONTROLLER)
	struct spi_cs_control cs_ctrl;
#endif /* DT_INST_0_WINBOND_W25QXX_CS_GPIOS_CONTROLLER */
	struct spi_config spi_cfg;
#if defined(CONFIG_MULTITHREADING)
	struct k_sem sem;
#endif /* CONFIG_MULTITHREADING */
};


#endif /* ZEPHYR_DRIVERS_FLASH_SPI_FLASH_W25QXXDV_H_ */
