/*
 * Copyright (c) 2016 BayLibre, SAS
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_SYSCFG_REGISTERS_H_
#define _STM32_SYSCFG_REGISTERS_H_

#include "../common/soc_syscfg_common.h"

/* SYSCFG registers */
struct stm32_syscfg {
	u32_t memrmp;
	u32_t cfgr1;
	union syscfg_exticr exticr1;
	union syscfg_exticr exticr2;
	union syscfg_exticr exticr3;
	union syscfg_exticr exticr4;
	u32_t scsr;
	u32_t cfgr2;
	u32_t swpr;
	u32_t skr;
};

#endif /* _STM32_SYSCFG_REGISTERS_H_ */
