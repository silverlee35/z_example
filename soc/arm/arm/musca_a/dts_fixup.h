/*
 * Copyright (c) 2018 Linaro Limited
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* SoC level DTS fixup file */

#define DT_NUM_IRQ_PRIO_BITS	DT_ARM_V8M_NVIC_E000E100_ARM_NUM_IRQ_PRIORITY_BITS

#if defined (CONFIG_ARM_NONSECURE_FIRMWARE)
/* CMSDK APB Timers */
#define DT_CMSDK_APB_TIMER0			DT_ARM_CMSDK_TIMER_40000000_BASE_ADDRESS
#define DT_CMSDK_APB_TIMER_0_IRQ		DT_ARM_CMSDK_TIMER_40000000_IRQ_0

#define DT_CMSDK_APB_TIMER1			DT_ARM_CMSDK_TIMER_40001000_BASE_ADDRESS
#define DT_CMSDK_APB_TIMER_1_IRQ 		DT_ARM_CMSDK_TIMER_40001000_IRQ_0

/* CMSDK APB Dual Timer */
#define DT_CMSDK_APB_DTIMER			DT_ARM_CMSDK_DTIMER_40002000_BASE_ADDRESS
#define DT_CMSDK_APB_DUALTIMER_IRQ		DT_ARM_CMSDK_DTIMER_40002000_IRQ_0

/* CMSDK APB Universal Asynchronous Receiver-Transmitter (UART) */
#define DT_PL011_PORT0_BASE_ADDRESS		DT_ARM_PL011_40101000_BASE_ADDRESS
#define DT_PL011_PORT0_IRQ_TX			DT_ARM_PL011_40101000_IRQ_TX
#define DT_PL011_PORT0_IRQ_RX			DT_ARM_PL011_40101000_IRQ_RX
#define DT_PL011_PORT0_IRQ_RXTIM		DT_ARM_PL011_40101000_IRQ_RXTIM
#define DT_PL011_PORT0_IRQ_ERR			DT_ARM_PL011_40101000_IRQ_ERR
#define DT_PL011_PORT0_IRQ_PRI			DT_ARM_PL011_40101000_IRQ_0_PRIORITY
#define DT_PL011_PORT0_BAUD_RATE		DT_ARM_PL011_40101000_CURRENT_SPEED
#define DT_PL011_PORT0_NAME			DT_ARM_PL011_40101000_LABEL

#define DT_PL011_PORT1_BASE_ADDRESS		DT_ARM_PL011_40102000_BASE_ADDRESS
#define DT_PL011_PORT1_IRQ_TX			DT_ARM_PL011_40102000_IRQ_TX
#define DT_PL011_PORT1_IRQ_RX			DT_ARM_PL011_40102000_IRQ_RX
#define DT_PL011_PORT1_IRQ_RXTIM		DT_ARM_PL011_40102000_IRQ_RXTIM
#define DT_PL011_PORT1_IRQ_ERR			DT_ARM_PL011_40102000_IRQ_ERR
#define DT_PL011_PORT1_IRQ_PRI			DT_ARM_PL011_40102000_IRQ_0_PRIORITY
#define DT_PL011_PORT1_BAUD_RATE		DT_ARM_PL011_40102000_CURRENT_SPEED
#define DT_PL011_PORT1_NAME			DT_ARM_PL011_40102000_LABEL

/* SCC */
#define DT_ARM_SCC_BASE_ADDRESS			DT_ARM_SCC_4010C000_BASE_ADDRESS

#else

/* CMSDK APB Timers */
#define DT_CMSDK_APB_TIMER0			DT_ARM_CMSDK_TIMER_50000000_BASE_ADDRESS
#define DT_CMSDK_APB_TIMER_0_IRQ		DT_ARM_CMSDK_TIMER_50000000_IRQ_0

#define DT_CMSDK_APB_TIMER1			DT_ARM_CMSDK_TIMER_50001000_BASE_ADDRESS
#define DT_CMSDK_APB_TIMER_1_IRQ 		DT_ARM_CMSDK_TIMER_50001000_IRQ_0

/* CMSDK APB Dual Timer */
#define DT_CMSDK_APB_DTIMER			DT_ARM_CMSDK_DTIMER_50002000_BASE_ADDRESS
#define DT_CMSDK_APB_DUALTIMER_IRQ		DT_ARM_CMSDK_DTIMER_50002000_IRQ_0

/* CMSDK APB Universal Asynchronous Receiver-Transmitter (UART) */
#define DT_PL011_PORT0_BASE_ADDRESS		DT_ARM_PL011_50101000_BASE_ADDRESS
#define DT_PL011_PORT0_IRQ_TX			DT_ARM_PL011_50101000_IRQ_TX
#define DT_PL011_PORT0_IRQ_RX			DT_ARM_PL011_50101000_IRQ_RX
#define DT_PL011_PORT0_IRQ_RXTIM		DT_ARM_PL011_50101000_IRQ_RXTIM
#define DT_PL011_PORT0_IRQ_ERR			DT_ARM_PL011_50101000_IRQ_ERR
#define DT_PL011_PORT0_IRQ_PRI			DT_ARM_PL011_50101000_IRQ_0_PRIORITY
#define DT_PL011_PORT0_BAUD_RATE		DT_ARM_PL011_50101000_CURRENT_SPEED
#define DT_PL011_PORT0_NAME			DT_ARM_PL011_50101000_LABEL

#define DT_PL011_PORT1_BASE_ADDRESS		DT_ARM_PL011_50102000_BASE_ADDRESS
#define DT_PL011_PORT1_IRQ_TX			DT_ARM_PL011_50102000_IRQ_TX
#define DT_PL011_PORT1_IRQ_RX			DT_ARM_PL011_50102000_IRQ_RX
#define DT_PL011_PORT1_IRQ_RXTIM		DT_ARM_PL011_50102000_IRQ_RXTIM
#define DT_PL011_PORT1_IRQ_ERR			DT_ARM_PL011_50102000_IRQ_ERR
#define DT_PL011_PORT1_IRQ_PRI			DT_ARM_PL011_50102000_IRQ_0_PRIORITY
#define DT_PL011_PORT1_BAUD_RATE		DT_ARM_PL011_50102000_CURRENT_SPEED
#define DT_PL011_PORT1_NAME			DT_ARM_PL011_50102000_LABEL

/* SCC */
#define DT_ARM_SCC_BASE_ADDRESS			DT_ARM_SCC_5010C000_BASE_ADDRESS

#endif

/* End of SoC Level DTS fixup file */
