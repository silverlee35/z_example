/*
 * Copyright (c) 2019 Oticon A/S
 *
 * SPDX-License-Identifier: Apache-2.0
 */


#ifndef EDTT_DRIVER_H
#define EDTT_DRIVER_H

#include <stdlib.h>
#include <stdbool.h>
#include "zephyr/types.h"

#define EDTTT_NONBLOCK 0
#define EDTTT_BLOCK 1

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Generic EDTT interface
 */
bool edtt_start(void);
bool edtt_stop(void);
int edtt_read(u8_t *ptr, size_t size, int flags);
int edtt_write(u8_t *ptr, size_t size, int flags);

#if defined(CONFIG_ARCH_POSIX)
/**
 * Exclusive functions for the BabbleSim driver
 */
void enable_edtt_mode(void);
void set_edtt_autoshutdown(bool mode);
#endif /* CONFIG_POSIX_ARCH */

#ifdef __cplusplus
}
#endif

#endif /* EDTT_DRIVER_H */
