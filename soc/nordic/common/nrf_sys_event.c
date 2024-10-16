/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <nrf_sys_event.h>

static K_SEM_DEFINE(lock_sem, 1, 1);
static uint16_t global_constlat_count;

#if defined(CONFIG_SOC_SERIES_NRF54HX)

#include <hal/nrf_lrcconf.h>

static int enable_global_conslat(void)
{
	nrf_lrcconf_task_trigger(NRF_LRCCONF010, NRF_LRCCONF_TASK_CONSTLAT_ENABLE);

	while (!nrf_lrcconf_constlatstat_check(NRF_LRCCONF010)) {
		k_yield();
	}

	return 0;
}

static int disable_global_conslat(void)
{
	nrf_lrcconf_task_trigger(NRF_LRCCONF010, NRF_LRCCONF_TASK_CONSTLAT_DISABLE);

	while (nrf_lrcconf_constlatstat_check(NRF_LRCCONF010)) {
		k_yield();
	}

	return 0;
}

#elif defined(CONFIG_SOC_SERIES_NRF54LX)

#include <hal/nrf_power.h>

static inline bool nrf_power_constlatstat_check(NRF_POWER_Type *p_reg)
{
	return p_reg->CONSTLATSTAT & POWER_CONSTLATSTAT_STATUS_Msk;
}

static int enable_global_conslat(void)
{
	nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_CONSTLAT);

	while (!nrf_power_constlatstat_check(NRF_POWER)) {
		k_yield();
	}

	return 0;
}

static int disable_global_conslat(void)
{
	nrf_power_task_trigger(NRF_POWER, NRF_POWER_TASK_LOWPWR);

	while (nrf_power_constlatstat_check(NRF_POWER)) {
		k_yield();
	}

	return 0;
}

#else

#include <nrfx_power.h>

static int enable_global_conslat(void)
{
	return nrfx_power_constlat_mode_request() == NRFX_SUCCESS ? 0 : -EAGAIN;
}

static int disable_global_conslat(void)
{
	return nrfx_power_constlat_mode_free() == NRFX_SUCCESS ? 0 : -EAGAIN;
}

#endif

int nrf_sys_event_request_global_constlat(void)
{
	int ret = 0;

	(void)k_sem_take(&lock_sem, K_FOREVER);

	if (global_constlat_count == 0) {
		ret = enable_global_conslat();
	}

	if (ret == 0) {
		global_constlat_count++;
	}

	k_sem_give(&lock_sem);
	return ret;
}

int nrf_sys_event_release_global_constlat(void)
{
	int ret = 0;

	(void)k_sem_take(&lock_sem, K_FOREVER);

	if (global_constlat_count == 1) {
		ret = disable_global_conslat();
	}

	if (ret == 0) {
		global_constlat_count--;
	}

	k_sem_give(&lock_sem);
	return ret;
}
