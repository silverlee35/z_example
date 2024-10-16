/*
 * Copyright (c) 2019 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <lvgl.h>
#include "lvgl_display.h"

void lvgl_flush_cb_mono(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
	uint16_t w = area->x2 - area->x1 + 1;
	uint16_t h = area->y2 - area->y1 + 1;
	struct lvgl_disp_data *data = (struct lvgl_disp_data *)lv_display_get_user_data(display);
	const struct device *display_dev = data->display_dev;
	struct display_buffer_descriptor desc;
	const bool is_epd = data->cap.screen_info & SCREEN_INFO_EPD;
	const bool is_last = lv_display_flush_is_last(display);

	/* Needed because LVGL reserves 2x4 bytes in the buffer for the color palette. */
	px_map += 8;

	if (is_epd && !data->blanking_on && !is_last) {
		/*
		 * Turn on display blanking when using an EPD
		 * display. This prevents updates and the associated
		 * flicker if the screen is rendered in multiple
		 * steps.
		 */
		display_blanking_on(display_dev);
		data->blanking_on = true;
	}

	desc.buf_size = (w * h) / 8U;
	desc.width = w;
	desc.pitch = w;
	desc.height = h;
	display_write(display_dev, area->x1, area->y1, &desc, (void *)px_map);
	if (data->cap.screen_info & SCREEN_INFO_DOUBLE_BUFFER) {
		display_write(display_dev, area->x1, area->y1, &desc, (void *)px_map);
	}

	if (is_epd && is_last && data->blanking_on) {
		/*
		 * The entire screen has now been rendered. Update the
		 * display by disabling blanking.
		 */
		display_blanking_off(display_dev);
		data->blanking_on = false;
	}

	lv_display_flush_ready(display);
}

void lvgl_set_px_cb_mono(lv_display_t *display, uint8_t *buf, int32_t buf_w, int32_t x, int32_t y,
			 lv_color_t color, lv_opa_t opa)
{
	struct lvgl_disp_data *data = (struct lvgl_disp_data *)lv_display_get_user_data(display);
	uint8_t *buf_xy;
	uint8_t bit;

	if (data->cap.screen_info & SCREEN_INFO_MONO_VTILED) {
		buf_xy = buf + x + y / 8 * buf_w;

		if (data->cap.screen_info & SCREEN_INFO_MONO_MSB_FIRST) {
			bit = 7 - y % 8;
		} else {
			bit = y % 8;
		}
	} else {
		buf_xy = buf + x / 8 + y * buf_w / 8;

		if (data->cap.screen_info & SCREEN_INFO_MONO_MSB_FIRST) {
			bit = 7 - x % 8;
		} else {
			bit = x % 8;
		}
	}

	if (data->cap.current_pixel_format == PIXEL_FORMAT_MONO10) {
		if (lv_color_to_u32(color) == 0) {
			*buf_xy &= ~BIT(bit);
		} else {
			*buf_xy |= BIT(bit);
		}
	} else {
		if (lv_color_to_u32(color) == 0) {
			*buf_xy |= BIT(bit);
		} else {
			*buf_xy &= ~BIT(bit);
		}
	}
}

void lvgl_rounder_cb_mono(lv_event_t *e)
{
	lv_area_t *area = lv_event_get_param(e);
	lv_display_t *display = lv_event_get_user_data(e);
	struct lvgl_disp_data *data = (struct lvgl_disp_data *)lv_display_get_user_data(display);

	if (data->cap.screen_info & SCREEN_INFO_X_ALIGNMENT_WIDTH) {
		area->x1 = 0;
		area->x2 = data->cap.x_resolution - 1;
	} else {
		if (data->cap.screen_info & SCREEN_INFO_MONO_VTILED) {
			area->y1 &= ~0x7;
			area->y2 |= 0x7;
		} else {
			area->x1 &= ~0x7;
			area->x2 |= 0x7;
		}
	}
}
