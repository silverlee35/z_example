/*
 * Copyright (c) 2018-2020 Jan Van Winkel <jan.van_winkel@dxplore.eu>
 * Copyright (c) 2020 Teslabs Engineering S.L.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_LIB_GUI_LVGL_LV_CONF_H_
#define ZEPHYR_LIB_GUI_LVGL_LV_CONF_H_

#include <sys/util.h>

#ifdef CONFIG_LVGL_USE_DEBUG
#include <sys/__assert.h>
#define LV_DEBUG_ASSERT(expr, msg, value) __ASSERT(expr, msg)
#endif

/* Graphical settings */

#define LV_HOR_RES_MAX	CONFIG_LVGL_HOR_RES_MAX
#define LV_VER_RES_MAX	CONFIG_LVGL_VER_RES_MAX

#ifdef CONFIG_LVGL_COLOR_DEPTH_32
#define LV_COLOR_DEPTH	32
#elif defined(CONFIG_LVGL_COLOR_DEPTH_16)
#define LV_COLOR_DEPTH	16
#elif defined(CONFIG_LVGL_COLOR_DEPTH_8)
#define LV_COLOR_DEPTH	8
#elif defined(CONFIG_LVGL_COLOR_DEPTH_1)
#define LV_COLOR_DEPTH	1
#endif

#define LV_COLOR_16_SWAP IS_ENABLED(CONFIG_LVGL_COLOR_16_SWAP)

#define LV_COLOR_SCREEN_TRANSP IS_ENABLED(CONFIG_LVGL_COLOR_SCREEN_TRANSP)

#ifdef CONFIG_LVGL_COLOR_TRANSP_RED
#define LV_COLOR_TRANSP LV_COLOR_RED
#elif defined(CONFIG_LVGL_COLOR_TRANSP_GREEN)
#define LV_COLOR_TRANSP LV_COLOR_LIME
#elif defined(CONFIG_LVGL_COLOR_TRANSP_BLUE)
#define LV_COLOR_TRANSP LV_COLOR_BLUE
#elif defined(CONFIG_LVGL_COLOR_TRANSP_CUSTOM)
#define LV_COLOR_TRANSP \
	LV_COLOR_MAKE(CONFIG_LVGL_CUSTOM_COLOR_TRANSP_RED, \
		      CONFIG_LVGL_CUSTOM_COLOR_TRANSP_GREEN, \
		      CONFIG_LVGL_CUSTOM_COLOR_TRANSP_BLUE)
#endif

#define LV_ANTIALIAS IS_ENABLED(CONFIG_LVGL_ANTIALIAS)

#define LV_DISP_DEF_REFR_PERIOD	CONFIG_LVGL_DISP_DEF_REFR_PERIOD

#define LV_DPI CONFIG_LVGL_DPI

#define LV_DISP_SMALL_LIMIT CONFIG_LVGL_DISP_SMALL_LIMIT
#define LV_DISP_MEDIUM_LIMIT CONFIG_LVGL_DISP_MEDIUM_LIMIT
#define LV_DISP_LARGE_LIMIT CONFIG_LVGL_DISP_LARGE_LIMIT

typedef short lv_coord_t;

/* Memory manager settings */

#define LV_MEM_CUSTOM 1

#define LV_MEMCPY_MEMSET_STD 1

#if defined(CONFIG_LVGL_MEM_POOL_HEAP_LIB_C)

#define LV_MEM_CUSTOM_INCLUDE	"stdlib.h"
#define LV_MEM_CUSTOM_ALLOC	malloc
#define LV_MEM_CUSTOM_FREE	free

#else

#define LV_MEM_CUSTOM_INCLUDE	"lvgl_mem.h"
#define LV_MEM_CUSTOM_ALLOC	lvgl_malloc
#define LV_MEM_CUSTOM_FREE	lvgl_free

#endif

#define LV_ENABLE_GC 0

/* Input device settings */

#define LV_INDEV_DEF_READ_PERIOD CONFIG_LVGL_INDEV_DEF_READ_PERIOD

#define LV_INDEV_DEF_DRAG_LIMIT CONFIG_LVGL_INDEV_DEF_DRAG_LIMIT

#define LV_INDEV_DEF_DRAG_THROW CONFIG_LVGL_INDEV_DEF_DRAG_THROW

#define LV_INDEV_DEF_LONG_PRESS_TIME CONFIG_LVGL_INDEV_DEF_LONG_PRESS_TIME

#define LV_INDEV_DEF_LONG_PRESS_REP_TIME CONFIG_LVGL_INDEV_DEF_LONG_PRESS_REP_TIME

#define LV_INDEV_DEF_GESTURE_LIMIT CONFIG_LVGL_INDEV_DEF_GESTURE_LIMIT

#define LV_INDEV_DEF_GESTURE_MIN_VELOCITY CONFIG_LVGL_INDEV_DEF_GESTURE_MIN_VELOCITY

/* Feature usage */

#define LV_USE_ANIMATION IS_ENABLED(CONFIG_LVGL_USE_ANIMATION)

#if LV_USE_ANIMATION
typedef void *lv_anim_user_data_t;
#endif

#define LV_USE_SHADOW IS_ENABLED(CONFIG_LVGL_USE_SHADOW)

#if LV_USE_SHADOW
#define LV_SHADOW_CACHE_SIZE CONFIG_LVGL_SHADOW_CACHE_SIZE
#endif

#define LV_USE_OUTLINE IS_ENABLED(CONFIG_LVGL_USE_OUTLINE)

#define LV_USE_PATTERN IS_ENABLED(CONFIG_LVGL_USE_PATTERN)

#define LV_USE_VALUE_STR IS_ENABLED(CONFIG_LVGL_USE_VALUE_STR)

#define LV_USE_BLEND_MODES IS_ENABLED(CONFIG_LVGL_USE_BLEND_MODES)

#define LV_USE_OPA_SCALE IS_ENABLED(CONFIG_LVGL_USE_OPA_SCALE)

#define LV_USE_IMG_TRANSFORM IS_ENABLED(CONFIG_LVGL_USE_IMG_TRANSFORM)

#define LV_USE_GROUP IS_ENABLED(CONFIG_LVGL_USE_GROUP)

#if LV_USE_GROUP
typedef void *lv_group_user_data_t;
#endif

#define LV_USE_GPU IS_ENABLED(CONFIG_LVGL_USE_GPU)

#define LV_USE_FILESYSTEM IS_ENABLED(CONFIG_LVGL_USE_FILESYSTEM)

#if LV_USE_FILESYSTEM
typedef void *lv_fs_drv_user_data_t;
#endif

#define LV_USE_USER_DATA 1

#define LV_USE_PERF_MONITOR IS_ENABLED(CONFIG_LVGL_USE_PERF_MONITOR)

#define LV_USE_API_EXTENSION_V6 IS_ENABLED(CONFIG_LVGL_USE_API_EXTENSION_V6)
#define LV_USE_API_EXTENSION_V7 IS_ENABLED(CONFIG_LVGL_USE_API_EXTENSION_V7)

/* Image decoder and cache */

#define LV_IMG_CF_INDEXED IS_ENABLED(CONFIG_LVGL_IMG_CF_INDEXED)

#define LV_IMG_CF_ALPHA IS_ENABLED(CONFIG_LVGL_IMG_CF_ALPHA)

#define LV_IMG_CACHE_DEF_SIZE CONFIG_LVGL_IMG_CACHE_DEF_SIZE

typedef void *lv_img_decoder_user_data_t;

/* Compiler settings */

#define LV_BIG_ENDIAN_SYSTEM IS_ENABLED(CONFIG_BIG_ENDIAN)

#define LV_ATTRIBUTE_TICK_INC

#define LV_ATTRIBUTE_TASK_HANDLER

#define LV_ATTRIBUTE_MEM_ALIGN

#define LV_ATTRIBUTE_LARGE_CONST

#define LV_EXPORT_CONST_INT(int_value)

/* HAL settings */

#define LV_TICK_CUSTOM			1
#define LV_TICK_CUSTOM_INCLUDE		"kernel.h"
#define LV_TICK_CUSTOM_SYS_TIME_EXPR	(k_uptime_get_32())

typedef void *lv_disp_drv_user_data_t;
typedef void *lv_indev_drv_user_data_t;

/* Log settings */

#if CONFIG_LVGL_LOG_LEVEL == 0
#define LV_USE_LOG 0
#else
#define LV_USE_LOG 1

#if CONFIG_LVGL_LOG_LEVEL == 1
#define LV_LOG_LEVEL LV_LOG_LEVEL_ERROR
#elif CONFIG_LVGL_LOG_LEVEL == 2
#define LV_LOG_LEVEL LV_LOG_LEVEL_WARN
#elif CONFIG_LVGL_LOG_LEVEL == 3
#define LV_LOG_LEVEL LV_LOG_LEVEL_INFO
#elif CONFIG_LVGL_LOG_LEVEL == 4
#define LV_LOG_LEVEL LV_LOG_LEVEL_TRACE
#endif

#define LV_LOG_PRINTF 0
#endif

/* Debug settings */

#define LV_USE_DEBUG IS_ENABLED(CONFIG_LVGL_USE_DEBUG)

#if LV_USE_DEBUG

#define LV_USE_ASSERT_NULL IS_ENABLED(CONFIG_LVGL_USE_ASSERT_NULL)

#define LV_USE_ASSERT_MEM IS_ENABLED(CONFIG_LVGL_USE_ASSERT_MEM)

#define LV_USE_ASSERT_MEM_INTEGRITY IS_ENABLED(CONFIG_LVGL_USE_ASSERT_MEM_INTEGRITY)

#define LV_USE_ASSERT_STR IS_ENABLED(CONFIG_LVGL_USE_ASSERT_STR)

#define LV_USE_ASSERT_OBJ IS_ENABLED(CONFIG_LVGL_USE_ASSERT_OBJ)

#define LV_USE_ASSERT_STYLE IS_ENABLED(CONFIG_LVGL_USE_ASSERT_STYLE)

#endif /* LV_USE_DEBUG */

/* THEME USAGE */

/* Empty theme */
#define LV_USE_THEME_EMPTY IS_ENABLED(CONFIG_LVGL_USE_THEME_EMPTY)

#if LV_USE_THEME_EMPTY
#define LV_THEME_DEFAULT_FLAG 0
#define LV_THEME_DEFAULT_INIT lv_theme_empty_init
#endif

/* Material theme */
#define LV_USE_THEME_MATERIAL IS_ENABLED(CONFIG_LVGL_USE_THEME_MATERIAL)

#if LV_USE_THEME_MATERIAL
#define LV_THEME_DEFAULT_INIT lv_theme_material_init

#if defined(CONFIG_LVGL_THEME_MATERIAL_LIGHT)
#define LVGL_THEME_MATERIAL_LIGHT_DARK_FLAG LV_THEME_MATERIAL_FLAG_LIGHT
#elif defined(CONFIG_LVGL_THEME_MATERIAL_DARK)
#define LVGL_THEME_MATERIAL_LIGHT_DARK_FLAG LV_THEME_MATERIAL_FLAG_DARK
#else
#define LVGL_THEME_MATERIAL_LIGHT_DARK_FLAG 0
#endif

#if defined(CONFIG_LVGL_THEME_MATERIAL_FLAG_NO_TRANSITION)
#define LVGL_THEME_MATERIAL_FLAG_NO_TRANSITION LV_THEME_MATERIAL_FLAG_NO_TRANSITION
#else
#define LVGL_THEME_MATERIAL_FLAG_NO_TRANSITION 0
#endif

#if defined(CONFIG_LVGL_THEME_MATERIAL_FLAG_NO_FOCUS)
#define LVGL_THEME_MATERIAL_FLAG_NO_FOCUS LV_THEME_MATERIAL_FLAG_NO_FOCUS
#else
#define LVGL_THEME_MATERIAL_FLAG_NO_FOCUS 0
#endif

#define LV_THEME_DEFAULT_FLAG \
	(LVGL_THEME_MATERIAL_LIGHT_DARK_FLAG | \
	LVGL_THEME_MATERIAL_FLAG_NO_TRANSITION | \
	LVGL_THEME_MATERIAL_FLAG_NO_FOCUS)

#endif

/* Mono-color theme */
#define LV_USE_THEME_MONO IS_ENABLED(CONFIG_LVGL_USE_THEME_MONO)

#if LV_USE_THEME_MONO
#define LV_THEME_DEFAULT_INIT lv_theme_mono_init
#define LV_THEME_DEFAULT_FLAG 0
#endif

/* Custom theme */
#if CONFIG_LVGL_USE_THEME_CUSTOM

lv_theme_t *LVGL_THEME_CUSTOM_INIT_FUNCTION(
	lv_color_t color_primary, lv_color_t color_secondary, uint32_t flags,
	const lv_font_t *font_small, const lv_font_t *font_normal,
	const lv_font_t *font_subtitle, const lv_font_t *font_title);

#define LV_THEME_DEFAULT_INIT LVGL_THEME_CUSTOM_INIT_FUNCTION

#endif

/* Theme primary color */
#if defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_WHITE)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_WHITE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_SILVER)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_SILVER
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_GRAY)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_GRAY
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_BLACK)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_BLACK
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_RED)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_RED
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_MAROON)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_MAROON
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_YELLOW)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_YELLOW
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_OLIVE)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_OLIVE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_LIME)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_LIME
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_GREEN)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_GREEN
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_CYAN)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_CYAN
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_AQUA)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_AQUA
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_TEAL)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_TEAL
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_BLUE)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_BLUE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_NAVY)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_NAVY
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_MAGENTA)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_MAGENTA
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_PURPLE)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_PURPLE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_ORANGE)
#define LV_THEME_DEFAULT_COLOR_PRIMARY LV_COLOR_ORANGE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_CUSTOM)
#define LV_THEME_DEFAULT_COLOR_PRIMARY \
	LV_COLOR_MAKE(CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_CUSTOM_RED, \
		      CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_CUSTOM_GREEN, \
		      CONFIG_LVGL_THEME_DEFAULT_COLOR_PRIMARY_CUSTOM_BLUE)
#endif

/* Theme secondary color */
#if defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_WHITE)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_WHITE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_SILVER)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_SILVER
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_GRAY)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_GRAY
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_BLACK)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_BLACK
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_RED)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_RED
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_MAROON)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_MAROON
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_YELLOW)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_YELLOW
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_OLIVE)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_OLIVE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_LIME)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_LIME
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_GREEN)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_GREEN
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_CYAN)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_CYAN
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_AQUA)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_AQUA
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_TEAL)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_TEAL
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_BLUE)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_BLUE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_NAVY)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_NAVY
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_MAGENTA)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_MAGENTA
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_PURPLE)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_PURPLE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_ORANGE)
#define LV_THEME_DEFAULT_COLOR_SECONDARY LV_COLOR_ORANGE
#elif defined(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_CUSTOM)
#define LV_THEME_DEFAULT_COLOR_SECONDARY \
	LV_COLOR_MAKE(CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_CUSTOM_RED, \
		      CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_CUSTOM_GREEN, \
		      CONFIG_LVGL_THEME_DEFAULT_COLOR_SECONDARY_CUSTOM_BLUE)
#endif

/* Theme small font */
#if defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_8)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_10)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_10)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_12)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_12)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_14)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_14)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_16)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_16)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_18)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_18)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_20)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_20)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_22)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_22)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_24)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_24)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_26)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_26)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_28)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_28)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_30)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_30)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_32)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_32)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_34)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_34)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_36)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_36)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_38)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_38)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_40)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_40)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_42)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_42)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_44)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_44)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_46)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_46)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_48)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_48)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_12_SUBPX)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_12_subpx)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_MONTSERRAT_28_COMPRESSED)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_montserrat_28_compressed)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_DEJAVU_16_PERSIAN_HEBREW)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_dejavu_16_persian_hebrew)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_SIMSUN_16_CJK)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_simsun_16_cjk)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_UNSCII_8)
#define LV_THEME_DEFAULT_FONT_SMALL (&lv_font_unscii_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SMALL_CUSTOM)
extern void *lv_theme_default_font_small_custom_ptr;
#define LV_THEME_DEFAULT_FONT_SMALL \
	((lv_font_t *)lv_theme_default_font_small_custom_ptr)
#endif

/* Theme normal font */
#if defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_8)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_10)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_10)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_12)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_12)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_14)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_14)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_16)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_16)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_18)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_18)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_20)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_20)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_22)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_22)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_24)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_24)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_26)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_26)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_28)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_28)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_30)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_30)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_32)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_32)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_34)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_34)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_36)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_36)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_38)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_38)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_40)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_40)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_42)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_42)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_44)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_44)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_46)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_46)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_48)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_48)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_12_SUBPX)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_12_subpx)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_MONTSERRAT_28_COMPRESSED)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_montserrat_28_compressed)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_DEJAVU_16_PERSIAN_HEBREW)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_dejavu_16_persian_hebrew)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_SIMSUN_16_CJK)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_simsun_16_cjk)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_UNSCII_8)
#define LV_THEME_DEFAULT_FONT_NORMAL (&lv_font_unscii_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_NORMAL_CUSTOM)
extern void *lv_theme_default_font_normal_custom_ptr;
#define LV_THEME_DEFAULT_FONT_NORMAL \
	((lv_font_t *)lv_theme_default_font_normal_custom_ptr)
#endif

/* Theme subtitle font */
#if defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_8)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_10)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_10)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_12)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_12)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_14)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_14)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_16)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_16)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_18)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_18)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_20)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_20)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_22)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_22)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_24)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_24)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_26)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_26)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_28)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_28)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_30)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_30)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_32)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_32)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_34)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_34)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_36)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_36)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_38)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_38)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_40)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_40)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_42)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_42)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_44)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_44)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_46)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_46)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_48)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_48)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_12_SUBPX)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_12_subpx)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_MONTSERRAT_28_COMPRESSED)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_montserrat_28_compressed)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_DEJAVU_16_PERSIAN_HEBREW)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_dejavu_16_persian_hebrew)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_SIMSUN_16_CJK)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_simsun_16_cjk)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_UNSCII_8)
#define LV_THEME_DEFAULT_FONT_SUBTITLE (&lv_font_unscii_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_SUBTITLE_CUSTOM)
extern void *lv_theme_default_font_subtitle_custom_ptr;
#define LV_THEME_DEFAULT_FONT_SUBTITLE \
	((lv_font_t *)lv_theme_default_font_subtitle_custom_ptr)
#endif

/* Theme title font */
#if defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_8)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_10)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_10)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_12)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_12)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_14)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_14)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_16)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_16)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_18)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_18)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_20)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_20)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_22)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_22)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_24)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_24)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_26)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_26)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_28)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_28)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_30)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_30)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_32)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_32)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_34)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_34)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_36)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_36)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_38)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_38)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_40)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_40)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_42)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_42)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_44)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_44)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_46)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_46)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_48)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_48)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_12_SUBPX)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_12_subpx)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_MONTSERRAT_28_COMPRESSED)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_montserrat_28_compressed)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_DEJAVU_16_PERSIAN_HEBREW)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_dejavu_16_persian_hebrew)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_SIMSUN_16_CJK)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_simsun_16_cjk)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_UNSCII_8)
#define LV_THEME_DEFAULT_FONT_TITLE (&lv_font_unscii_8)
#elif defined(CONFIG_LVGL_THEME_DEFAULT_FONT_TITLE_CUSTOM)
extern void *lv_theme_default_font_title_custom_ptr;
#define LV_THEME_DEFAULT_FONT_TITLE \
	((lv_font_t *)lv_theme_default_font_title_custom_ptr)
#endif

/* FONT USAGE */

#define LV_FONT_MONTSERRAT_8 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_8)

#define LV_FONT_MONTSERRAT_10 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_10)

#define LV_FONT_MONTSERRAT_12 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_12)

#define LV_FONT_MONTSERRAT_14 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_14)

#define LV_FONT_MONTSERRAT_16 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_16)

#define LV_FONT_MONTSERRAT_18 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_18)

#define LV_FONT_MONTSERRAT_20 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_20)

#define LV_FONT_MONTSERRAT_22 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_22)

#define LV_FONT_MONTSERRAT_24 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_24)

#define LV_FONT_MONTSERRAT_26 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_26)

#define LV_FONT_MONTSERRAT_28 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_28)

#define LV_FONT_MONTSERRAT_30 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_30)

#define LV_FONT_MONTSERRAT_32 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_32)

#define LV_FONT_MONTSERRAT_34 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_34)

#define LV_FONT_MONTSERRAT_36 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_36)

#define LV_FONT_MONTSERRAT_38 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_38)

#define LV_FONT_MONTSERRAT_40 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_40)

#define LV_FONT_MONTSERRAT_42 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_42)

#define LV_FONT_MONTSERRAT_44 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_44)

#define LV_FONT_MONTSERRAT_46 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_46)

#define LV_FONT_MONTSERRAT_48 IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_48)

#define LV_FONT_MONTSERRAT_12_SUBPX IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_12_SUBPX)

#define LV_FONT_MONTSERRAT_28_COMPRESSED IS_ENABLED(CONFIG_LVGL_FONT_MONTSERRAT_28_COMPRESSED)

#define LV_FONT_DEJAVU_16_PERSIAN_HEBREW IS_ENABLED(CONFIG_LVGL_FONT_DEJAVU_16_PERSIAN_HEBREW)

#define LV_FONT_SIMSUN_16_CJK IS_ENABLED(CONFIG_LVGL_FONT_SIMSUN_16_CJK)

#define LV_FONT_UNSCII_8 IS_ENABLED(CONFIG_LVGL_FONT_UNSCII_8)

#define LV_USE_FONT_COMPRESSED IS_ENABLED(CONFIG_LVGL_USE_FONT_COMPRESSED)

#define LV_USE_FONT_SUBPX IS_ENABLED(CONFIG_LVGL_USE_FONT_SUBPX)

#if LV_USE_FONT_SUBPX
#define LV_FONT_SUBPX_BGR IS_ENABLED(CONFIG_LVGL_FONT_SUBPX_BGR)
#endif

#define LV_FONT_CUSTOM_DECLARE

typedef void *lv_font_user_data_t;

/* Text settings */

#ifdef CONFIG_LVGL_TXT_ENC_ASCII
#define LV_TXT_ENC LV_TXT_ENC_ASCII
#elif defined(CONFIG_LVGL_TXT_ENC_UTF8)
#define LV_TXT_ENC LV_TXT_ENC_UTF8
#endif

#define LV_TXT_BREAK_CHARS CONFIG_LVGL_TXT_BREAK_CHARS

#define LV_TXT_LINE_BREAK_LONG_LEN CONFIG_LVGL_TXT_LINE_BREAK_LONG_LEN

#define LV_TXT_LINE_BREAK_LONG_PRE_MIN_LEN  \
	CONFIG_LVGL_TXT_LINE_BREAK_LONG_PRE_MIN_LEN

#define LV_TXT_LINE_BREAK_LONG_POST_MIN_LEN \
	CONFIG_LVGL_TXT_LINE_BREAK_LONG_POST_MIN_LEN

#define LV_TXT_COLOR_CMD CONFIG_LVGL_TXT_COLOR_CMD

#define LV_USE_BIDI IS_ENABLED(CONFIG_LVGL_USE_BIDI)

#if LV_USE_BIDI

#ifdef CONFIG_LVGL_BIDI_DIR_LTR
#define LV_BIDI_BASE_DIR_DEF LV_BIDI_DIR_LTR
#elif defined(CONFIG_LVGL_BIDI_DIR_RTL)
#define LV_BIDI_BASE_DIR_DEF LV_BIDI_DIR_RTL
#else
#define LV_BIDI_BASE_DIR_DEF LV_BIDI_DIR_AUTO
#endif

#endif

#define LV_USE_ARABIC_PERSIAN_CHARS IS_ENABLED(CONFIG_LVGL_USE_ARABIC_PERSIAN_CHARS)

#define LV_SPRINTF_CUSTOM 1

#if LV_SPRINTF_CUSTOM
#define LV_SPRINTF_INCLUDE "stdio.h"
#define lv_snprintf snprintf
#define lv_vsnprintf vsnprintf
#endif

/* LV_OBJ SETTINGS */

typedef void *lv_obj_user_data_t;

#define LV_USE_OBJ_REALIGN IS_ENABLED(CONFIG_LVGL_USE_OBJ_REALIGN)

#if defined(CONFIG_LVGL_EXT_CLICK_AREA_OFF)
#define LV_USE_EXT_CLICK_AREA  LV_EXT_CLICK_AREA_OFF
#elif defined(CONFIG_LVGL_EXT_CLICK_AREA_TINY)
#define LV_USE_EXT_CLICK_AREA  LV_EXT_CLICK_AREA_TINY
#elif defined(CONFIG_LVGL_EXT_CLICK_AREA_FULL)
#define LV_USE_EXT_CLICK_AREA  LV_EXT_CLICK_AREA_FULL
#endif

/* LV OBJ X USAGE */

#define LV_USE_ARC IS_ENABLED(CONFIG_LVGL_USE_ARC)

#define LV_USE_BAR IS_ENABLED(CONFIG_LVGL_USE_BAR)

#define LV_USE_BTN IS_ENABLED(CONFIG_LVGL_USE_BTN)

#define LV_USE_BTNMATRIX IS_ENABLED(CONFIG_LVGL_USE_BTNMATRIX)

#define LV_USE_CALENDAR IS_ENABLED(CONFIG_LVGL_USE_CALENDAR)

#if LV_USE_CALENDAR
#define LV_CALENDAR_WEEK_STARTS_MONDAY IS_ENABLED(CONFIG_LVGL_CALENDAR_WEEK_STARTS_MONDAY)
#endif

#define LV_USE_CANVAS IS_ENABLED(CONFIG_LVGL_USE_CANVAS)

#define LV_USE_CHECKBOX IS_ENABLED(CONFIG_LVGL_USE_CHECKBOX)

#define LV_USE_CHART IS_ENABLED(CONFIG_LVGL_USE_CHART)
#if LV_USE_CHART
#define LV_CHART_AXIS_TICK_LABEL_MAX_LEN \
	CONFIG_LVGL_CHART_AXIS_TICK_LABEL_MAX_LEN
#endif

#define LV_USE_CONT IS_ENABLED(CONFIG_LVGL_USE_CONT)

#define LV_USE_CPICKER IS_ENABLED(CONFIG_LVGL_USE_CPICKER)

#define LV_USE_DROPDOWN IS_ENABLED(CONFIG_LVGL_USE_DROPDOWN)
#if LV_USE_DROPDOWN
#define LV_DROPDOWN_DEF_ANIM_TIME CONFIG_LVGL_DROPDOWN_DEF_ANIM_TIME
#endif

#define LV_USE_GAUGE IS_ENABLED(CONFIG_LVGL_USE_GAUGE)

#define LV_USE_IMG IS_ENABLED(CONFIG_LVGL_USE_IMG)

#define LV_USE_IMGBTN IS_ENABLED(CONFIG_LVGL_USE_IMGBTN)
#if LV_USE_IMGBTN
#define LV_IMGBTN_TILED IS_ENABLED(CONFIG_LVGL_IMGBTN_TILED)
#endif

#define LV_USE_KEYBOARD IS_ENABLED(CONFIG_LVGL_USE_KEYBOARD)

#define LV_USE_LABEL IS_ENABLED(CONFIG_LVGL_USE_LABEL)
#if LV_USE_LABEL
#define LV_LABEL_DEF_SCROLL_SPEED CONFIG_LVGL_LABEL_DEF_SCROLL_SPEED
#define LV_LABEL_WAIT_CHAR_COUNT \
	CONFIG_LVGL_LABEL_WAIT_CHAR_COUNT
#define LV_LABEL_TEXT_SEL IS_ENABLED(CONFIG_LVGL_LABEL_TEXT_SEL)
#define LV_LABEL_LONG_TXT_HINT IS_ENABLED(CONFIG_LVGL_LABEL_LONG_TXT_HINT)
#endif

#define LV_USE_LED IS_ENABLED(CONFIG_LVGL_USE_LED)
#if LV_USE_LED
#define LV_LED_BRIGHT_MIN CONFIG_LVGL_LED_BRIGHT_MIN
#define LV_LED_BRIGHT_MAX CONFIG_LVGL_LED_BRIGHT_MAX
#endif

#define LV_USE_LINE IS_ENABLED(CONFIG_LVGL_USE_LINE)

#define LV_USE_LIST IS_ENABLED(CONFIG_LVGL_USE_LIST)
#if LV_USE_LIST
#define LV_LIST_DEF_ANIM_TIME CONFIG_LVGL_LIST_DEF_ANIM_TIME
#endif

#define LV_USE_LINEMETER IS_ENABLED(CONFIG_LVGL_USE_LINEMETER)
#if LV_USE_LINEMETER
#if defined(CONFIG_LVGL_LINEMETER_PRECISE_NO_EXTRA)
#define LV_LINEMETER_PRECISE 0
#elif defined(CONFIG_LVGL_LINEMETER_PRECISE_SOME_EXTRA)
#define LV_LINEMETER_PRECISE 1
#elif defined(CONFIG_LVGL_LINEMETER_PRECISE_BEST)
#define LV_LINEMETER_PRECISE 2
#endif
#endif

#define LV_USE_OBJMASK IS_ENABLED(CONFIG_LVGL_USE_OBJMASK)

#define LV_USE_MSGBOX IS_ENABLED(CONFIG_LVGL_USE_MSGBOX)

#define LV_USE_PAGE IS_ENABLED(CONFIG_LVGL_USE_PAGE)
#if LV_USE_PAGE
#define LV_PAGE_DEF_ANIM_TIME CONFIG_LVGL_PAGE_DEF_ANIM_TIME
#endif

#define LV_USE_SPINNER IS_ENABLED(CONFIG_LVGL_USE_SPINNER)
#if LV_USE_SPINNER
#define LV_SPINNER_DEF_ARC_LENGTH CONFIG_LVGL_SPINNER_DEF_ARC_LENGTH
#define LV_SPINNER_DEF_SPIN_TIME CONFIG_LVGL_SPINNER_DEF_SPIN_TIME

#if defined(CONFIG_LVGL_SPINNER_DEF_ANIM_SPINNING_ARC)
#define LV_SPINNER_DEF_ANIM LV_SPINNER_TYPE_SPINNING_ARC
#elif defined(CONFIG_LVGL_SPINNER_DEF_ANIM_FILLSPIN_ARC)
#define LV_SPINNER_DEF_ANIM LV_SPINNER_TYPE_FILLSPIN_ARC
#elif defined(CONFIG_LVGL_SPINNER_DEF_ANIM_CONSTANT_ARC)
#define LV_SPINNER_DEF_ANIM LV_SPINNER_TYPE_CONSTANT_ARC
#endif
#endif

#define LV_USE_ROLLER IS_ENABLED(CONFIG_LVGL_USE_ROLLER)
#if LV_USE_ROLLER
#define LV_ROLLER_DEF_ANIM_TIME	CONFIG_LVGL_ROLLER_DEF_ANIM_TIME
#define LV_ROLLER_INF_PAGES	CONFIG_LVGL_ROLLER_INF_PAGES
#endif

#define LV_USE_SLIDER IS_ENABLED(CONFIG_LVGL_USE_SLIDER)

#define LV_USE_SPINBOX IS_ENABLED(CONFIG_LVGL_USE_SPINBOX)

#define LV_USE_SWITCH IS_ENABLED(CONFIG_LVGL_USE_SWITCH)

#define LV_USE_TEXTAREA IS_ENABLED(CONFIG_LVGL_USE_TEXTAREA)
#if LV_USE_TEXTAREA
#define LV_TA_DEF_CURSOR_BLINK_TIME CONFIG_LVGL_TA_DEF_CURSOR_BLINK_TIME
#define LV_TA_DEF_PWD_SHOW_TIME CONFIG_LVGL_TA_DEF_PWD_SHOW_TIME
#endif

#define LV_USE_TABLE IS_ENABLED(CONFIG_LVGL_USE_TABLE)
#if LV_USE_TABLE
#define LV_TABLE_COL_MAX CONFIG_LVGL_TABLE_COL_MAX
#endif

#define LV_USE_TABVIEW IS_ENABLED(CONFIG_LVGL_USE_TABVIEW)
#if LV_USE_TABVIEW
#define LV_TABVIEW_DEF_ANIM_TIME CONFIG_LVGL_TABVIEW_DEF_ANIM_TIME
#endif

#define LV_USE_TILEVIEW IS_ENABLED(CONFIG_LVGL_USE_TILEVIEW)
#if LV_USE_TILEVIEW
#define LV_TILEVIEW_DEF_ANIM_TIME CONFIG_LVGL_TILEVIEW_DEF_ANIM_TIME
#endif

#define LV_USE_WIN IS_ENABLED(CONFIG_LVGL_USE_WIN)

#endif /* ZEPHYR_LIB_GUI_LVGL_LV_CONF_H_ */
