/**
 * lv_conf.h
 * Configuration for LVGL v8.3.x on Raspberry Pi 5 (SDL2, 16-bit RGB565)
 * Compatible with SquareLine Studio exports.
 */

#ifndef LV_CONF_H
#define LV_CONF_H

/*********************
 * OPERATING SYSTEM
 *********************/
#define LV_USE_OS               LV_OS_POSIX   /* Linux / Raspberry Pi */

/*********************
 * COLOR SETTINGS
 *********************/
#define LV_COLOR_DEPTH          16            /* SquareLine expects 16-bit */
#define LV_COLOR_16_SWAP        0

/*********************
 * DISPLAY SETTINGS
 *********************/
#define LV_HOR_RES_MAX          800           /* set to your screen width  */
#define LV_VER_RES_MAX          480           /* set to your screen height */
#define LV_USE_LOG              1
#define LV_LOG_PRINTF           1

/*********************
 * DRAW ENGINE
 *********************/
#define LV_USE_DRAW_SW          1             /* software renderer (required) */
#define LV_USE_GPU_SDL          0             /* disable SDL GPU acceleration */

/*********************
 * FILESYSTEM
 *********************/
#define LV_USE_FS_STDIO         1
#define LV_USE_FS_POSIX         1

/* Assign drive letters for the filesystem back-ends */
#define LV_FS_POSIX_LETTER      'P'
#define LV_FS_POSIX_PATH        "/"
#define LV_FS_STDIO_LETTER      'S'
#define LV_FS_STDIO_PATH        "/"

/*********************
 * LAYOUTS
 *********************/
#define LV_USE_FLEX             1
#define LV_USE_GRID             1

/*********************
 * THEME
 *********************/
#define LV_USE_THEME_DEFAULT            1
#define LV_THEME_DEFAULT_DARK           0
#define LV_THEME_DEFAULT_GROW           1
#define LV_THEME_DEFAULT_FONT_SMALL     &lv_font_montserrat_14
#define LV_THEME_DEFAULT_FONT_NORMAL    &lv_font_montserrat_20
#define LV_THEME_DEFAULT_FONT_LARGE     &lv_font_montserrat_32

/*********************
 * WIDGETS
 *********************/
#define LV_USE_ARC              1
#define LV_USE_BAR              1
#define LV_USE_BTN              1
#define LV_USE_BTNMATRIX        1
#define LV_USE_CANVAS           1
#define LV_USE_CHECKBOX         1
#define LV_USE_DROPDOWN         1
#define LV_USE_IMG              1
#define LV_USE_LABEL            1
#define LV_USE_LED              1
#define LV_USE_LINE             1
#define LV_USE_LIST             1
#define LV_USE_METER            1
#define LV_USE_MSGBOX           1
#define LV_USE_SLIDER           1
#define LV_USE_SPINBOX          1
#define LV_USE_SPINNER          1
#define LV_USE_SWITCH           1
#define LV_USE_TABLE            1
#define LV_USE_TABVIEW          1
#define LV_USE_TEXTAREA         1
#define LV_USE_TILEVIEW         1
#define LV_USE_CHART            1
#define LV_USE_KEYBOARD         1

/*********************
 * INPUT DEVICES
 *********************/
#define LV_USE_INDEV            1
#define LV_USE_POINTER          1
#define LV_USE_KEYBOARD         1
#define LV_USE_ENCODER          0

/*********************
 * FONT CONFIGURATION
 *********************/
#define LV_FONT_MONTSERRAT_12   1
#define LV_FONT_MONTSERRAT_14   1
#define LV_FONT_MONTSERRAT_16   1
#define LV_FONT_MONTSERRAT_20   1
#define LV_FONT_MONTSERRAT_24   1
#define LV_FONT_MONTSERRAT_28   1
#define LV_FONT_MONTSERRAT_32   1
#define LV_FONT_MONTSERRAT_40   1
#define LV_FONT_MONTSERRAT_48   1
#define LV_FONT_MONTSERRAT_60   0
#define LV_FONT_MONTSERRAT_72   0
#define LV_FONT_MONTSERRAT_80   0
#define LV_FONT_FMT_TXT_LARGE   1
#define LV_FONT_DEFAULT         &lv_font_montserrat_20

/*********************
 * PERFORMANCE & DEBUG
 *********************/
#define LV_USE_ASSERT_NULL      1
#define LV_USE_ASSERT_MEM       1
#define LV_USE_ASSERT_STR       1
#define LV_USE_ASSERT_OBJ       1
#define LV_USE_ASSERT_STYLE     1
#define LV_MEM_CUSTOM           0
#define LV_USE_PERF_MONITOR     0
#define LV_USE_REFR_DEBUG       0

/*********************
 * MISC
 *********************/
#define LV_USE_SNAPSHOT         0
#define LV_USE_SYS_MONITOR      0
#define LV_USE_PROFILER         0

#endif /* LV_CONF_H */
