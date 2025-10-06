/*
 * SPDX-FileCopyrightText: 2023-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once
#include "esp_lcd_types.h"

/* LCD color formats */
#define ESP_LCD_COLOR_FORMAT_RGB565 (1)
#define ESP_LCD_COLOR_FORMAT_RGB888 (2)

/* LCD display color format */
#define BSP_LCD_COLOR_FORMAT (ESP_LCD_COLOR_FORMAT_RGB565)
/* LCD display color bytes endianess */
#define BSP_LCD_BIGENDIAN (1)
/* LCD display color bits */
#define BSP_LCD_BITS_PER_PIXEL (16)
#define BSP_LCD_BIT_PER_PIXEL (18)
#define BSP_RGB_DATA_WIDTH (16)

/* LCD display definition */
#define BSP_LCD_H_RES (480)
#define BSP_LCD_V_RES (480)

/**
 * @brief RGB timing structure
 *
 * @note  refresh_rate = (pclk_hz * data_width) / (h_res + hsync_pulse_width + hsync_back_porch + hsync_front_porch)
 *                                              / (v_res + vsync_pulse_width + vsync_back_porch + vsync_front_porch)
 *                                              / bits_per_pixel
 *
 */
#define ZX2D10GE01R_TIMING()           \
    {                                  \
        .pclk_hz = (16 * 1000 * 1000), \
        .h_res = BSP_LCD_H_RES,        \
        .v_res = BSP_LCD_V_RES,        \
        .hsync_pulse_width = 10,       \
        .hsync_back_porch = 10,        \
        .hsync_front_porch = 10,       \
        .vsync_pulse_width = 2,        \
        .vsync_back_porch = 18,        \
        .vsync_front_porch = 14,       \
        .flags = {                     \
            .pclk_active_neg = false,  \
        },                             \
    }

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief BSP display configuration structure
     *
     */
    typedef struct
    {
        int max_transfer_sz; /*!< Maximum transfer size, in bytes. */
    } bsp_display_config_t;

    /**
     * @brief Create new display panel
     *
     * For maximum flexibility, this function performs only reset and initialization of the display.
     * You must turn on the display explicitly by calling esp_lcd_panel_disp_on_off().
     * The display's backlight is not turned on either. You can use bsp_display_backlight_on/off(),
     * bsp_display_brightness_set() (on supported boards) or implement your own backlight control.
     *
     * If you want to free resources allocated by this function, you can use esp_lcd API, ie.:
     *
     * \code{.c}
     * esp_lcd_panel_del(panel);
     * esp_lcd_panel_io_del(io);
     * spi_bus_free(spi_num_from_configuration);
     * \endcode
     *
     * @param[in]  config    display configuration
     * @param[out] ret_panel esp_lcd panel handle
     * @param[out] ret_io    esp_lcd IO handle
     * @return
     *      - ESP_OK         On success
     *      - Else           esp_lcd failure
     */
    esp_err_t bsp_display_new(const bsp_display_config_t *config, esp_lcd_panel_handle_t *ret_panel, esp_lcd_panel_io_handle_t *ret_io);

    /**
     * @brief Initialize display's brightness
     *
     * Brightness is controlled with PWM signal to a pin controlling backlight.
     *
     * @return
     *      - ESP_OK                On success
     *      - ESP_ERR_INVALID_ARG   Parameter error
     */
    esp_err_t bsp_display_brightness_init(void);

    /**
     * @brief Set display's brightness
     *
     * Brightness is controlled with PWM signal to a pin controlling backlight.
     * Brightness must be already initialized by calling bsp_display_brightness_init() or bsp_display_new()
     *
     * @param[in] brightness_percent Brightness in [%]
     * @return
     *      - ESP_OK                On success
     *      - ESP_ERR_INVALID_ARG   Parameter error
     */
    esp_err_t bsp_display_brightness_set(int brightness_percent);

    int bsp_display_brightness_get(void);
    /**
     * @brief Turn on display backlight
     *
     * Brightness is controlled with PWM signal to a pin controlling backlight.
     * Brightness must be already initialized by calling bsp_display_brightness_init() or bsp_display_new()
     *
     * @return
     *      - ESP_OK                On success
     *      - ESP_ERR_INVALID_ARG   Parameter error
     */
    esp_err_t bsp_display_backlight_on(void);

    /**
     * @brief Turn off display backlight
     *
     * Brightness is controlled with PWM signal to a pin controlling backlight.
     * Brightness must be already initialized by calling bsp_display_brightness_init() or bsp_display_new()
     *
     * @return
     *      - ESP_OK                On success
     *      - ESP_ERR_INVALID_ARG   Parameter error
     */
    esp_err_t bsp_display_backlight_off(void);

#ifdef __cplusplus
}
#endif