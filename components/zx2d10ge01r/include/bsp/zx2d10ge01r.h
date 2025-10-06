#pragma once

#include "sdkconfig.h"
#include <driver/gpio.h>
#include "bsp/config.h"
#include "bsp/display.h"

#include "lvgl.h"
#include "esp_lvgl_port.h"

/**************************************************************************************************
 *  BSP Board Name
 **************************************************************************************************/

/** @defgroup boardname Board Name
 *  @brief BSP Board Name
 *  @{
 */
#define BSP_BOARD_ZX2D10GE01R
/** @} */ // end of boardname

/**************************************************************************************************
 *  BSP Capabilities
 **************************************************************************************************/

#define BSP_CAPS_DISPLAY 1
#define BSP_CAPS_TOUCH 0
#define BSP_CAPS_BUTTONS 1
#define BSP_CAPS_KNOB 1
#define BSP_CAPS_AUDIO 0
#define BSP_CAPS_AUDIO_SPEAKER 0
#define BSP_CAPS_AUDIO_MIC 0
#define BSP_CAPS_LED 1
#define BSP_CAPS_SDCARD 0
#define BSP_CAPS_IMU 0

/**************************************************************************************************
 * ESP-BSP pinout
 **************************************************************************************************/

/* Encoder */
#define BSP_ENCODER_A (GPIO_NUM_5)
#define BSP_ENCODER_B (GPIO_NUM_6)

typedef enum
{
    BSP_BTN_PRESS = GPIO_NUM_3,
} bsp_button_t;

/** @defgroup g07_usb USB
 *  @brief USB BSP API
 *  @{
 */
#define BSP_USB_POS (GPIO_NUM_20)
#define BSP_USB_NEG (GPIO_NUM_19)
/** @} */ // end of usb

/** @defgroup g06_led Leds
 *  @brief Leds BSP API
 *  @{
 */
#define BSP_RGB_CTRL (GPIO_NUM_4)
/** @} */ // end of leds

/* Display */
#define BSP_LCD_VSYNC (GPIO_NUM_48)
#define BSP_LCD_HSYNC (GPIO_NUM_40)
#define BSP_LCD_DE (GPIO_NUM_39)
#define BSP_LCD_PCLK (GPIO_NUM_45)
#define BSP_LCD_DISP (GPIO_NUM_NC)
#define BSP_LCD_DATA0 (GPIO_NUM_47)
#define BSP_LCD_DATA1 (GPIO_NUM_41)
#define BSP_LCD_DATA2 (GPIO_NUM_0)
#define BSP_LCD_DATA3 (GPIO_NUM_42)
#define BSP_LCD_DATA4 (GPIO_NUM_14)
#define BSP_LCD_DATA5 (GPIO_NUM_8)
#define BSP_LCD_DATA6 (GPIO_NUM_13)
#define BSP_LCD_DATA7 (GPIO_NUM_18)
#define BSP_LCD_DATA8 (GPIO_NUM_12)
#define BSP_LCD_DATA9 (GPIO_NUM_11)
#define BSP_LCD_DATA10 (GPIO_NUM_17)
#define BSP_LCD_DATA11 (GPIO_NUM_10)
#define BSP_LCD_DATA12 (GPIO_NUM_16)
#define BSP_LCD_DATA13 (GPIO_NUM_9)
#define BSP_LCD_DATA14 (GPIO_NUM_15)
#define BSP_LCD_DATA15 (GPIO_NUM_46)

#define BSP_LCD_IO_SPI_CS (GPIO_NUM_21)
#define BSP_LCD_IO_SPI_SCL (GPIO_NUM_47)
#define BSP_LCD_IO_SPI_SDA (GPIO_NUM_41)

#define BSP_LCD_BACKLIGHT (GPIO_NUM_38)
#define BSP_LCD_RST (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_RST (GPIO_NUM_NC)
#define BSP_LCD_TOUCH_INT (GPIO_NUM_NC)

#define LVGL_BUFFER_HEIGHT (CONFIG_BSP_DISPLAY_LVGL_BUF_HEIGHT)

#ifdef __cplusplus
extern "C"
{
#endif

/**************************************************************************************************
 *
 * LCD interface
 * *
 * LVGL is used as graphics library. LVGL is NOT thread safe, therefore the user must take LVGL mutex
 * by calling bsp_display_lock() before calling any LVGL API (lv_...) and then give the mutex with
 * bsp_display_unlock().
 *
 * If you want to use the display without LVGL, see bsp/display.h API and use BSP version with 'noglib' suffix.
 **************************************************************************************************/
#define BSP_LCD_PIXEL_CLOCK_HZ (16 * 1000 * 1000)
#define BSP_LCD_SPI_NUM (SPI3_HOST)

#if (BSP_CONFIG_NO_GRAPHIC_LIB == 0)
#define BSP_LCD_DRAW_BUFF_SIZE (BSP_LCD_H_RES * CONFIG_BSP_LCD_RGB_BOUNCE_BUFFER_HEIGHT)
#define BSP_LCD_DRAW_BUFF_DOUBLE (0)

    /**
     * @brief BSP display configuration structure
     */
    typedef struct
    {
        lvgl_port_cfg_t lvgl_port_cfg; /*!< LVGL port configuration */
        uint32_t buffer_size;          /*!< Size of the buffer for the screen in pixels */
        uint32_t trans_size;
        bool double_buffer; /*!< True, if should be allocated two buffers */
        struct
        {
            unsigned int buff_dma : 1;    /*!< Allocated LVGL buffer will be DMA capable */
            unsigned int buff_spiram : 1; /*!< Allocated LVGL buffer will be in PSRAM */
        } flags;
    } bsp_display_cfg_t;

    /**
     * @brief Initialize display
     *
     * This function initializes SPI, display controller and starts LVGL handling task.
     *
     * @return Pointer to LVGL display or NULL when error occurred
     */
    lv_display_t *bsp_display_start(void);

    /**
     * @brief Initialize display
     *
     * This function initializes SPI, display controller and starts LVGL handling task.
     * LCD backlight must be enabled separately by calling bsp_display_brightness_set()
     *
     * @param cfg display configuration
     *
     * @return Pointer to LVGL display or NULL when error occurred
     */
    lv_display_t *bsp_display_start_with_config(const bsp_display_cfg_t *cfg);

    /**
     * @brief Get pointer to input device (touch, buttons, ...)
     *
     * @note The LVGL input device is initialized in bsp_display_start() function.
     *
     * @return Pointer to LVGL input device or NULL when not initialized
     */
    lv_indev_t *bsp_display_get_input_dev(void);

    /**
     * @brief Take LVGL mutex
     *
     * @param timeout_ms Timeout in [ms]. 0 will block indefinitely.
     * @return true  Mutex was taken
     * @return false Mutex was NOT taken
     */
    bool bsp_display_lock(uint32_t timeout_ms);

    /**
     * @brief Give LVGL mutex
     *
     */
    void bsp_display_unlock(void);

    /**
     * @brief Rotate screen
     *
     * Display must be already initialized by calling bsp_display_start()
     *
     * @param[in] disp Pointer to LVGL display
     * @param[in] rotation Angle of the display rotation
     */
    void bsp_display_rotate(lv_display_t *disp, lv_disp_rotation_t rotation);
#endif // BSP_CONFIG_NO_GRAPHIC_LIB == 0

    /**
     * @brief Initialize RGB LED
     *
     * @return
     *     - ESP_OK Success
     *     - ESP_ERR_INVALID_ARG Parameter error
     */
    esp_err_t bsp_led_init();

    /**
     * @brief Set RGB for a specific pixel
     *
     * @param r: red part of color
     * @param g: green part of color
     * @param b: blue part of color
     *
     * @return
     *      - ESP_OK: Set RGB for a specific pixel successfully
     *      - ESP_ERR_INVALID_ARG: Set RGB for a specific pixel failed because of invalid parameters
     *      - ESP_FAIL: Set RGB for a specific pixel failed because other error occurred
     */
    esp_err_t bsp_led_rgb_set(uint8_t r, uint8_t g, uint8_t b);

#ifdef __cplusplus
}
#endif