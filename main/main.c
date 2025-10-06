#include <stdint.h>
#include <stdlib.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "bsp/esp-bsp.h"

#define TAG "MAIN"

#define LVGL_TASK_MAX_DELAY_MS 200
#define LVGL_TASK_MIN_DELAY_MS 1
#define LVGL_TASK_STACK_SIZE (8 * 1024)
#define LVGL_TASK_PRIORITY 2

static void lvgl_port_task(void *arg)
{
    ESP_LOGI(TAG, "Starting LVGL task");

    bsp_display_start();

    extern void lv_demo_benchmark(void);
    lv_demo_benchmark();

    ESP_ERROR_CHECK(bsp_display_backlight_on());

    uint32_t task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
    while (1)
    {
        if (bsp_display_lock(-1))
        {
            task_delay_ms = lv_timer_handler();
            bsp_display_unlock();
        }
        if (task_delay_ms > LVGL_TASK_MAX_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MAX_DELAY_MS;
        }
        else if (task_delay_ms < LVGL_TASK_MIN_DELAY_MS)
        {
            task_delay_ms = LVGL_TASK_MIN_DELAY_MS;
        }
        vTaskDelay(pdMS_TO_TICKS(task_delay_ms));
    }
}

void app_main(void)
{

    bsp_led_init();

    bsp_led_rgb_set(0, 0, 255); // Blue

    xTaskCreatePinnedToCore(lvgl_port_task, "LVGL", LVGL_TASK_STACK_SIZE, NULL, LVGL_TASK_PRIORITY, NULL, 1);

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}