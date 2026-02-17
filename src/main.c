/**
 * @file main.c
 * @brief WS2812B LED Application
 * @description Main application with stability fixes and logging
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ws2812b.h"

void app_main(void)
{   
    ws2812b_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    
    ws2812b_clear();
    vTaskDelay(pdMS_TO_TICKS(100));

    while (1) {

        ws2812b_set_all(255, 0, 0); //GBR
        ws2812b_refresh();
    }
}