/**
 * @file ws2812b.h
 * @brief WS2812B LED Strip Driver Header
 * @description Public API for controlling WS2812B addressable RGB LEDs
 */

#ifndef WS2812B_H
#define WS2812B_H

#include <stdint.h>
#include "esp_err.h"

// Configuration
#define LED_STRIP_GPIO      11
#define LED_STRIP_COUNT     48

/**
 * @brief Initialize WS2812B LED strip
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ws2812b_init(void);

/**
 * @brief Refresh LED strip (send data to LEDs)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t ws2812b_refresh(void);

/**
 * @brief Set individual LED color
 * @param index LED index (0 to LED_STRIP_COUNT-1)
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void ws2812b_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Set all LEDs to the same color
 * @param r Red value (0-255)
 * @param g Green value (0-255)
 * @param b Blue value (0-255)
 */
void ws2812b_set_all(uint8_t r, uint8_t g, uint8_t b);

/**
 * @brief Clear all LEDs (turn off)
 */
void ws2812b_clear(void);

#endif // WS2812B_H
