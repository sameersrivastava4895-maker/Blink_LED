/**
 * @file ws2812b.c
 * @brief WS2812B LED Strip Driver Implementation
 * @description Driver for controlling WS2812B addressable RGB LEDs using ESP32 RMT
 */

#include "ws2812b.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"

// Internal timing configuration (nanoseconds)
// WS2812B standard: T0H=400ns, T0L=850ns, T1H=800ns, T1L=450ns
#define LED_STRIP_RMT_RES_HZ  (40 * 1000 * 1000) // 40MHz = 25ns resolution
#define WS2812_T0H_NS       400
#define WS2812_T0L_NS       850
#define WS2812_T1H_NS       800
#define WS2812_T1L_NS       450
#define WS2812_RESET_US     500 // Increased reset for better stability

// Internal state
static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;
static uint8_t led_strip_pixels[LED_STRIP_COUNT * 3];

// RMT Encoder Structure
typedef struct {
    rmt_encoder_t base;
    rmt_encoder_t *bytes_encoder;
    rmt_encoder_t *copy_encoder;
    int state;
    rmt_symbol_word_t reset_code;
} rmt_led_strip_encoder_t;

// RMT Encoder Implementation
static size_t rmt_encode_led_strip(rmt_encoder_t *encoder, rmt_channel_handle_t channel,
                                    const void *primary_data, size_t data_size,
                                    rmt_encode_state_t *ret_state)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encode_state_t session_state = RMT_ENCODING_RESET;
    rmt_encode_state_t state = RMT_ENCODING_RESET;
    size_t encoded_symbols = 0;
    
    switch (led_encoder->state) {
    case 0:
        encoded_symbols += led_encoder->bytes_encoder->encode(led_encoder->bytes_encoder, channel,
                                                               primary_data, data_size, &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = 1;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
        __attribute__((fallthrough));
    case 1:
        encoded_symbols += led_encoder->copy_encoder->encode(led_encoder->copy_encoder, channel,
                                                              &led_encoder->reset_code,
                                                              sizeof(led_encoder->reset_code), &session_state);
        if (session_state & RMT_ENCODING_COMPLETE) {
            led_encoder->state = RMT_ENCODING_RESET;
            state |= RMT_ENCODING_COMPLETE;
        }
        if (session_state & RMT_ENCODING_MEM_FULL) {
            state |= RMT_ENCODING_MEM_FULL;
            goto out;
        }
    }
out:
    *ret_state = state;
    return encoded_symbols;
}

static esp_err_t rmt_del_led_strip_encoder(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_del_encoder(led_encoder->bytes_encoder);
    rmt_del_encoder(led_encoder->copy_encoder);
    free(led_encoder);
    return ESP_OK;
}

static esp_err_t rmt_led_strip_encoder_reset(rmt_encoder_t *encoder)
{
    rmt_led_strip_encoder_t *led_encoder = __containerof(encoder, rmt_led_strip_encoder_t, base);
    rmt_encoder_reset(led_encoder->bytes_encoder);
    rmt_encoder_reset(led_encoder->copy_encoder);
    led_encoder->state = RMT_ENCODING_RESET;
    return ESP_OK;
}

static esp_err_t rmt_new_led_strip_encoder(rmt_encoder_handle_t *ret_encoder)
{
    esp_err_t ret = ESP_OK;
    rmt_led_strip_encoder_t *led_encoder = calloc(1, sizeof(rmt_led_strip_encoder_t));
    if (!led_encoder) return ESP_ERR_NO_MEM;
    
    led_encoder->base.encode = rmt_encode_led_strip;
    led_encoder->base.del = rmt_del_led_strip_encoder;
    led_encoder->base.reset = rmt_led_strip_encoder_reset;
    
    rmt_bytes_encoder_config_t bytes_encoder_config = {
        .bit0 = {
            .level0 = 1,
            .duration0 = (uint32_t)(WS2812_T0H_NS * (uint64_t)LED_STRIP_RMT_RES_HZ / 1000000000ULL),
            .level1 = 0,
            .duration1 = (uint32_t)(WS2812_T0L_NS * (uint64_t)LED_STRIP_RMT_RES_HZ / 1000000000ULL),
        },
        .bit1 = {
            .level0 = 1,
            .duration0 = (uint32_t)(WS2812_T1H_NS * (uint64_t)LED_STRIP_RMT_RES_HZ / 1000000000ULL),
            .level1 = 0,
            .duration1 = (uint32_t)(WS2812_T1L_NS * (uint64_t)LED_STRIP_RMT_RES_HZ / 1000000000ULL),
        },
        .flags.msb_first = 1
    };
    
    ret = rmt_new_bytes_encoder(&bytes_encoder_config, &led_encoder->bytes_encoder);
    if (ret != ESP_OK) goto err;
    
    rmt_copy_encoder_config_t copy_encoder_config = {};
    ret = rmt_new_copy_encoder(&copy_encoder_config, &led_encoder->copy_encoder);
    if (ret != ESP_OK) goto err;
    
    uint32_t reset_ticks = LED_STRIP_RMT_RES_HZ / 1000000 * WS2812_RESET_US;
    led_encoder->reset_code = (rmt_symbol_word_t) {
        .level0 = 0, .duration0 = reset_ticks,
        .level1 = 0, .duration1 = reset_ticks,
    };
    
    *ret_encoder = &led_encoder->base;
    return ESP_OK;
    
err:
    if (led_encoder) {
        if (led_encoder->bytes_encoder) rmt_del_encoder(led_encoder->bytes_encoder);
        if (led_encoder->copy_encoder) rmt_del_encoder(led_encoder->copy_encoder);
        free(led_encoder);
    }
    return ret;
}

// Public API Implementation

esp_err_t ws2812b_init(void)
{
    // Configure GPIO
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << LED_STRIP_GPIO),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_drive_capability(LED_STRIP_GPIO, GPIO_DRIVE_CAP_3);
    
    // Configure RMT TX channel
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .gpio_num = LED_STRIP_GPIO,
        .mem_block_symbols = 128, // Increased from 64 to fit 3 LEDs (72 symbols) comfortably
        .resolution_hz = LED_STRIP_RMT_RES_HZ,
        .trans_queue_depth = 4,
        .flags.invert_out = false,
        .flags.with_dma = false,
    };
    
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));
    ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&led_encoder));
    ESP_ERROR_CHECK(rmt_enable(led_chan));
    
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    
    return ESP_OK;
}

esp_err_t ws2812b_refresh(void)
{
    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels, sizeof(led_strip_pixels), &tx_config));
    ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
    return ESP_OK;
}

void ws2812b_set_pixel(uint32_t index, uint8_t r, uint8_t g, uint8_t b)
{
    if (index < LED_STRIP_COUNT) {
        // Swapping to RBG order based on user observation (Red -> Blue)
        led_strip_pixels[index * 3 + 0] = r;
        led_strip_pixels[index * 3 + 1] = b;
        led_strip_pixels[index * 3 + 2] = g;
    }
}

void ws2812b_set_all(uint8_t r, uint8_t g, uint8_t b)
{
    for (uint32_t i = 0; i < LED_STRIP_COUNT; i++) {
        ws2812b_set_pixel(i, r, g, b);
    }
}

void ws2812b_clear(void)
{
    memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
    ws2812b_refresh();
}