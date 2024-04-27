/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 *
 * 2024/04/27 modified by Hoozz
 *
 */
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/rmt_tx.h"
#include "led_strip_encoder.h"
#include "ws2812b.h"

/* 10MHz resolution, 1 tick = 0.1us (led strip needs a high resolution) */
#define RMT_LED_STRIP_RESOLUTION_HZ 10000000
#define RMT_LED_STRIP_GPIO_NUM      GPIO_NUM_4

static const char *TAG = "WS2812B";

static uint8_t led_strip_pixels[WS2812B_LED_NUMBERS * 3];

static rmt_transmit_config_t tx_config = {
	.loop_count = 0, // no transfer loop
};

static rmt_channel_handle_t led_chan = NULL;
static rmt_encoder_handle_t led_encoder = NULL;

/**
 * @brief Simple helper function, converting HSV color space to RGB color space
 *
 * Wiki: https://en.wikipedia.org/wiki/HSL_and_HSV
 *
 */
void ws2812b_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
	h %= 360; // h -> [0,360]
	uint32_t rgb_max = v * 2.55f;
	uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

	uint32_t i = h / 60;
	uint32_t diff = h % 60;

	// RGB adjustment amount by hue
	uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

	switch (i) {
	case 0:
		*r = rgb_max;
		*g = rgb_min + rgb_adj;
		*b = rgb_min;
		break;
	case 1:
		*r = rgb_max - rgb_adj;
		*g = rgb_max;
		*b = rgb_min;
		break;
	case 2:
		*r = rgb_min;
		*g = rgb_max;
		*b = rgb_min + rgb_adj;
		break;
	case 3:
		*r = rgb_min;
		*g = rgb_max - rgb_adj;
		*b = rgb_max;
		break;
	case 4:
		*r = rgb_min + rgb_adj;
		*g = rgb_min;
		*b = rgb_max;
		break;
	default:
		*r = rgb_max;
		*g = rgb_min;
		*b = rgb_max - rgb_adj;
		break;
	}
}

void ws2812b_off(void)
{
	memset(led_strip_pixels, 0, sizeof(led_strip_pixels));
	ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels,
				     sizeof(led_strip_pixels), &tx_config));
	ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

void ws2812b_set_pixel(uint8_t num, uint32_t rgb)
{
	uint8_t red = (uint8_t)((rgb >> 16) & 0xff);
	uint8_t green = (uint8_t)((rgb >> 8) & 0xff);
	uint8_t blue = (uint8_t)((rgb >> 0) & 0xff);

	num %= WS2812B_LED_NUMBERS;
	led_strip_pixels[num * 3 + 0] = green;
	led_strip_pixels[num * 3 + 1] = red;
	led_strip_pixels[num * 3 + 2] = blue;
}

void ws2812b_copy_to_buffer(uint32_t rgb_buf[])
{
	for (int i = 0; i < WS2812B_LED_NUMBERS; i++)
		ws2812b_set_pixel(i, rgb_buf[i]);
}

void ws2812b_clear(uint32_t rgb)
{
	uint8_t red = (uint8_t)((rgb >> 16) & 0xff);
	uint8_t green = (uint8_t)((rgb >> 8) & 0xff);
	uint8_t blue = (uint8_t)((rgb >> 0) & 0xff);

	for (int i = 0; i < WS2812B_LED_NUMBERS * 3; i += 3) {
		led_strip_pixels[i + 0] = green;
		led_strip_pixels[i + 1] = red;
		led_strip_pixels[i + 2] = blue;
	}
}

uint32_t ws2812b_get_pixel(uint8_t num)
{
	uint32_t retval = 0;

	num %= WS2812B_LED_NUMBERS;
	retval |= led_strip_pixels[num * 3 + 1] << 16;
	retval |= led_strip_pixels[num * 3 + 0] << 8;
	retval |= led_strip_pixels[num * 3 + 2] << 0;

	return retval;
}

void ws2812b_refresh(void)
{
	/* Flush RGB values to LEDs */
	ESP_ERROR_CHECK(rmt_transmit(led_chan, led_encoder, led_strip_pixels,
			sizeof(led_strip_pixels), &tx_config));
	ESP_ERROR_CHECK(rmt_tx_wait_all_done(led_chan, portMAX_DELAY));
}

void ws2812b_init(void)
{
	ESP_LOGI(TAG, "Create RMT TX channel");
	rmt_tx_channel_config_t tx_chan_config = {
		/* select source clock */
		.clk_src = RMT_CLK_SRC_DEFAULT,
		.gpio_num = RMT_LED_STRIP_GPIO_NUM,
		/* increase the block size can make the LED less flickering */
		.mem_block_symbols = 64,
		.resolution_hz = RMT_LED_STRIP_RESOLUTION_HZ,
		/* set the number of transactions that can be pending in the background */
		.trans_queue_depth = 4,
	};
	ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &led_chan));

	ESP_LOGI(TAG, "Install led strip encoder");
	led_strip_encoder_config_t encoder_config = {
		.resolution = RMT_LED_STRIP_RESOLUTION_HZ,
	};
	ESP_ERROR_CHECK(rmt_new_led_strip_encoder(&encoder_config, &led_encoder));

	ESP_LOGI(TAG, "Enable RMT TX channel");
	ESP_ERROR_CHECK(rmt_enable(led_chan));
}
