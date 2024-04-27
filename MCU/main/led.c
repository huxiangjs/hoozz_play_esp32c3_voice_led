/*
 * MIT License
 *
 * Copyright (c) 2024 huxiangjs
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "led.h"
#include "ws2812b.h"

static const char *TAG = "LED";

/* Default value for prompt */
#define DEFAULT_VALUE		10

#define __MAYBE_UNUSED		__attribute__((unused))

static QueueHandle_t queue = NULL;
static uint8_t status = LED_STATUS_NONE;

/**
 * @brief Set the value of RGB lights
 *
 * @param rgb Color value (the lowest bit is blue)
 */
void led_set_rgb(uint32_t rgb)
{
	ws2812b_clear(rgb);
	ws2812b_refresh();
}

/**
 * @brief Get RGB value
 *
 * @return uint32_t Color value
 */
uint32_t led_get_rgb(void)
{
	return ws2812b_get_pixel(0);
}

// Red-On
static __MAYBE_UNUSED void led_red_on(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = (rgb & 0x00FFFF) | (DEFAULT_VALUE << 16);
	led_set_rgb(rgb);
}

// Red-Off
static __MAYBE_UNUSED void led_red_off(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = rgb & 0x00FFFF;
	led_set_rgb(rgb);
}

// Red light jump
#if 0
static __MAYBE_UNUSED void led_red_step(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = (rgb & 0x00FFFF) | (rgb & 0xFF0000 ? 0 : (DEFAULT_VALUE << 16));
	led_set_rgb(rgb);
}
#else
static __MAYBE_UNUSED void led_red_step(void)
{
	uint32_t rgb_buf[WS2812B_LED_NUMBERS] = { 0 };
	static uint8_t index;

	index %= WS2812B_LED_NUMBERS;
	rgb_buf[index] = DEFAULT_VALUE << 16;
	ws2812b_copy_to_buffer(rgb_buf);
	ws2812b_refresh();

	index++;
}
#endif

// Green-On
static __MAYBE_UNUSED void led_green_on(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = (rgb & 0xFF00FF) | (DEFAULT_VALUE << 8);
	led_set_rgb(rgb);
}

// Green-Off
static __MAYBE_UNUSED void led_green_off(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = rgb & 0xFF00FF;
	led_set_rgb(rgb);
}

// Green light jump
static __MAYBE_UNUSED void led_green_step(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = (rgb & 0xFF00FF) | (rgb & 0x00FF00 ? 0 : (DEFAULT_VALUE << 8));
	led_set_rgb(rgb);
}

// Blue-On
static __MAYBE_UNUSED void led_blue_on(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = (rgb & 0xFFFF00) | DEFAULT_VALUE;
	led_set_rgb(rgb);
}

// Blue-Off
static __MAYBE_UNUSED void led_blue_off(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = rgb & 0xFFFF00;
	led_set_rgb(rgb);
}

// Blue light jump
static __MAYBE_UNUSED void led_blue_step(void)
{
	uint32_t rgb = led_get_rgb();
	rgb = (rgb & 0xFFFF00) | (rgb & 0x0000FF ? 0 : DEFAULT_VALUE);
	led_set_rgb(rgb);
}

static __MAYBE_UNUSED void led_color_step(void)
{
	static uint16_t hue = 0;
	uint32_t red = 0;
	uint32_t green = 0;
	uint32_t blue = 0;

	/* Build RGB pixels */
	ws2812b_hsv2rgb(hue, 100, 100, &red, &green, &blue);
	led_set_rgb((red << 16) | (green << 8) | (blue << 0));

	hue++;
}

static void led_task(void *pvParameters)
{
	TickType_t timeout = portMAX_DELAY;

	while(1) {
		// if(xQueueReceive(queue, &status, timeout)) {}
		if(xQueueReceive(queue, &status, timeout)) {
			led_blue_off();
		}

		switch(status) {
		case LED_STATUS_NONE:
			timeout = portMAX_DELAY;
			led_green_off();
			led_red_off();
			break;
#if 0
		case LED_STATUS_WIFI_CONNECT:
			timeout = portMAX_DELAY;
			led_green_on();
			led_red_off();
			break;
		case LED_STATUS_WIFI_DISCONNECT:
			timeout = portMAX_DELAY;
			led_green_off();
			led_red_on();
			break;
#endif
		case LED_STATUS_SMARTCONFIG:
			timeout = pdMS_TO_TICKS(50);
			// led_green_off();
			led_red_step();
			break;
		case LED_STATUS_UPGRADE:
			timeout = pdMS_TO_TICKS(50);
			led_red_off();
			led_green_step();
			break;
		case LED_STATUS_COLOR_PICK:
			timeout = pdMS_TO_TICKS(50);
			led_color_step();
			break;
		}
	}

	vQueueDelete(queue);
	vTaskDelete(NULL);
}

void led_init(void)
{
	ESP_LOGI(TAG, "LED Init");

	/* Create message queue */
	queue = xQueueCreate(5, sizeof(uint8_t));
	/* Start task */
	xTaskCreate(led_task, "led_task", 512, NULL, tskIDLE_PRIORITY, NULL);
}

void led_status_set(uint8_t new_status)
{
	if (queue)
		xQueueSend(queue, (void *)&new_status, (TickType_t)0);
}

uint8_t led_status_get(void)
{
	return status;
}
