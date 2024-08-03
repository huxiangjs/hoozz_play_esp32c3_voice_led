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
#include <stdbool.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "led_effects.h"
#include "ws2812b.h"
#include "event_bus.h"

static const char *TAG = "LED";

#define WS2812B_DOUT_GPIO			GPIO_NUM_4

/* Default value for prompt */
#define SMARTCONFIG_DEFAULT_VALUE		10

/* Force refresh flag */
#define FORCE_REFRESH_FLAG			0xffffffff

#define __MAYBE_UNUSED		__attribute__((unused))

struct led_effects_update {
	uint8_t id;
	uint32_t color;
};

static QueueHandle_t queue = NULL;
static uint8_t effects_id = LED_EFFECTS_NONE;

/**
 * @brief Set the value of RGB lights
 *
 * @param rgb Color value (the lowest bit is blue)
 */
static __MAYBE_UNUSED void led_effects_set_rgb(uint32_t rgb)
{
	ws2812b_clear(rgb);
	ws2812b_refresh();
}

static __MAYBE_UNUSED void led_effects_smartconfig_step(void)
{
	uint32_t rgb_buf[WS2812B_LED_NUMBERS] = { 0 };
	static uint8_t index;

	index %= WS2812B_LED_NUMBERS;
	rgb_buf[index] = SMARTCONFIG_DEFAULT_VALUE << 16;
	ws2812b_copy_to_buffer(rgb_buf);
	ws2812b_refresh();

	index++;
}

static __MAYBE_UNUSED uint32_t led_effects_color_pick_step(void)
{
	static uint16_t hue = 0;
	uint32_t red = 0;
	uint32_t green = 0;
	uint32_t blue = 0;

	/* Build RGB pixels */
	ws2812b_hsv2rgb(hue, 100, 100, &red, &green, &blue);
	hue++;

	return (uint32_t)((red << 16) | (green << 8) | (blue << 0));
}

#define COLOR_GRADIENT_STEP_SIZE	5

static __MAYBE_UNUSED uint32_t led_effects_color_gradient_step(uint32_t expect, uint32_t current)
{
	uint8_t expect_rgb[3];
	uint8_t current_rgb[3];
	uint8_t step_size;
	uint8_t index;

	for (index = 0; index < 3; index++) {
		expect_rgb[index] = (uint8_t)((expect >> (index << 3)) & 0xff);
		current_rgb[index] = (uint8_t)((current >> (index << 3)) & 0xff);

		if (current_rgb[index] > expect_rgb[index]) {
			step_size = current_rgb[index] - expect_rgb[index];
			step_size = step_size > COLOR_GRADIENT_STEP_SIZE ? COLOR_GRADIENT_STEP_SIZE : step_size;
			current_rgb[index] -= step_size;
		} else if (current_rgb[index] < expect_rgb[index]) {
			step_size = expect_rgb[index] - current_rgb[index];
			step_size = step_size > COLOR_GRADIENT_STEP_SIZE ? COLOR_GRADIENT_STEP_SIZE : step_size;
			current_rgb[index] += step_size;
		}
	}

	return (uint32_t)((current_rgb[0] << 0) | (current_rgb[1] << 8) | (current_rgb[2] << 16));
}

static bool led_event_notify_callback(struct event_bus_msg *msg)
{
	switch (msg->type) {
	case EVENT_BUS_START_SMART_CONFIG:
		led_effects_play(LED_EFFECTS_SMARTCONFIG, 0);
		break;
	case EVENT_BUS_STOP_SMART_CONFIG:
		led_effects_play(LED_EFFECTS_NONE, 0);
		break;
	}

	return false;
}

static void led_effects_task(void *pvParameters)
{
	TickType_t timeout = portMAX_DELAY;
	struct led_effects_update update;
	uint32_t color = 0x000000;
	uint32_t last_color = color;
	int back_color = -1;

	while(1) {
		if(xQueueReceive(queue, &update, timeout)) {
			if (effects_id != LED_EFFECTS_NONE &&
			    update.id != LED_EFFECTS_NONE)
				continue; /* Drop effects */
			else
				effects_id = update.id;
		}

		switch(effects_id) {
		case LED_EFFECTS_NONE:
			/* recover */
			if (back_color >= 0) {
				color = back_color;
				back_color = -1;
			}
			last_color = FORCE_REFRESH_FLAG;
			effects_id = LED_EFFECTS_NONE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_ALL_ON:
			color = 0xffffff;
			effects_id = LED_EFFECTS_NONE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_ALL_OFF:
			color = 0x000000;
			effects_id = LED_EFFECTS_NONE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_WIFI_CONNECT:
			effects_id = LED_EFFECTS_NONE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_WIFI_DISCONNECT:
			effects_id = LED_EFFECTS_NONE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_SMARTCONFIG:
			back_color = color;
			led_effects_smartconfig_step();
			effects_id = LED_EFFECTS_SMARTCONFIG;
			timeout = pdMS_TO_TICKS(50);
			break;
		case LED_EFFECTS_UPGRADE:
			effects_id = LED_EFFECTS_UPGRADE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_COLOR_PICK:
			color = led_effects_color_pick_step();
			effects_id = LED_EFFECTS_COLOR_PICK;
			timeout = pdMS_TO_TICKS(50);
			break;
		case LED_EFFECTS_COLOR_FILL:
			color = update.color;
			effects_id = LED_EFFECTS_NONE;
			timeout = portMAX_DELAY;
			break;
		case LED_EFFECTS_COLOR_GRADIENT:
			color = led_effects_color_gradient_step(update.color, color);
			if (color != update.color) {
				effects_id = LED_EFFECTS_COLOR_GRADIENT;
				timeout = pdMS_TO_TICKS(40);
			} else {
				effects_id = LED_EFFECTS_NONE;
				timeout = portMAX_DELAY;
			}
			break;
		}

		/* Update color */
		if (color ^ last_color) {
			struct event_bus_msg msg = {
				.type = EVENT_BUS_LED_COLOR_UPDATED,
				.param1 = color,
			};
			event_bus_send(&msg);

			led_effects_set_rgb(color);
			last_color = color;
		}
	}

	vQueueDelete(queue);
	vTaskDelete(NULL);
}

void led_effects_init(void)
{
	int ret;

	ESP_LOGI(TAG, "LED Init");

	ws2812b_init(WS2812B_DOUT_GPIO);
	ws2812b_off();

	/* Create message queue */
	queue = xQueueCreate(5, sizeof(struct led_effects_update));
	ESP_ERROR_CHECK(queue == NULL);

	event_bus_register(led_event_notify_callback);

	/* Start task */
	ret = xTaskCreate(led_effects_task, "led_effects_task", 1024, NULL, tskIDLE_PRIORITY, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);
}

void led_effects_play(uint8_t id, uint32_t param)
{
	struct led_effects_update update = {
		.id = id,
		.color = param,
	};

	xQueueSend(queue, (void *)&update, (TickType_t)0);
}
