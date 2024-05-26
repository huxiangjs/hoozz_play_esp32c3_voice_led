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
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

#include "audio.h"
#include "ws2812b.h"
#include "led_effects.h"
#include "wifi.h"
#include "event_bus.h"
#include "recognition.h"
#include "simple_ctrl.h"

static const char *TAG = "APP-MAIN";

static bool allow_wifi_config;
static uint32_t last_color;

#define LED_CMD_SET_COLOR		0x00
#define LED_CMD_GET_COLOR		0x01

#define LED_RESULT_OK			0x00
#define LED_RESULT_FAIL			0x01

static void app_show_info(void)
{
	int size = esp_get_free_heap_size();

	ESP_LOGI(TAG, "Free heap size: %dbyte", size);
}

static int app_led_request(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	uint32_t rgb = 0;

	if (vaild_size < 1) {
		ESP_LOGE(TAG, "Information command is incorrect");
		return -1;
	}

	if (buff_size - buf_offs < 5) {
		ESP_LOGE(TAG, "Not enough buffer space");
		return -1;
	}

	if (buffer[buf_offs] == LED_CMD_SET_COLOR && vaild_size == 4) {
		/* Set color */
		rgb |= buffer[buf_offs + 1] << 0;
		rgb |= buffer[buf_offs + 2] << 8;
		rgb |= buffer[buf_offs + 3] << 16;
		// led_effects_play(LED_EFFECTS_COLOR_FILL, rgb);
		led_effects_play(LED_EFFECTS_COLOR_GRADIENT, rgb);
		ESP_LOGI(TAG, "Set color: #%06x", (unsigned int)rgb);
		/* Set return (2byte) */
		buffer[buf_offs + 1] = LED_RESULT_OK;
		return 2;
	} else if (buffer[buf_offs] == LED_CMD_GET_COLOR && vaild_size == 1) {
		ESP_LOGI(TAG, "Get color: #%06x", (unsigned int)last_color);
		/* Set return (5byte) */
		buffer[buf_offs + 1] = LED_RESULT_OK;
		buffer[buf_offs + 2] = (uint8_t)((last_color >> 0) & 0xff);
		buffer[buf_offs + 3] = (uint8_t)((last_color >> 8) & 0xff);
		buffer[buf_offs + 4] = (uint8_t)((last_color >> 16) & 0xff);
		return 5;
	} else {
		ESP_LOGE(TAG, "Lllegal command");
		return -1;
	}

	return 0;
}

static void app_led_notify(uint32_t color)
{
	char rgb[3];

	last_color = color;

	rgb[0] = (uint8_t)((color >> 0) & 0xff);
	rgb[1] = (uint8_t)((color >> 8) & 0xff);
	rgb[2] = (uint8_t)((color >> 16) & 0xff);

	simple_ctrl_notify(rgb, sizeof(rgb));
}

static bool app_event_notify_callback(struct event_bus_msg *msg)
{
	switch (msg->type) {
	case EVENT_BUS_STARTUP:
		break;
	case EVENT_BUS_START_SMART_CONFIG:
		break;
	case EVENT_BUS_STOP_SMART_CONFIG:
		break;
	case EVENT_BUS_AUDIO_RECOGNITION:
		if (allow_wifi_config) {
			ESP_LOGI(TAG, "Allow wifi config");
			if (msg->param1 == 1) {
				allow_wifi_config = false;
				wifi_smartconfig();
			}
		} else {
			if (msg->param1 == 1) {
				led_effects_play(LED_EFFECTS_ALL_ON, 0);
				printf("LED ON\n");
			} else {
				led_effects_play(LED_EFFECTS_ALL_OFF, 0);
				printf("LED OFF\n");
			}
		}
		break;
	case EVENT_BUS_LED_COLOR_UPDATED:
		printf("Color updated\n");
		app_led_notify(msg->param1);
		break;
	}

	return false;
}

extern "C" void app_main(void)
{
	struct event_bus_msg msg = { EVENT_BUS_STARTUP, 0 };

	app_show_info();
	ESP_ERROR_CHECK(nvs_flash_init());

	/* Event bus */
	event_bus_init();
	event_bus_register(app_event_notify_callback);
	event_bus_send(&msg);

	/* LED */
	led_effects_init();

	/* Audio recognition */
	recognition_init();
	vTaskDelay(pdMS_TO_TICKS(100));

	/* Wifi */
	wifi_init();

	/* Network ctrl */
	simple_ctrl_init();
	simple_ctrl_set_name("VOICE LED");
	simple_ctrl_set_class_id(CLASS_ID_VOICE_LED);

	allow_wifi_config = true;
	vTaskDelay(pdMS_TO_TICKS(10000));
	allow_wifi_config = false;

	simple_ctrl_request_register(app_led_request);
}
