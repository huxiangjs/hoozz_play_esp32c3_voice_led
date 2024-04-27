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
#include "led.h"
#include "wifi.h"
#include "event_bus.h"
#include "recognition.h"

static const char *TAG = "APP-MAIN";

static bool allow_wifi_config;

static void app_show_info(void)
{
	int size = esp_get_free_heap_size();

	ESP_LOGI(TAG, "Free heap size: %dbyte", size);
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
				led_set_rgb(0xffffff);
				printf("LED ON\n");
			} else {
				led_set_rgb(0x000000);
				printf("LED OFF\n");
			}
		}
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
	ws2812b_init();
	led_init();
	led_set_rgb(0x000000);

	/* Audio recognition */
	recognition_init();
	vTaskDelay(pdMS_TO_TICKS(100));

	/* Wifi */
	wifi_init();

	allow_wifi_config = true;
	vTaskDelay(pdMS_TO_TICKS(10000));
	allow_wifi_config = false;
}
