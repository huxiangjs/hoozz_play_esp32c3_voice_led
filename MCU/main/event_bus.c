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
#include <stdint.h>
#include <stdbool.h>
#include <malloc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include "esp_log.h"
#include "event_bus.h"

static const char *TAG = "EVENT-BUS";

/* Maximum number of events */
#define MAX_EVENT_NUM		10

static QueueHandle_t queue = NULL;
static SemaphoreHandle_t register_mutex;

struct event_notify {
	event_notify_callback call;
	struct event_notify *next;
};

static struct event_notify *notify_link_head = NULL;
static struct event_notify *notify_link_tail = NULL;

void event_bus_send(struct event_bus_msg *msg)
{
	xQueueSend(queue, (void *)msg, (TickType_t)0);
}

void event_bus_register(event_notify_callback callback)
{
	struct event_notify *notify;

	notify = (struct event_notify *)malloc(sizeof(struct event_notify));
	ESP_ERROR_CHECK(notify == NULL);

	notify->call = callback;
	notify->next = NULL;

	xSemaphoreTake(register_mutex, portMAX_DELAY);
	if (notify_link_head) {
		notify_link_tail->next = notify;
		notify_link_tail = notify_link_tail->next;
	} else {
		notify_link_head = notify;
		notify_link_tail = notify_link_head;
	}
	xSemaphoreGive(register_mutex);

	// printf("Register [%p]\n", callback);
}

static void event_bus_task(void *pvParameters)
{
	struct event_bus_msg msg;
	struct event_notify *iter;
	bool processed;

	while(1) {
		if(xQueueReceive(queue, &msg, portMAX_DELAY)) {
			iter = notify_link_head;
			while (iter) {
				processed = iter->call(&msg);
				if (processed)
					break;
				// printf("Call [%p]\n", iter->call);
				iter = iter->next;
			}
		}
	}

	vQueueDelete(queue);
	vTaskDelete(NULL);
}

void event_bus_init(void)
{
	int ret;

	ESP_LOGI(TAG, "Event Bus Init");

	/* Create message queue */
	queue = xQueueCreate(MAX_EVENT_NUM, sizeof(struct event_bus_msg));
	ESP_ERROR_CHECK(queue == NULL);

	register_mutex = xSemaphoreCreateMutex();

	/* Start task */
	ret = xTaskCreate(event_bus_task, "event_bus_task", 2048, NULL, tskIDLE_PRIORITY, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);
}
