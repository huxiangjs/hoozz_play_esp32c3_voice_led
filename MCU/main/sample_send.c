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
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio.h"
#include "event_bus.h"

static const char *TAG = "SAMPLE-SEND";

#define EVENT_DATA_SIZE			1024
#define EVENT_QUEUE_NUM_MAX		100
#define EVENT_REMOTE_TCP_PORT		17171

struct sample_event {
	uint8_t event;
	uint8_t data[EVENT_DATA_SIZE];
	uint16_t valid_size;
};

static struct sample_event sample_tmp;
static QueueHandle_t event_queue;
static TaskHandle_t handle;

// #define SAMPLE_SEND_TO_SERIAL

#if defined(SAMPLE_SEND_TO_SERIAL)

#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"

#define SERIAL_BUFFER_SIZE		1024

static void usb_uart_config(void)
{
	/* Configure USB SERIAL JTAG */
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
		.tx_buffer_size = SERIAL_BUFFER_SIZE,
		.rx_buffer_size = SERIAL_BUFFER_SIZE,
	};

	/*
	 * You need:
	 * 	menuconfig → Component config → ESP System Settings → Channel for console secondary output
	 * then, disable log output
	 */
	ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_serial_jtag_config));
	// usb_serial_jtag_driver_uninstall();
}

/**
 * @brief blocking call
 *
 * @param data data
 * @param size data size (unit: byte)
 */
static inline void usb_uart_write_bytes(const void *data, uint32_t size)
{
	usb_serial_jtag_write_bytes(data, size, portMAX_DELAY);
	usb_serial_jtag_ll_txfifo_flush();
}

static void sample_send_task(void *pvParameters)
{
	struct sample_event event;
	uint16_t count;
	size_t write_size;

	usb_uart_config();

	while (1) {
		/* Receive frame events */
		if(xQueueReceive(event_queue, &event, portMAX_DELAY)) {
			/* Continuous frame */
			if (event.event & AUDIO_EVENT_VOICE_FRAME) {
				/* Write to USB Serial */
				count = 0;
				while (count < event.valid_size) {
					write_size = event.valid_size - count < SERIAL_BUFFER_SIZE ?
						     event.valid_size - count : SERIAL_BUFFER_SIZE;
					usb_uart_write_bytes(event.data + count, write_size);
					count += write_size;
				}
			}
		}
	}

	vQueueDelete(event_queue);
	vTaskDelete(NULL);
}

#else

#include <unistd.h>
#include <sys/socket.h>
#include <errno.h>
#include <arpa/inet.h>
#include "esp_netif.h"

#define REMOTE_IP			"192.168.31.251"

static int sock = -1;

static void sample_send_task(void *pvParameters)
{
	const char *host_ip = REMOTE_IP;
	struct sample_event event;
	struct sockaddr_in dest_addr;
	int err;

	ESP_LOGI(TAG, "Wait network ready...");
	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	ESP_LOGI(TAG, "Network ready");

	inet_pton(AF_INET, host_ip, &dest_addr.sin_addr);
	dest_addr.sin_family = AF_INET;
	dest_addr.sin_port = htons(EVENT_REMOTE_TCP_PORT);

	while (1) {
		/* Receive frame events */
		if(xQueueReceive(event_queue, &event, portMAX_DELAY)) {
			/* Start */
			if (event.event & AUDIO_EVENT_VOICE_START) {
				ESP_LOGI(TAG, "START");
				if (sock != -1) {
					shutdown(sock, 0);
					close(sock);
					sock = -1;
				}
				while (1) {
					sock =  socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
					if (sock < 0) {
						ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
						sock = -1;
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						continue;
					}
					ESP_LOGI(TAG, "Socket created, connecting to %s:%d", host_ip, EVENT_REMOTE_TCP_PORT);

					err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
					if (err) {
						ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
						shutdown(sock, 0);
						close(sock);
						sock = -1;
						vTaskDelay(1000 / portTICK_PERIOD_MS);
						continue;
					}
					ESP_LOGI(TAG, "Successfully connected [%s:%d]", host_ip, EVENT_REMOTE_TCP_PORT);
					break;
				}
			}

			/* Continuous frame */
			if (event.event & AUDIO_EVENT_VOICE_FRAME) {
				ESP_LOGI(TAG, "+ %d bytes", (int)event.valid_size);
				if (sock != -1) {
					err = send(sock, event.data, event.valid_size, 0);
					if (err != (int)event.valid_size) {
						ESP_LOGE(TAG, "Error value: %d", err);
						ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
					}
				}
			}

			/* Stop */
			if (event.event & AUDIO_EVENT_VOICE_STOP) {
				ESP_LOGI(TAG, "STOP");
				if (sock != -1) {
					shutdown(sock, 0);
					close(sock);
					sock = -1;
					ESP_LOGI(TAG, "Socket closed");
				}
				ESP_LOGI(TAG, "Free heap size: %dbytes", (int)esp_get_free_heap_size());
			}
		}
	}

	close(sock);
	vQueueDelete(event_queue);
	vTaskDelete(NULL);
}
#endif

void sample_send_event(uint8_t event, void *data, uint16_t size)
{
	/* Check frame size */
	ESP_ERROR_CHECK(size > EVENT_DATA_SIZE);

	/* Send event */
	sample_tmp.event = event;
	memcpy(sample_tmp.data, data, size);
	sample_tmp.valid_size = size;
	if (xQueueSend(event_queue, (void *)&sample_tmp, (TickType_t)0) != pdTRUE)
		ESP_LOGE(TAG, "Send sample event failed");
}

static bool sample_send_notify_callback(struct event_bus_msg *msg)
{
	if (msg->type == EVENT_BUS_WIFI_CONNECTED)
		xTaskNotifyGive(handle);

	return false;
}

void sample_send_init(void)
{
	int ret;

	/* Create event queue */
	event_queue = xQueueCreate(EVENT_QUEUE_NUM_MAX, sizeof(struct sample_event));
	ESP_ERROR_CHECK(event_queue == NULL);

	/* Create sample_send task */
	ret = xTaskCreate(sample_send_task, "sample_send_task", 1024 * 6, NULL,
			  tskIDLE_PRIORITY + 1, &handle);
	ESP_ERROR_CHECK(ret != pdPASS);

	event_bus_register(sample_send_notify_callback);
}
