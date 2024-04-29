/*
 * MIT License
 *
 * Copyright (c) 2024 Hoozz (huxiangjs)
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
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>
#include <lwip/sockets.h>
#include <lwip/dns.h>
#include <lwip/netdb.h>
#include <esp_log.h>
#include "event_bus.h"

static const char *TAG = "SIMPLE-CTRL";

#define DISCOVER_UDP_PORT		54542
#define DISCOVER_SAY			"HOOZZ?"
#define DISCOVER_ONLINE_SAY		"HOOZZ!"
#define DISCOVER_RESPOND		"YES"
#define DISCOVER_BUFFER_SIZE		16
#define DISCOVER_BROADCAST_NUM		40
#define DISCOVER_BROADCAST_ADDRESS	"255.255.255.255"

static int discover_socket;
static SemaphoreHandle_t network_ready;

static void simple_ctrl_do_discover(void)
{
	char buffer[DISCOVER_BUFFER_SIZE];
	struct sockaddr_in addr_in;
	socklen_t addr_size = sizeof(addr_in);
	int recv_len;
	uint8_t count;
	int ret;

	/* Full address */
	addr_in.sin_family = AF_INET;
	addr_in.sin_port = htons(DISCOVER_UDP_PORT);
	ret = inet_pton(AF_INET, DISCOVER_BROADCAST_ADDRESS,
			(void *)&addr_in.sin_addr);
	ESP_ERROR_CHECK(ret < 0);
	count = DISCOVER_BROADCAST_NUM;
	while (count--) {
		snprintf(buffer, sizeof(buffer), DISCOVER_ONLINE_SAY);
		sendto(discover_socket, buffer, strlen(buffer), 0,
		       (struct sockaddr*)&addr_in, addr_size);
		vTaskDelay(pdMS_TO_TICKS(100));
	}
	ESP_LOGI(TAG, "online broadcast has been sent");

	while (1) {
		/* Wait data */
		recv_len = recvfrom(discover_socket, &buffer,
				    DISCOVER_BUFFER_SIZE - 1, 0,
				    (struct sockaddr*)&addr_in,
				    &addr_size);
		if (recv_len >= 0) {
			ESP_LOGI(TAG, "recvfrom: %s:%d",
				 inet_ntoa(addr_in.sin_addr),
				 ntohs(addr_in.sin_port));

			buffer[recv_len] = '\0';
			ESP_LOGI(TAG, "recvdata: %s (%dbytes)", buffer, recv_len);

			/* If the flag is discover, then respond */
			if(!strcmp(buffer, DISCOVER_SAY)) {
				snprintf(buffer, sizeof(buffer), DISCOVER_RESPOND);
				sendto(discover_socket, buffer, strlen(buffer), 0,
				       (struct sockaddr*)&addr_in, addr_size);
				ESP_LOGI(TAG, "discover has responded");
			}
		} else {
			ESP_LOGI(TAG, "recv_len < 0, exit discover");
			break;
		}
	}
}

static void simple_ctrl_discover_task(void *pvParameters)
{
	while (1) {
		ESP_LOGI(TAG, "Wait network ready...");
		xSemaphoreTake(network_ready, portMAX_DELAY);

		ESP_LOGI(TAG, "Start discover...");
		simple_ctrl_do_discover();
	}

	vTaskDelete(NULL);
}

static void simple_ctrl_discover_init(void)
{
	int operate = 1;
	struct sockaddr_in address;
	int ret;

	ESP_LOGI(TAG, "Discover Init");

	/* Create UDP */
	discover_socket = socket(AF_INET, SOCK_DGRAM, 0);
	ESP_ERROR_CHECK(discover_socket < 0);

	/* Allow binding address reuse */
	setsockopt(discover_socket, SOL_SOCKET, SO_REUSEADDR, &operate, sizeof(operate));
	/* Enable broadcast */
	setsockopt(discover_socket, SOL_SOCKET, SO_BROADCAST, &operate, sizeof(operate));

	/* Bind port */
	address.sin_family = AF_INET;
	address.sin_port = htons(DISCOVER_UDP_PORT);
	address.sin_addr.s_addr = INADDR_ANY;
	ret = bind(discover_socket, (struct sockaddr *)&address, sizeof (address));
	ESP_ERROR_CHECK(ret < 0);

	xSemaphoreGive(network_ready);
}

static void simple_ctrl_discover_destroy(void)
{
	close(discover_socket);
}

static bool simple_ctrl_notify_callback(struct event_bus_msg *msg)
{
	switch (msg->type) {
	case EVENT_BUS_WIFI_CONNECTED:
		simple_ctrl_discover_init();
		break;
	case EVENT_BUS_WIFI_DISCONNECTED:
		simple_ctrl_discover_destroy();
		break;
	}

	return false;
}

void simple_ctrl_init(void)
{
	network_ready = xSemaphoreCreateBinary();
	ESP_ERROR_CHECK(network_ready == NULL);

	xTaskCreate(simple_ctrl_discover_task, "simple_ctrl_discover_task", 2048,
		    NULL, tskIDLE_PRIORITY, NULL);

	event_bus_register(simple_ctrl_notify_callback);
}
