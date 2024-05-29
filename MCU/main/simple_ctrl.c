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
#include <esp_mac.h>
#include "event_bus.h"
#include "encryp.h"
#include "class_id.h"

static const char *TAG = "SIMPLE-CTRL";

#define SIMPLE_CTRL_INFO_ID_LENGTH	14
#define SIMPLE_CTRL_INFO_ID_DEFAULT	"00000000000000"
#define SIMPLE_CTRL_INFO_NAME_LENGTH	64
#define SIMPLE_CTRL_INFO_NAME_DEFAULT	"Unnamed"

#define DISCOVER_UDP_PORT		54542
#define DISCOVER_SAY			"HOOZZ?"
#define DISCOVER_RESPOND		"HOOZZ:"
#define DISCOVER_BUFFER_SIZE		(sizeof(DISCOVER_RESPOND) + 2 + SIMPLE_CTRL_INFO_ID_LENGTH + SIMPLE_CTRL_INFO_NAME_LENGTH)
#define DISCOVER_BROADCAST_NUM		40
#define DISCOVER_BROADCAST_ADDRESS	"255.255.255.255"

#define BODY_TCP_PORT			DISCOVER_UDP_PORT
#define BODY_TCP_MAX_ACCEPT		5
#define BODY_TCP_BUFFER_SIZE		128
#define BODY_TCP_TIMEOUT		30

#define CTRL_DATA_TYPE_PING		0x00
#define CTRL_DATA_TYPE_INFO		0x01
#define CTRL_DATA_TYPE_REQUEST		0x02
#define CTRL_DATA_TYPE_NOTIFY		0x03

#define CTRL_INFO_TYPE_GETNAME		0x00
#define CTRL_INFO_TYPE_SETNAME		0x01
#define CTRL_INFO_TYPE_GETCLASSID	0x02

#define CTRL_RETURN_OK			0x00
#define CTRL_RETURN_FAIL		0x01

#define CTRL_DATA_HEADER		"HOOZZ"
#define CTRL_DATA_HEADER_SIZE		(sizeof(CTRL_DATA_HEADER) - 1)

struct simple_ctrl_handle {
	uint8_t data_type;
	uint8_t encryp_type;
	uint32_t data_len;
	uint32_t ret_len;
};

static int discover_socket;
static TaskHandle_t discover_handle;
static SemaphoreHandle_t send_mutex;

static char info_name[SIMPLE_CTRL_INFO_NAME_LENGTH + 1] = SIMPLE_CTRL_INFO_NAME_DEFAULT;
static uint8_t info_class_id = CLASS_ID_UNKNOWN;
static char info_id[SIMPLE_CTRL_INFO_ID_LENGTH + 1] = SIMPLE_CTRL_INFO_ID_DEFAULT;

static void simple_ctrl_init_info_id(void)
{
	uint8_t mac[6];

	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA) != ESP_OK);
	sprintf(info_id, "%02x%02x%02x%02x%02x%02x%02x",
		mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], 0);

	ESP_LOGI(TAG, "ID(%s)", info_id);
}

static inline void simple_ctrl_discover_set_respond(char *buf, size_t buf_size)
{
	/* |--MAGIC(6bytes)--|--CLASS(2byte)--|--ID(14bytes)--|--NAME(nbytes)--| */
	snprintf(buf, buf_size, "%s%02x%s%s", DISCOVER_RESPOND, info_class_id, info_id, info_name);
}

static void simple_ctrl_discover_handle(void)
{
	char buffer[DISCOVER_BUFFER_SIZE];
	struct sockaddr_in addr_in;
	socklen_t addr_size = sizeof(addr_in);
	int recv_len;
	uint8_t count;
	int ret;
	int operate = 1;
	struct sockaddr_in address;

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

	/* Full address */
	addr_in.sin_family = AF_INET;
	addr_in.sin_port = htons(DISCOVER_UDP_PORT);
	ret = inet_pton(AF_INET, DISCOVER_BROADCAST_ADDRESS,
			(void *)&addr_in.sin_addr);
	ESP_ERROR_CHECK(ret < 0);
	count = DISCOVER_BROADCAST_NUM;
	while (count--) {
		simple_ctrl_discover_set_respond(buffer, sizeof(buffer));
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
				simple_ctrl_discover_set_respond(buffer, sizeof(buffer));
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
	ESP_LOGI(TAG, "Discover Running");

	while (1) {
		ESP_LOGI(TAG, "Wait network ready...");
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		ESP_LOGI(TAG, "Start discover...");
		simple_ctrl_discover_handle();
	}

	vTaskDelete(NULL);
}

static bool simple_ctrl_notify_callback(struct event_bus_msg *msg)
{
	switch (msg->type) {
	case EVENT_BUS_WIFI_CONNECTED:
		xTaskNotifyGive(discover_handle);
		break;
	case EVENT_BUS_WIFI_DISCONNECTED:
		close(discover_socket);
		break;
	}

	return false;
}

static void simple_ctrl_handle_reset(struct simple_ctrl_handle *handle)
{
	memset(handle, 0, sizeof(struct simple_ctrl_handle));
}

static int simple_ctrl_none_request(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	/* Do nothing */
	return 0;
}

static int (*request_handle)(char *buffer, int buf_offs, int vaild_size, int buff_size)
	= simple_ctrl_none_request;

void simple_ctrl_request_register(int (*request)(char *buffer, int buf_offs, int vaild_size, int buff_size))
{
	request_handle = request;
}

static uint8_t encryp_type = ENCRYP_TYPE_NONE;

void simple_ctrl_set_encryp_type(uint8_t type)
{
	encryp_type = type;
}

void simple_ctrl_set_class_id(uint8_t new_id)
{
	info_class_id = new_id;
}

void simple_ctrl_set_name(const char *new_name)
{
	size_t len = strlen(new_name);
	size_t copy_size;

	copy_size = SIMPLE_CTRL_INFO_NAME_LENGTH;
	copy_size = copy_size < len ? copy_size : len;

	memcpy(info_name, new_name, copy_size);
	info_name[copy_size] = '\0';
}

static int simple_ctrl_ping(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	int ret = -1;
	uint8_t ping_interval;

	if (vaild_size != 1) {
		ESP_LOGE(TAG, "Ping command is incorrect");
		return -1;
	}

	ping_interval = buffer[buf_offs];
	ESP_LOGD(TAG, "Remote ping interval: %u\n", ping_interval);

	buffer[buf_offs] = CTRL_RETURN_OK;
	ret = 1;

	return ret;
}

static int simple_ctrl_info(char *buffer, int buf_offs, int vaild_size, int buff_size)
{
	size_t len;
	int ret = -1;
	uint8_t info_cmd;

	if (vaild_size < 1) {
		ESP_LOGE(TAG, "Information command is incorrect");
		return -1;
	}

	info_cmd = buffer[buf_offs];

	switch (info_cmd) {
	case CTRL_INFO_TYPE_GETNAME:
		len = strlen(info_name);
		if ((len + 2) > (buff_size - buf_offs)) {
			ESP_LOGE(TAG, "Buffer does not have enough space");
			return -1;
		}
		buffer[buf_offs + 1] = CTRL_RETURN_OK;
		memcpy(buffer + buf_offs + 2, info_name, len);
		ret = 2 + (int)len;
		break;
	case CTRL_INFO_TYPE_SETNAME:
		len = vaild_size - 1;
		if (len > SIMPLE_CTRL_INFO_NAME_LENGTH) {
			buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
			ret = 2;
		} else {
			memcpy(info_name, buffer + buf_offs + 1, len);
			info_name[len] = '\0';
			buffer[buf_offs + 1] = CTRL_RETURN_OK;
			ret = 2;
		}
		break;
	case CTRL_INFO_TYPE_GETCLASSID:
		ret = 3;
		if (ret > (buff_size - buf_offs)) {
			ESP_LOGE(TAG, "Buffer does not have enough space");
			return -1;
		}
		buffer[buf_offs + 1] = CTRL_RETURN_OK;
		buffer[buf_offs + 2] = info_class_id;
		break;
	default:
		buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
		ret = 2;
		break;
	}

	return ret;
}

static int simple_ctrl_handle_pad(struct simple_ctrl_handle *handle,
				  char *buffer, int vaild_size, int buff_size)
{
	int ret;

	if (handle->encryp_type != encryp_type) {
		ESP_LOGE(TAG, "Encryption method does not match");
		return -1;
	}

	ret = decryp_do(handle->encryp_type, buffer, vaild_size, buff_size);
	if (ret < 0)
		return ret;
	vaild_size = ret;

	if ((ret < CTRL_DATA_HEADER_SIZE) ||
	    memcmp(CTRL_DATA_HEADER, buffer, CTRL_DATA_HEADER_SIZE)) {
		ESP_LOGE(TAG, "Data format is incorrect");
		return -1;
	}
	vaild_size -= CTRL_DATA_HEADER_SIZE;

	switch (handle->data_type) {
	case CTRL_DATA_TYPE_PING:
		ret = simple_ctrl_ping(buffer, CTRL_DATA_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_DATA_TYPE_INFO:
		ret = simple_ctrl_info(buffer, CTRL_DATA_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_DATA_TYPE_REQUEST:
		ret = request_handle(buffer, CTRL_DATA_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_DATA_TYPE_NOTIFY:
		ret = 0;
		break;
	}

	handle->ret_len = 0;

	if (ret > 0) {
		ret += CTRL_DATA_HEADER_SIZE;
		handle->ret_len = (uint32_t)ret;
	}

	return ret;
}

static volatile int fds[BODY_TCP_MAX_ACCEPT];

void simple_ctrl_notify(char *buffer, int size)
{
	int index;
	int ret;
	uint8_t data_info[6];
	int data_size;

	ret = encryp_do(encryp_type, buffer, size, size);
	if (ret < 0)
		return;
	size = ret;
	data_size = size + CTRL_DATA_HEADER_SIZE;	// TODO: enable encryp

	data_info[0] = CTRL_DATA_TYPE_NOTIFY;
	data_info[1] = encryp_type;
	data_info[2] = (uint8_t)((data_size >> 0) & 0xff);
	data_info[3] = (uint8_t)((data_size >> 8) & 0xff);
	data_info[4] = (uint8_t)((data_size >> 16) & 0xff);
	data_info[5] = (uint8_t)((data_size >> 24) & 0xff);

	ESP_LOGD(TAG, "Data size: %d", data_size);

	for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
		if (fds[index] != -1) {
			xSemaphoreTake(send_mutex, portMAX_DELAY);

			/* Send info */
			ret = send(fds[index], data_info, sizeof(data_info), 0);
			if (ret != sizeof(data_info)) {
				ESP_LOGE(TAG, "Send data info fail: %d", ret);
				continue;
			}

			/* Send header */
			ret = send(fds[index], CTRL_DATA_HEADER, CTRL_DATA_HEADER_SIZE, 0);
			if (ret != CTRL_DATA_HEADER_SIZE) {
				ESP_LOGE(TAG, "Send data header fail: %d", ret);
				continue;
			}

			/* Send data */
			ret = send(fds[index], buffer, size, 0);
			if (ret != size) {
				ESP_LOGE(TAG, "Send data fail: %d", ret);
				continue;
			}

			xSemaphoreGive(send_mutex);

			ESP_LOGD(TAG, "Notification completed (%d)", fds[index]);
		}
	}
}

static void simple_ctrl_body_handle(void)
{
	int operate = 1;
	int ret;
	int sel_ret;
	static int body_socket;
	struct sockaddr_in address;
	struct sockaddr_in addr_in;
	socklen_t addr_size = sizeof(addr_in);
	int index;
	int max_fd;
	int newconn;
	fd_set readfds;
	char buffer[BODY_TCP_BUFFER_SIZE];
	struct simple_ctrl_handle handle;
	uint32_t count;
	uint32_t size;
	struct timeval timeout;

	timeout.tv_sec = BODY_TCP_TIMEOUT;
	timeout.tv_usec = 0;

	/* Create UDP */
	body_socket = socket(AF_INET, SOCK_STREAM, 0);
	ESP_ERROR_CHECK(body_socket < 0);

	/* Allow binding address reuse */
	setsockopt(body_socket, SOL_SOCKET, SO_REUSEADDR, &operate, sizeof(operate));

	/* Bind port */
	address.sin_family = AF_INET;
	address.sin_port = htons(BODY_TCP_PORT);
	address.sin_addr.s_addr = INADDR_ANY;
	ret = bind(body_socket, (struct sockaddr *)&address, sizeof (address));
	ESP_ERROR_CHECK(ret < 0);

	/*
	 * Start listen
	 *
	 * TCP listen backlog = 1, then the maximum waiting length is 1 + 1 = 2
	 * (backlog represents the maximum length of the waiting queue)
	 */
	ret = listen(body_socket, 1);
	ESP_ERROR_CHECK(ret < 0);

	/* Initialize all to invalid */
	for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++)
		fds[index] = -1;

	while (1) {
		/* Set select */
		FD_ZERO(&readfds);
		FD_SET(body_socket, &readfds);
		max_fd = body_socket;
		/* Find all valid sockets */
		for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
			if (fds[index] != -1) {
				FD_SET(fds[index], &readfds);
				if (fds[index] > max_fd)
					max_fd = fds[index];
			}
		}

		/* Wait (OK > 0; Timeout == 0; Failed < 0) */
		sel_ret = select(max_fd + 1, &readfds, NULL, NULL, &timeout);
		ESP_ERROR_CHECK(sel_ret < 0);
		ESP_LOGD(TAG, "Select resule: %d", sel_ret);

		/* Need to accept or receive? */
		if (FD_ISSET(body_socket, &readfds)) {
			newconn = accept(body_socket, (struct sockaddr *)&addr_in, &addr_size);
			if (newconn >= 0) {
				ESP_LOGI(TAG, "%s:%d has been connected (%d)", inet_ntoa(addr_in.sin_addr),
					 ntohs(addr_in.sin_port), newconn);
			} else {
				ESP_LOGI(TAG, "newconn < 0, drop it");
				continue;
			}

			for (index = 0; (index < BODY_TCP_MAX_ACCEPT) && (fds[index] != -1); index++);
			if (index < BODY_TCP_MAX_ACCEPT) {
				/* Set read timeout */
				setsockopt(newconn, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
				fds[index] = newconn;
			} else {
				ESP_LOGW(TAG, "The connection has reached the set maximum");
				close(newconn);
				ESP_LOGI(TAG, "Closed (%d)", newconn);
			}
		}

		for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
			if (fds[index] != -1) {
				if (FD_ISSET(fds[index], &readfds)) {
					/* Read control fields */
					ret = recv(fds[index], buffer, 6, 0);
					if (ret > 0) {
						if (ret < 6) {
							ESP_LOGW(TAG, "Control field size mismatch, closed (%d)", fds[index]);
							goto closefd;
						}

						simple_ctrl_handle_reset(&handle);

						/* Fill parameter */
						handle.data_type = buffer[0];
						handle.encryp_type = buffer[1];
						handle.data_len = (buffer[2] << 0) |
								  (buffer[3] << 8) |
								  (buffer[4] << 16) |
								  (buffer[5] << 24);
						ESP_LOGI(TAG, "(%d) data_type:%02x, encryp_type:%02x, data_len:%lu", fds[index],
							 handle.data_type, handle.encryp_type, handle.data_len);

						count = 0;
						while (count < handle.data_len) {
							size = handle.data_len > (uint32_t)sizeof(buffer) ?
							       sizeof(buffer) : handle.data_len;
							/* Read pack data */
							ret = recv(fds[index], buffer, size, 0);
							if (ret <= 0) {
								ESP_LOGI(TAG, "Read exception or timeout, disconnected (%d)", fds[index]);
								goto closefd;
							}
							ESP_LOGD(TAG, "(%d) Expect: %lu; Result: %d", fds[index], size, ret);

							count += ret;
							ESP_LOGD(TAG, "(%d) Handle [%lu/%lu]", fds[index], count, handle.data_len);

							/* Handle data */
							ret = simple_ctrl_handle_pad(&handle, buffer, ret, sizeof(buffer));
							if (ret > 0) {
								uint8_t data_info[6];
								int vaild_size = ret;

								data_info[0] = handle.data_type;
								data_info[1] = handle.encryp_type;
								data_info[2] = (uint8_t)((handle.ret_len >> 0) & 0xff);
								data_info[3] = (uint8_t)((handle.ret_len >> 8) & 0xff);
								data_info[4] = (uint8_t)((handle.ret_len >> 16) & 0xff);
								data_info[5] = (uint8_t)((handle.ret_len >> 24) & 0xff);

								xSemaphoreTake(send_mutex, portMAX_DELAY);

								/* Send info */
								ret = send(fds[index], data_info, sizeof(data_info), 0);
								if (ret != sizeof(data_info)) {
									ESP_LOGE(TAG, "(%d) Send data info fail: %d", fds[index], ret);
									goto closefd;
								}
								ESP_LOGD(TAG, "(%d) Send info: %d", fds[index], ret);

								/* Send data */
								ret = send(fds[index], buffer, vaild_size, 0);
								if (ret != vaild_size) {
									ESP_LOGE(TAG, "(%d) Send data info fail: %d", fds[index], ret);
									goto closefd;
								}
								ESP_LOGD(TAG, "(%d) Send data: %d", fds[index], ret);

								xSemaphoreGive(send_mutex);
							} else if (ret < 0) {
								ESP_LOGI(TAG, "Handle exception, disconnected (%d)", fds[index]);
								goto closefd;
							}
						}

						ESP_LOGI(TAG, "(%d) Handle done", fds[index]);
					} else {
						ESP_LOGI(TAG, "Read exception or timeout, disconnected (%d)", fds[index]);
						goto closefd;
					}
				} else if (sel_ret == 0) {
					ESP_LOGI(TAG, "Select timeout, disconnected (%d)", fds[index]);
closefd:
					close(fds[index]);
					fds[index] = -1;
				}
			}
		}
	}

	close(body_socket);
}

static void simple_ctrl_body_task(void *pvParameters)
{
	ESP_LOGI(TAG, "Ctrl Body Running");

	while (1) {
		simple_ctrl_body_handle();
		vTaskDelay(pdMS_TO_TICKS(1000));
	}

	vTaskDelete(NULL);
}

void simple_ctrl_init(void)
{
	int ret;

	simple_ctrl_init_info_id();

	ret = xTaskCreate(simple_ctrl_discover_task, "simple_ctrl_discover_task", 2048,
			  NULL, tskIDLE_PRIORITY + 1, &discover_handle);
	ESP_ERROR_CHECK(ret != pdPASS);

	send_mutex = xSemaphoreCreateMutex();
	ret = xTaskCreate(simple_ctrl_body_task, "simple_ctrl_body_task", 4096,
			  NULL, tskIDLE_PRIORITY + 1, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);

	event_bus_register(simple_ctrl_notify_callback);
}
