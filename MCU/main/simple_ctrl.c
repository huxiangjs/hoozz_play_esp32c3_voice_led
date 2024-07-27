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
#include "crypto.h"
#include "class_id.h"
#include "spiffs.h"

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

#define NOTIFY_BUFFER_SIZE		128

#define CTRL_LOAD_TYPE_PING		0x00
#define CTRL_LOAD_TYPE_INFO		0x01
#define CTRL_LOAD_TYPE_REQUEST		0x02
#define CTRL_LOAD_TYPE_NOTIFY		0x03

#define CTRL_INFO_TYPE_GETNAME		0x00
#define CTRL_INFO_TYPE_SETNAME		0x01
#define CTRL_INFO_TYPE_GETCLASSID	0x02
#define CTRL_INFO_TYPE_SETPASSWD	0x03

#define CTRL_RETURN_OK			0x00
#define CTRL_RETURN_FAIL		0x01

#define CTRL_LOAD_MAGIC			"HOOZZ"
#define CTRL_LOAD_HEADER_SIZE		16

#define CTRL_INFO_PASSWD_LENGTH		16

#define SPIFFS_PASSWD_FILE_NAME		"VOICE-LED-PASSWD"

struct simple_ctrl_handle {
	uint8_t load_type;
	uint8_t crypto_type;
	uint32_t load_len;
	uint32_t ret_len;
	struct crypto in_crypto;
	struct crypto out_crypto;
	char passwd_data[CTRL_INFO_PASSWD_LENGTH];
	int passwd_len;
};

static int discover_socket;
static TaskHandle_t discover_handle;
static SemaphoreHandle_t send_mutex;

static char info_name[SIMPLE_CTRL_INFO_NAME_LENGTH + 1] = SIMPLE_CTRL_INFO_NAME_DEFAULT;
static uint8_t info_class_id = CLASS_ID_UNKNOWN;
static char info_id[SIMPLE_CTRL_INFO_ID_LENGTH + 1] = SIMPLE_CTRL_INFO_ID_DEFAULT;

static char crypto_passwd[1 + CTRL_INFO_PASSWD_LENGTH];
static uint8_t crypto_type = CRYPTO_TYPE_AES128ECB;

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
	/* |--MAGIC(6bytes)--|--CLASS(2bytes)--|--ID(14bytes)--|--NAME(nbytes)--| */
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
	memcpy(handle->passwd_data, crypto_passwd + 1, crypto_passwd[0]);
	handle->passwd_len = crypto_passwd[0];
	crypto_init(&handle->in_crypto, crypto_type, handle->passwd_data, handle->passwd_len);
	crypto_init(&handle->out_crypto, crypto_type, handle->passwd_data, handle->passwd_len);
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
	case CTRL_INFO_TYPE_SETPASSWD:
		len = vaild_size - 1;
		if (len > CTRL_INFO_PASSWD_LENGTH) {
			buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
			ret = 2;
			ESP_LOGE(TAG, "Password is too long");
		} else {
			memcpy(crypto_passwd + 1, buffer + buf_offs + 1, len);
			crypto_passwd[0] = (char)len;
			ret = spiffs_save(SPIFFS_PASSWD_FILE_NAME, crypto_passwd,
					  sizeof(crypto_passwd));
			if (ret) {
				buffer[buf_offs + 1] = CTRL_RETURN_FAIL;
				ret = 2;
				ESP_LOGE(TAG, "Access password saving failed");
			} else {
				buffer[buf_offs + 1] = CTRL_RETURN_OK;
				ret = 2;
				ESP_LOGI(TAG, "Access password has been changed");
			}
		}
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

	if (handle->crypto_type != crypto_type) {
		ESP_LOGE(TAG, "Encryption method does not match");
		return -1;
	}

	ret = crypto_de(&handle->in_crypto, buffer, vaild_size, buff_size);
	if (ret < 0)
		return ret;
	vaild_size = ret;

	if ((vaild_size < CTRL_LOAD_HEADER_SIZE) ||
	    memcmp(CTRL_LOAD_MAGIC, buffer, sizeof(CTRL_LOAD_MAGIC))) {
		ESP_LOGE(TAG, "Magic is incorrect");
		return -1;
	}
	/* |--Magic(6bytes)--|--Reserved(6bytes)--|--Data size(4bytes)--| */
	vaild_size = (buffer[12] << 0) |
		     (buffer[13] << 8) |
		     (buffer[14] << 16) |
		     (buffer[15] << 24);

	switch (handle->load_type) {
	case CTRL_LOAD_TYPE_PING:
		ret = simple_ctrl_ping(buffer, CTRL_LOAD_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_LOAD_TYPE_INFO:
		ret = simple_ctrl_info(buffer, CTRL_LOAD_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_LOAD_TYPE_REQUEST:
		ret = request_handle(buffer, CTRL_LOAD_HEADER_SIZE, vaild_size, buff_size);
		break;
	case CTRL_LOAD_TYPE_NOTIFY:
		ret = 0;
		break;
	}

	handle->ret_len = 0;

	if (ret > 0) {
		buffer[12] = (uint8_t)((ret >> 0) & 0xff);
		buffer[13] = (uint8_t)((ret >> 8) & 0xff);
		buffer[14] = (uint8_t)((ret >> 16) & 0xff);
		buffer[15] = (uint8_t)((ret >> 24) & 0xff);
		ret += CTRL_LOAD_HEADER_SIZE;
		handle->ret_len = (uint32_t)ret;
	}

	return ret;
}

static volatile int fds[BODY_TCP_MAX_ACCEPT];

void simple_ctrl_notify(char *buffer, int size)
{
	int index;
	int ret;
	uint8_t load_info[6];
	char header[CTRL_LOAD_HEADER_SIZE] = CTRL_LOAD_MAGIC;
	char enbuf[NOTIFY_BUFFER_SIZE];
	int load_size;
	struct crypto handle;

	if (size > NOTIFY_BUFFER_SIZE) {
		ESP_LOGE(TAG, "Not enough space in the enbuf");
		return;
	}
	memcpy(enbuf, buffer, size);

	crypto_init(&handle, crypto_type, crypto_passwd + 1, crypto_passwd[0]);

	header[12] = (uint8_t)((size >> 0) & 0xff);
	header[13] = (uint8_t)((size >> 8) & 0xff);
	header[14] = (uint8_t)((size >> 16) & 0xff);
	header[15] = (uint8_t)((size >> 24) & 0xff);
	ret = crypto_en(&handle, header, sizeof(header), sizeof(header));
	if (ret != sizeof(header))
		return;
	load_size = sizeof(header);

	ret = crypto_en(&handle, enbuf, size, sizeof(enbuf));
	if (ret < 0)
		return;
	size = ret;
	load_size += size;

	load_info[0] = CTRL_LOAD_TYPE_NOTIFY;
	load_info[1] = crypto_type;
	load_info[2] = (uint8_t)((load_size >> 0) & 0xff);
	load_info[3] = (uint8_t)((load_size >> 8) & 0xff);
	load_info[4] = (uint8_t)((load_size >> 16) & 0xff);
	load_info[5] = (uint8_t)((load_size >> 24) & 0xff);

	ESP_LOGD(TAG, "Load size: %d", load_size);

	for (index = 0; index < BODY_TCP_MAX_ACCEPT; index++) {
		if (fds[index] != -1) {
			xSemaphoreTake(send_mutex, portMAX_DELAY);

			/* Send info */
			ret = send(fds[index], load_info, sizeof(load_info), 0);
			if (ret != sizeof(load_info)) {
				ESP_LOGE(TAG, "Send load info fail: %d", ret);
				xSemaphoreGive(send_mutex);
				continue;
			}

			/* Send header */
			ret = send(fds[index], header, sizeof(header), 0);
			if (ret != sizeof(header)) {
				ESP_LOGE(TAG, "Send load header fail: %d", ret);
				xSemaphoreGive(send_mutex);
				continue;
			}

			/* Send load */
			ret = send(fds[index], enbuf, size, 0);
			if (ret != size) {
				ESP_LOGE(TAG, "Send load fail: %d", ret);
				xSemaphoreGive(send_mutex);
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
						handle.load_type = buffer[0];
						handle.crypto_type = buffer[1];
						handle.load_len = (buffer[2] << 0) |
								  (buffer[3] << 8) |
								  (buffer[4] << 16) |
								  (buffer[5] << 24);
						ESP_LOGI(TAG, "(%d) load_type:%02x, crypto_type:%02x, load_len:%lu", fds[index],
							 handle.load_type, handle.crypto_type, handle.load_len);

						count = 0;
						while (count < handle.load_len) {
							size = handle.load_len > (uint32_t)sizeof(buffer) ?
							       sizeof(buffer) : handle.load_len;
							/* Read pack data */
							ret = recv(fds[index], buffer, size, 0);
							if (ret <= 0) {
								ESP_LOGI(TAG, "Read exception or timeout, disconnected (%d)", fds[index]);
								goto closefd;
							}
							ESP_LOGD(TAG, "(%d) Expect: %lu; Result: %d", fds[index], size, ret);

							count += ret;
							ESP_LOGD(TAG, "(%d) Handle [%lu/%lu]", fds[index], count, handle.load_len);

							/* Handle data */
							ret = simple_ctrl_handle_pad(&handle, buffer, ret, sizeof(buffer));
							if (ret > 0) {
								uint8_t load_info[6];
								int vaild_size = ret;

								/* Encrypto */
								ret = crypto_en(&handle.out_crypto, buffer, vaild_size, sizeof(buffer));
								if (ret < 0) {
									ESP_LOGE(TAG, "(%d) Encryption failed: %d", fds[index], ret);
									goto closefd;
								}
								vaild_size = ret;

								load_info[0] = handle.load_type;
								load_info[1] = crypto_type;
								load_info[2] = (uint8_t)((vaild_size >> 0) & 0xff);
								load_info[3] = (uint8_t)((vaild_size >> 8) & 0xff);
								load_info[4] = (uint8_t)((vaild_size >> 16) & 0xff);
								load_info[5] = (uint8_t)((vaild_size >> 24) & 0xff);

								xSemaphoreTake(send_mutex, portMAX_DELAY);

								/* Send info */
								ret = send(fds[index], load_info, sizeof(load_info), 0);
								if (ret != sizeof(load_info)) {
									ESP_LOGE(TAG, "(%d) Send load info fail: %d", fds[index], ret);
									xSemaphoreGive(send_mutex);
									goto closefd;
								}
								ESP_LOGD(TAG, "(%d) Send load info: %d", fds[index], ret);

								/* Send data */
								ret = send(fds[index], buffer, vaild_size, 0);
								if (ret != vaild_size) {
									ESP_LOGE(TAG, "(%d) Send load data fail: %d", fds[index], ret);
									xSemaphoreGive(send_mutex);
									goto closefd;
								}
								ESP_LOGD(TAG, "(%d) Send load data: %d", fds[index], ret);

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

	spiffs_load(SPIFFS_PASSWD_FILE_NAME, crypto_passwd, sizeof(crypto_passwd));

	ret = xTaskCreate(simple_ctrl_discover_task, "simple_ctrl_discover_task", 2048,
			  NULL, tskIDLE_PRIORITY + 2, &discover_handle);
	ESP_ERROR_CHECK(ret != pdPASS);

	send_mutex = xSemaphoreCreateMutex();
	ret = xTaskCreate(simple_ctrl_body_task, "simple_ctrl_body_task", 4096,
			  NULL, tskIDLE_PRIORITY + 2, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);

	event_bus_register(simple_ctrl_notify_callback);
}
