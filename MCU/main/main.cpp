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

#include "audio.h"

static const char *TAG = "APP-MAIN";

// #define DEBUG_DATA_TO_SERIAL

#if defined(DEBUG_DATA_TO_SERIAL)

#include "driver/usb_serial_jtag.h"
#include "hal/usb_serial_jtag_ll.h"

static void usb_uart_config(void)
{
	/* Configure USB SERIAL JTAG */
	usb_serial_jtag_driver_config_t usb_serial_jtag_config = {
		.tx_buffer_size = 1024,
		.rx_buffer_size = 1024,
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
#endif

static void app_show_info(void)
{
	int size = esp_get_free_heap_size();

	ESP_LOGI(TAG, "Free heap size: %dbyte", size);
}

static void audio_event_callback(uint8_t event, void *data, uint32_t size)
{
#if defined(DEBUG_DATA_TO_SERIAL)
	uint8_t *p = (uint8_t *)data;
	uint32_t count = 0;
	size_t write_size;
#endif

	if (event == AUDIO_EVENT_VOICE_STOP) {
		printf("Output: %ums\n", audio_size_to_time(size));

#if defined(DEBUG_DATA_TO_SERIAL)
		/* Write to USB Serial */
		while (count < size) {
			write_size = size - count < 1024 ? size - count : 1024;
			usb_uart_write_bytes(p + count, write_size);
			count += write_size;
		}
#endif
	}
}

static struct audio_handler handler;

extern "C" void app_main(void)
{
	app_show_info();
#if defined(DEBUG_DATA_TO_SERIAL)
	usb_uart_config();
#endif

	handler.event = audio_event_callback;
	audio_init(&handler);
}
