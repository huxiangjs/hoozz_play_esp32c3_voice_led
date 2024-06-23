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

#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>

#include "sdkconfig.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "argtable3/argtable3.h"
#include "driver/i2c.h"
#include "driver/i2s_tdm.h"

#include <fvad.h>
#include <es7243e.h>

#include "audio.h"
#include "sample_send.h"

// #define DEBUG

#define ES7243E_ADDR			0x10		// AD0 AD1 AD2 is low

#define AUDIO_SAMPLE_DEPTH		(16 / 8)	// 16-bit: 2-byte
#define AUDIO_SAMPLE_RATE		16000		// 16kHz

#define AUDIO_MIN_TIME			300		// 400ms
#define AUDIO_FILTER_TIME		100		// 100ms

// Sampled and sent to TCP
// #define SAMPLE_SEND_IP		"192.168.31.251"

static const char *TAG = "AUDIO";

static i2s_chan_handle_t rx_chan;
static TaskHandle_t audio_task_handle;
static TaskHandle_t i2s_task_handle;
static uint8_t audio_event;
static uint8_t audio_vad_state;
static struct audio_handler audio_hand;

static uint16_t audio_frame_time;
static uint16_t audio_sample_point;
static uint16_t audio_frame_size;
static uint8_t *audio_frame_buf;

static uint16_t audio_max_time;
static uint16_t audio_time_count;

static void audio_i2c_init(void)
{
	i2c_config_t conf = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = GPIO_NUM_2,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_io_num = GPIO_NUM_3,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 100000,
	};

	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
	ESP_ERROR_CHECK(i2c_param_config(I2C_NUM_0, &conf));
}

static void audio_i2c_scan(void)
{
	uint8_t address;
	char buffer[512];
	uint16_t index = 0;

	index += snprintf(buffer + index, sizeof(buffer) - index,
			  "\r\n     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f\r\n");
	for (int i = 0; i < 128; i += 16) {
		index += snprintf(buffer + index, sizeof(buffer) - index,"%02x: ", i);
		for (int j = 0; j < 16; j++) {
			address = i + j;
			i2c_cmd_handle_t cmd = i2c_cmd_link_create();
			i2c_master_start(cmd);
			i2c_master_write_byte(cmd, (address << 1) | I2C_MASTER_WRITE, true);
			i2c_master_stop(cmd);
			esp_err_t ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 50 / portTICK_PERIOD_MS);
			i2c_cmd_link_delete(cmd);
			if (ret == ESP_OK) {
				index += snprintf(buffer + index, sizeof(buffer) - index, "%02x ", address);
			} else if (ret == ESP_ERR_TIMEOUT) {
				index += snprintf(buffer + index, sizeof(buffer) - index, "UU ");
			} else {
				index += snprintf(buffer + index, sizeof(buffer) - index, "-- ");
			}
		}
		index += snprintf(buffer + index, sizeof(buffer) - index, "\r\n");
	}

	ESP_LOGI(TAG, "%s", buffer);
}

static void audio_i2s_init(void)
{
	i2s_chan_config_t rx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
	ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, NULL, &rx_chan));

	i2s_tdm_config_t rx_tdm_cfg = {
		.clk_cfg  = I2S_TDM_CLK_DEFAULT_CONFIG(AUDIO_SAMPLE_RATE),
		.slot_cfg = I2S_TDM_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT,
							    I2S_SLOT_MODE_MONO,
							    I2S_TDM_SLOT0),
		.gpio_cfg = {
			.mclk = GPIO_NUM_10,
			.bclk = GPIO_NUM_6,
			.ws   = GPIO_NUM_7,
			.dout = I2S_GPIO_UNUSED,
			.din  = GPIO_NUM_5,
			.invert_flags = {
				.mclk_inv = false,
				.bclk_inv = false,
				.ws_inv   = false, // Left Channel
				// .ws_inv   = true, // Right Channel
			},
		},
	};

	rx_tdm_cfg.clk_cfg.mclk_multiple = I2S_MCLK_MULTIPLE_512;
	rx_tdm_cfg.slot_cfg.big_endian = true;

	ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(rx_chan, &rx_tdm_cfg));
}

static void audio_callback_task(void *args)
{
#if !defined(SAMPLE_SEND_IP)
	struct audio_handler *handler = (struct audio_handler *)args;
#endif

	while (1) {
		/* Wait */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

#if 0
		/*
		 * Show voice status
		 * (ESP32 needs to be followed by \r\n before outputting to the serial port)
		 */
		if (audio_event & AUDIO_EVENT_VOICE_START)
			printf("[");
		if (audio_event & AUDIO_EVENT_VOICE_FRAME)
			printf("=");
		if (audio_event & AUDIO_EVENT_VOICE_STOP)
			printf("]");
		if (audio_event & AUDIO_EVENT_VOICE_OVER)
			printf(" OVER");
		if (audio_event & AUDIO_EVENT_VOICE_DROP)
			printf(" DROP");
		if (audio_event & AUDIO_EVENT_VOICE_STOP)
			printf(" %dms\r\n", audio_time_count);
#endif

#if defined(SAMPLE_SEND_IP)
		sample_send_event(audio_event, audio_frame_buf, audio_frame_size);
#else
		/* Process */
		handler->event(audio_event, audio_frame_buf, audio_frame_size);
#endif
		/* Done */
		xTaskNotifyGive(i2s_task_handle);
	}

	vTaskDelete(NULL);
}

static void audio_i2s_read_task(void *args)
{
	size_t r_bytes = 0;
	uint16_t keep_count = 0;
	bool busy = false;
	int ret;

	Fvad *vad = fvad_new();
	ESP_ERROR_CHECK(!vad);
	ESP_ERROR_CHECK(fvad_set_sample_rate(vad, AUDIO_SAMPLE_RATE) < 0);
	ESP_ERROR_CHECK(fvad_set_mode(vad, 0) < 0);

	/* Enable the RX channel */
	ESP_ERROR_CHECK(i2s_channel_enable(rx_chan));

	while (1) {
		/* Read i2s data */
		if (i2s_channel_read(rx_chan, audio_frame_buf, audio_frame_size, &r_bytes, 1000/*ms*/) == ESP_OK) {
#if 0
			ESP_LOGI(TAG, "Read Task: i2s read %d bytes, first data: %d", r_bytes, *((short int *)audio_frame_size));
#endif
			if (audio_frame_size != r_bytes) {
				ESP_LOGE(TAG, "%d bytes of data were expected, but only %d bytes were received",
					 audio_frame_size, r_bytes);
				continue;
			}

			/* We discard all data received while there is audio being processed */
			if (busy) {
				if (ulTaskNotifyTake(pdTRUE, 0) == pdTRUE)
					busy = false;
				else
					continue;
			}

			/* Do VAD */
			ret = fvad_process(vad, (int16_t *)audio_frame_buf, audio_sample_point);
			if (ret < 0) {
				ESP_LOGE(TAG, "VAD processing failed");
				continue;
			}
			ret = !!ret;
#if defined(DEBUG)
			printf("%c\n", ret ? '#' : '-');
#endif

			/*
			 * If the last speech was too long, wait until the speech
			 * is inactive before starting again.
			 */
			if (audio_event & AUDIO_EVENT_VOICE_OVER) {
				if (ret)
					continue;
				else
					audio_event = 0;
			}

			/* Changed */
			if (audio_vad_state ^ ret) {
				if (ret) {
					audio_time_count = audio_frame_time;
					audio_vad_state = ret;
					audio_event = AUDIO_EVENT_VOICE_START | AUDIO_EVENT_VOICE_FRAME;
					busy = true;
					xTaskNotifyGive(audio_task_handle);
				} else if (keep_count >= AUDIO_FILTER_TIME) {
					audio_time_count += audio_frame_time;
					audio_vad_state = ret;
					audio_event = AUDIO_EVENT_VOICE_FRAME | AUDIO_EVENT_VOICE_STOP;
					if (audio_time_count < AUDIO_MIN_TIME)
						audio_event |= AUDIO_EVENT_VOICE_DROP;
					busy = true;
					xTaskNotifyGive(audio_task_handle);
				}
			} else {
				/* Single frame */
				if (audio_vad_state) {
					audio_time_count += audio_frame_time;
					audio_event = AUDIO_EVENT_VOICE_FRAME;
					if (audio_time_count >= audio_max_time) {
						audio_event |= AUDIO_EVENT_VOICE_STOP | AUDIO_EVENT_VOICE_OVER;
						audio_vad_state = 0;
					}
					busy = true;
					xTaskNotifyGive(audio_task_handle);
				}
			}

			keep_count = ret ? 0 : (keep_count + audio_frame_time);

		} else {
			ESP_LOGE(TAG, "i2s read failed");
		}
	}

	/* Disable the RX channel */
	ESP_ERROR_CHECK(i2s_channel_disable(rx_chan));
	fvad_free(vad);
	vTaskDelete(NULL);
}

static bool audio_i2c_write_reg(uint8_t addr, uint8_t data)
{
	uint8_t buf[2] = {addr, data};

	esp_err_t ret = i2c_master_write_to_device(I2C_NUM_0,
						   ES7243E_ADDR,
						   buf, sizeof(buf),
						   50 / portTICK_PERIOD_MS);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "i2c write fail, errno: 0x%03x (%d)", ret, ret);
		return false;
	}

	return true;
}

void audio_init(struct audio_handler *handler)
{
	int ret;

	if (handler == NULL) {
		ESP_LOGE(TAG, "handler is null");
		return;
	}

	audio_hand = *handler;
	audio_frame_time = handler->frame_time;
	audio_max_time = handler->max_time;
	audio_sample_point = AUDIO_SAMPLE_RATE * audio_frame_time / 1000;
	audio_frame_size = audio_sample_point * AUDIO_SAMPLE_DEPTH;
	audio_frame_buf = (uint8_t *)calloc(1, audio_frame_size);
	ESP_ERROR_CHECK(audio_frame_buf == NULL);

	ESP_LOGI(TAG, "audio_frame_time = %dms", audio_frame_time);
	ESP_LOGI(TAG, "audio_max_time = %dms", audio_max_time);
	ESP_LOGI(TAG, "audio_sample_point = %d", audio_sample_point);
	ESP_LOGI(TAG, "audio_frame_size = %d", audio_frame_size);

	/* I2C init */
	audio_i2c_init();
	audio_i2c_scan();
	vTaskDelay(10 / portTICK_PERIOD_MS);

	/* ES7243E init */
	struct es7243e es = { .write_reg = audio_i2c_write_reg, };
	es7243e_init(&es);

	/* I2S init */
	audio_i2s_init();

#if defined(SAMPLE_SEND_IP)
	sample_send_init(SAMPLE_SEND_IP);
#endif

	/* Start task */
	if (audio_hand.event) {
		ret = xTaskCreate(audio_callback_task, "audio_callback_task", 2048,
				  &audio_hand, tskIDLE_PRIORITY + 1, &audio_task_handle);
		ESP_ERROR_CHECK(ret != pdPASS);
	} else {
		ESP_LOGE(TAG, "No nedd to processing");
	}

	ret = xTaskCreate(audio_i2s_read_task, "audio_i2s_read_task", 4096,
			  NULL, tskIDLE_PRIORITY + 1, &i2s_task_handle);
	ESP_ERROR_CHECK(ret != pdPASS);
}
