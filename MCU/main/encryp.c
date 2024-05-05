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
#include <esp_log.h>
#include "encryp.h"

static const char *TAG = "ENCRYP";

struct algorithm {
	int (*encryp)(char *buffer, int vaild_size, int buff_size);
	int (*decryp)(char *buffer, int vaild_size, int buff_size);
};

static int encryp_none(char *buffer, int vaild_size, int buff_size)
{
	/* Do nothing */

	return vaild_size;
}

static int decryp_none(char *buffer, int vaild_size, int buff_size)
{
	/* Do nothing */

	return vaild_size;
}

static struct algorithm list[ENCRYP_TYPE_MAX] = {
	[ENCRYP_TYPE_NONE] = { encryp_none, decryp_none },	/* 0x00: No encryption */
};

int encryp_do(uint8_t encryp_type, char *buffer, int vaild_size, int buff_size)
{
	if (encryp_type >= ENCRYP_TYPE_MAX) {
		ESP_LOGE(TAG, "Unsupported encryption type: 0x%02x", encryp_type);
		return -1;
	}

	return list[encryp_type].encryp(buffer, vaild_size, buff_size);
}

int decryp_do(uint8_t encryp_type, char *buffer, int vaild_size, int buff_size)
{
	if (encryp_type >= ENCRYP_TYPE_MAX) {
		ESP_LOGE(TAG, "Unsupported encryption type: 0x%02x", encryp_type);
		return -1;
	}

	return list[encryp_type].decryp(buffer, vaild_size, buff_size);
}
