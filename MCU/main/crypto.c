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
#include <string.h>
#include "crypto.h"

static const char *TAG = "CRYPTO";

struct algorithm {
	int (*en)(struct crypto *handle, char *buffer, int vaild_size, int buff_size);
	int (*de)(struct crypto *handle, char *buffer, int vaild_size, int buff_size);
};

static int en_none(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	/* Do nothing */

	return vaild_size;
}

static int de_none(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	/* Do nothing */

	return vaild_size;
}

static int do_xor(struct crypto *handle, char *buffer, int vaild_size)
{
	int index;

	if (handle->plen == 0)
		goto out;

	for (index = 0; index < vaild_size; index++) {
		buffer[index] ^= handle->passwd[handle->count % handle->plen];
		handle->count++;
	}

out:
	return vaild_size;
}

static int en_xor(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_xor(handle, buffer, vaild_size);
}

static int de_xor(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return do_xor(handle, buffer, vaild_size);
}

static struct algorithm list[CRYPTO_TYPE_MAX] = {
	[CRYPTO_TYPE_NONE] = { en_none, de_none },	/* 0x00: No encryption */
	[CRYPTO_TYPE_XOR] = { en_xor, de_xor },		/* 0x01: Simple xor replacement */
};

int crypto_en(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return list[handle->type].en(handle, buffer, vaild_size, buff_size);
}

int crypto_de(struct crypto *handle, char *buffer, int vaild_size, int buff_size)
{
	return list[handle->type].de(handle, buffer, vaild_size, buff_size);
}

int crypto_init(struct crypto *handle, int type, char *passwd, int plen)
{
	if (!handle) {
		ESP_LOGE(TAG, "A null pointer is used");
		return -1;
	}

	if (type >= CRYPTO_TYPE_MAX) {
		ESP_LOGE(TAG, "Unsupported encryption type: 0x%02x", type);
		return -1;
	}

	handle->count = 0;
	handle->type = type;

	if (passwd && plen) {
		handle->passwd = passwd;
		handle->plen = plen;
	} else {
		handle->passwd = NULL;
		handle->plen = 0;
	}

	return 0;
}
