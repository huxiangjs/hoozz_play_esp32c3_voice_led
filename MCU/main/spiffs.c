/*
 * MIT License
 *
 * Copyright (c) 2023 Hoozz <huxiangjs@foxmail.com>
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
 *
 * refs: esp-idf/examples/storage/spiffs/main/spiffs_example_main.c
 *
 */

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

static const char *TAG = "SPIFFS";

void spiffs_init(void)
{
	ESP_LOGI(TAG, "Initializing SPIFFS");

	esp_vfs_spiffs_conf_t conf = {
	  .base_path = "/spiffs",
	  .partition_label = NULL,
	  .max_files = 5,
	  .format_if_mount_failed = true
	};

	// Use settings defined above to initialize and mount SPIFFS filesystem.
	// Note: esp_vfs_spiffs_register is an all-in-one convenience function.
	esp_err_t ret = esp_vfs_spiffs_register(&conf);

	if (ret != ESP_OK) {
		if (ret == ESP_FAIL) {
			ESP_LOGE(TAG, "Failed to mount or format filesystem");
		} else if (ret == ESP_ERR_NOT_FOUND) {
			ESP_LOGE(TAG, "Failed to find SPIFFS partition");
		} else {
			ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
		}
		return;
	}

	size_t total = 0, used = 0;
	ret = esp_spiffs_info(conf.partition_label, &total, &used);
	if (ret != ESP_OK) {
		ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s). Formatting...", esp_err_to_name(ret));
		esp_spiffs_format(conf.partition_label);
		return;
	} else {
		ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
	}

	// Check consistency of reported partiton size info.
	if (used > total) {
		ESP_LOGW(TAG, "Number of used bytes cannot be larger than total. Performing SPIFFS_check().");
		ret = esp_spiffs_check(conf.partition_label);
		// Could be also used to mend broken files, to clean unreferenced pages, etc.
		// More info at https://github.com/pellepl/spiffs/wiki/FAQ#powerlosses-contd-when-should-i-run-spiffs_check
		if (ret != ESP_OK) {
			ESP_LOGE(TAG, "SPIFFS_check() failed (%s)", esp_err_to_name(ret));
			return;
		} else {
			ESP_LOGI(TAG, "SPIFFS_check() successful");
		}
	}

	// All done, unmount partition and disable SPIFFS
	// esp_vfs_spiffs_unregister(conf.partition_label);
	// ESP_LOGI(TAG, "SPIFFS unmounted");
}

int spiffs_save(char *name, char *data, int size)
{
	char path[32];
	FILE* f;
	int ret;

	sprintf(path, "/spiffs/%s", name);
	ESP_LOGI(TAG, "Open: %s", path);

	f = fopen(path, "wb");
	if (f == NULL) {
		ESP_LOGE(TAG, "Failed to open file for writing");
		return -1;
	}

	ret = fwrite(data, 1, size, f);
	if (ret != size)
		ESP_LOGE(TAG, "File write failed, ret:%d", ret);
	else
		ESP_LOGI(TAG, "File written");

	fclose(f);

	return 0;
}

int spiffs_load(char *name, char *buff, int size)
{
	char path[32];
	FILE* f;
	int ret;

	sprintf(path, "/spiffs/%s", name);
	ESP_LOGI(TAG, "Open %s", path);

	f = fopen(path, "rb");
	if (f == NULL) {
		ESP_LOGI(TAG, "Failed to open file for reading");
		return -1;
	}

	ret = fread(buff, 1, size, f);
	fclose(f);

	if (ret <= 0) {
		ESP_LOGE(TAG, "Reading error or file is empty");
		return -1;
	} else {
		ESP_LOGI(TAG, "File read size: %d bytes", ret);
	}

	return 0;
}
