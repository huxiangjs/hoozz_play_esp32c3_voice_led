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
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "audio.h"
#include "event_bus.h"

#include <algorithm>
#include <cstdint>
#include <iterator>
#include "tensorflow/lite/core/c/common.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

#include "model_config.h"

static const char *TAG = "RECOGNITION";

// Arena size is a guesstimate, followed by use of
// MicroInterpreter::arena_used_bytes() on both the AudioPreprocessor and
// MicroSpeech models and using the larger of the two results.
constexpr size_t kSpeechArenaSize = 28584;   // xtensa p6
alignas(16) uint8_t g_speech_arena[kSpeechArenaSize];
constexpr size_t kFeatureArenaSize = 10240;  // xtensa p6
alignas(16) uint8_t g_feature_arena[kFeatureArenaSize];

using Features = int8_t[tflite_feature_count][tflite_feature_size];
Features g_features;

constexpr int kAudioSampleDurationCount =
	tflite_feature_duration_ms * tflite_model_audio_sample_frequency / 1000;
constexpr int kAudioSampleStrideCount =
	tflite_feature_stride_ms * tflite_model_audio_sample_frequency / 1000;
constexpr int kAudioSampleOverlapCount = kAudioSampleDurationCount - kAudioSampleStrideCount;
constexpr int16_t silence_audio_data[kAudioSampleDurationCount] = {0};

using MicroSpeechOpResolver = tflite::MicroMutableOpResolver<4>;
using AudioPreprocessorOpResolver = tflite::MicroMutableOpResolver<18>;

struct sample_event {
	uint8_t event;
	int16_t data[kAudioSampleDurationCount];
};
static struct sample_event sample_tmp;

#define AUDIO_EVENT_NUM_MAX		10

static QueueHandle_t event_queue;
static SemaphoreHandle_t semaphore;

#define RECOGNITION_THRESHOLD		0.60

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

TfLiteStatus RegisterOps(MicroSpeechOpResolver& op_resolver)
{
	TF_LITE_ENSURE_STATUS(op_resolver.AddReshape());
	TF_LITE_ENSURE_STATUS(op_resolver.AddFullyConnected());
	TF_LITE_ENSURE_STATUS(op_resolver.AddDepthwiseConv2D());
	TF_LITE_ENSURE_STATUS(op_resolver.AddSoftmax());

	return kTfLiteOk;
}

TfLiteStatus RegisterOps(AudioPreprocessorOpResolver& op_resolver)
{
	TF_LITE_ENSURE_STATUS(op_resolver.AddReshape());
	TF_LITE_ENSURE_STATUS(op_resolver.AddCast());
	TF_LITE_ENSURE_STATUS(op_resolver.AddStridedSlice());
	TF_LITE_ENSURE_STATUS(op_resolver.AddConcatenation());
	TF_LITE_ENSURE_STATUS(op_resolver.AddMul());
	TF_LITE_ENSURE_STATUS(op_resolver.AddAdd());
	TF_LITE_ENSURE_STATUS(op_resolver.AddDiv());
	TF_LITE_ENSURE_STATUS(op_resolver.AddMinimum());
	TF_LITE_ENSURE_STATUS(op_resolver.AddMaximum());
	TF_LITE_ENSURE_STATUS(op_resolver.AddWindow());
	TF_LITE_ENSURE_STATUS(op_resolver.AddFftAutoScale());
	TF_LITE_ENSURE_STATUS(op_resolver.AddRfft());
	TF_LITE_ENSURE_STATUS(op_resolver.AddEnergy());
	TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBank());
	TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBankSquareRoot());
	TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBankSpectralSubtraction());
	TF_LITE_ENSURE_STATUS(op_resolver.AddPCAN());
	TF_LITE_ENSURE_STATUS(op_resolver.AddFilterBankLog());

	return kTfLiteOk;
}

TfLiteStatus LoadMicroSpeechModelAndPerformInference(const Features& features)
{
	ESP_LOGI(TAG, "MicroSpeech model size = %u bytes", tflite_speech_model_size);

	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	const tflite::Model* model = tflite::GetModel(tflite_speech_model_data);
	ESP_ERROR_CHECK(model->version() != TFLITE_SCHEMA_VERSION);

	MicroSpeechOpResolver op_resolver;
	ESP_ERROR_CHECK(RegisterOps(op_resolver) != kTfLiteOk);

	tflite::MicroInterpreter interpreter(model, op_resolver, g_speech_arena, kSpeechArenaSize);

	ESP_ERROR_CHECK(interpreter.AllocateTensors() != kTfLiteOk);

	ESP_LOGI(TAG, "MicroSpeech model arena size = %u", interpreter.arena_used_bytes());

	TfLiteTensor* input = interpreter.input(0);
	ESP_ERROR_CHECK(input == nullptr);

	// check input shape is compatible with our feature data size
	ESP_ERROR_CHECK(tflite_feature_element_count != input->dims->data[input->dims->size - 1]);

	TfLiteTensor* output = interpreter.output(0);
	ESP_ERROR_CHECK(output == nullptr);

	// check output shape is compatible with our number of prediction categories
	ESP_LOGI(TAG, "MicroSpeech model output = %u", output->dims->data[output->dims->size - 1]);
	ESP_ERROR_CHECK(tflite_category_count != output->dims->data[output->dims->size - 1]);

	float output_scale = output->params.scale;
	int output_zero_point = output->params.zero_point;

	std::copy_n(&features[0][0], tflite_feature_element_count, tflite::GetTensorData<int8_t>(input));
	ESP_ERROR_CHECK(interpreter.Invoke() != kTfLiteOk);

	// Dequantize output values
	float category_predictions[tflite_category_count];
	MicroPrintf("MicroSpeech category predictions:");
	for (int i = 0; i < tflite_category_count; i++) {
		category_predictions[i] = (tflite::GetTensorData<int8_t>(output)[i] - output_zero_point) * output_scale;
		MicroPrintf("  %.4f %s", static_cast<double>(category_predictions[i]), tflite_category_labels[i]);
	}
	int prediction_index = std::distance(std::begin(category_predictions),
			       std::max_element(std::begin(category_predictions),
			       std::end(category_predictions)));
	MicroPrintf("Highest score: [%s]", tflite_category_labels[prediction_index]);

	if (category_predictions[prediction_index] > RECOGNITION_THRESHOLD) {
		struct event_bus_msg msg = {
			.type = EVENT_BUS_AUDIO_RECOGNITION,
			.param1 = 0,
		};
		if (prediction_index == 2) {
			msg.param1 = 1;
			event_bus_send(&msg);
			ESP_LOGI(TAG, "Case 1");
		} else if (prediction_index == 3) {
			msg.param1 = 2;
			event_bus_send(&msg);
			ESP_LOGI(TAG, "Case 2");
		}
	}
	return kTfLiteOk;
}

TfLiteStatus GenerateSingleFeature(const int16_t* audio_data, const int audio_data_size,
				   int8_t* feature_output, tflite::MicroInterpreter* interpreter)
{
	TfLiteTensor* input = interpreter->input(0);
	ESP_ERROR_CHECK(input == nullptr);

	// check input shape is compatible with our audio sample size
	ESP_ERROR_CHECK(kAudioSampleDurationCount != audio_data_size);
	ESP_ERROR_CHECK(kAudioSampleDurationCount != input->dims->data[input->dims->size - 1]);

	TfLiteTensor* output = interpreter->output(0);
	ESP_ERROR_CHECK(output == nullptr);

	// check output shape is compatible with our feature size
	ESP_ERROR_CHECK(tflite_feature_size != output->dims->data[output->dims->size - 1]);

	std::copy_n(audio_data, audio_data_size, tflite::GetTensorData<int16_t>(input));
	ESP_ERROR_CHECK(interpreter->Invoke() != kTfLiteOk);
	std::copy_n(tflite::GetTensorData<int8_t>(output), tflite_feature_size, feature_output);

	return kTfLiteOk;
}

static void recognition_task(void *args)
{
	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	const tflite::Model* model = tflite::GetModel(tflite_feature_model_data);
	ESP_ERROR_CHECK(model->version() != TFLITE_SCHEMA_VERSION);

	AudioPreprocessorOpResolver op_resolver;
	ESP_ERROR_CHECK(RegisterOps(op_resolver) != kTfLiteOk);

	tflite::MicroInterpreter interpreter(model, op_resolver, g_feature_arena, kFeatureArenaSize);

	ESP_ERROR_CHECK(interpreter.AllocateTensors() != kTfLiteOk);

	ESP_LOGI(TAG, "AudioPreprocessor model arena size = %u", interpreter.arena_used_bytes());

	struct sample_event event = {0, 0};
	size_t feature_index = 0;

	while (1) {
		/* Receive frame events */
		if(xQueueReceive(event_queue, &event, portMAX_DELAY)) {
			/* Start */
			if (event.event & AUDIO_EVENT_VOICE_START) {
				ESP_LOGD(TAG, "RECOGNITION START");
				feature_index = 0;
			}

			/* Generate features */
			if ((feature_index < tflite_feature_count) &&
			    (event.event & AUDIO_EVENT_VOICE_FRAME)) {
				ESP_LOGD(TAG, "RECOGNITION FRAME");
				GenerateSingleFeature(event.data,
						      kAudioSampleDurationCount,
						      g_features[feature_index],
						      &interpreter);
				feature_index++;
			}

			/* Stop */
			if ((event.event & AUDIO_EVENT_VOICE_STOP) &&
			    !(event.event & AUDIO_EVENT_VOICE_DROP) &&
			    !(event.event & AUDIO_EVENT_VOICE_OVER)) {
				ESP_LOGI(TAG, "Voice length: %dms",
					 feature_index * tflite_feature_stride_ms);

				/*
				 * Filling silence features
				 * (We have to do this to satisfy the input of the micro-speech model)
				 */
				while (feature_index < tflite_feature_count) {
					GenerateSingleFeature(silence_audio_data,
							      kAudioSampleDurationCount,
							      g_features[feature_index],
							      &interpreter);
					feature_index++;
				}

				/* Single recognition */
				unsigned int start_time;
				unsigned int end_time;
				start_time = esp_log_timestamp();
				LoadMicroSpeechModelAndPerformInference(g_features);
				end_time = esp_log_timestamp();
				ESP_LOGI(TAG, "Time cost:  +%ums", end_time - start_time);
				ESP_LOGI(TAG, "Free heap size: %dbytes", (int)esp_get_free_heap_size());
			}

			/* Give the end signal */
			if (event.event & AUDIO_EVENT_VOICE_STOP)
				xSemaphoreGive(semaphore);
		}
	}

	vQueueDelete(event_queue);
	vTaskDelete(NULL);
}

static void audio_event_callback(uint8_t event, void *data, uint16_t size)
{
#if defined(DEBUG_DATA_TO_SERIAL)
	uint8_t *p = (uint8_t *)data;
	uint16_t count = 0;
	size_t write_size;

	/* Write to USB Serial */
	while (count < size) {
		write_size = size - count < 1024 ? size - count : 1024;
		usb_uart_write_bytes(p + count, write_size);
		count += write_size;
	}
#endif

	/* Check frame size */
	ESP_ERROR_CHECK(size != kAudioSampleStrideCount * 2);

	/* Copy overlap part */
	for (uint16_t index = 0; index < kAudioSampleOverlapCount; index++)
		sample_tmp.data[index] = sample_tmp.data[kAudioSampleStrideCount + index];

	/* Copy new data */
	int16_t *frame = (int16_t *)data;
	for (uint16_t index = 0; index < kAudioSampleStrideCount; index++)
		sample_tmp.data[kAudioSampleOverlapCount + index] = frame[index];

	/* Send event */
	sample_tmp.event = event;
	if (xQueueSend(event_queue, (void *)&sample_tmp, (TickType_t)0) != pdTRUE)
		ESP_LOGE(TAG, "Send sample event failed");

	/* Waiting for recognition to end */
	if (event & AUDIO_EVENT_VOICE_STOP)
		xSemaphoreTake(semaphore, portMAX_DELAY);
}

void recognition_init(void)
{
	int ret;
	struct audio_handler handler = {
		.frame_time = (uint16_t)tflite_feature_stride_ms,
		.max_time = (uint16_t)((tflite_feature_count + 1) * tflite_feature_stride_ms),
		.event = audio_event_callback,
	};

	semaphore = xSemaphoreCreateBinary();
	ESP_ERROR_CHECK(semaphore == NULL);

	/* Create event queue */
	event_queue = xQueueCreate(AUDIO_EVENT_NUM_MAX, sizeof(struct sample_event));
	ESP_ERROR_CHECK(event_queue == NULL);

	/* Create recognition task */
	ret = xTaskCreate(recognition_task, "recognition_task", 4096 * 2, NULL,
			  tskIDLE_PRIORITY + 1, NULL);
	ESP_ERROR_CHECK(ret != pdPASS);

#if defined(DEBUG_DATA_TO_SERIAL)
	usb_uart_config();
#endif
	audio_init(&handler);
}
