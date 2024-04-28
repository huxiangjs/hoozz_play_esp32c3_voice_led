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
#include "micro_model_settings.h"
#include "models/audio_preprocessor_int8_model_data.h"
#include "models/micro_speech_quantized_model_data.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"

static const char *TAG = "RECOGNITION";

// Arena size is a guesstimate, followed by use of
// MicroInterpreter::arena_used_bytes() on both the AudioPreprocessor and
// MicroSpeech models and using the larger of the two results.
constexpr size_t kArenaSize = 28584;  // xtensa p6
alignas(16) uint8_t g_arena[kArenaSize];

using Features = int8_t[kFeatureCount][kFeatureSize];
Features g_features;

constexpr int kAudioSampleDurationCount = kFeatureDurationMs * kAudioSampleFrequency / 1000;
constexpr int kAudioSampleStrideCount = kFeatureStrideMs * kAudioSampleFrequency / 1000;

using MicroSpeechOpResolver = tflite::MicroMutableOpResolver<4>;
using AudioPreprocessorOpResolver = tflite::MicroMutableOpResolver<18>;

#define RECOGNITION_THRESHOLD		0.75

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
	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	const tflite::Model* model = tflite::GetModel(g_micro_speech_quantized_model_data);
	ESP_ERROR_CHECK(model->version() != TFLITE_SCHEMA_VERSION);

	MicroSpeechOpResolver op_resolver;
	ESP_ERROR_CHECK(RegisterOps(op_resolver) != kTfLiteOk);

	tflite::MicroInterpreter interpreter(model, op_resolver, g_arena, kArenaSize);

	ESP_ERROR_CHECK(interpreter.AllocateTensors() != kTfLiteOk);

	MicroPrintf("MicroSpeech model arena size = %u", interpreter.arena_used_bytes());

	TfLiteTensor* input = interpreter.input(0);
	ESP_ERROR_CHECK(input == nullptr);

	// check input shape is compatible with our feature data size
	ESP_ERROR_CHECK(kFeatureElementCount != input->dims->data[input->dims->size - 1]);

	TfLiteTensor* output = interpreter.output(0);
	ESP_ERROR_CHECK(output == nullptr);

	// check output shape is compatible with our number of prediction categories
	MicroPrintf("MicroSpeech model output = %u", output->dims->data[output->dims->size - 1]);
	ESP_ERROR_CHECK(kCategoryCount != output->dims->data[output->dims->size - 1]);

	float output_scale = output->params.scale;
	int output_zero_point = output->params.zero_point;

	std::copy_n(&features[0][0], kFeatureElementCount, tflite::GetTensorData<int8_t>(input));
	ESP_ERROR_CHECK(interpreter.Invoke() != kTfLiteOk);

	// Dequantize output values
	float category_predictions[kCategoryCount];
	MicroPrintf("MicroSpeech category predictions:");
	for (int i = 0; i < kCategoryCount; i++) {
		category_predictions[i] = (tflite::GetTensorData<int8_t>(output)[i] - output_zero_point) * output_scale;
		MicroPrintf("  %.4f %s", static_cast<double>(category_predictions[i]), kCategoryLabels[i]);
	}
	int prediction_index = std::distance(std::begin(category_predictions),
			       std::max_element(std::begin(category_predictions),
			       std::end(category_predictions)));
	MicroPrintf("RESULT: %s", kCategoryLabels[prediction_index]);

	if (category_predictions[prediction_index] > RECOGNITION_THRESHOLD) {
		struct event_bus_msg msg = {
			.type = EVENT_BUS_AUDIO_RECOGNITION,
			.param1 = 0,
		};
		if (prediction_index == 2) {
			msg.param1 = 1;
			event_bus_send(&msg);
			MicroPrintf("Case 1");
		} else if (prediction_index == 3) {
			msg.param1 = 2;
			event_bus_send(&msg);
			MicroPrintf("Case 2");
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
	ESP_ERROR_CHECK(kFeatureSize != output->dims->data[output->dims->size - 1]);

	std::copy_n(audio_data, audio_data_size, tflite::GetTensorData<int16_t>(input));
	ESP_ERROR_CHECK(interpreter->Invoke() != kTfLiteOk);
	std::copy_n(tflite::GetTensorData<int8_t>(output), kFeatureSize, feature_output);

	return kTfLiteOk;
}

TfLiteStatus GenerateFeatures(const int16_t* audio_data, const size_t audio_data_size,
			      Features* features_output)
{
	// Map the model into a usable data structure. This doesn't involve any
	// copying or parsing, it's a very lightweight operation.
	const tflite::Model* model = tflite::GetModel(g_audio_preprocessor_int8_model_data);
	ESP_ERROR_CHECK(model->version() != TFLITE_SCHEMA_VERSION);

	AudioPreprocessorOpResolver op_resolver;
	ESP_ERROR_CHECK(RegisterOps(op_resolver) != kTfLiteOk);

	tflite::MicroInterpreter interpreter(model, op_resolver, g_arena, kArenaSize);

	ESP_ERROR_CHECK(interpreter.AllocateTensors() != kTfLiteOk);

	MicroPrintf("AudioPreprocessor model arena size = %u", interpreter.arena_used_bytes());

	size_t remaining_samples = audio_data_size;
	size_t feature_index = 0;
	while (remaining_samples >= kAudioSampleDurationCount &&
	       feature_index < kFeatureCount) {
		TF_LITE_ENSURE_STATUS(GenerateSingleFeature(audio_data,
							    kAudioSampleDurationCount,
							    (*features_output)[feature_index],
							    &interpreter));
		feature_index++;
		audio_data += kAudioSampleStrideCount;
		remaining_samples -= kAudioSampleStrideCount;
	}

	return kTfLiteOk;
}

TfLiteStatus AudioRecognition(const int16_t* audio_data, const size_t audio_data_size)
{
	unsigned int start_time;
	unsigned int end_time;

	start_time = esp_log_timestamp();
	TF_LITE_ENSURE_STATUS(GenerateFeatures(audio_data, audio_data_size, &g_features));
	end_time = esp_log_timestamp();
	ESP_LOGI(TAG, "%s:%d:  +%ums", __func__, __LINE__, end_time - start_time);

	start_time = esp_log_timestamp();
	TF_LITE_ENSURE_STATUS(LoadMicroSpeechModelAndPerformInference(g_features));
	end_time = esp_log_timestamp();
	ESP_LOGI(TAG, "%s:%d:  +%ums", __func__, __LINE__, end_time - start_time);

	return kTfLiteOk;
}

static void audio_event_callback(uint8_t event, void *data, uint32_t size)
{
#if defined(DEBUG_DATA_TO_SERIAL)
	uint8_t *p = (uint8_t *)data;
	uint32_t count = 0;
	size_t write_size;
#endif

	if (event == AUDIO_EVENT_VOICE_STOP) {
		ESP_LOGI(TAG, "Output: %ums", audio_size_to_time(size));

#if defined(DEBUG_DATA_TO_SERIAL)
		/* Write to USB Serial */
		while (count < size) {
			write_size = size - count < 1024 ? size - count : 1024;
			usb_uart_write_bytes(p + count, write_size);
			count += write_size;
		}
#endif
		AudioRecognition((int16_t *)data, size / sizeof(int16_t));
	}
}

void recognition_init(void)
{
	struct audio_handler handler = {audio_event_callback};

#if defined(DEBUG_DATA_TO_SERIAL)
	usb_uart_config();
#endif
	audio_init(&handler);
}
