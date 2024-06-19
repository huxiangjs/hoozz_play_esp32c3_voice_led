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

#ifndef __MODEL_CONFIG_H_
#define __MODEL_CONFIG_H_

/* Test only */
#define FUNCTION_TEST

#if defined(FUNCTION_TEST)

#include "micro_model_settings.h"
#include "models/micro_speech_quantized_model_data.h"

#define tflite_model_data g_micro_speech_quantized_model_data
#define tflite_model_size g_micro_speech_quantized_model_data_size
#define tflite_model_audio_sample_frequency kAudioSampleFrequency
#define tflite_category_count kCategoryCount
#define tflite_category_labels kCategoryLabels
#define tflite_feature_size kFeatureSize
#define tflite_feature_count kFeatureCount
#define tflite_feature_element_count kFeatureElementCount
#define tflite_feature_stride_ms kFeatureStrideMs
#define tflite_feature_duration_ms kFeatureDurationMs

#include "models/audio_preprocessor_int8_model_data.h"

#define tflite_feature_model_data g_audio_preprocessor_int8_model_data
#define tflite_feature_model_size g_audio_preprocessor_int8_model_data_size

#else

#include "models/model.cc"

#endif /* FUNCTION_TEST */

#endif /* __MODEL_CONFIG_H_ */
