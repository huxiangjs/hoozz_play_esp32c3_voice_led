# The C3 hardware AES backend in esp32-common/tcp_crypto.c uses the
# IDF's mbedtls port (aes/esp_aes.h) and its hal types (hal/aes_types.h)
set(tcp_crypto_aes_inc "")
if(CONFIG_IDF_TARGET_ESP32C3)
	string(REPLACE "\\" "/" tcp_crypto_idf_path "$ENV{IDF_PATH}")
	set(tcp_crypto_aes_inc
		"${tcp_crypto_idf_path}/components/mbedtls/port/include"
		"${tcp_crypto_idf_path}/components/hal/include")
endif()

idf_component_register(
	SRCS
		"../components/tflm/project/examples/micro_speech/models/audio_preprocessor_int8_model_data.cc"
		"../components/tflm/project/examples/micro_speech/models/micro_speech_quantized_model_data.cc"
		"esp32-common/led_strip_encoder.c"
		"esp32-common/ws2812b.c"
		"esp32-common/wifi.c"
		"esp32-common/event_bus.c"
		"esp32-common/simple_ctrl.c"
		"esp32-common/crypto.c"
		"esp32-common/store.c"
		"esp32-common/ota.c"
		"esp32-common/tcp_crypto.c"
		"esp32-common/tiny-AES-c/aes.c"
		"main.cpp"
		"audio.c"
		"recognition.cpp"
		"sample_send.c"
		"led_effects.c"

        INCLUDE_DIRS
		"."
		"esp32-common"
		"../components/tflm/project"
		"../components/tflm/project/third_party/gemmlowp"
		"../components/tflm/project/third_party/flatbuffers/include"
		"../components/tflm/project/third_party/kissfft"
		"../components/tflm/project/third_party/kissfft/tools"
		"../components/tflm/project/third_party/ruy"
		"../components/tflm/project/examples/micro_speech"
		${tcp_crypto_aes_inc}
)

add_definitions(
	-fno-exceptions
	-Wno-deprecated-declarations
	-DTF_LITE_STATIC_MEMORY
	-fno-unwind-tables
	-ffunction-sections
	-fdata-sections
	-fmessage-length=0
)
