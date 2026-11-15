# Build-time version info (the git branch + short sha + "-dirty" of this
# repo at build time), generated into build/include/git_version.h by
# esp32-common/git_version.cmake, which rewrites the file only when the
# git state changes (so incremental builds pick up version changes, like
# the ESP8266 project's Makefile rule).
#
# The block is skipped during CMake's early expansion of component
# requirements (script mode, CMAKE_BUILD_EARLY_EXPANSION): there
# add_custom_command/add_custom_target are not allowed.
if(NOT DEFINED CMAKE_BUILD_EARLY_EXPANSION)
set(app_git_version_h ${CMAKE_BINARY_DIR}/include/git_version.h)
add_custom_command(
	OUTPUT ${app_git_version_h}
	COMMAND ${CMAKE_COMMAND}
		-DGIT_REPO=${CMAKE_CURRENT_SOURCE_DIR}
		-DOUT_FILE=${app_git_version_h}
		-P ${CMAKE_CURRENT_SOURCE_DIR}/esp32-common/git_version.cmake
	DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/esp32-common/git_version.cmake
	COMMENT "Checking app git version info"
)
# The header is generated at build time; create the directory up front
# because idf_component_register() validates INCLUDE_DIRS at configure time
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/include)
endif()

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
		"esp32-common/ota_flash.c"
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
		${CMAKE_BINARY_DIR}/include
		${tcp_crypto_aes_inc}
)

if(NOT DEFINED CMAKE_BUILD_EARLY_EXPANSION)
# main must be built after the version header is generated
add_custom_target(app_git_version DEPENDS ${app_git_version_h})
add_dependencies(__idf_main app_git_version)
endif()

add_definitions(
	-fno-exceptions
	-Wno-deprecated-declarations
	-DTF_LITE_STATIC_MEMORY
	-fno-unwind-tables
	-ffunction-sections
	-fdata-sections
	-fmessage-length=0
)
