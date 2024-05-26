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

#ifndef __LED_EFFECTS_H_
#define __LED_EFFECTS_H_

#include <stdint.h>

#define LED_EFFECTS_NONE		0x00	// Idle
#define LED_EFFECTS_ALL_ON		0x01	// All LEDs are on
#define LED_EFFECTS_ALL_OFF		0x02	// All LEDs are off
#define LED_EFFECTS_WIFI_CONNECT	0x03	// Wi-Fi is connected
#define LED_EFFECTS_WIFI_DISCONNECT	0x04	// Wi-Fi is disconnected
#define LED_EFFECTS_SMARTCONFIG		0x05	// Smart config
#define LED_EFFECTS_UPGRADE		0x06	// Firmware update
#define LED_EFFECTS_COLOR_PICK		0x07	// Pick color
#define LED_EFFECTS_COLOR_FILL		0x08	// Fill color
#define LED_EFFECTS_COLOR_GRADIENT	0x09	// Gradient color

#ifdef __cplusplus
extern "C" {
#endif

void led_effects_init(void);
void led_effects_play(uint8_t id, uint32_t param);

#ifdef __cplusplus
}
#endif

#endif /* __LED_EFFECTS_H_ */
