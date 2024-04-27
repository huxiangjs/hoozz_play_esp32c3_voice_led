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

#ifndef __LED_H_
#define __LED_H_

#include <stdint.h>

#define LED_STATUS_NONE			0	// Stateless, all LEDs are off
#define LED_STATUS_WIFI_CONNECT		1	// Wi-Fi is connected
#define LED_STATUS_WIFI_DISCONNECT	2	// Wi-Fi is disconnected
#define LED_STATUS_SMARTCONFIG		3	// Smart config
#define LED_STATUS_UPGRADE		4	// Firmware update
#define LED_STATUS_COLOR_PICK		5	// Pick color

#ifdef __cplusplus
extern "C" {
#endif

void led_init(void);
void led_status_set(uint8_t new_status);
void led_set_rgb(uint32_t rgb);
uint32_t led_get_rgb(void);

#ifdef __cplusplus
}
#endif

#endif /* __LED_H_ */
