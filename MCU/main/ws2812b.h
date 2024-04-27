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

#ifndef __WS2812B_H_
#define __WS2812B_H_

#define WS2812B_LED_NUMBERS         4

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void ws2812b_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r,
		     uint32_t *g, uint32_t *b);
void ws2812b_off(void);
void ws2812b_set_pixel(uint8_t num, uint32_t rgb);
void ws2812b_copy_to_buffer(uint32_t rgb_buf[]);
void ws2812b_clear(uint32_t rgb);
uint32_t ws2812b_get_pixel(uint8_t num);
void ws2812b_refresh(void);
void ws2812b_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __WS2812B_H_ */
