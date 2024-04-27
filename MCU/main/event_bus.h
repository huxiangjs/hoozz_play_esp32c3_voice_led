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

#ifndef __EVENT_BUS_H_
#define __EVENT_BUS_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define EVENT_BUS_STARTUP		0x00		// Startup
#define EVENT_BUS_START_SMART_CONFIG	0x01		// Smart config start
#define EVENT_BUS_STOP_SMART_CONFIG	0x02		// Smart config stop
#define EVENT_BUS_AUDIO_RECOGNITION	0x03		// Audio recognition result

struct event_bus_msg {
	uint8_t type;
	uint32_t param1;
};

typedef bool (*event_notify_callback)(struct event_bus_msg *msg);

void event_bus_send(struct event_bus_msg *msg);
void event_bus_register(event_notify_callback callback);
void event_bus_init(void);

#ifdef __cplusplus
}
#endif

#endif /* __EVENT_BUS_H_ */
