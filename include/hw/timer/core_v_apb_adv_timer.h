/*
 * CORE-V MCU Advanced Timer
 *
 * Copyright (c) 2021-2022 PLCT Lab.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_CORE_V_APB_ABV_TIMER_H
#define HW_CORE_V_APB_ABV_TIMER_H

#include "hw/qdev-properties.h"
#include "hw/sysbus.h"
#include "hw/ptimer.h"
#include "hw/clock.h"
#include "qom/object.h"

#define TYPE_CORE_V_APB_ADV_TIMER "core-v.mcu.apb-adv-timer"
OBJECT_DECLARE_SIMPLE_TYPE(CoreVApbAdvTimerState, CORE_V_APB_ADV_TIMER)

#define CORE_V_APB_ADV_TIMER_NR_TIMERS   4
#define CORE_V_APB_ADV_TIMER_NR_CHANNELS 4
#define CORE_V_APB_ADV_TIMER_NR_EVENTS 4

typedef struct CoreVApbAdvTimer {
    uint8_t id;
    int8_t direction;
    uint16_t counter;
    struct ptimer_state *timer;

    uint32_t cmd;
    uint32_t config;
    uint16_t th_hi;
    uint16_t th_lo;
    bool is_2nd[CORE_V_APB_ADV_TIMER_NR_CHANNELS];
    uint32_t channels[CORE_V_APB_ADV_TIMER_NR_CHANNELS];
    uint32_t channels_lut[CORE_V_APB_ADV_TIMER_NR_CHANNELS];
    bool events[CORE_V_APB_ADV_TIMER_NR_CHANNELS];

} CoreVApbAdvTimer;

struct CoreVApbAdvTimerState {
    /* <private> */
    SysBusDevice parent_obj;

    /* <publib> */
    MemoryRegion mmio;

    uint32_t event_cfg;
    uint32_t cg;
    CoreVApbAdvTimer timers[CORE_V_APB_ADV_TIMER_NR_TIMERS];
    qemu_irq irq[CORE_V_APB_ADV_TIMER_NR_EVENTS];
};

enum ch_mode {
    OP_SET,
    OP_TOGRST,
    OP_SETRST,
    OP_TOG,
    OP_RST,
    OP_TOGSET,
    OP_RSTSET,
};
#endif /* HW_CORE_V_APB_ABV_TIMER_H */
