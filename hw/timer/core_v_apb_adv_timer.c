/*
 * CORE-V MCU Advanced Timer
 *
 * Copyright (c) 2022 PLCT Lab.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qapi/error.h"
#include "trace.h"
#include "hw/sysbus.h"
#include "hw/irq.h"
#include "hw/registerfields.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/timer/core_v_apb_adv_timer.h"
#include "migration/vmstate.h"

#define DEBUG_ADT_WRITE(X)

#define TIMER_REGS_SIZE 0x40
#define REF_CLK_FREQ 32000 /* 32K */
#define CLK_SYSTEM 50000000 /* 50M */

REG32(TIM_CMD,               0x00)
    FIELD(TIM_CMD,           START,      0,  1)
    FIELD(TIM_CMD,           STOP,       1,  1)
    FIELD(TIM_CMD,           UPDATE,     2,  1)
    FIELD(TIM_CMD,           RESET,      3,  1)
    FIELD(TIM_CMD,           ARM,        4,  1)
REG32(TIM_CONFIG,        0x04)
    FIELD(TIM_CONFIG,        INSEL,      0,  8)
    FIELD(TIM_CONFIG,        MODE,       8,  3)
    FIELD(TIM_CONFIG,        CLKSEL,     11, 1)
    FIELD(TIM_CONFIG,        UPDOWNSEL,  12, 1)
    FIELD(TIM_CONFIG,        PRESC,      16, 8)
REG32(TIM_THRESHOLD,     0x08)
    FIELD(TIM_THRESHOLD,     TH_LO,      0,  16)
    FIELD(TIM_THRESHOLD,     TH_HI,      16, 16)
REG32(TIM_TH_CHANNEL0,   0x0C)
    FIELD(TIM_TH_CHANNEL0,   TH,         0,  16)
    FIELD(TIM_TH_CHANNEL0,   MODE,       16, 3)
REG32(TIM_TH_CHANNEL1,   0x10)
    FIELD(TIM_TH_CHANNEL1,   TH,         0,  16)
    FIELD(TIM_TH_CHANNEL1,   MODE,       16, 3)
REG32(TIM_TH_CHANNEL2,   0x14)
    FIELD(TIM_TH_CHANNEL2,   TH,         0,  16)
    FIELD(TIM_TH_CHANNEL2,   MODE,       16, 3)
REG32(TIM_TH_CHANNEL3,   0x18)
    FIELD(TIM_TH_CHANNEL3,   TH,         0,  16)
    FIELD(TIM_TH_CHANNEL3,   MODE,       16, 3)
REG32(TIM_LUT_CHANNEL0,  0x1C)
    FIELD(TIM_LUT_CHANNEL0,  LUT,        0,  16)
    FIELD(TIM_LUT_CHANNEL0,  FLT,        16, 2)
REG32(TIM_LUT_CHANNEL1,  0x20)
    FIELD(TIM_LUT_CHANNEL1,  LUT,        0,  16)
    FIELD(TIM_LUT_CHANNEL1,  FLT,        16, 2)
REG32(TIM_LUT_CHANNEL2,  0x24)
    FIELD(TIM_LUT_CHANNEL2,  LUT,        0,  16)
    FIELD(TIM_LUT_CHANNEL2,  FLT,        16, 2)
REG32(TIM_LUT_CHANNEL3,  0x28)
    FIELD(TIM_LUT_CHANNEL3,  LUT,        0,  16)
    FIELD(TIM_LUT_CHANNEL3,  FLT,        16, 2)
REG32(TIM_COUNTER,       0x2C)

REG32(EVENT_CFG,        0x100)
    FIELD(EVENT_CFG,        SEL0,       0,  4)
    FIELD(EVENT_CFG,        SEL1,       4,  4)
    FIELD(EVENT_CFG,        SEL2,       8,  4)
    FIELD(EVENT_CFG,        SEL3,       12, 4)
    FIELD(EVENT_CFG,        ENA,        16, 4)
REG32(CG,               0x104)
    FIELD(CG,               ENA,        0,  16)

static inline CoreVApbAdvTimerState *timer_to_state(CoreVApbAdvTimer *t)
{
    const CoreVApbAdvTimer (*timers)[] = (void *)t - (t->id * sizeof(*t));
    return container_of(timers, CoreVApbAdvTimerState, timers);
}

static inline bool timer_enabled(CoreVApbAdvTimer *t)
{
    return !!(timer_to_state(t)->cg & BIT(t->id));
}

static void trigger_counter_update(CoreVApbAdvTimer *t)
{
    bool end = false;
    bool old_event[4];
    int i;

    t->counter += t->direction;
    if (((t->direction > 0) && (t->counter == t->th_hi)) ||
        ((t->direction < 0) && (t->counter == (uint16_t)(t->th_lo - 1)))) {
        end = true;
        if (!(t->config & R_TIM_CONFIG_UPDOWNSEL_MASK)) {
            t->direction = -t->direction;
        }
    }

    for (i = 0; i < CORE_V_APB_ADV_TIMER_NR_CHANNELS; i++) {
        uint16_t cmp = t->channels[i] & 0xFFFF;
        uint8_t cmp_op = t->channels[i] >> 16;
        bool match = (cmp == t->counter);
        bool second = t->config & R_TIM_CONFIG_UPDOWNSEL_MASK ? end : match;

        old_event[i] = t->events[i];
        switch (cmp_op) {
        case OP_SET:
            t->events[i] = match ? 1 : t->events[i];
            break;
        case OP_TOGRST:
            if (t->config & R_TIM_CONFIG_UPDOWNSEL_MASK) {
                t->events[i] = match ? !t->events[i] :
                                       second ? false : t->events[i];
            } else {
                if (match && !t->is_2nd[i]) {
                    t->events[i] = !t->events[i];
                    t->is_2nd[i] = true;
                } else if (match && t->is_2nd[i]) {
                    t->events[i] = false;
                    t->is_2nd[i] = false;
                }
            }
            break;
        case OP_SETRST:
            if (t->config & R_TIM_CONFIG_UPDOWNSEL_MASK) {
                t->events[i] = match ? true : second ? false : t->events[i];
            } else {
                if (match && !t->is_2nd[i]) {
                    t->events[i] = true;
                    t->is_2nd[i] = true;
                } else if (match && t->is_2nd[i]) {
                    t->events[i] = false;
                    t->is_2nd[i] = false;
                }
            }
            break;
        case OP_TOG:
            t->events[i] = match ? !t->events[i] : t->events[i];
            break;
        case OP_RST:
            t->events[i] = match ? false : t->events[i];
            break;
        case OP_TOGSET:
            if (t->config & R_TIM_CONFIG_UPDOWNSEL_MASK) {
                t->events[i] = match ? !t->events[i] :
                                       second ? true : t->events[i];
            } else {
                if (match && !t->is_2nd[i]) {
                    t->events[i] = !t->events[i];
                    t->is_2nd[i] = true;
                } else if (match && t->is_2nd[i]) {
                    t->events[i] = true;
                    t->is_2nd[i] = false;
                }
            }
            break;
        case OP_RSTSET:
            if (t->config & R_TIM_CONFIG_UPDOWNSEL_MASK) {
                t->events[i] = match ? false : second ? true : t->events[i];
            } else {
                if (match && !t->is_2nd[i]) {
                    t->events[i] = false;
                    t->is_2nd[i] = true;
                } else if (match && t->is_2nd[i]) {
                    t->events[i] = true;
                    t->is_2nd[i] = false;
                }
            }
            break;
        default:
            t->is_2nd[i] = false;
            break;
        }
    }

    for (i = 0; i < CORE_V_APB_ADV_TIMER_NR_EVENTS; i++) {
        CoreVApbAdvTimerState *s = timer_to_state(t);
        uint8_t select =  (s->event_cfg >> (i << 2)) & 0xF;
        uint8_t id = select >> 2;
        uint8_t ch_id = select & 0x3;
        if ((t->id == id) && ((s->event_cfg >> (16 + i)) & 0x1)) {
            if (!old_event[ch_id] && t->events[ch_id]) {
                qemu_irq_raise(s->irq[i]);
            } else if (old_event[ch_id] && !t->events[ch_id]) {
                qemu_irq_lower(s->irq[i]);
            }
        }
    }

    if ((t->config & R_TIM_CONFIG_UPDOWNSEL_MASK) && end) {
        t->counter = t->th_lo;
    }
}

static bool get_ext_input(CoreVApbAdvTimer *t)
{
    uint8_t select = FIELD_EX32(t->config, TIM_CONFIG, INSEL);

    if (select < 32) {
        /* There is no external input currently */
        return 0;
    } else {
        uint8_t timer_id = (select - 32) >> 2;
        return timer_to_state(t)->timers[timer_id].events[select & 0x3];
    }
}

static void core_v_apb_adv_timer_tick(void *opaque)
{
    CoreVApbAdvTimer *t = opaque;

    if (!timer_enabled(t)) {
        ptimer_stop(t->timer);
        return;
    }

    ptimer_set_limit(t->timer, 1, 1);

    uint8_t mode = FIELD_EX32(t->config, TIM_CONFIG, MODE);

    bool count_valid = false;
    bool int_sig = get_ext_input(t);
    switch (mode) {
    case 0:
        count_valid = true;
        break;
    case 1:
        count_valid = !int_sig;
        break;
    case 2:
       count_valid = int_sig;
       break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "Don't support other input signal mode currently\n");
    }
    if (count_valid) {
        trigger_counter_update(t);
    }
}

static void core_v_apb_adv_timer_reset(DeviceState *dev)
{
    CoreVApbAdvTimerState *s = CORE_V_APB_ADV_TIMER(dev);

    int i, j;
    for (i = 0; i < CORE_V_APB_ADV_TIMER_NR_TIMERS; i++) {
        CoreVApbAdvTimer *t = &s->timers[i];

        t->direction = 1;
        t->cmd = 0;
        t->config = 0;
        t->th_hi = 0;
        t->th_lo = 0;

        for (j = 0; j < CORE_V_APB_ADV_TIMER_NR_CHANNELS; j++) {
            t->channels[j] = 0;
            t->channels_lut[j] = 0;
            t->events[j] = false;
            t->is_2nd[j] = false;
        }

        ptimer_transaction_begin(t->timer);
        ptimer_stop(t->timer);
        ptimer_transaction_commit(t->timer);
    }
    s->event_cfg = 0;
    s->cg = 0;
}

static uint64_t core_v_apb_adv_timer_read(void *opaque, hwaddr addr,
                                          unsigned int size)
{
    CoreVApbAdvTimerState *s = CORE_V_APB_ADV_TIMER(opaque);

    switch (addr) {
    case A_EVENT_CFG:
        return s->event_cfg;
    case A_CG:
        return s->cg;
    }

    uint8_t id = addr / TIMER_REGS_SIZE;
    hwaddr offset = addr % TIMER_REGS_SIZE;

    if (id >= CORE_V_APB_ADV_TIMER_NR_TIMERS) {
        return 0;
    }

    CoreVApbAdvTimer *t = &s->timers[id];

    switch (offset) {
    case A_TIM_CONFIG:
        return t->config;
    case A_TIM_THRESHOLD:
        return ((uint32_t)t->th_hi << 16) | t->th_lo;
    case A_TIM_TH_CHANNEL0:
        return t->channels[0];
    case A_TIM_TH_CHANNEL1:
        return t->channels[1];
    case A_TIM_TH_CHANNEL2:
        return t->channels[2];
    case A_TIM_TH_CHANNEL3:
        return t->channels[3];
    case A_TIM_LUT_CHANNEL0:
        return t->channels_lut[0];
    case A_TIM_LUT_CHANNEL1:
        return t->channels_lut[1];
    case A_TIM_LUT_CHANNEL2:
        return t->channels_lut[2];
    case A_TIM_LUT_CHANNEL3:
        return t->channels_lut[3];
    case A_TIM_COUNTER:
        return t->counter;
    }

    return 0;
}

static void core_v_apb_adv_timer_write(void *opaque, hwaddr addr,
                                       uint64_t value, unsigned int size)
{
    CoreVApbAdvTimerState *s = CORE_V_APB_ADV_TIMER(opaque);

    if (addr >= A_EVENT_CFG) {
        switch (addr) {
        case A_EVENT_CFG:
            s->event_cfg = value & 0xFFFFF;
            break;
        case A_CG:
            s->cg = value & ((1 << CORE_V_APB_ADV_TIMER_NR_TIMERS) - 1);
            break;
        }

        return;
    }

    uint8_t id = addr / TIMER_REGS_SIZE;
    hwaddr offset = addr % TIMER_REGS_SIZE;

    DEBUG_ADT_WRITE(printf("adt write (%d-%x), %x\n", id, (uint32_t)offset,
                           (uint32_t)value);)
    if (id >= CORE_V_APB_ADV_TIMER_NR_TIMERS) {
        return;
    }

    CoreVApbAdvTimer *t = &s->timers[id];
    switch (offset) {
    case A_TIM_CMD:
        t->cmd = value & 0x1F;
        if (t->cmd & R_TIM_CMD_START_MASK) {
            ptimer_transaction_begin(t->timer);
            ptimer_set_limit(t->timer, 1, 1);
            ptimer_run(t->timer, 0);
            ptimer_transaction_commit(t->timer);
        }

        if (t->cmd & R_TIM_CMD_STOP_MASK) {
            ptimer_transaction_begin(t->timer);
            ptimer_stop(t->timer);
            ptimer_transaction_commit(t->timer);
        }

        if (t->cmd & R_TIM_CMD_RESET_MASK) {
            t->direction = 1;
            t->counter = t->th_lo;
        }
        break;
    case A_TIM_CONFIG:
        t->config = value & 0xFF1FFF;
        uint32_t presc = FIELD_EX32(t->config, TIM_CONFIG, PRESC);
        presc = presc == 0 ? 1 : presc;

        if (t->config & R_TIM_CONFIG_CLKSEL_MASK) {
            /* reference clock */
            ptimer_transaction_begin(t->timer);
            ptimer_set_freq(t->timer, REF_CLK_FREQ / presc);
            ptimer_transaction_commit(t->timer);
        } else {
            ptimer_transaction_begin(t->timer);
            ptimer_set_freq(t->timer, CLK_SYSTEM / presc);
            ptimer_transaction_commit(t->timer);
        }
        break;
    case A_TIM_THRESHOLD:
        t->th_lo = value & 0xFFFF;
        t->th_hi = value >> 16;
        break;
    case A_TIM_TH_CHANNEL0:
        t->channels[0] = value & 0x7FFFF;
        break;
    case A_TIM_TH_CHANNEL1:
        t->channels[1] = value & 0x7FFFF;
        break;
    case A_TIM_TH_CHANNEL2:
        t->channels[2] = value & 0x7FFFF;
        break;
    case A_TIM_TH_CHANNEL3:
        t->channels[3] = value & 0x7FFFF;
        break;
    case A_TIM_LUT_CHANNEL0:
        t->channels_lut[0] = value & 0x3FFFF;
        break;
    case A_TIM_LUT_CHANNEL1:
        t->channels_lut[1] = value & 0x3FFFF;
        break;
    case A_TIM_LUT_CHANNEL2:
        t->channels_lut[2] = value & 0x3FFFF;
        break;
    case A_TIM_LUT_CHANNEL3:
        t->channels_lut[3] = value & 0x3FFFF;
        break;
    }
}

static const MemoryRegionOps core_v_apb_adv_timer_ops = {
    .read = core_v_apb_adv_timer_read,
    .write = core_v_apb_adv_timer_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_core_v_apb_adv_timer = {
    .name = TYPE_CORE_V_APB_ADV_TIMER ".timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(id, CoreVApbAdvTimer),
        VMSTATE_INT8(direction, CoreVApbAdvTimer),
        VMSTATE_UINT16(counter, CoreVApbAdvTimer),
        VMSTATE_PTIMER(timer, CoreVApbAdvTimer),
        VMSTATE_UINT32(cmd, CoreVApbAdvTimer),
        VMSTATE_UINT32(config, CoreVApbAdvTimer),
        VMSTATE_UINT16(th_hi, CoreVApbAdvTimer),
        VMSTATE_UINT16(th_lo, CoreVApbAdvTimer),
        VMSTATE_BOOL_ARRAY(is_2nd, CoreVApbAdvTimer,
                           CORE_V_APB_ADV_TIMER_NR_CHANNELS),
        VMSTATE_UINT32_ARRAY(channels, CoreVApbAdvTimer,
                             CORE_V_APB_ADV_TIMER_NR_CHANNELS),
        VMSTATE_UINT32_ARRAY(channels_lut, CoreVApbAdvTimer,
                             CORE_V_APB_ADV_TIMER_NR_CHANNELS),
        VMSTATE_BOOL_ARRAY(events, CoreVApbAdvTimer,
                           CORE_V_APB_ADV_TIMER_NR_CHANNELS),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_core_v_apb_adv_timer_state = {
    .name = TYPE_CORE_V_APB_ADV_TIMER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(timers, CoreVApbAdvTimerState,
                             CORE_V_APB_ADV_TIMER_NR_TIMERS,
                             1, vmstate_core_v_apb_adv_timer,
                             CoreVApbAdvTimer),
        VMSTATE_UINT32(event_cfg, CoreVApbAdvTimerState),
        VMSTATE_UINT32(cg, CoreVApbAdvTimerState),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_apb_adv_timer_init(Object *obj)
{
    CoreVApbAdvTimerState *s = CORE_V_APB_ADV_TIMER(obj);

    qdev_init_gpio_out(DEVICE(obj), s->irq, CORE_V_APB_ADV_TIMER_NR_TIMERS);

    memory_region_init_io(&s->mmio, obj, &core_v_apb_adv_timer_ops, s,
                          TYPE_CORE_V_APB_ADV_TIMER, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void core_v_apb_adv_init_timer(CoreVApbAdvTimerState *s, uint8_t id)
{
    CoreVApbAdvTimer *t = &s->timers[id];

    t->id = id;
    t->direction = 1;

    t->timer = ptimer_init(core_v_apb_adv_timer_tick, t, PTIMER_POLICY_LEGACY);

    ptimer_transaction_begin(t->timer);
    ptimer_set_freq(t->timer, CLK_SYSTEM);
    ptimer_transaction_commit(t->timer);
}

static void core_v_apb_adv_timer_realize(DeviceState *dev, Error **errp)
{
    CoreVApbAdvTimerState *s = CORE_V_APB_ADV_TIMER(dev);

    int i;
    for (i = 0; i < CORE_V_APB_ADV_TIMER_NR_TIMERS; i++) {
        core_v_apb_adv_init_timer(s, i);
    }
}

static void core_v_apb_adv_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = core_v_apb_adv_timer_reset;
    dc->vmsd = &vmstate_core_v_apb_adv_timer_state;
    dc->realize = core_v_apb_adv_timer_realize;
}

static const TypeInfo core_v_apb_adv_timer_info = {
    .name           = TYPE_CORE_V_APB_ADV_TIMER,
    .parent         = TYPE_SYS_BUS_DEVICE,
    .instance_size  = sizeof(CoreVApbAdvTimerState),
    .instance_init  = core_v_apb_adv_timer_init,
    .class_init     = core_v_apb_adv_timer_class_init,
};

static void core_v_apb_adv_timer_register(void)
{
    type_register_static(&core_v_apb_adv_timer_info);
}

type_init(core_v_apb_adv_timer_register)
