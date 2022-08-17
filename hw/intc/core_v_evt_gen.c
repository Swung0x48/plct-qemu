/*
 * CORE-V MCU Event Generator
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
#include "qapi/error.h"
#include "migration/vmstate.h"
#include "qemu/module.h"
#include "qemu/timer.h"
#include "hw/intc/core_v_evt_gen.h"
#include "target/riscv/cpu.h"
#include "hw/irq.h"
#include "trace.h"

#define REG_SW_EVENT      0x000
#define REG_FC_MASK0      0x004
#define REG_FC_MASK7      0x020
#define REG_CL_MASK0      0x024
#define REG_CL_MASK7      0x040
#define REG_PR_MASK0      0x044
#define REG_PR_MASK7      0x060
#define REG_ERR0          0x064
#define REG_ERR7          0x080
#define REG_TIMER_LO      0x084
#define REG_TIMER_HI      0x088
#define REG_FIFO          0x090

#define APB_EVENTS_START   160

#define DEBUG_EG_READ(X)
#define DEBUG_EG_WRITE(X)
#define DEBUG_EG_EXEC(X)

static void update_irq(CoreVEvtGenState *s)
{
    int i;
    int error = 0;

    for (i = 0; i <= 7; i++) {
        if (s->err_reg[i]) {
            error = 1;
            break;
        }
    }
    qemu_set_irq(s->irq[1], error);
}

static void dec_events(CoreVEvtGenState *s)
{
    uint8_t index = s->round >> 5;
    uint8_t pos = s->round & 0x1F;
    s->err_reg[index] &= ~(1 << pos);
    s->events[s->round]--;
    update_irq(s);
}

static void round_events(void *opaque)
{
    CoreVEvtGenState *s = opaque;
    uint8_t round = s->round;
    round++;
    while (s->fc_ready) {
        if (s->events[round]) {
            /*
             * only trigger event to fabric controller and peripherals,
             * nothing for cluster events currently
             */
            uint8_t index = round >> 5;
            uint32_t pos = 1 << (round & 0x1F);
            if (~s->pr_mask[index] & pos) {
                qemu_set_irq(s->irq[4], round);
            }

            if (~s->fc_mask[index] & pos) {
                s->fc_ready = false;
                s->round = round;
                qemu_irq_raise(s->irq[0]);
                break;
            }
        }

        if (s->round == round) {
            break;
        }

        if (++round == EVENT_NUM) {
            round = 0;
        }
    }
}

static void core_v_evt_gen_set_irq(void *opaque, int num, int level)
{
    CoreVEvtGenState *s = opaque;
    if (level == 1) {
        if (s->events[num] == 3) {
            uint8_t index = num >> 5;
            uint8_t pos = num & 0x1f;
            s->err_reg[index] |= 1 << pos;
            update_irq(s);
        }

        if (num == s->timer_lo) {
            qemu_irq_raise(s->irq[2]);
        }

        if (num == s->timer_hi) {
            qemu_irq_raise(s->irq[3]);
        }
        s->events[num]++;
    }

    round_events(s);
}

static void core_v_evt_gen_write(void *opaque, hwaddr addr, uint64_t value,
                                 unsigned size)
{
    CoreVEvtGenState *s = opaque;
    int i;

    DEBUG_EG_WRITE(printf("EventGen write %d, %x\n", (uint32_t)addr,
                          (uint32_t)value);)
    switch (addr) {
    case REG_SW_EVENT:
        s->sw_event = value & 0xff;
        for (i = 0; i < 7; i++) {
            if (value & (1 << i)) {
                core_v_evt_gen_set_irq(opaque, APB_EVENTS_START + i, 1);
            }
        }
        break;
    case REG_FC_MASK0 ... REG_FC_MASK7:
        s->fc_mask[(addr - REG_FC_MASK0) >> 2] = value;
        break;
    case REG_CL_MASK0 ... REG_CL_MASK7:
        s->cl_mask[(addr - REG_CL_MASK0) >> 2] = value;
        break;
    case REG_PR_MASK0 ... REG_PR_MASK7:
        s->pr_mask[(addr - REG_PR_MASK0) >> 2] = value;
        break;
    case REG_TIMER_LO:
        s->timer_lo = value & 0xff;
        break;
    case REG_TIMER_HI:
        s->timer_hi = value & 0xff;
        break;
    default:
        break;
    }
}

static uint64_t core_v_evt_gen_read(void *opaque, hwaddr addr, unsigned size)
{
    CoreVEvtGenState *s = opaque;
    uint32_t value = 0;

    switch (addr) {
    case REG_SW_EVENT:
        value = s->sw_event;
        break;
    case REG_FC_MASK0 ... REG_FC_MASK7:
        value = s->fc_mask[(addr - REG_FC_MASK0) >> 2];
        break;
    case REG_PR_MASK0 ... REG_PR_MASK7:
        value = s->pr_mask[(addr - REG_PR_MASK0) >> 2];
        break;
    case REG_ERR0 ... REG_ERR7:
        value = s->err_reg[(addr - REG_ERR0) >> 2];
        s->err_reg[(addr - REG_ERR0) >> 2] = 0;
        break;
    case REG_TIMER_LO:
        value = s->timer_lo;
        break;
    case REG_TIMER_HI:
        value = s->timer_hi;
        break;
    case REG_FIFO:
        value = s->round;
        qemu_irq_lower(s->irq[0]);
        dec_events(s);
        s->fc_ready = true;
        round_events(s);
        break;
    default:
        break;
    }
    DEBUG_EG_READ(printf("EventGen read %d, %x\n", (uint32_t)addr,
                  (uint32_t)value);)
    return value;
}

static const MemoryRegionOps core_v_evt_gen_ops = {
    .read = core_v_evt_gen_read,
    .write = core_v_evt_gen_write,
    .valid = {
        .min_access_size = 1,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void core_v_evt_gen_reset(DeviceState *d)
{
    CoreVEvtGenState *s = CORE_V_EVT_GEN(d);
    int i = 0;

    s->sw_event = 0;
    for (; i < 8; i++) {
        s->fc_mask[i] = 0xFFFFFFFF;
        s->cl_mask[i] = 0xFFFFFFFF;
        s->pr_mask[i] = 0xFFFFFFFF;
        s->err_reg[i] = 0;
    }

    s->timer_lo = 0xFF;
    s->timer_hi = 0xFF;
    s->fc_ready = true;
    memset(s->events, 0, EVENT_NUM);
}

static void core_v_evt_gen_realize(DeviceState *dev, Error **errp)
{
    CoreVEvtGenState *s = CORE_V_EVT_GEN(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->mem, OBJECT(s), &core_v_evt_gen_ops, s,
                          "core_v_evt_gen", 0x1000);
    sysbus_init_mmio(sbd, &s->mem);
}

static void core_v_evt_gen_init(Object *obj)
{
    CoreVEvtGenState *s = CORE_V_EVT_GEN(obj);

    qdev_init_gpio_out(DEVICE(obj), s->irq, 5);
    qdev_init_gpio_in(DEVICE(obj), core_v_evt_gen_set_irq, EVENT_NUM);
}

static const VMStateDescription vmstate_core_v_evt_gen = {
    .name = "core_v_evt_gen",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(sw_event, CoreVEvtGenState),
        VMSTATE_UINT32_ARRAY(fc_mask, CoreVEvtGenState, 8),
        VMSTATE_UINT32_ARRAY(cl_mask, CoreVEvtGenState, 8),
        VMSTATE_UINT32_ARRAY(pr_mask, CoreVEvtGenState, 8),
        VMSTATE_UINT32_ARRAY(err_reg, CoreVEvtGenState, 8),
        VMSTATE_UINT8(timer_lo, CoreVEvtGenState),
        VMSTATE_UINT8(timer_hi, CoreVEvtGenState),
        VMSTATE_UINT8(round, CoreVEvtGenState),
        VMSTATE_UINT8_ARRAY(events, CoreVEvtGenState, EVENT_NUM),
        VMSTATE_BOOL(fc_ready, CoreVEvtGenState),
        VMSTATE_TIMER(r_timer, CoreVEvtGenState),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_evt_gen_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = core_v_evt_gen_realize;
    dc->reset = core_v_evt_gen_reset;
    dc->vmsd = &vmstate_core_v_evt_gen;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo core_v_evt_gen_type_info = {
    .name = TYPE_CORE_V_EVT_GEN,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVEvtGenState),
    .instance_init = core_v_evt_gen_init,
    .class_init = core_v_evt_gen_class_init,
};

static void core_v_evt_gen_register_types(void)
{
    type_register_static(&core_v_evt_gen_type_info);
}

type_init(core_v_evt_gen_register_types)
