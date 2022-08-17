/*
 * CORE-V MCU APB Timer
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
#include "qemu/timer.h"
#include "qom/object.h"
#include "hw/timer/core_v_timer.h"
#include "migration/vmstate.h"
#include "hw/irq.h"

#define TIMER_CFG_LO       0x000
#define TIMER_CFG_HI       0x004
#define TIMER_CNT_LO       0x008
#define TIMER_CNT_HI       0x00C
#define TIMER_CMP_LO       0x010
#define TIMER_CMP_HI       0x014
#define TIMER_START_LO     0x018
#define TIMER_START_HI     0x01C
#define TIMER_RESET_LO     0x020
#define TIMER_RESET_HI     0x024

#define TIMER_CFG_EN             0x00000001
#define TIMER_CFG_RST            0x00000002
#define TIMER_CFG_IRQEN          0x00000004
#define TIMER_CFG_IEM            0x00000008
#define TIMER_CFG_CLR            0x00000010
#define TIMER_CFG_ONESHOT        0x00000020
#define TIMER_CFG_PRESCALER_EN   0x00000040
#define TIMER_CFG_REF_CLK_EN     0x00000080
#define TIMER_CFG_PRESCALER_MASK 0x0000FF00
#define TIMER_CFG_MODE_64        0x80000000

#define PRESCALER_SHIFT   0x8

#define REF_CLK 32000    /* 32K */
#define DEFAULT_CLK 50000000    /* 5M */

#define DEBUG_TIMER_READ(X)
#define DEBUG_TIMER_WRITE(X)

static uint32_t core_v_timer_get_count(CoreVTimerState *s, uint8_t index)
{
    bool high = false;

    if ((index == 1) && (s->cfg[0] & TIMER_CFG_MODE_64)) {
        index = 0;
        high = true;
    }

    if (s->cfg[index] & TIMER_CFG_EN) {
        uint64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
        uint64_t cnt = (now - s->start_time[index]) * s->freq[index] /
                       NANOSECONDS_PER_SECOND;
        return high ? (cnt >> 32) : (uint32_t)cnt;
    } else {
        return high ? s->cnt[1] : s->cnt[index];
    }
}

static void core_v_set_freq(CoreVTimerState *s, uint8_t index)
{
    int prescaler = 0;
    if (s->cfg[index] & TIMER_CFG_PRESCALER_EN) {
        prescaler = (s->cfg[index] & TIMER_CFG_PRESCALER_MASK) >>
                    PRESCALER_SHIFT;
    }
    if (s->cfg[index] & TIMER_CFG_REF_CLK_EN) {
        s->freq[index] = REF_CLK >> prescaler;
    } else {
        s->freq[index] = DEFAULT_CLK >> prescaler;
    }
}

static void sync_cnts(CoreVTimerState *s)
{
    s->cnt[1] = core_v_timer_get_count(s, 1);
    s->cnt[0] = core_v_timer_get_count(s, 0);
}

static void core_v_update_timer(CoreVTimerState *s, uint8_t index)
{
    if (s->cfg[index] & TIMER_CFG_RST) {
        s->cnt[index] = 0;
        if ((index == 0) && (s->cfg[index] & TIMER_CFG_MODE_64)) {
            s->cnt[1] = 0;
        }
        s->cfg[index] &= ~TIMER_CFG_RST;
    }

    if ((index == 1) && (s->cfg[0] & TIMER_CFG_MODE_64)) {
        return;
    }

    uint64_t cnt = s->cnt[index];
    uint64_t cmp = s->cmp[index];

    if (s->cfg[index] & TIMER_CFG_EN) {
        if ((index == 0) && (s->cfg[index] & TIMER_CFG_MODE_64)) {
            cnt |= (uint64_t)s->cnt[1] << 32;
            cmp |= (uint64_t)s->cmp[1] << 32;
            timer_del(&s->timer[1]);
        }
        s->start_time[index] = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) -
                               cnt * NANOSECONDS_PER_SECOND / s->freq[index];
        timer_mod(&s->timer[index], qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                    (cmp - cnt) * NANOSECONDS_PER_SECOND /
                                    s->freq[index]);
    } else if (s->start_time[index]) {
        timer_del(&s->timer[index]);
        cnt = (qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) - s->start_time[index]) *
              s->freq[index] / 1000000000;
        s->cnt[index] = cnt;
        if ((index == 0) && (s->cfg[index] & TIMER_CFG_MODE_64)) {
            s->cnt[1] = cnt >> 32;
        }
    }
}

static void core_v_timer_write(void *opaque, hwaddr addr, uint64_t value,
                               unsigned size)
{
    CoreVTimerState *s = opaque;
    DEBUG_TIMER_WRITE(printf("timer write 0x%x, 0x%x\n", (uint32_t)addr,
                             (uint32_t)value);)
    switch (addr) {
    case TIMER_CFG_LO:
        if (s->cfg[0] & TIMER_CFG_EN) {
            sync_cnts(s);
        }
        s->cfg[0] = value;
        core_v_set_freq(s, 0);
        core_v_update_timer(s, 0);
        break;
    case TIMER_CFG_HI:
        if (s->cfg[1] & TIMER_CFG_EN) {
            sync_cnts(s);
        }
        s->cfg[1] = value;
        core_v_set_freq(s, 1);
        core_v_update_timer(s, 1);
        break;
    case TIMER_CNT_LO:
        s->cnt[0] = value;
        core_v_update_timer(s, 0);
        break;
    case TIMER_CNT_HI:
        s->cnt[1] = value;
        core_v_update_timer(s, 1);
        break;
    case TIMER_CMP_LO:
        s->cmp[0] = value;
        if (s->cfg[0] & TIMER_CFG_EN) {
            sync_cnts(s);
        }
        core_v_update_timer(s, 0);
        break;
    case TIMER_CMP_HI:
        s->cmp[1] = value;
        if (s->cfg[1] & TIMER_CFG_EN) {
            sync_cnts(s);
        }
        core_v_update_timer(s, 1);
        break;
    case TIMER_START_LO:
        s->cfg[0] |= TIMER_CFG_EN;
        core_v_update_timer(s, 0);
        break;
    case TIMER_START_HI:
        s->cfg[1] |= TIMER_CFG_EN;
        core_v_update_timer(s, 1);
        break;
    case TIMER_RESET_LO:
        s->cfg[0] |= TIMER_CFG_RST;
        core_v_update_timer(s, 0);
        break;
    case TIMER_RESET_HI:
        s->cfg[1] |= TIMER_CFG_RST;
        core_v_update_timer(s, 1);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "bad CORE-V Timer register: %x\n",
                      (unsigned)addr);
    }
}

static uint64_t core_v_timer_read(void *opaque, hwaddr addr, unsigned size)
{
    CoreVTimerState *s = opaque;
    uint32_t value = 0;

    switch (addr) {
    case TIMER_CFG_LO:
        value = s->cfg[0];
        break;
    case TIMER_CFG_HI:
        value = s->cfg[1];
        break;
    case TIMER_CNT_LO:
        value = core_v_timer_get_count(s, 0);
        break;
    case TIMER_CNT_HI:
        value = core_v_timer_get_count(s, 1);
        break;
    case TIMER_CMP_LO:
        value = s->cmp[0];
        break;
    case TIMER_CMP_HI:
        value = s->cmp[1];
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "bad CORE-V Timer register: %x\n",
                      (unsigned)addr);
        return 0;
    }

    DEBUG_TIMER_READ(printf("timer read 0x%x, 0x%x\n", (uint32_t)addr,
                            (uint32_t)value);)
    return value;
}

static const MemoryRegionOps core_v_timer_ops = {
    .read = core_v_timer_read,
    .write = core_v_timer_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static void core_v_timer_hit(void *opaque, uint8_t index)
{
    CoreVTimerState *s = opaque;

    if (s->cfg[index] & TIMER_CFG_ONESHOT) {
        timer_del(&s->timer[index]);
        s->cfg[index] &= ~TIMER_CFG_EN;
        s->cnt[index] = s->cmp[index];
        if ((index == 0) && (s->cfg[index] & TIMER_CFG_MODE_64)) {
            s->cfg[1]  &= ~TIMER_CFG_EN;
            s->cnt[1] = s->cmp[1];
        }
    } else if (s->cfg[index] & TIMER_CFG_CLR) {
        s->cfg[index] |= TIMER_CFG_RST;
    } else {
        s->cnt[index] += 1;
        if ((index == 0) && (s->cfg[index] & TIMER_CFG_MODE_64)) {
            s->cnt[1] = s->cmp[1];
        }
    }

    if (s->cfg[index] & TIMER_CFG_IRQEN) {
        qemu_irq_raise(s->irq[index]);
    }

    core_v_update_timer(s, index);
}

static void core_v_lo_timer_hit(void *opaque)
{
    core_v_timer_hit(opaque, 0);
}

static void core_v_hi_timer_hit(void *opaque)
{
    core_v_timer_hit(opaque, 1);
}

static void core_v_timer_realize(DeviceState *dev, Error **errp)
{
    CoreVTimerState *s = CORE_V_TIMER(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    timer_init_ns(&s->timer[0], QEMU_CLOCK_VIRTUAL, core_v_lo_timer_hit, s);
    timer_init_ns(&s->timer[1], QEMU_CLOCK_VIRTUAL, core_v_hi_timer_hit, s);

    memory_region_init_io(&s->iomem, OBJECT(s), &core_v_timer_ops, s,
                          TYPE_CORE_V_TIMER, 0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void evt_gen_trigger_timer(void *opaque, int num, int level)
{
    CoreVTimerState *s = opaque;

    if (level && s->cfg[num] & TIMER_CFG_IEM) {
        s->cfg[num] |= TIMER_CFG_EN;
        core_v_update_timer(s, num);
    }
}

static void core_v_timer_init(Object *obj)
{
    CoreVTimerState *s = CORE_V_TIMER(obj);

    qdev_init_gpio_out(DEVICE(obj), s->irq, 2);

    qdev_init_gpio_in(DEVICE(obj), evt_gen_trigger_timer, 2);
}

static void core_v_timer_reset(DeviceState *dev)
{
    CoreVTimerState *s = CORE_V_TIMER(dev);
    int i;

    for (i = 0; i < 2; i++) {
        s->cfg[i] = 0;
        s->cmp[i] = 0;
        s->cnt[i] = 0;
    }
}

static const VMStateDescription vmstate_core_v_timer = {
    .name = "core_v_timer",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32_ARRAY(cfg, CoreVTimerState, 2),
        VMSTATE_UINT32_ARRAY(cnt, CoreVTimerState, 2),
        VMSTATE_UINT32_ARRAY(cmp, CoreVTimerState, 2),
        VMSTATE_UINT32_ARRAY(freq, CoreVTimerState, 2),
        VMSTATE_TIMER_ARRAY(timer, CoreVTimerState, 2),
        VMSTATE_UINT64_ARRAY(start_time, CoreVTimerState, 2),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->realize = core_v_timer_realize;
    dc->reset = core_v_timer_reset;
    dc->vmsd = &vmstate_core_v_timer;
}

static const TypeInfo core_v_timer_info = {
    .name          = TYPE_CORE_V_TIMER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVTimerState),
    .instance_init = core_v_timer_init,
    .class_init    = core_v_timer_class_init,
};

static void core_v_timer_register(void)
{
    type_register_static(&core_v_timer_info);
}

type_init(core_v_timer_register)
