/*
 *  NUCLEI TIMER (Timer Unit) interface
 *
 * Copyright (c) 2020 Gao ZhiYuan <alapha23@gmail.com>
 * Copyright (c) 2020-2021 PLCT Lab.All rights reserved.
 *
 * This provides a parameterizable timer controller based on NucLei's Systimer.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qapi/error.h"
#include "qemu/error-report.h"
#include "qemu/timer.h"
#include "target/riscv/cpu.h"
#include "hw/intc/nuclei_systimer.h"
#include "hw/registerfields.h"
#include "hw/qdev-properties.h"
#include "migration/vmstate.h"
#include "hw/intc/nuclei_eclic.h"

static uint64_t cpu_riscv_read_rtc(uint64_t timebase_freq)
{
    return muldiv64(qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL),
        timebase_freq, NANOSECONDS_PER_SECOND);
}

static void nuclei_timer_reset(DeviceState *dev)
{
    NucLeiSYSTIMERState *s = NUCLEI_SYSTIMER(dev);
    s->mtime_lo = 0x0;
    s->mtime_hi = 0x0;
    s->mtimecmp_lo = 0xFFFFFFFF;
    s->mtimecmp_hi = 0xFFFFFFFF;
    s->msftrst = 0x0;
    s->mtimectl = 0x0;
    s->msip = 0x0;
}

static void nuclei_timer_update_compare(NucLeiSYSTIMERState *s)
{
    CPUState *cpu = qemu_get_cpu(0);
    CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
    uint64_t cmp, real_time;
    int64_t diff;

    // real_time =  s->mtime_lo | ((uint64_t)s->mtime_hi << 32);

    real_time= cpu_riscv_read_rtc(s->timebase_freq);

    cmp = (uint64_t)s->mtimecmp_lo | ((uint64_t)s->mtimecmp_hi <<32);
    env->timecmp =  cmp;

    diff = cmp - real_time;

    if ( real_time >= cmp) {
        qemu_set_irq(s->timer_irq, 1);
    }
    else {
    	qemu_set_irq(s->timer_irq, 0);

        if (s->mtimecmp_hi != 0xffffffff) {
            // set up future timer interrupt
            uint64_t next_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                muldiv64(diff, NANOSECONDS_PER_SECOND, s->timebase_freq);
            timer_mod(env->timer, next_ns);
	    }
    }
}

static uint64_t nuclei_timer_read(void *opaque, hwaddr offset,
                                    unsigned size)
{
    NucLeiSYSTIMERState *s = NUCLEI_SYSTIMER(opaque);
    CPUState *cpu = qemu_get_cpu(0);
    CPURISCVState *env = cpu ? cpu->env_ptr : NULL;
    uint64_t value = 0;

    switch (offset) {
    case NUCLEI_SYSTIMER_REG_MTIMELO:
        value = cpu_riscv_read_rtc(s->timebase_freq);
        s->mtime_lo =  value & 0xffffffff;
        s->mtime_hi = (value >> 32) & 0xffffffff;
        value = s->mtime_lo;
        break;
    case NUCLEI_SYSTIMER_REG_MTIMEHI:
        value =  s->mtime_hi;
        break;
    case NUCLEI_SYSTIMER_REG_MTIMECMPLO:
        s->mtimecmp_lo = (env->timecmp) & 0xFFFFFFFF;
        value = s->mtimecmp_lo;
        break;
    case NUCLEI_SYSTIMER_REG_MTIMECMPHI:
        s->mtimecmp_hi = (env->timecmp >> 32) & 0xFFFFFFFF;
        value = s->mtimecmp_hi;
        break;
    case NUCLEI_SYSTIMER_REG_MSFTRST:
        value =  s->msftrst;
        break;
    case NUCLEI_SYSTIMER_REG_MTIMECTL:
        value = s->mtimectl;
        break;
    case NUCLEI_SYSTIMER_REG_MSIP:
        value = s->msip;
        break;
    default:
        break;
    }

    return (value & 0xFFFFFFFF);
}

static void nuclei_timer_write(void *opaque, hwaddr offset,
                                 uint64_t value, unsigned size)
{
    NucLeiSYSTIMERState *s = NUCLEI_SYSTIMER(opaque);
    CPUState *cpu = qemu_get_cpu(0);
    CPURISCVState *env = cpu ? cpu->env_ptr : NULL;

    value = value & 0xFFFFFFFF;

    switch (offset) {
    case NUCLEI_SYSTIMER_REG_MTIMELO:
        s->mtime_lo = value;
        env->timer->expire_time |= (value &0xFFFFFFFF);
        break;
    case NUCLEI_SYSTIMER_REG_MTIMEHI:
        s->mtime_hi = value;
        env->timer->expire_time |= ((value << 32)&0xFFFFFFFF);
        break;
    case NUCLEI_SYSTIMER_REG_MTIMECMPLO:
        s->mtimecmp_lo = value;
        s->mtimecmp_hi = 0xFFFFFFFF;
        env->timecmp  |= (value &0xFFFFFFFF);
        nuclei_timer_update_compare(s);
        break;
    case NUCLEI_SYSTIMER_REG_MTIMECMPHI:
        s->mtimecmp_hi = value;
        env->timecmp  |= ((value << 32)&0xFFFFFFFF);
        nuclei_timer_update_compare(s);
        break;
    case NUCLEI_SYSTIMER_REG_MSFTRST:
	if (!(value & 0x80000000) == 0)
            nuclei_timer_reset((DeviceState *)s);
        break;
    case NUCLEI_SYSTIMER_REG_MTIMECTL:
        s->mtimectl = value;
        break;
    case NUCLEI_SYSTIMER_REG_MSIP:
        s->msip = value;
        if ((s->msip & 0x1) == 1) {
            qemu_set_irq(s->soft_irq, 1);
        }else{
            qemu_set_irq(s->soft_irq, 0);
        }

        break;
    default:
        break;
    }
}

static const MemoryRegionOps nuclei_timer_ops = {
    .read = nuclei_timer_read,
    .write = nuclei_timer_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
};

static Property nuclei_systimer_properties[] = {
    DEFINE_PROP_UINT32("aperture-size", NucLeiSYSTIMERState, aperture_size, 0x1000),
    DEFINE_PROP_UINT32("timebase-freq", NucLeiSYSTIMERState, timebase_freq, NUCLEI_NUCLEI_TIMEBASE_FREQ),
    DEFINE_PROP_END_OF_LIST(),
};

static void nuclei_timer_instance_init(Object *obj)
{
    NucLeiSYSTIMERState *s = NUCLEI_SYSTIMER(obj);

    memory_region_init_io(&s->mmio, obj, &nuclei_timer_ops,
                          s,TYPE_NUCLEI_SYSTIMER, s->aperture_size);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->mmio);
}

static void nuclei_mtimecmp_cb(void *opaque) {
    RISCVCPU *cpu = RISCV_CPU(qemu_get_cpu(0));
    CPURISCVState *env = &cpu->env;
    NucLeiSYSTIMERState *s = NUCLEI_SYSTIMER(opaque);
    qemu_set_irq(s->timer_irq, 1);
    timer_del(env->timer);
}

static void nuclei_timer_realize(DeviceState *dev, Error **errp)
{
    RISCVCPU *cpu = RISCV_CPU(qemu_get_cpu(0));
    CPURISCVState *env = &cpu->env;
    NucLeiSYSTIMERState *s = NUCLEI_SYSTIMER(dev);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->soft_irq);
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->timer_irq);

    env->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                   &nuclei_mtimecmp_cb, s);
    env->timecmp = 0;
}

static void nuclei_timer_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    dc->realize = nuclei_timer_realize;
    dc->reset = nuclei_timer_reset;
    dc->desc = "NucLei Systimer Timer";
    device_class_set_props(dc, nuclei_systimer_properties);
}

static const TypeInfo nuclei_timer_info = {
    .name = TYPE_NUCLEI_SYSTIMER,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(NucLeiSYSTIMERState),
    .instance_init = nuclei_timer_instance_init,
    .class_init = nuclei_timer_class_init,
};

static void nuclei_timer_register_types(void)
{
    type_register_static(&nuclei_timer_info);
}
type_init(nuclei_timer_register_types);