/*
 * QEMU CORE-V MCU APB GPIO Controller
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
#include "qemu/timer.h"
#include "qapi/error.h"
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "hw/sd/sd.h"
#include "hw/gpio/core_v_apb_gpio.h"
#include "hw/irq.h"

#define DEBUG_GPIO_CONNECT_INOUT
#define DEBUG_GPIO_READ(X)
#define DEBUG_GPIO_WRITE(X)

#define REG_SETGPIO 0x000
#define REG_CLRGPIO 0x004
#define REG_TOGGPIO 0x008

#define REG_PIN0 0x010
#define REG_PIN1 0x014
#define REG_PIN2 0x018
#define REG_PIN3 0x01C

#define REG_OUT0 0x020
#define REG_OUT1 0x024
#define REG_OUT2 0x028
#define REG_OUT3 0x02C

#define REG_SETSEL 0x030
#define REG_RDSTAT 0x034
#define REG_SETDIR 0x038
#define REG_SETINT 0x03C
#define REG_INTACK 0x040

static uint64_t corev_gpio_read(void *opaque, hwaddr offset, unsigned size)
{
    CoreVGpioState *s = (CoreVGpioState *)opaque;
    uint32_t value = 0;
    uint8_t i;

    switch (offset) {
    case REG_RDSTAT:
        value |= s->dir[s->select] << 24;
        value |= s->inttype[s->select] << 17;
        value |= s->inten[s->select] << 16;
        value |= s->in[s->select] << 12;
        value |= s->out[s->select] << 8;
        value |= s->select;
        break;
    case REG_OUT0:
        for (i = 0; (i < COREV_N_GPIO) && (i < 32); i++) {
            value = (s->out[i] << i);
        }
        break;
    case REG_PIN0:
        for (i = 0; (i < COREV_N_GPIO) && (i < 32); i++) {
            value = (s->in[i] << i);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
                __func__, offset);
        break;
    }

    DEBUG_GPIO_READ(printf("GPIO read %x, %x\n", (uint32_t)offset,
                           (uint32_t)value);)
    return value;
}

static void update_irq(CoreVGpioState *s)
{
    int i;
    bool int_fall[COREV_N_GPIO];
    bool int_rise[COREV_N_GPIO];
    bool int_low[COREV_N_GPIO];
    bool int_hi[COREV_N_GPIO];

    for (i = 0; i < COREV_N_GPIO; i++) {
        int_fall[i] = (s->inttype[i] & 0x1) && s->fall[i] && s->inten[i];
        int_rise[i] = (s->inttype[i] & 0x2) && s->rise[i] && s->inten[i];
        int_low[i] = (s->inttype[i] == 0) && !s->in[i] &&
                     !s->block_int[i] && s->inten[i];
        int_hi[i] = (s->inttype[i] == 4) && s->in[i] &&
                    !s->block_int[i] && s->inten[i];
        s->block_int[i] |= int_low[i] || int_hi[i];
        qemu_set_irq(s->irq[i], int_fall[i] || int_rise[i] ||
                                int_low[i] || int_hi[i]);
    }
}


static void core_v_gpio_in(void *opaque, int num, int level)
{
    CoreVGpioState *s = (CoreVGpioState *)opaque;
    if (!s->in[num] && level) {
        s->rise[num] = true;
    } else if (s->in[num] && !level) {
        s->fall[num] = true;
    }
    s->in[num] = level;
    update_irq(s);
    s->rise[num] = false;
    s->fall[num] = false;
}

static void corev_gpio_write(void *opaque, hwaddr offset, uint64_t value,
                             unsigned size)
{
    CoreVGpioState *s = (CoreVGpioState *)opaque;
    int i;

     DEBUG_GPIO_WRITE(printf("GPIO write %x, %x\n", (uint32_t)offset,
                             (uint32_t)value);)
    switch (offset) {
    case REG_SETSEL:
        s->select = value & 0x3F;
        break;
    case REG_SETDIR:
        s->select = value & 0x3F;
        s->dir[s->select] = (value >> 24) & 0x3;
        break;
    case REG_SETINT:
        s->select = value & 0x3F;
        s->inttype[s->select] = (value >> 17) & 0x7;
        s->inten[s->select] = (value >> 16) & 0x1;
        update_irq(s);
        break;
    case REG_SETGPIO:
        s->select = value & 0x3F;
        s->out[s->select] = true;
        break;
    case REG_CLRGPIO:
        s->select = value & 0x3F;
        s->out[s->select] = false;
        break;
    case REG_TOGGPIO:
        s->select = value & 0x3F;
        s->out[s->select] = !s->out[s->select];
        break;
    case REG_OUT0:
        for (i = 0; (i < COREV_N_GPIO) && (i < 32); i++) {
            s->out[i] = (value >> i) & 0x1;
        }
        break;
    case REG_INTACK:
        s->block_int[value & 0xff] = false;
        update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR, "%s: Bad offset %"HWADDR_PRIx"\n",
            __func__, offset);
    }

#ifdef DEBUG_GPIO_CONNECT_INOUT
    for (i = 0; i < COREV_N_GPIO; i++) {
        if (s->dir[i] & 0x1) {
            core_v_gpio_in(s, i, s->out[i]);
        }
    }
#endif
    return;
}

static void corev_gpio_reset(DeviceState *dev)
{
    CoreVGpioState *s = CORE_V_GPIO(dev);
    int i;

    s->select = 0;
    for (i = 0; i < COREV_N_GPIO; i++) {
        s->dir[i] = 0;
        s->inttype[i] = 0;
        s->inten[i] = false;
        s->out[i] = false;
        s->in[i] = false;
        s->rise[i] = false;
        s->fall[i] = false;
        s->block_int[i] = false;
    }
}

static const MemoryRegionOps corev_gpio_ops = {
    .read = corev_gpio_read,
    .write = corev_gpio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
};

static const VMStateDescription vmstate_corev_gpio = {
    .name = "core_v_gpio",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(select, CoreVGpioState),
        VMSTATE_BOOL_ARRAY(inten, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_UINT8_ARRAY(inttype, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_BOOL_ARRAY(out, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_UINT8_ARRAY(dir, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_BOOL_ARRAY(rise, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_BOOL_ARRAY(fall, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_BOOL_ARRAY(block_int, CoreVGpioState, COREV_N_GPIO),
        VMSTATE_END_OF_LIST()
    }
};

static void corev_gpio_init(Object *obj)
{
    CoreVGpioState *s = CORE_V_GPIO(obj);
    DeviceState *dev = DEVICE(obj);
    SysBusDevice *sbd = SYS_BUS_DEVICE(obj);

    memory_region_init_io(&s->iomem, obj, &corev_gpio_ops, s, "corev_gpio",
                          0x1000);
    sysbus_init_mmio(sbd, &s->iomem);
    qdev_init_gpio_out(dev, s->irq, COREV_N_GPIO);
    qdev_init_gpio_in(DEVICE(obj), core_v_gpio_in, COREV_N_GPIO);
}

static void corev_gpio_realize(DeviceState *dev, Error **errp)
{
}

static void corev_gpio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_corev_gpio;
    dc->realize = &corev_gpio_realize;
    dc->reset = &corev_gpio_reset;
}

static const TypeInfo corev_gpio_info = {
    .name          = TYPE_CORE_V_GPIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVGpioState),
    .instance_init = corev_gpio_init,
    .class_init    = corev_gpio_class_init,
};

static void corev_gpio_register_types(void)
{
    type_register_static(&corev_gpio_info);
}

type_init(corev_gpio_register_types)
