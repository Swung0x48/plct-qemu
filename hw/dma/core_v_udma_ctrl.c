/*
 * CORE-V MCU UDMA Subsystem CTRL UNIT
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
#include "hw/dma/core_v_udma.h"
#include "hw/irq.h"
#include "trace.h"

#define DEBUG_CTRL(X)

#define REG_CG        0x000
#define REG_CFG_EVT   0x004
#define REG_RST       0x008

static void core_v_udma_ctrl_event(void *opaque, int num, int level)
{
    CoreVUDMACtrlState *s = opaque;
    int i;

    for (i = 0; i < UDMA_CTRL_EVENT_NUM; i++) {
        if (((s->cfg_evt >> (i * 8)) & 0xff) == level) {
            qemu_irq_raise(s->irq[i]);
        }
    }
}

static void gate_update(CoreVUDMACtrlState *s)
{
    int i;
    for (i = 0; i < N_PERIPHS; i++) {
        if (s->cg & (1 << i)) {
            qemu_irq_raise(s->gate[i]);
        } else {
            qemu_irq_lower(s->gate[i]);
        }
    }
}

static void core_v_udma_ctrl_write(void *opaque, hwaddr addr,
                                   uint64_t value, unsigned size)
{
    CoreVUDMACtrlState *s = opaque;
    DEBUG_CTRL(printf("udma ctrl write %x, %x\n", (uint32_t)addr,
                      (uint32_t)value);)
    switch (addr) {
    case REG_CG:
        s->cg = value;
        gate_update(s);
        break;
    case REG_CFG_EVT:
        s->cfg_evt = value;
        break;
    case REG_RST:
        s->rst = value;
        break;
    default:
        break;
    }
}

static uint64_t core_v_udma_ctrl_read(void *opaque, hwaddr addr,
                                      unsigned size)
{
    CoreVUDMACtrlState *s = opaque;
    uint32_t value = 0;
    switch (addr) {
    case REG_CG:
        value = s->cg;
        break;
    case REG_CFG_EVT:
        value = s->cfg_evt;
        break;
    case REG_RST:
        value = s->rst;
        break;
    default:
        break;
    }
    DEBUG_CTRL(printf("udma ctrl read %x, %x\n", (uint32_t)addr,
                      (uint32_t)value);)
    return value;
}

static const MemoryRegionOps core_v_udma_ctrl_ops = {
    .read = core_v_udma_ctrl_read,
    .write = core_v_udma_ctrl_write,
    .valid = {
        .min_access_size = 4,
        .max_access_size = 4,
    },
    .endianness = DEVICE_LITTLE_ENDIAN,
};

static void core_v_udma_ctrl_reset(DeviceState *d)
{
    CoreVUDMACtrlState *s = CORE_V_UDMA_CTRL(d);
    s->cg = 0;
    s->cfg_evt = 0;
    s->rst = 0;
}

static void core_v_udma_ctrl_realize(DeviceState *dev, Error **errp)
{
    CoreVUDMACtrlState *s = CORE_V_UDMA_CTRL(dev);
    SysBusDevice *sbd = SYS_BUS_DEVICE(dev);

    memory_region_init_io(&s->iomem, OBJECT(s), &core_v_udma_ctrl_ops, s,
                          "core_v_udma_ctrl", UDMA_PER_SIZE);
    sysbus_init_mmio(sbd, &s->iomem);
}

static void core_v_udma_ctrl_init(Object *obj)
{
    CoreVUDMACtrlState *s = CORE_V_UDMA_CTRL(obj);

    qdev_init_gpio_out_named(DEVICE(obj), s->irq, "event", UDMA_CTRL_EVENT_NUM);
    qdev_init_gpio_out_named(DEVICE(obj), s->gate, "gate", N_PERIPHS);
    qdev_init_gpio_in(DEVICE(obj), core_v_udma_ctrl_event, 1);
}

static const VMStateDescription vmstate_core_v_udma_ctrl = {
    .name = "core_v_udma_ctrl",
    .version_id = 0,
    .minimum_version_id = 0,
    .fields = (VMStateField[]) {
        VMSTATE_UINT32(cg, CoreVUDMACtrlState),
        VMSTATE_UINT32(cfg_evt, CoreVUDMACtrlState),
        VMSTATE_UINT32(rst, CoreVUDMACtrlState),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_udma_ctrl_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = core_v_udma_ctrl_realize;
    dc->reset = core_v_udma_ctrl_reset;
    dc->vmsd = &vmstate_core_v_udma_ctrl;
    set_bit(DEVICE_CATEGORY_MISC, dc->categories);
}

static const TypeInfo core_v_udma_ctrl_type_info = {
    .name = TYPE_CORE_V_UDMA_CTRL,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVUDMACtrlState),
    .instance_init = core_v_udma_ctrl_init,
    .class_init = core_v_udma_ctrl_class_init,
};

static void core_v_udma_ctrl_register_types(void)
{
    type_register_static(&core_v_udma_ctrl_type_info);
}

type_init(core_v_udma_ctrl_register_types)
