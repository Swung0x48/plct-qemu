/*
 * QEMU CORE-V MCU I2C Slave Device
 *
 * Copyright (c) 2022 PLCT Lab
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
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/i2c/core_v_i2cs.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "trace.h"
#include "hw/registerfields.h"

#define DEBUG_I2CS_READ(X)
#define DEBUG_I2CS_WRITE(X)
#define DEBUG_I2CS_I2C_READ(X)
#define DEBUG_I2CS_I2C_WRITE(X)

REG32(ADDRESS, 0x00)
REG32(ENABLE, 0x04)
REG32(DEBOUNCE_LEN, 0x08)
REG32(SCL_DELAY_LEN, 0x0C)
REG32(SDA_DELAY_LEN, 0x10)
REG32(I2A, 0x40)
REG32(I2A_STAT, 0x44)
REG32(A2I, 0x48)
REG32(A2I_STAT, 0x4C)
REG32(I2A_WRITE_DATA_PORT, 0x80)
REG32(I2A_READ_DATA_PORT, 0x84)
REG32(I2A_FLUSH, 0x88)
REG32(I2A_WRITE_FLAGS, 0x8C)
REG32(I2A_READ_FLAGS, 0x90)
REG32(A2I_WRITE_DATA_PORT, 0xC0)
REG32(A2I_READ_DATA_PORT, 0xC4)
REG32(A2I_FLUSH, 0xC8)
REG32(A2I_WRITE_FLAGS, 0xCC)
REG32(A2I_READ_FLAGS, 0xD0)
REG32(INTS, 0x100)
REG32(INTS_EN, 0x104)
REG32(I2A_WRITE_FLAGS_SELECT, 0x108)
REG32(I2A_READ_FLAGS_SELECT, 0x10C)
REG32(INTS2A, 0x140)
REG32(INTS2A_EN, 0x144)
REG32(A2I_WRITE_FLAGS_SELECT, 0x148)
REG32(A2I_READ_FLAGS_SELECT, 0x14C)


static inline void update_flags(Fifo8 *fifo, uint8_t *wrflags,
                                uint8_t *rdflags)
{
    uint32_t left = fifo8_num_free(fifo);
    uint32_t used = fifo8_num_used(fifo);

    if (left >= 128) {
        *wrflags = 0;
    } else if (left >= 64) {
        *wrflags = 1;
    } else if (left >= 32) {
        *wrflags = 2;
    } else if (left >= 8) {
        *wrflags = 3;
    } else if (left >= 4) {
        *wrflags = 4;
    } else if (left >= 2) {
        *wrflags = 5;
    } else if (left >= 1) {
        *wrflags = 6;
    } else {
        *wrflags = 7;
    }

    if (used >= 128) {
        *rdflags = 7;
    } else if (used >= 64) {
        *rdflags = 6;
    } else if (used >= 32) {
        *rdflags = 5;
    } else if (used >= 8) {
        *rdflags = 4;
    } else if (used >= 4) {
        *rdflags = 3;
    } else if (used >= 2) {
        *rdflags = 2;
    } else if (used >= 1) {
        *rdflags = 1;
    } else {
        *rdflags = 0;
    }
}

static void update_i2a_irq(CoreVI2csState *s)
{
    s->ints = 0;
    if (s->i2a_write_flags_select & (1 << s->i2a_write_flag)) {
        s->ints |= s->ints_en & 0x4;
    }

    if (s->i2a_read_flags_select & (1 << s->i2a_read_flag)) {
        s->ints |= s->ints_en & 0x2;
    }

    s->ints |= (s->ints_en & 0x1) && s->a2i_stat;

    if (s->ints) {
        qemu_set_irq(s->irq[I2C_INTERRUPT], 1);
    } else {
        qemu_set_irq(s->irq[I2C_INTERRUPT], 0);
    }
}

static void update_a2i_irq(CoreVI2csState *s)
{
    s->ints2a = 0;
    if (s->a2i_write_flags_select & (1 << s->a2i_write_flag)) {
        s->ints2a |= s->ints2a_en & 0x4;
    }

    if (s->a2i_read_flags_select & (1 << s->a2i_read_flag)) {
        s->ints2a |= s->ints2a_en & 0x2;
    }

    s->ints2a |= (s->ints2a_en & 0x1) && s->i2a_stat;

    if (s->ints2a) {
        qemu_set_irq(s->irq[APB_INTERRUPT], 1);
    } else {
        qemu_set_irq(s->irq[APB_INTERRUPT], 0);
    }
}

static uint64_t corev_i2cs_read(void *opaque, hwaddr addr,
                              unsigned int size)
{
    CoreVI2csState *s = opaque;
    uint64_t ret = 0;

    switch (addr >> 2) {
    case R_ADDRESS:
        ret = s->address;
        break;
    case R_ENABLE:
        ret = s->enable;
        break;
    case R_DEBOUNCE_LEN:
        ret = s->debounce_len;
        break;
    case R_SCL_DELAY_LEN:
        ret = s->scl_delay_len;
        break;
    case R_SDA_DELAY_LEN:
        ret = s->sda_delay_len;
        break;
    case R_I2A:
        s->i2a_stat = 0;
        ret = s->i2a;
        break;
    case R_I2A_STAT:
        ret = s->i2a_stat;
        break;
    case R_A2I:
        ret = s->a2i;
        break;
    case R_A2I_STAT:
        ret = s->a2i_stat;
        break;
    case R_I2A_READ_DATA_PORT:
        if (!fifo8_is_empty(&s->i2a_fifo)) {
            ret = fifo8_pop(&s->i2a_fifo);
            update_flags(&s->i2a_fifo, &s->i2a_write_flag, &s->i2a_read_flag);
            update_i2a_irq(s);
        }
        break;
    case R_I2A_FLUSH:
        ret = s->i2a_flush;
        break;
    case R_I2A_WRITE_FLAGS:
        ret = s->i2a_write_flag;
        break;
    case R_I2A_READ_FLAGS:
        ret = s->i2a_read_flag;
        break;
    case R_A2I_FLUSH:
        ret = s->a2i_flush;
        break;
    case R_A2I_WRITE_FLAGS:
        ret = s->a2i_write_flag;
        break;
    case R_A2I_READ_FLAGS:
        ret = s->a2i_read_flag;
        break;
    case R_INTS:
        ret = s->ints;
        break;
    case R_INTS_EN:
        ret = s->ints_en;
        break;
    case R_I2A_WRITE_FLAGS_SELECT:
        ret = s->i2a_write_flags_select;
        break;
    case R_I2A_READ_FLAGS_SELECT:
        ret = s->i2a_read_flags_select;
        break;
    case R_INTS2A:
        ret = s->ints2a;
        break;
    case R_INTS2A_EN:
        ret = s->ints2a_en;
        break;
    case R_A2I_WRITE_FLAGS_SELECT:
        ret = s->a2i_write_flags_select;
        break;
    case R_A2I_READ_FLAGS_SELECT:
        ret = s->a2i_read_flags_select;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }
    DEBUG_I2CS_READ(printf("i2cs read %x, %x\n", (uint32_t)addr >> 2,
                           (uint32_t)ret);)
    return ret;
}

static void corev_i2cs_write(void *opaque, hwaddr addr,
                            uint64_t val64, unsigned int size)
{
    CoreVI2csState *s = opaque;
    uint32_t value = val64;
    DEBUG_I2CS_WRITE(printf("i2cs write %x, %x\n", (uint32_t)addr >> 2,
                            value);)

    switch (addr >> 2) {
    case R_ADDRESS:
        s->address = value & 0x7F;
        if (s->i2c != NULL) {
            i2c_slave_set_address(s->i2c, s->address);
        }
        break;
    case R_ENABLE:
        s->enable = value & 0x1;
        break;
    case R_DEBOUNCE_LEN:
        s->debounce_len = value;
        break;
    case R_SCL_DELAY_LEN:
        s->scl_delay_len = value;
        break;
    case R_SDA_DELAY_LEN:
        s->sda_delay_len = value;
        break;
    case R_A2I:
        s->a2i = value;
        s->a2i_stat = true;
        update_i2a_irq(s);
        break;
    case R_I2A_FLUSH:
        s->i2a_flush = value & 1;
        if (s->i2a_flush) {
            fifo8_reset(&s->i2a_fifo);
            update_flags(&s->i2a_fifo, &s->i2a_write_flag, &s->i2a_read_flag);
            update_i2a_irq(s);
        }
        break;
    case R_A2I_WRITE_DATA_PORT:
        /* Receive overruns do not overwrite FIFO contents. */
        if (!fifo8_is_full(&s->a2i_fifo)) {
            fifo8_push(&s->a2i_fifo, value);
            update_flags(&s->a2i_fifo, &s->a2i_write_flag, &s->a2i_read_flag);
            update_a2i_irq(s);
        }
        break;
    case R_A2I_FLUSH:
        s->a2i_flush = value & 1;
        if (s->a2i_flush) {
            fifo8_reset(&s->a2i_fifo);
            update_flags(&s->a2i_fifo, &s->a2i_write_flag, &s->a2i_read_flag);
            update_a2i_irq(s);
        }
        break;
    case R_INTS2A_EN:
        s->ints2a_en = value & 0x7;
        update_a2i_irq(s);
        break;
    case R_A2I_WRITE_FLAGS_SELECT:
        s->a2i_write_flags_select = value;
        update_a2i_irq(s);
        break;
    case R_A2I_READ_FLAGS_SELECT:
        s->a2i_read_flags_select = value;
        update_a2i_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps corev_i2cs_ops = {
    .read = corev_i2cs_read,
    .write = corev_i2cs_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 1,
    .impl.max_access_size = 4,
};


static void corev_i2cs_init(Object *obj)
{
    CoreVI2csState *s = CORE_V_I2CS(obj);

    memory_region_init_io(&s->iomem, obj, &corev_i2cs_ops, s,
                          TYPE_CORE_V_I2CS, 0x1000);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq, 3);
}

static void corev_i2cs_realize(DeviceState *dev, Error **errp)
{
    CoreVI2csState *s = CORE_V_I2CS(dev);

    s->i2c = NULL;
    fifo8_create(&s->i2a_fifo, 256);
    fifo8_create(&s->a2i_fifo, 256);
}

static void corev_i2cs_reset(DeviceState *dev)
{
    CoreVI2csState *s = CORE_V_I2CS(dev);
    s->address = DEFAULT_I2CS_ADDRESS;
    s->debounce_len = 20;
    s->scl_delay_len = 20;
    s->sda_delay_len = 8;
    fifo8_reset(&s->a2i_fifo);
    fifo8_reset(&s->i2a_fifo);
    s->a2i_write_flag = 0;
    s->a2i_read_flag = 0;
    s->i2a_write_flag = 0;
    s->i2a_read_flag = 0;
}

static const VMStateDescription vmstate_corev_i2cs = {
    .name = TYPE_CORE_V_I2CS,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_UINT8(address, CoreVI2csState),
        VMSTATE_BOOL(enable, CoreVI2csState),
        VMSTATE_UINT8(debounce_len, CoreVI2csState),
        VMSTATE_UINT8(scl_delay_len, CoreVI2csState),
        VMSTATE_UINT8(sda_delay_len, CoreVI2csState),
        VMSTATE_UINT8(i2a, CoreVI2csState),
        VMSTATE_BOOL(i2a_stat, CoreVI2csState),
        VMSTATE_UINT8(a2i, CoreVI2csState),
        VMSTATE_BOOL(a2i_stat, CoreVI2csState),
        VMSTATE_BOOL(i2a_flush, CoreVI2csState),
        VMSTATE_UINT8(i2a_write_flag, CoreVI2csState),
        VMSTATE_UINT8(i2a_read_flag, CoreVI2csState),
        VMSTATE_BOOL(a2i_flush, CoreVI2csState),
        VMSTATE_UINT8(a2i_write_flag, CoreVI2csState),
        VMSTATE_UINT8(a2i_read_flag, CoreVI2csState),
        VMSTATE_UINT8(ints, CoreVI2csState),
        VMSTATE_UINT8(ints_en, CoreVI2csState),
        VMSTATE_UINT8(i2a_write_flags_select, CoreVI2csState),
        VMSTATE_UINT8(i2a_read_flags_select, CoreVI2csState),
        VMSTATE_UINT8(ints2a, CoreVI2csState),
        VMSTATE_UINT8(ints2a_en, CoreVI2csState),
        VMSTATE_UINT8(a2i_write_flags_select, CoreVI2csState),
        VMSTATE_UINT8(a2i_read_flags_select, CoreVI2csState),

        VMSTATE_FIFO8(i2a_fifo, CoreVI2csState),
        VMSTATE_FIFO8(a2i_fifo, CoreVI2csState),
        VMSTATE_STRUCT_POINTER(i2c, CoreVI2csState, vmstate_i2c_slave,
                               I2CSlave),
        VMSTATE_UINT8(offset, CoreVI2csState),
        VMSTATE_END_OF_LIST()
    }
};

static void corev_i2cs_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = corev_i2cs_reset;
    dc->realize = corev_i2cs_realize;
    dc->vmsd = &vmstate_corev_i2cs;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo corev_i2cs_info = {
    .name          = TYPE_CORE_V_I2CS,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVI2csState),
    .instance_init = corev_i2cs_init,
    .class_init    = corev_i2cs_class_init,
};

static uint8_t i2cs_read_reg(CoreVI2csState *s)
{
    uint8_t ret = 0;

    switch (s->offset) {
    case R_ADDRESS:
        ret = s->address;
        break;
    case R_ENABLE:
        ret = s->enable;
        break;
    case R_DEBOUNCE_LEN:
        ret = s->debounce_len;
        break;
    case R_SCL_DELAY_LEN:
        ret = s->scl_delay_len;
        break;
    case R_SDA_DELAY_LEN:
        ret = s->sda_delay_len;
        break;
    case R_I2A:
        ret = s->i2a;
        break;
    case R_I2A_STAT:
        ret = s->i2a_stat;
        break;
    case R_A2I:
        ret = s->a2i;
        s->a2i_stat = false;
        break;
    case R_A2I_STAT:
        ret = s->a2i_stat;
        break;
    case R_I2A_FLUSH:
        ret = s->i2a_flush;
        break;
    case R_I2A_WRITE_FLAGS:
        ret = s->i2a_write_flag;
        break;
    case R_I2A_READ_FLAGS:
        ret = s->i2a_read_flag;
        break;
    case R_A2I_READ_DATA_PORT:
        if (!fifo8_is_empty(&s->a2i_fifo)) {
            ret = fifo8_pop(&s->a2i_fifo);
            update_flags(&s->a2i_fifo, &s->a2i_write_flag, &s->a2i_read_flag);
            update_a2i_irq(s);
        }
        break;
    case R_A2I_FLUSH:
        ret = s->a2i_flush;
        break;
    case R_A2I_WRITE_FLAGS:
        ret = s->a2i_write_flag;
        break;
    case R_A2I_READ_FLAGS:
        ret = s->a2i_read_flag;
        break;
    case R_INTS:
        ret = s->ints;
        break;
    case R_INTS_EN:
        ret = s->ints_en;
        break;
    case R_I2A_WRITE_FLAGS_SELECT:
        ret = s->i2a_write_flags_select;
        break;
    case R_I2A_READ_FLAGS_SELECT:
        ret = s->i2a_read_flags_select;
        break;
    case R_INTS2A:
        ret = s->ints2a;
        break;
    case R_INTS2A_EN:
        ret = s->ints2a_en;
        break;
    case R_A2I_WRITE_FLAGS_SELECT:
        ret = s->a2i_write_flags_select;
        break;
    case R_A2I_READ_FLAGS_SELECT:
        ret = s->a2i_read_flags_select;
        break;
    default:
        ret = 0;
        break;
    }
    return ret;
}

static uint8_t i2cs_write_reg(CoreVI2csState *s, uint8_t value)
{
    uint8_t ret = 0;

    switch (s->offset) {
    case R_I2A:
        s->i2a = value;
        s->i2a_stat = true;
        update_a2i_irq(s);
        break;
    case R_I2A_WRITE_DATA_PORT:
        /* Receive overruns do not overwrite FIFO contents. */
        if (!fifo8_is_full(&s->i2a_fifo)) {
            fifo8_push(&s->i2a_fifo, value);
            update_flags(&s->i2a_fifo, &s->i2a_write_flag, &s->i2a_read_flag);
            update_i2a_irq(s);
        }
        break;
    case R_I2A_FLUSH:
        s->i2a_flush = value & 0x1;
        if (s->i2a_flush) {
            fifo8_reset(&s->i2a_fifo);
            update_flags(&s->i2a_fifo, &s->i2a_write_flag, &s->i2a_read_flag);
            update_i2a_irq(s);
        }
        break;
    case R_A2I_FLUSH:
        s->a2i_flush = value & 0x1;
        if (s->a2i_flush) {
            fifo8_reset(&s->a2i_fifo);
            update_flags(&s->a2i_fifo, &s->a2i_write_flag, &s->a2i_read_flag);
            update_a2i_irq(s);
        }
        break;
    case R_INTS_EN:
        s->ints_en = value & 0x7;
        update_i2a_irq(s);
        break;
    case R_I2A_WRITE_FLAGS_SELECT:
        s->i2a_write_flags_select = value;
        update_i2a_irq(s);
        break;
    case R_I2A_READ_FLAGS_SELECT:
        s->i2a_read_flags_select = value;
        update_i2a_irq(s);
        break;
    default:
        break;
    }
    return ret;
}

static uint8_t i2cs_receive_byte(SMBusDevice *dev)
{
    CoreVI2csState *s = CORE_V_SMBUS_SLAVE(dev)->state;

    uint8_t val = i2cs_read_reg(s);
    return val;
}

static int i2cs_write_data(SMBusDevice *dev, uint8_t *buf, uint8_t len)
{
    CoreVI2csState *s = CORE_V_SMBUS_SLAVE(dev)->state;

    /* len is guaranteed to be > 0 */
    s->offset = buf[0];
    buf++;
    len--;

    for (; len > 0; len--) {
        i2cs_write_reg(s, *buf++);
    }

    return 0;
}

static void smbus_i2cs_realize(DeviceState *dev, Error **errp)
{
}

static const VMStateDescription vmstate_smbus_i2cs = {
    .name = "smbus-i2cs",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields      = (VMStateField[]) {
        VMSTATE_SMBUS_DEVICE(i2c, CoreVI2cSlaveDevice),
        VMSTATE_STRUCT_POINTER(state, CoreVI2cSlaveDevice, vmstate_corev_i2cs,
                               CoreVI2csState),
        VMSTATE_END_OF_LIST()
    }
};

static void smbus_i2cs_class_initfn(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);
    SMBusDeviceClass *sc = SMBUS_DEVICE_CLASS(klass);

    dc->realize = smbus_i2cs_realize;
    sc->receive_byte = i2cs_receive_byte;
    sc->write_data = i2cs_write_data;
    dc->vmsd = &vmstate_smbus_i2cs;
    /* Reason: init_data */
    dc->user_creatable = false;
}

static const TypeInfo smbus_i2cs_info = {
    .name          = TYPE_CORE_V_SMBUS_SLAVE,
    .parent        = TYPE_SMBUS_DEVICE,
    .instance_size = sizeof(CoreVI2cSlaveDevice),
    .class_init    = smbus_i2cs_class_initfn,
};

static void corev_i2cs_register_types(void)
{
    type_register_static(&corev_i2cs_info);
    type_register_static(&smbus_i2cs_info);
}

type_init(corev_i2cs_register_types)

void corev_i2cs_init_one(I2CBus *smbus, uint8_t address,
                         CoreVI2csState *state)
{
    DeviceState *dev;

    dev = qdev_new(TYPE_CORE_V_SMBUS_SLAVE);
    qdev_prop_set_uint8(dev, "address", address);
    qdev_realize_and_unref(dev, (BusState *)smbus, &error_fatal);

    CoreVI2cSlaveDevice *s = CORE_V_SMBUS_SLAVE(dev);
    s->state = state;
    state->i2c = I2C_SLAVE(dev);
}
