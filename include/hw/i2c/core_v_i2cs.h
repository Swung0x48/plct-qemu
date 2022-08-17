/*
 * CORE-V MCU I2C Slave Device
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

#ifndef HW_CORE_V_I2CS_H
#define HW_CORE_V_I2CS_H

#include "hw/sysbus.h"
#include "qom/object.h"
#include "hw/i2c/i2c.h"
#include "hw/i2c/smbus_slave.h"
#include "qemu/fifo8.h"

#define DEFAULT_I2CS_ADDRESS 0x6F

#define TYPE_CORE_V_I2CS "core-v.i2cs"
OBJECT_DECLARE_SIMPLE_TYPE(CoreVI2csState, CORE_V_I2CS)

struct CoreVI2csState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;
    uint8_t address;          /* 0x00 */
    bool enable;              /* 0x01 */
    uint8_t debounce_len;     /* 0x02 */
    uint8_t scl_delay_len;    /* 0x03 */
    uint8_t sda_delay_len;    /* 0x04 */
    uint8_t i2a;              /* 0x10 */
    bool i2a_stat;            /* 0x11 */
    uint8_t a2i;              /* 0x12 */
    bool a2i_stat;            /* 0x13 */

    bool i2a_flush;               /* 0x22 */
    uint8_t i2a_write_flag;       /* 0x23 */
    uint8_t i2a_read_flag;        /* 0x24 */

    bool a2i_flush;               /* 0x32 */
    uint8_t a2i_write_flag;       /* 0x33 */
    uint8_t a2i_read_flag;        /* 0x34 */
    uint8_t ints;                 /* 0x40 */
    uint8_t ints_en;              /* 0x41 */
    uint8_t i2a_write_flags_select; /* 0x42 */
    uint8_t i2a_read_flags_select;  /* 0x43 */
    uint8_t ints2a;                 /* 0x50 */
    uint8_t ints2a_en;              /* 0x51 */
    uint8_t a2i_write_flags_select; /* 0x52 */
    uint8_t a2i_read_flags_select;  /* 0x53 */

    Fifo8 i2a_fifo;
    Fifo8 a2i_fifo;

    I2CSlave *i2c;

    uint8_t offset;
    qemu_irq irq[2];
};

struct CoreVI2cSlaveDevice {
    SMBusDevice i2c;
    CoreVI2csState *state;
};

#define TYPE_CORE_V_SMBUS_SLAVE "core-v.smbus.slave"
OBJECT_DECLARE_SIMPLE_TYPE(CoreVI2cSlaveDevice, CORE_V_SMBUS_SLAVE)

#define I2C_INTERRUPT 0
#define APB_INTERRUPT 1

void corev_i2cs_init_one(I2CBus *smbus, uint8_t address,
                         CoreVI2csState *state);
#endif /* HW_CORE_V_I2CS_H */
