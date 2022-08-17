/*
 * QEMU CORE-V MCU UDMA I2C device
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
#include "hw/sysbus.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/i2c/core_v_udma_i2c.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "trace.h"
#include "hw/registerfields.h"
#include "hw/i2c/smbus_eeprom.h"

#define DEBUG_I2C(X)
#define DEBUG_I2C_SLATE

REG32(RX_SADDR, 0x00)
REG32(RX_SIZE, 0x04)
REG32(RX_CFG, 0x08)
    FIELD(RX_CFG, CONTINUE, 0, 1)
    FIELD(RX_CFG, EN, 4, 1)
    FIELD(RX_CFG, PENDING, 5, 1)
    FIELD(RX_CFG, CLR, 6, 1)
REG32(TX_SADDR, 0x10)
REG32(TX_SIZE, 0x14)
REG32(TX_CFG, 0x18)
    FIELD(TX_CFG, CONTINUE, 0, 1)
    FIELD(TX_CFG, EN, 4, 1)
    FIELD(TX_CFG, PENDING, 5, 1)
    FIELD(TX_CFG, CLR, 6, 1)
REG32(STATUS, 0x20)
    FIELD(STATUS, BUSY, 0, 1)
    FIELD(STATUS, alarm, 1, 1)
REG32(SETUP, 0x24)
    FIELD(SETUP, DO_RESET, 0, 1)

static bool dma_rx_transfer_byte(CoreVUDMAI2cState *s, uint8_t byte)
{
    if ((s->rx_cfg & R_RX_CFG_EN_MASK) && s->rx_cur_size) {
        cpu_physical_memory_write(s->rx_cur_saddr, &byte, 1);
        s->rx_cur_size--;
        s->rx_cur_saddr++;
        if (s->rx_cur_size == 0) {
            qemu_irq_raise(s->irq[I2C_IRQ_CH_RX]);
            if (s->rx_cfg & R_RX_CFG_CONTINUE_MASK) {
                s->rx_cur_size = s->rx_size;
                s->rx_cur_saddr = s->rx_saddr;
            } else {
                s->rx_cfg &= ~R_RX_CFG_EN_MASK;
            }
        }
        return true;
    }
    return false;
}

static bool dma_tx_transfer_byte(CoreVUDMAI2cState *s, uint8_t* cmd)
{
    if ((s->tx_cfg & R_TX_CFG_EN_MASK) && s->tx_cur_size) {
        cpu_physical_memory_read(s->tx_cur_saddr, cmd, 1);
        s->tx_cur_size--;
        s->tx_cur_saddr++;
        if (s->tx_cur_size == 0) {
            qemu_irq_raise(s->irq[I2C_IRQ_CH_TX]);
            if (s->tx_cfg & R_RX_CFG_CONTINUE_MASK) {
                s->tx_cur_size = s->tx_size;
                s->tx_cur_saddr = s->tx_saddr;
            } else {
                s->tx_cfg &= ~R_TX_CFG_EN_MASK;
            }
        }
        return true;
    }
    return false;
}

static void handle_i2c_cmd(CoreVUDMAI2cState *s)
{
    uint8_t data = 0;

    do {
        switch (s->cmd_status) {
        case I2C_ST_WAIT_IN_CMD:
            if (!dma_tx_transfer_byte(s, &data)) {
                return;
            }
            switch (data >> 4) {
            case I2C_CMD_START:
                s->cmd_status = I2C_ST_CMD_DONE;
                s->start_transfer = false;
                break;
            case I2C_CMD_STOP:
                if (s->start_transfer) {
                    i2c_end_transfer(s->bus);
                }
                s->start_transfer = false;
                s->cmd_status = I2C_ST_CMD_DONE;
                break;
            case I2C_CMD_RD_NACK:
                s->is_nack = true;
                /* FALL THROUGH */
            case I2C_CMD_RD_ACK:
                s->cmd_status = I2C_ST_READ;
                break;
            case I2C_CMD_WR:
                s->cmd_status = I2C_ST_GET_DATA;
                break;
            case I2C_CMD_WAIT:
                s->cmd_status = I2C_ST_GET_WAIT;
                break;
            case I2C_CMD_RPT:
                s->cmd_status = I2C_ST_GET_RPT;
                break;
            case I2C_CMD_CFG:
                s->cmd_status = I2C_ST_GET_CFG_MSB;
                break;
            case I2C_CMD_WAIT_EV:
                s->event_select = data & 3;
                s->cmd_status = I2C_ST_WAIT_EV;
                break;
            }
            break;
        case I2C_ST_WAIT_EV:
            if (s->trigger_event[s->event_select]) {
                s->trigger_event[s->event_select] = false;
                s->cmd_status = I2C_ST_WAIT_IN_CMD;
            } else {
                return;
            }
            break;
        case I2C_ST_CMD_DONE:
            s->cmd_status = I2C_ST_WAIT_IN_CMD;
            break;
        case I2C_ST_READ:
            g_assert(s->start_transfer);
            s->recv_data = i2c_recv(s->bus);
            if (s->is_nack) {
                i2c_nack(s->bus);
                s->is_nack = false;
                if (s->start_transfer) {
                    i2c_end_transfer(s->bus);
                    s->start_transfer = false;
                }
            }
            s->cmd_status = I2C_ST_STORE_DATA;
            break;
        case I2C_ST_GET_DATA: /* FALL THROUGH */
        case I2C_ST_WRITE:
            if (!s->start_transfer) {
                if (dma_tx_transfer_byte(s, &data)) {
                    if (!i2c_start_transfer(s->bus, data >> 1, data & 1)) {
                        s->start_transfer = true;
                        if (s->repeat_cmd) {
                            s->repeat_cmd--;
                        } else {
                            s->cmd_status = I2C_ST_WAIT_IN_CMD;
                        }
                        return;
                    } else {
                        qemu_log_mask(LOG_GUEST_ERROR,
                                     "requesting i2c bus for 0x%02x failed\n",
                                      data >> 1);
                        return;
                    }
                } else {
                    return;
                }
            }

            if (dma_tx_transfer_byte(s, &data)) {
                if (i2c_send(s->bus, data)) {
                    i2c_end_transfer(s->bus);
                    s->start_transfer = false;
                }
            } else {
                return;
            }

            if (s->repeat_cmd) {
                s->repeat_cmd--;
            } else {
                s->cmd_status = I2C_ST_WAIT_IN_CMD;
            }
            break;
        case I2C_ST_STORE_DATA:
            if (dma_rx_transfer_byte(s, s->recv_data)) {
                if (s->repeat_cmd == 0) {
                    s->cmd_status = I2C_ST_WAIT_IN_CMD;
                } else {
                    s->repeat_cmd--;
                    s->cmd_status = I2C_ST_READ;
                }
            } else {
                return;
            }
            break;
        case I2C_ST_SKIP_CMD:
            s->cmd_status = I2C_ST_WAIT_IN_CMD;
            break;
        case I2C_ST_GET_WAIT:
            if (dma_tx_transfer_byte(s, &data)) {
                /* do nothing for wait cmd */
                s->cmd_status = I2C_ST_WAIT_IN_CMD;
            } else {
                return;
            }
            break;
        case I2C_ST_GET_RPT:
            if (dma_tx_transfer_byte(s, &data)) {
                if (data == 0) {
                    s->repeat_cmd = 0;
                    s->cmd_status = I2C_ST_SKIP_CMD;
                } else {
                    s->repeat_cmd = data - 1;
                    s->cmd_status = I2C_ST_WAIT_IN_CMD;
                }
            } else {
                return;
            }
            break;
        case I2C_ST_GET_CFG_MSB:
            /* current do nothing for clock divide num */
            if (!dma_tx_transfer_byte(s, &data)) {
                return;
            }
            /* FALL THROUGH */
        case I2C_ST_GET_CFG_LSB:
            if (!dma_tx_transfer_byte(s, &data)) {
                return;
            }
            s->cmd_status = I2C_ST_WAIT_IN_CMD;
            break;
        default:
            break;
        }
    } while (s->cmd_status != I2C_ST_WAIT_IN_CMD);
}

static void update_timer(CoreVUDMAI2cState *s)
{
    timer_del(s->dma_handle);
    if (s->enable && ((s->tx_cfg & R_TX_CFG_EN_MASK) ||
                      (s->rx_cfg & R_RX_CFG_EN_MASK))) {
        timer_mod(s->dma_handle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                 (NANOSECONDS_PER_SECOND / 100000));
    }
}

static uint64_t udma_i2c_read(void *opaque, hwaddr addr,
                              unsigned int size)
{
    CoreVUDMAI2cState *s = opaque;
    uint64_t ret = 0;

    if (!s->enable) {
        return 0;
    }

    switch (addr >> 2) {
    case R_RX_SADDR:
        ret = s->rx_cur_saddr;
        break;
    case R_RX_SIZE:
        ret = s->rx_cur_size;
        break;
    case R_RX_CFG:
        ret = s->rx_cfg;
        break;
    case R_TX_SADDR:
        ret = s->tx_cur_saddr;
        break;
    case R_TX_SIZE:
        ret = s->tx_cur_size;
        break;
    case R_TX_CFG:
        ret = s->tx_cfg;
        break;
    case R_STATUS:
        ret = s->status;
        s->status = 0;
        break;
    case R_SETUP:
        ret = s->setup;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }
    DEBUG_I2C(printf("udma i2c read %d, %x\n", (uint32_t)addr >> 2,
              (uint32_t)ret);)
    return ret;
}

static void reset_status(CoreVUDMAI2cState *s)
{
    s->cmd_status = I2C_ST_WAIT_IN_CMD;
    s->event_select = 0;
    s->repeat_cmd = 0;
    s->recv_data = 0;
    s->is_nack = false;

    if (s->start_transfer) {
        i2c_end_transfer(s->bus);
        s->start_transfer = false;
    }
}

static void udma_i2c_write(void *opaque, hwaddr addr,
                            uint64_t val64, unsigned int size)
{
    CoreVUDMAI2cState *s = opaque;
    uint32_t value = val64;
    DEBUG_I2C(printf("udma i2c write %d, %x\n", (uint32_t)addr >> 2, value);)
    if (!s->enable) {
        return;
    }

    switch (addr >> 2) {
    case R_RX_SADDR:
        s->rx_saddr = value;
        break;
    case R_RX_SIZE:
        s->rx_size = value;
        break;
    case R_RX_CFG:
        if (!(s->rx_cfg & R_RX_CFG_EN_MASK) && (value & R_RX_CFG_EN_MASK)) {
            s->rx_cur_saddr = s->rx_saddr;
            s->rx_cur_size = s->rx_size;
        }
        if (s->rx_cfg & R_RX_CFG_CLR_MASK) {
            s->rx_cur_saddr = 0;
            s->rx_cur_size = 0;
            s->rx_cfg = value & R_RX_CFG_CONTINUE_MASK;
        } else {
            s->rx_cfg = value & ~R_RX_CFG_CLR_MASK;
        }
        update_timer(s);
        break;
    case R_TX_SADDR:
        s->tx_saddr = value;
        break;
    case R_TX_SIZE:
        s->tx_size = value;
        break;
    case R_TX_CFG:
        if (!(s->tx_cfg & R_TX_CFG_EN_MASK) && (value & R_TX_CFG_EN_MASK)) {
            s->tx_cur_saddr = s->tx_saddr;
            s->tx_cur_size = s->tx_size;
        }
        if (s->tx_cfg & R_TX_CFG_CLR_MASK) {
            s->tx_cur_saddr = 0;
            s->tx_cur_size = 0;
            s->tx_cfg = value & R_TX_CFG_CONTINUE_MASK;
        } else {
            s->tx_cfg = value & ~R_TX_CFG_CLR_MASK;
        }
        update_timer(s);
        break;
    case R_SETUP:
        if (value & R_SETUP_DO_RESET_MASK) {
            reset_status(s);
        }
        update_timer(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static void trigger_dma_transfer(void *opaque)
{
    CoreVUDMAI2cState *s = opaque;
    handle_i2c_cmd(s);
    update_timer(s);
}

static const MemoryRegionOps udma_i2c_ops = {
    .read = udma_i2c_read,
    .write = udma_i2c_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_udma_i2c = {
    .name = TYPE_CORE_V_UDMA_I2C,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enable, CoreVUDMAI2cState),
        VMSTATE_TIMER_PTR(dma_handle, CoreVUDMAI2cState),
        VMSTATE_UINT32(rx_saddr, CoreVUDMAI2cState),
        VMSTATE_UINT32(rx_size, CoreVUDMAI2cState),
        VMSTATE_UINT32(rx_cfg, CoreVUDMAI2cState),
        VMSTATE_UINT32(rx_cur_saddr, CoreVUDMAI2cState),
        VMSTATE_UINT32(rx_cur_size, CoreVUDMAI2cState),
        VMSTATE_UINT32(tx_saddr, CoreVUDMAI2cState),
        VMSTATE_UINT32(tx_size, CoreVUDMAI2cState),
        VMSTATE_UINT32(tx_cfg, CoreVUDMAI2cState),
        VMSTATE_UINT32(tx_cur_saddr, CoreVUDMAI2cState),
        VMSTATE_UINT32(tx_cur_size, CoreVUDMAI2cState),
        VMSTATE_UINT32(status, CoreVUDMAI2cState),
        VMSTATE_UINT32(setup, CoreVUDMAI2cState),
        VMSTATE_UINT8(event_select, CoreVUDMAI2cState),
        VMSTATE_UINT8(cmd_status, CoreVUDMAI2cState),
        VMSTATE_UINT8(repeat_cmd, CoreVUDMAI2cState),
        VMSTATE_BOOL(is_nack, CoreVUDMAI2cState),
        VMSTATE_BOOL(start_transfer, CoreVUDMAI2cState),
        VMSTATE_BOOL_ARRAY(trigger_event, CoreVUDMAI2cState,
                           UDMA_CTRL_EVENT_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_udma_ctrl_event(void *opaque, int num, int level)
{
    CoreVUDMAI2cState *s = opaque;
    if (num == UDMA_CTRL_EVENT_NUM) {
        s->enable = level;
        if (s->enable) {
            update_timer(s);
        }
    } else {
        s->trigger_event[num] = level;
    }
}

static void udma_i2c_init(Object *obj)
{
    CoreVUDMAI2cState *s = CORE_V_UDMA_I2C(obj);

    memory_region_init_io(&s->iomem, obj, &udma_i2c_ops, s,
                          TYPE_CORE_V_UDMA_I2C, UDMA_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq, UDMA_PER_EVENT_NUM);
    qdev_init_gpio_in(DEVICE(obj), core_v_udma_ctrl_event,
                      UDMA_CTRL_EVENT_NUM + 1);
}

static void udma_i2c_realize(DeviceState *dev, Error **errp)
{
    CoreVUDMAI2cState *s = CORE_V_UDMA_I2C(dev);

    s->dma_handle = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 trigger_dma_transfer, s);
    s->bus = i2c_init_bus(dev, "udma_i2c_bus");
#ifdef DEBUG_I2C_SLATE
    uint8_t *eeprom_buf = g_malloc0(32 * 1024);
    smbus_eeprom_init_one(s->bus, 0x6F, eeprom_buf);
#endif
}

static void udma_i2c_reset(DeviceState *dev)
{
    CoreVUDMAI2cState *s = CORE_V_UDMA_I2C(dev);
    int i;

    s->rx_saddr = 0;
    s->rx_size = 0;
    s->rx_cfg = 0;
    s->tx_saddr = 0;
    s->tx_size = 0;
    s->tx_cfg = 0;
    s->status = 0;
    s->setup = 0;
    s->rx_cur_saddr = 0;
    s->rx_cur_size = 0;
    s->tx_cur_saddr = 0;
    s->tx_cur_size = 0;
    s->enable = false;

    for (i = 0; i < UDMA_CTRL_EVENT_NUM; i++) {
        s->trigger_event[i] = false;
    }
    reset_status(s);
}

static void udma_i2c_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = udma_i2c_reset;
    dc->realize = udma_i2c_realize;
    dc->vmsd = &vmstate_udma_i2c;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo udma_i2c_info = {
    .name          = TYPE_CORE_V_UDMA_I2C,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVUDMAI2cState),
    .instance_init = udma_i2c_init,
    .class_init    = udma_i2c_class_init,
};

static void udma_i2c_register_types(void)
{
    type_register_static(&udma_i2c_info);
}

type_init(udma_i2c_register_types)
