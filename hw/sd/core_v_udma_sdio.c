/*
 * QEMU CORE-V MCU UDMA SDIO
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
#include "hw/sd/core_v_udma_sdio.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "trace.h"
#include "hw/registerfields.h"

#define DEBUG_SDIO_EXEC(X)
#define DEBUG_SDIO_SDCARD

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
REG32(CMD_OP, 0x20)
    FIELD(CMD_OP, RSP, 0, 3)
    FIELD(CMD_OP, OP, 8, 6)
REG32(CMD_ARG, 0x24)
REG32(DATA_SETUP, 0x28)
    FIELD(DATA_SETUP, EN, 0, 1)
    FIELD(DATA_SETUP, RW, 1, 1)
    FIELD(DATA_SETUP, QUAD, 2, 1)
    FIELD(DATA_SETUP, BLOCK_NUM, 8, 8)
    FIELD(DATA_SETUP, BLOCK_SIZE, 16, 10)
REG32(START, 0x2C)
REG32(RESP0, 0x30)
REG32(RESP1, 0x34)
REG32(RESP2, 0x38)
REG32(RESP3, 0x3C)
REG32(CLK_DIV, 0x40)
    FIELD(CLK_DIV, DATA, 0, 8)
    FIELD(CLK_DIV, VALID, 8, 1)
REG32(STATUS, 0x44)
    FIELD(STATUS, EOT, 0, 1)
    FIELD(STATUS, ERR, 1, 1)
    FIELD(STATUS, RSP_TIMEOUT, 16, 1)
    FIELD(STATUS, RSP_WRONG_RSP, 17, 1)
    FIELD(STATUS, RSP_BUSY_TIMEOUT, 18, 1)
    FIELD(STATUS, DATA_RSP_TIMEOUT, 24, 1)

static void dma_rx_transfer(CoreVUDMASdioState *s)
{
    if ((s->rx_cfg & R_RX_CFG_EN_MASK) && sdbus_data_ready(&s->bus)) {
        uint32_t size = s->block_size * s->block_num;
        while (size) {
            uint8_t data = sdbus_read_byte(&s->bus);
            if (s->rx_cur_size) {
                cpu_physical_memory_write(s->rx_cur_saddr, &data, 1);
                s->rx_cur_size -= 1;
                s->rx_cur_saddr += 1;
            } else {
                s->status |= R_STATUS_DATA_RSP_TIMEOUT_MASK | R_STATUS_ERR_MASK;
                return;
            }
            size--;
            if (s->rx_cur_size == 0) {
                qemu_irq_raise(s->irq[SDIO_IRQ_CH_RX]);
                if (s->rx_cfg & R_RX_CFG_CONTINUE_MASK) {
                    s->rx_cur_size = s->rx_size;
                    s->rx_cur_saddr = s->rx_saddr;
                } else {
                    s->rx_cfg &= ~R_RX_CFG_EN_MASK;
                }
            }
        }
        s->status |= R_STATUS_EOT_MASK;
    } else {
        s->status |= R_STATUS_DATA_RSP_TIMEOUT_MASK | R_STATUS_ERR_MASK;
    }
}

static void dma_tx_transfer(CoreVUDMASdioState *s)
{
    if (s->tx_cfg & R_TX_CFG_EN_MASK) {
        uint32_t size = s->block_size * s->block_num;
        while (size) {
            if (s->tx_cur_size) {
                uint8_t data;
                cpu_physical_memory_read(s->tx_cur_saddr, &data, 1);
                s->tx_cur_size--;
                s->tx_cur_saddr++;
                sdbus_write_byte(&s->bus, data);
                size--;
                if (s->tx_cur_size == 0) {
                    qemu_irq_raise(s->irq[SDIO_IRQ_CH_TX]);
                    if (s->tx_cfg & R_RX_CFG_CONTINUE_MASK) {
                        s->tx_cur_size = s->tx_size;
                        s->tx_cur_saddr = s->tx_saddr;
                    } else {
                        s->rx_cfg &= ~R_TX_CFG_EN_MASK;
                    }
                }
            } else {
                s->status |= R_STATUS_DATA_RSP_TIMEOUT_MASK | R_STATUS_ERR_MASK;
                return;
            }
        }
        s->status |= R_STATUS_EOT_MASK;
    } else {
        s->status |= R_STATUS_DATA_RSP_TIMEOUT_MASK | R_STATUS_ERR_MASK;
    }
}

static void handle_sdio_cmd(CoreVUDMASdioState *s)
{
    SDRequest request;
    uint8_t rsp[16];
    int rlen;

    /* clear previous status when start new cmd */
    s->status &= 0x03;
    request.cmd = s->cmd_op;
    request.arg = s->cmd_arg;

    rlen = sdbus_do_command(&s->bus, &request, rsp);
    if (rlen < 0) {
        goto error;
    }

    memset(s->resp, 0, 16);
    if (s->cmd_rsp != RSP_TYPE_NULL) {
        if (rlen == 0 || (rlen == 4 && (s->cmd_rsp == RSP_TYPE_136))) {
            goto error;
        }
        if (rlen != 4 && rlen != 16) {
            goto error;
        }
        if (rlen == 4) {
            s->resp[0] = ldl_be_p(&rsp[0]);
            s->resp[1] = s->resp[2] = s->resp[3] = 0;
        } else {
            s->resp[3] = ldl_be_p(&rsp[0]);
            s->resp[2] = ldl_be_p(&rsp[4]);
            s->resp[1] = ldl_be_p(&rsp[8]);
            s->resp[0] = ldl_be_p(&rsp[12]);
        }
    }

    DEBUG_SDIO_EXEC(printf("sdio cmd %x(%x): %d (%x %x %x %x)\n", s->cmd_op,
                           s->cmd_arg, rlen, s->resp[0], s->resp[1],
                           s->resp[2], s->resp[3]);)

    if (s->data_setup & R_DATA_SETUP_EN_MASK) {
        if (s->data_setup & R_DATA_SETUP_RW_MASK) {
            dma_rx_transfer(s);
        } else {
            dma_tx_transfer(s);
        }
    } else {
        s->status |= R_STATUS_EOT_MASK;
    }
    return;

error:
    s->status |= R_STATUS_RSP_TIMEOUT_MASK | R_STATUS_ERR_MASK;
}

static uint64_t udma_sdio_read(void *opaque, hwaddr addr,
                               unsigned int size)
{
    CoreVUDMASdioState *s = opaque;
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
    case R_RESP0:
    case R_RESP1:
    case R_RESP2:
    case R_RESP3:
        ret = s->resp[(addr >> 2) - R_RESP0];
        break;
    case R_CLK_DIV:
        ret = s->clk_div;
        break;
    case R_STATUS:
        ret = s->status;
        s->status = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }

    return ret;
}

static void update_irq(CoreVUDMASdioState *s)
{
    if (s->status & R_STATUS_EOT_MASK) {
        qemu_irq_raise(s->irq[SDIO_IRQ_EOT]);
    } else {
        qemu_irq_lower(s->irq[SDIO_IRQ_EOT]);
    }

    if (s->status & R_STATUS_ERR_MASK) {
        qemu_irq_raise(s->irq[SDIO_IRQ_ERR]);
    } else {
        qemu_irq_lower(s->irq[SDIO_IRQ_ERR]);
    }
}

static void udma_sdio_write(void *opaque, hwaddr addr,
                            uint64_t val64, unsigned int size)
{
    CoreVUDMASdioState *s = opaque;
    uint32_t value = val64;

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
        break;
    case R_CMD_OP:
        s->cmd_op = (value & R_CMD_OP_OP_MASK) >> R_CMD_OP_OP_SHIFT;
        s->cmd_rsp = (value & R_CMD_OP_RSP_MASK) >> R_CMD_OP_RSP_SHIFT;
        break;
    case R_CMD_ARG:
        s->cmd_arg = value;
        break;
    case R_DATA_SETUP:
        s->data_setup = value;
        s->block_size = ((value & R_DATA_SETUP_BLOCK_SIZE_MASK) >>
                         R_DATA_SETUP_BLOCK_SIZE_SHIFT) + 1;
        s->block_num = ((value & R_DATA_SETUP_BLOCK_NUM_MASK) >>
                        R_DATA_SETUP_BLOCK_NUM_SHIFT) + 1;
        break;
    case R_START:
        handle_sdio_cmd(s);
        update_irq(s);
        break;
    case R_CLK_DIV:
        s->clk_div = value;
        break;
    case R_STATUS:
        s->status &= ~(value & 0x3);
        update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps udma_sdio_ops = {
    .read = udma_sdio_read,
    .write = udma_sdio_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_udma_sdio = {
    .name = TYPE_CORE_V_UDMA_SDIO,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enable, CoreVUDMASdioState),
        VMSTATE_UINT32(rx_saddr, CoreVUDMASdioState),
        VMSTATE_UINT32(rx_cur_saddr, CoreVUDMASdioState),
        VMSTATE_UINT32(rx_size, CoreVUDMASdioState),
        VMSTATE_UINT32(rx_cur_size, CoreVUDMASdioState),
        VMSTATE_UINT32(rx_cfg, CoreVUDMASdioState),
        VMSTATE_UINT32(tx_saddr, CoreVUDMASdioState),
        VMSTATE_UINT32(tx_cur_saddr, CoreVUDMASdioState),
        VMSTATE_UINT32(tx_size, CoreVUDMASdioState),
        VMSTATE_UINT32(tx_cur_size, CoreVUDMASdioState),
        VMSTATE_UINT32(tx_cfg, CoreVUDMASdioState),
        VMSTATE_UINT8(cmd_op, CoreVUDMASdioState),
        VMSTATE_UINT8(cmd_rsp, CoreVUDMASdioState),
        VMSTATE_UINT32(cmd_arg, CoreVUDMASdioState),
        VMSTATE_UINT32(data_setup, CoreVUDMASdioState),
        VMSTATE_UINT32(block_num, CoreVUDMASdioState),
        VMSTATE_UINT32(block_size, CoreVUDMASdioState),
        VMSTATE_UINT32_ARRAY(resp, CoreVUDMASdioState, 4),
        VMSTATE_UINT32(clk_div, CoreVUDMASdioState),
        VMSTATE_UINT32(status, CoreVUDMASdioState),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_udma_ctrl_event(void *opaque, int num, int level)
{
    CoreVUDMASdioState *s = opaque;
    s->enable = level;
}

static void udma_sdio_init(Object *obj)
{
    CoreVUDMASdioState *s = CORE_V_UDMA_SDIO(obj);

    qbus_init(&s->bus, sizeof(s->bus),
              TYPE_CORE_V_UDMA_SDBUS, DEVICE(s), "sd-bus");

    memory_region_init_io(&s->iomem, obj, &udma_sdio_ops, s,
                          TYPE_CORE_V_UDMA_SDIO, UDMA_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq, UDMA_PER_EVENT_NUM);
    qdev_init_gpio_in(DEVICE(obj), core_v_udma_ctrl_event, 1);
}

static void udma_sdio_realize(DeviceState *dev, Error **errp)
{
    CoreVUDMASdioState *s = CORE_V_UDMA_SDIO(dev);
#ifdef DEBUG_SDIO_SDCARD
    DriveInfo *dinfo = drive_get(IF_SD, 0, 0);
    DeviceState *card = qdev_new(TYPE_SD_CARD);

    qdev_prop_set_drive_err(card, "drive", blk_by_legacy_dinfo(dinfo),
                            &error_fatal);
    qdev_realize_and_unref(card, BUS(&s->bus), &error_fatal);
#endif
}

static void udma_sdio_reset(DeviceState *dev)
{
    CoreVUDMASdioState *s = CORE_V_UDMA_SDIO(dev);
    s->rx_saddr = 0;
    s->rx_size = 0;
    s->rx_cfg = 0;
    s->tx_saddr = 0;
    s->tx_size = 0;
    s->tx_cfg = 0;
    s->rx_cur_saddr = 0;
    s->rx_cur_size = 0;
    s->tx_cur_saddr = 0;
    s->tx_cur_size = 0;
    s->cmd_op = 0;
    s->cmd_rsp = 0;
    s->cmd_arg = 0;
    s->data_setup = 0;
    s->block_size = 0;
    s->block_num = 0;
    s->status = 0;
    s->clk_div = 0;
    s->enable = false;
}

static void udma_sdio_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = udma_sdio_reset;
    dc->realize = udma_sdio_realize;
    dc->vmsd = &vmstate_udma_sdio;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo udma_sdio_info = {
    .name          = TYPE_CORE_V_UDMA_SDIO,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVUDMASdioState),
    .instance_init = udma_sdio_init,
    .class_init    = udma_sdio_class_init,
};

static const TypeInfo udma_sdio_bus_info = {
    .name = TYPE_CORE_V_UDMA_SDBUS,
    .parent = TYPE_SD_BUS,
    .instance_size = sizeof(SDBus),
};

static void udma_sdio_register_types(void)
{
    type_register_static(&udma_sdio_info);
    type_register_static(&udma_sdio_bus_info);
}

type_init(udma_sdio_register_types)
