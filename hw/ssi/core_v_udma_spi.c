/*
 * QEMU CORE-V MCU UDMA SPI
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
#include "hw/ssi/core_v_udma_spi.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "trace.h"
#include "hw/registerfields.h"

#define DEBUG_SPI_FLASH
#define DEBUG_SPI_READ(X)
#define DEBUG_SPI_WRITE(X)

REG32(RX_SADDR, 0x00)
REG32(RX_SIZE, 0x04)
REG32(RX_CFG, 0x08)
    FIELD(RX_CFG, CONTINUE, 0, 1)
    FIELD(RX_CFG, DATASIZE, 1, 2)
    FIELD(RX_CFG, EN, 4, 1)
    FIELD(RX_CFG, PENDING, 5, 1)
    FIELD(RX_CFG, CLR, 6, 1)
REG32(TX_SADDR, 0x10)
REG32(TX_SIZE, 0x14)
REG32(TX_CFG, 0x18)
    FIELD(TX_CFG, CONTINUE, 0, 1)
    FIELD(TX_CFG, DATASIZE, 1, 2)
    FIELD(TX_CFG, EN, 4, 1)
    FIELD(TX_CFG, PENDING, 5, 1)
    FIELD(TX_CFG, CLR, 6, 1)
REG32(CMD_SADDR, 0x20)
REG32(CMD_SIZE, 0x24)
REG32(CMD_CFG, 0x28)
    FIELD(CMD_CFG, CONTINUE, 0, 1)
    FIELD(CMD_CFG, EN, 4, 1)
    FIELD(CMD_CFG, PENDING, 5, 1)
    FIELD(CMD_CFG, CLR, 6, 1)
REG32(STATUS, 0x30)

static bool dma_rx_transfer(CoreVUDMASpiState *s, uint32_t data)
{
    if ((s->rx_cfg & R_RX_CFG_EN_MASK) && s->rx_cur_size) {
        cpu_physical_memory_write(s->rx_cur_saddr, &data, s->rx_data_size);
        s->rx_cur_size -= s->rx_data_size;
        s->rx_cur_saddr += s->rx_data_size;
        if (s->rx_cur_size == 0) {
            qemu_irq_raise(s->irq[SPI_IRQ_CH_RX]);
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

static bool dma_tx_transfer(CoreVUDMASpiState *s, uint32_t* data)
{
    if ((s->tx_cfg & R_TX_CFG_EN_MASK) && s->tx_cur_size) {
        cpu_physical_memory_read(s->tx_cur_saddr, data, s->tx_data_size);
        s->tx_cur_size -= s->tx_data_size;
        s->tx_cur_saddr += s->tx_data_size;
        if (s->tx_cur_size == 0) {
            qemu_irq_raise(s->irq[SPI_IRQ_CH_TX]);
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

static bool dma_cmd_transfer(CoreVUDMASpiState *s, uint32_t* cmd)
{
    if ((s->cmd_cfg & R_CMD_CFG_EN_MASK) && s->cmd_cur_size) {
        cpu_physical_memory_read(s->cmd_cur_saddr, cmd, 4);
        s->cmd_cur_size -= 4;
        s->cmd_cur_saddr += 4;
        if (s->cmd_cur_size == 0) {
            qemu_irq_raise(s->irq[SPI_IRQ_CH_CMD]);
            if (s->cmd_cfg & R_CMD_CFG_CONTINUE_MASK) {
                s->cmd_cur_size = s->cmd_size;
                s->cmd_cur_saddr = s->cmd_saddr;
            } else {
                s->cmd_cfg &= ~R_CMD_CFG_EN_MASK;
            }
        }
        return true;
    }
    return false;
}

static void fifo_push(CoreVUDMASpiState *s, uint64_t value)
{
    int n;

    if (s->fifo_len == SPI_CMD_FIFO_LEN) {
        error_report("%s: FIFO overflow", __func__);
        return;
    }
    n = (s->fifo_pos + s->fifo_len) & (SPI_CMD_FIFO_LEN - 1);
    s->fifo_len++;
    s->cmd_fifo[n] = value;
}

static uint64_t fifo_pop(CoreVUDMASpiState *s)
{
    uint32_t value;

    if (s->fifo_len == 0) {
        error_report("%s: FIFO underflow", __func__);
        return 0;
    }
    value = s->cmd_fifo[s->fifo_pos];
    s->fifo_len--;
    s->fifo_pos = (s->fifo_pos + 1) & (SPI_CMD_FIFO_LEN - 1);
    return value;
}

static uint32_t ssi_transfer_data(CoreVUDMASpiState *s, uint32_t data,
                                  uint8_t size)
{
    uint32_t recv_data = ssi_transfer(s->bus, data & 0xff);

    switch (size) {
    case 4:
        recv_data |= ssi_transfer(s->bus, (data >> 8) & 0xff) << 8;
        recv_data |= ssi_transfer(s->bus, (data >> 16) & 0xff) << 16;
        recv_data |= ssi_transfer(s->bus, (data >> 24) & 0xff) << 24;
        break;
    case 2:
        recv_data |= ssi_transfer(s->bus, (data >> 8) & 0xff) << 8;
        break;
    }

    return recv_data;
}

static void handle_spi_cmd(CoreVUDMASpiState *s)
{
    uint64_t cmd = 0;
    uint32_t data = 0;

    do {
        switch (s->cmd_status) {
        case SPI_IDLE:
            if (s->is_replay) {
                cmd = fifo_pop(s);
                if (((s->repeat_cmd == 0) && (cmd >> 32)) ||
                                              s->check_result) {
                    s->status = s->check_result ? SPI_ST_CHECK : SPI_ST_EOL;
                    s->check_result = false;
                    s->is_replay = false;
                } else if (cmd >> 32) {
                    s->repeat_cmd--;
                }
                fifo_push(s, cmd);
                s->cur_cmd = cmd;
            } else {
                if (!dma_cmd_transfer(s, &s->cur_cmd)) {
                    return;
                }
            }

            switch ((s->cur_cmd >> 28) & 0xF) {
            case SPI_CMD_CFG:
                /* DO NOTHING */
                s->cmd_status = SPI_WAIT_CYCLE;
                break;
            case SPI_CMD_SOT:
                qemu_set_irq(s->irq_cs, 0);
                s->cmd_status = SPI_WAIT_CYCLE;
                break;
            case SPI_CMD_SEND_CMD:
                s->bitsword = ((s->cur_cmd >> 16) & 0x1F) + 1;
                ssi_transfer_data(s, s->cur_cmd & ((1 << s->bitsword) - 1),
                                  s->bitsword >> 3);
                s->cmd_status = SPI_WAIT_DONE;
                break;
            case SPI_CMD_DUMMY:
                s->cmd_status = SPI_WAIT_DONE;
                break;
            case SPI_CMD_WAIT:
                switch ((cmd >> 8) & 0x3) {
                case SPI_WAIT_EVT:
                    s->event_select = s->cur_cmd & 3;
                    s->cmd_status = SPI_WAIT_EVENT;
                    break;
                case SPI_WAIT_CYC:
                    s->cmd_status = SPI_WAIT_CYCLE;
                    break;
                }
                break;
            case SPI_CMD_TX_DATA:
                s->cmd_status = SPI_SEND;
                s->bitsword = ((s->cur_cmd >> 16) & 0x1F) + 1;
                s->transfer_size = (s->cur_cmd & 0xffff) + 1;
                break;
            case SPI_CMD_FULL_DUPL:
                s->cmd_status = SPI_TRANSFER;
                s->recv_data = 0;
                s->recv_data_size = 0;
                s->bitsword = ((s->cur_cmd >> 16) & 0x1F) + 1;
                s->transfer_size = (s->cur_cmd & 0xffff) + 1;
                break;
            case SPI_CMD_RX_DATA:
                s->cmd_status = SPI_RECV;
                s->recv_data = 0;
                s->recv_data_size = 0;

                s->bitsword = ((s->cur_cmd >> 16) & 0x1F) + 1;
                s->transfer_size = (s->cur_cmd & 0xffff) + 1;
                break;
            case SPI_CMD_RPT:
                s->repeat_cmd = s->cur_cmd & 0xFFFF;
                s->cmd_status = SPI_DO_REPEAT;
                s->first_replay = true;
                s->status = SPI_ST_NONE;
                break;
            case SPI_CMD_EOT:
                qemu_set_irq(s->irq_cs, 1);
                qemu_set_irq(s->irq[SPI_IRQ_EOT], s->cur_cmd & 0x1);
                if (!(s->cur_cmd & 0x2)) {
                    s->cmd_status = SPI_CLEAR_CS;
                } else {
                    s->cmd_status = SPI_IDLE;
                }
                break;
            case SPI_CMD_RX_CHECK:
                s->cmd_status = SPI_WAIT_CHECK;
                break;

            case SPI_CMD_SETUP_UCA:
                if ((s->cur_cmd >> 27) & 1) {
                    s->tx_saddr = s->cur_cmd & UDMA_ADDR_MASK;
                    s->tx_cur_saddr = s->tx_saddr;
                } else {
                    s->rx_saddr = s->cur_cmd & UDMA_ADDR_MASK;
                    s->rx_cur_saddr = s->rx_saddr;
                }
                s->cmd_status = SPI_IDLE;
                break;
            case SPI_CMD_SETUP_UCS:
                if ((s->cur_cmd >> 27) & 1) {
                    s->tx_data_size = 1 << ((s->cur_cmd >> 25) & 0x3);
                    s->tx_size = s->cur_cmd & UDMA_TRANS_SIZE_MASK;
                    s->tx_cur_size = s->tx_size;
                    s->tx_cfg |= R_TX_CFG_EN_MASK;
                } else {
                    s->rx_data_size = 1 << ((s->cur_cmd >> 25) & 0x3);
                    s->rx_size = s->cur_cmd & UDMA_TRANS_SIZE_MASK;
                    s->rx_cur_size = s->rx_size;
                    s->rx_cfg |= R_RX_CFG_EN_MASK;
                }
                s->cmd_status = SPI_IDLE;
                break;
            }
            break;
        case SPI_SEND:
            while (s->transfer_size > 0) {
                if (dma_tx_transfer(s, &data)) {
                    ssi_transfer_data(s, data, s->tx_data_size);
                    s->transfer_size -= s->tx_data_size;
                } else {
                    return;
                }
            }
            s->cmd_status = SPI_WAIT_DONE;
            break;
        case SPI_TRANSFER:
            while (s->transfer_size > 0) {
                if (s->recv_data_size < s->rx_data_size) {
                    if (!dma_tx_transfer(s, &data)) {
                        return;
                    }
                    s->recv_data |= ssi_transfer_data(s, data,
                                                      s->tx_data_size) <<
                                    (s->recv_data_size << 3);
                    s->recv_data_size += s->tx_data_size;
                }

                if (s->recv_data_size >= s->rx_data_size) {
                    if (dma_rx_transfer(s, s->recv_data)) {
                        s->recv_data >>= (s->rx_data_size << 3);
                        s->recv_data_size -= s->rx_data_size;
                        s->transfer_size -= s->rx_data_size;
                    } else {
                        return;
                    }
                }
            }
            s->cmd_status = SPI_WAIT_DONE;
            break;
        case SPI_RECV:
            while (s->transfer_size > 0) {
                if (s->recv_data_size < s->rx_data_size) {
                    s->recv_data |= ssi_transfer_data(s, data, 1) <<
                                    (s->recv_data_size << 3);
                    s->recv_data_size++;
                }

                if (s->recv_data_size >= s->rx_data_size) {
                    if (dma_rx_transfer(s, s->recv_data)) {
                        s->recv_data >>= (s->rx_data_size << 3);
                        s->recv_data_size -= s->rx_data_size;
                        s->transfer_size -= s->rx_data_size;
                    } else {
                        return;
                    }
                }
            }
            s->cmd_status = SPI_WAIT_DONE;
            break;
        case SPI_CLEAR_CS: /* FALL THROUGH */
        case SPI_WAIT_CYCLE:  /* FALL THROUGH */
        case SPI_WAIT_DONE:
            s->cmd_status = SPI_IDLE;
            break;
        case SPI_DO_REPEAT:
            do {
                if (!dma_cmd_transfer(s, &s->cur_cmd)) {
                    return;
                }
                if ((s->cur_cmd >> 28) == SPI_CMD_RPT_END) {
                    s->is_replay = true;
                    s->cmd_status = SPI_IDLE;
                    break;
                } else {
                    fifo_push(s, s->cur_cmd |
                              ((uint64_t)s->first_replay << 32));
                    s->first_replay = false;
                }
            } while (true);
            break;
        case SPI_WAIT_CHECK:
            if (dma_tx_transfer(s, &data)) {
                uint16_t check_data = s->cur_cmd & 0xFFFF;
                s->check_result = false;
                switch ((s->cur_cmd >> 24) & 3) {
                case 0:
                    if ((uint16_t)data == check_data) {
                        s->check_result = true;
                    }
                    break;
                case 1:
                    if ((data & check_data) == check_data) {
                        s->check_result = true;
                    }
                    break;
                case 2:
                    if ((~(uint16_t)data & ~check_data) == ~check_data) {
                        s->check_result = true;
                    }
                    break;
                case 3:
                    if ((data & check_data) != check_data) {
                        s->check_result = true;
                    }
                    break;
                }
                s->cmd_status = SPI_IDLE;
                break;
            } else {
                return;
            }
        case SPI_WAIT_EVENT:
            if (s->trigger_event[s->event_select]) {
                s->cmd_status = SPI_IDLE;
                break;
            } else {
                return;
            }
        }
    } while (true);
}

static void update_timer(CoreVUDMASpiState *s)
{
    timer_del(s->dma_handle);
    if (s->enable && ((s->tx_cfg & R_TX_CFG_EN_MASK) ||
                      (s->rx_cfg & R_RX_CFG_EN_MASK) ||
                      (s->cmd_cfg & R_CMD_CFG_EN_MASK))) {
        timer_mod(s->dma_handle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                 (NANOSECONDS_PER_SECOND / 100000));
    }
}

static void trigger_dma_transfer(void *opaque)
{
    CoreVUDMASpiState *s = opaque;
    handle_spi_cmd(s);
    update_timer(s);
}


static uint64_t udma_spi_read(void *opaque, hwaddr addr,
                              unsigned int size)
{
    CoreVUDMASpiState *s = opaque;
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
    case R_CMD_SADDR:
        ret = s->cmd_cur_saddr;
        break;
    case R_CMD_SIZE:
        ret = s->cmd_cur_size;
        break;
    case R_CMD_CFG:
        ret = s->cmd_cfg;
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
    DEBUG_SPI_READ(printf("udma spi read %x, %x\n", (uint32_t)addr >> 2,
                          (uint32_t)ret);)
    return ret;
}

static void reset_status(CoreVUDMASpiState *s)
{
    int i;

    s->cmd_status = SPI_IDLE;
    s->is_replay = false;
    s->first_replay = false;
    s->event_select = 0;
    s->repeat_cmd = 0;
    s->recv_data = 0;
    s->recv_data_size = 0;
    s->check_result = false;
    s->fifo_pos = 0;
    s->fifo_len = 0;
    for (i = 0; i < UDMA_CTRL_EVENT_NUM; i++) {
        s->trigger_event[i] = false;
    }
}

static void udma_spi_write(void *opaque, hwaddr addr,
                            uint64_t val64, unsigned int size)
{
    CoreVUDMASpiState *s = opaque;
    uint32_t value = val64;
    bool enabled;

    DEBUG_SPI_WRITE(printf("udma spi write %x, %x\n", (uint32_t)addr >> 2,
                           value);)
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
        enabled = s->rx_cfg & R_RX_CFG_EN_MASK;
        if (value & R_RX_CFG_CLR_MASK) {
            s->rx_cur_saddr = 0;
            s->rx_cur_size = 0;
            s->rx_cfg = value & R_RX_CFG_CONTINUE_MASK;
        } else {
            s->rx_cfg = value;
        }
        s->rx_data_size = 1 << ((s->rx_cfg & R_RX_CFG_DATASIZE_MASK) >>
                                R_RX_CFG_DATASIZE_SHIFT);

        if (!enabled && (s->rx_cfg & R_RX_CFG_EN_MASK)) {
            s->rx_cur_saddr = s->rx_saddr;
            s->rx_cur_size = s->rx_size;
            timer_del(s->dma_handle);
            trigger_dma_transfer(s);
        }
        break;
    case R_TX_SADDR:
        s->tx_saddr = value;
        break;
    case R_TX_SIZE:
        s->tx_size = value;
        break;
    case R_TX_CFG:
        enabled = s->tx_cfg & R_TX_CFG_EN_MASK;
        if (value & R_TX_CFG_CLR_MASK) {
            s->tx_cur_saddr = 0;
            s->tx_cur_size = 0;
            s->tx_cfg = value & R_TX_CFG_CONTINUE_MASK;
        } else {
            s->tx_cfg = value;
        }
        s->tx_data_size = 1 << ((s->tx_cfg & R_TX_CFG_DATASIZE_MASK) >>
                                 R_TX_CFG_DATASIZE_SHIFT);

        if (!enabled && (s->tx_cfg & R_TX_CFG_EN_MASK)) {
            s->tx_cur_saddr = s->tx_saddr;
            s->tx_cur_size = s->tx_size;
            timer_del(s->dma_handle);
            trigger_dma_transfer(s);
        }
        break;
    case R_CMD_SADDR:
        s->cmd_saddr = value;
        break;
    case R_CMD_SIZE:
        s->cmd_size = value;
        break;
    case R_CMD_CFG:
        enabled = s->cmd_cfg & R_CMD_CFG_EN_MASK;
        if (value & R_CMD_CFG_CLR_MASK) {
            s->cmd_cur_saddr = 0;
            s->cmd_cur_size = 0;
            s->cmd_cfg = value & R_CMD_CFG_CONTINUE_MASK;
        } else {
            s->cmd_cfg = value;
        }

        if (!enabled && (s->cmd_cfg & R_CMD_CFG_EN_MASK)) {
            s->cmd_cur_saddr = s->cmd_saddr;
            s->cmd_cur_size = s->cmd_size;
            timer_del(s->dma_handle);
            trigger_dma_transfer(s);
        }
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps udma_spi_ops = {
    .read = udma_spi_read,
    .write = udma_spi_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_udma_spi = {
    .name = TYPE_CORE_V_UDMA_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enable, CoreVUDMASpiState),
        VMSTATE_TIMER_PTR(dma_handle, CoreVUDMASpiState),
        VMSTATE_UINT32(rx_saddr, CoreVUDMASpiState),
        VMSTATE_UINT32(rx_cur_saddr, CoreVUDMASpiState),
        VMSTATE_UINT32(rx_size, CoreVUDMASpiState),
        VMSTATE_UINT32(rx_cur_size, CoreVUDMASpiState),
        VMSTATE_UINT32(rx_cfg, CoreVUDMASpiState),
        VMSTATE_UINT8(rx_data_size, CoreVUDMASpiState),
        VMSTATE_UINT32(tx_saddr, CoreVUDMASpiState),
        VMSTATE_UINT32(tx_cur_saddr, CoreVUDMASpiState),
        VMSTATE_UINT32(tx_size, CoreVUDMASpiState),
        VMSTATE_UINT32(tx_cur_size, CoreVUDMASpiState),
        VMSTATE_UINT32(tx_cfg, CoreVUDMASpiState),
        VMSTATE_UINT8(tx_data_size, CoreVUDMASpiState),
        VMSTATE_UINT32(cmd_saddr, CoreVUDMASpiState),
        VMSTATE_UINT32(cmd_cur_saddr, CoreVUDMASpiState),
        VMSTATE_UINT32(cmd_size, CoreVUDMASpiState),
        VMSTATE_UINT32(cmd_cur_size, CoreVUDMASpiState),
        VMSTATE_UINT32(cmd_cfg, CoreVUDMASpiState),
        VMSTATE_INT32(transfer_size, CoreVUDMASpiState),
        VMSTATE_UINT32(status, CoreVUDMASpiState),
        VMSTATE_BOOL(is_replay, CoreVUDMASpiState),
        VMSTATE_BOOL(first_replay, CoreVUDMASpiState),
        VMSTATE_UINT8(cmd_status, CoreVUDMASpiState),
        VMSTATE_UINT8(event_select, CoreVUDMASpiState),
        VMSTATE_UINT32(recv_data, CoreVUDMASpiState),
        VMSTATE_UINT8(recv_data_size, CoreVUDMASpiState),
        VMSTATE_BOOL(check_result, CoreVUDMASpiState),
        VMSTATE_UINT8(repeat_cmd, CoreVUDMASpiState),
        VMSTATE_UINT32(cur_cmd, CoreVUDMASpiState),
        VMSTATE_UINT64_ARRAY(cmd_fifo, CoreVUDMASpiState, SPI_CMD_FIFO_LEN),
        VMSTATE_UINT8(fifo_pos, CoreVUDMASpiState),
        VMSTATE_UINT8(fifo_len, CoreVUDMASpiState),
        VMSTATE_BOOL_ARRAY(trigger_event, CoreVUDMASpiState,
                           UDMA_CTRL_EVENT_NUM),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_udma_ctrl_event(void *opaque, int num, int level)
{
    CoreVUDMASpiState *s = opaque;
    if (num == UDMA_CTRL_EVENT_NUM) {
        s->enable = level;
        if (s->enable) {
            update_timer(s);
        }
    } else {
        s->trigger_event[num] = level;
    }
}

static void udma_spi_init(Object *obj)
{
    CoreVUDMASpiState *s = CORE_V_UDMA_SPI(obj);

    memory_region_init_io(&s->iomem, obj, &udma_spi_ops, s,
                          TYPE_CORE_V_UDMA_SPI, UDMA_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq, UDMA_PER_EVENT_NUM);
    qdev_init_gpio_in(DEVICE(obj), core_v_udma_ctrl_event,
                      UDMA_CTRL_EVENT_NUM + 1);

    qdev_init_gpio_out_named(DEVICE(obj), &s->irq_cs, "spi_cs",  1);

}

static void udma_spi_realize(DeviceState *dev, Error **errp)
{
    CoreVUDMASpiState *s = CORE_V_UDMA_SPI(dev);

    s->dma_handle = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 trigger_dma_transfer, s);
    s->bus = ssi_create_bus(dev, "udma_spi_bus");
#ifdef DEBUG_SPI_FLASH
    DeviceState *flash = ssi_create_peripheral(s->bus, "n25q256a13");
    qdev_connect_gpio_out_named(dev, "spi_cs", 0,
                                qdev_get_gpio_in_named(flash, SSI_GPIO_CS,
                                                       0));
#endif
}

static void udma_spi_reset(DeviceState *dev)
{
    CoreVUDMASpiState *s = CORE_V_UDMA_SPI(dev);
    s->rx_saddr = 0;
    s->rx_size = 0;
    s->rx_cfg = 0;
    s->rx_data_size = 0;
    s->tx_saddr = 0;
    s->tx_size = 0;
    s->tx_cfg = 0;
    s->tx_data_size = 0;
    s->rx_cur_saddr = 0;
    s->rx_cur_size = 0;
    s->tx_cur_saddr = 0;
    s->tx_cur_size = 0;
    s->cmd_saddr = 0;
    s->cmd_size = 0;
    s->cmd_cfg = 0;
    s->status = 0;
    s->enable = false;
    reset_status(s);
}

static void udma_spi_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = udma_spi_reset;
    dc->realize = udma_spi_realize;
    dc->vmsd = &vmstate_udma_spi;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo udma_spi_info = {
    .name          = TYPE_CORE_V_UDMA_SPI,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVUDMASpiState),
    .instance_init = udma_spi_init,
    .class_init    = udma_spi_class_init,
};

static void udma_spi_register_types(void)
{
    type_register_static(&udma_spi_info);
}

type_init(udma_spi_register_types)
