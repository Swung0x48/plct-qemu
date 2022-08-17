/*
 * QEMU CORE-V MCU UDMA FILTER
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
#include "hw/misc/core_v_udma_filter.h"
#include "hw/irq.h"
#include "hw/qdev-properties.h"
#include "trace.h"
#include "hw/registerfields.h"

REG32(TX_CH0_ADD, 0x00)
REG32(TX_CH0_CFG, 0x04)
    FIELD(TX_CH0_CFG, DATASIZE, 0, 2)
    FIELD(TX_CH0_CFG, MODE, 8, 2)
REG32(TX_CH0_LEN0, 0x08)
REG32(TX_CH0_LEN1, 0x0C)
REG32(TX_CH0_LEN2, 0x10)
REG32(TX_CH1_ADD, 0x14)
REG32(TX_CH1_CFG, 0x18)
    FIELD(TX_CH1_CFG, DATASIZE, 0, 2)
    FIELD(TX_CH1_CFG, MODE, 8, 2)
REG32(TX_CH1_LEN0, 0x1C)
REG32(TX_CH1_LEN1, 0x20)
REG32(TX_CH1_LEN2, 0x24)
REG32(RX_CH_ADD, 0x28)
REG32(RX_CH_CFG, 0x2C)
    FIELD(RX_CH_CFG, DATASIZE, 0, 2)
    FIELD(RX_CH_CFG, MODE, 8, 2)
REG32(RX_CH_LEN0, 0x30)
REG32(RX_CH_LEN1, 0x34)
REG32(RX_CH_LEN2, 0x38)
REG32(AU_CFG, 0x3C)
    FIELD(AU_CFG, SIGNED, 0, 1)
    FIELD(AU_CFG, BYPASS, 1, 1)
    FIELD(AU_CFG, MODE, 8, 4)
    FIELD(AU_CFG, SHIFT, 16, 5)
REG32(AU_REG0, 0x40)
REG32(AU_REG1, 0x44)
REG32(BINCU_TH, 0x48)
REG32(BINCU_CNT, 0x4C)
    FIELD(BINCU_CNT, COUNTER, 0, 15)
    FIELD(BINCU_CNT, EN, 31, 1)
REG32(BINCU_SETUP, 0x50)
    FIELD(BINCU_SETUP, DATASIZE, 0, 2)
REG32(BINCU_VAL, 0x54)
REG32(FILT, 0x58)
    FIELD(FILT, MODE, 0, 4)
REG32(FILT_CMD, 0x5C)
REG32(STATUS, 0x60)
    FIELD(STATUS, DONE, 0, 1)

static inline int32_t DATA(uint32_t data, uint8_t size, bool is_signed)
{
    int32_t input, mask;

    switch (size) {
    case 0:
        input = (int8_t)data;
        mask = 0xFF;
        break;
    case 1:
        input = (int16_t)data;
        mask = 0xFFFF;
        break;
    default:
        input = (int32_t)data;
        mask = 0xFFFFFFFF;
        break;
    }

    if (!is_signed) {
        input &= mask;
    }

    return input;
}

static uint32_t update_bincu(CoreVUDMAFilterState *s, uint32_t data)
{
    uint8_t irq = 0;
    int32_t input = DATA(data, s->bincu_datasize, s->is_signed);

    if ((s->bincu_cnt & R_BINCU_CNT_EN_MASK) & (input > s->bincu_th)) {
        s->bincu_val++;
        if (s->bincu_val == (s->bincu_cnt & R_BINCU_CNT_COUNTER_MASK)) {
            irq = 1;
        }
    }

    qemu_set_irq(s->irq[FILTER_IRQ_ACT], irq);
    return data > s->bincu_th;
}

static uint32_t caculate_au_once(CoreVUDMAFilterState *s, uint32_t opa,
                                 uint32_t opb)
{
    int32_t srcA = DATA(opa, s->tx_ch_datasize[0], s->is_signed);
    int32_t srcB = DATA(opb, s->tx_ch_datasize[1], s->is_signed);

    switch ((s->au_cfg & R_AU_CFG_MODE_MASK) >> R_AU_CFG_MODE_SHIFT) {
    case 0:
        s->acc = srcA * srcB;
        break;
    case 1:
        s->acc = srcA * srcB + s->au_reg0;
        break;
    case 2:
        s->acc += srcA * srcB;
        break;
    case 3:
        s->acc = srcA * srcA;
        break;
    case 4:
        s->acc = srcA * srcA + srcB;
        break;
    case 5:
        s->acc = srcA * srcA - srcB;
        break;
    case 6:
        s->acc += srcA * srcA;
        break;
    case 7:
        s->acc = srcA * srcA + s->au_reg0;
        break;
    case 8:
        s->acc = srcA * s->au_reg1;
        break;
    case 9:
        s->acc = srcA * s->au_reg1 + srcB;
        break;
    case 10:
        s->acc = srcA * s->au_reg1 - srcB;
        break;
    case 11:
        s->acc = srcA * s->au_reg1 + s->au_reg0;
        break;
    case 12:
        s->acc += srcA * s->au_reg1;
        break;
    case 13:
        s->acc = srcA + srcB;
        break;
    case 14:
        s->acc = srcA - srcB;
        break;
    case 15:
        s->acc = srcA + s->au_reg0;
        break;
    }
    s->acc = s->au_cfg & R_AU_CFG_BYPASS_MASK ? srcA : s->acc;

    return s->acc >> ((s->au_cfg & R_AU_CFG_SHIFT_MASK) >>
                      R_AU_CFG_SHIFT_SHIFT);
}

static void dma_rx_transfer(CoreVUDMAFilterState *s, uint32_t data)
{

    cpu_physical_memory_write(s->rx_ch_addr, &data,
                              1 << s->rx_ch_datasize);
    switch (s->rx_ch_mode) {
    case MODE_LINEAR:
        s->rx_cur_addr += 1 << s->rx_ch_datasize;
        s->rx_cur_len0++;
        if (s->rx_cur_len0 == s->rx_ch_len0) {
            s->rx_done = true;
        }
        break;
    case MODE_2D_ROW:
        s->rx_cur_len0++;
        s->rx_cur_addr += 1 << s->rx_ch_datasize;
        if (s->rx_cur_len0 == s->rx_ch_len0) {
            s->rx_cur_len1++;
            if (s->rx_cur_len1 == s->rx_ch_len1) {
                s->rx_done = true;
                break;
            }
            s->rx_cur_addr = s->rx_ch_addr +
                                    s->rx_cur_len1 *
                                    s->rx_cur_len2;
            s->rx_cur_len0 = 0;
        }
        break;
    case MODE_2D_COL:
        s->rx_cur_len0++;
        s->rx_cur_addr += 1 << s->rx_ch_datasize;
        if (s->rx_cur_len0 == s->rx_ch_len0) {
            s->rx_cur_len1++;
            if (s->rx_cur_len1 == s->rx_ch_len1) {
                s->rx_done = true;
                break;
            }
            s->rx_cur_addr = s->rx_ch_addr + s->rx_cur_len1 *
                                             (1 << s->rx_ch_datasize);
            s->rx_cur_len0 = 0;
        }
        break;
    }
}

static uint32_t dma_tx_transfer(CoreVUDMAFilterState *s, int index)
{
    uint32_t data;
    cpu_physical_memory_read(s->tx_cur_addr[index], &data,
                             1 << s->tx_ch_datasize[index]);
    switch (s->tx_ch_mode[index]) {
    case MODE_LINEAR:
        s->tx_cur_addr[index] += 1 << s->tx_ch_datasize[index];
        s->tx_cur_len0[index]++;
        if (s->tx_cur_len0[index] == s->tx_ch_len0[index]) {
            s->tx_done[index] = true;
        }
        break;
    case MODE_SLIDING:
        s->tx_cur_len0[index]++;
        s->tx_cur_addr[index] += 1 << s->tx_ch_datasize[index];
        if (s->tx_cur_len0[index] == s->tx_ch_len0[index]) {
            s->tx_cur_len1[index]++;
            if (s->tx_cur_len1[index] == s->tx_ch_len1[index]) {
                s->tx_done[index] = true;
                break;
            }
            s->tx_cur_addr[index] = s->tx_ch_addr[index] +
                                    s->tx_cur_len1[index] *
                                    (1 << s->tx_ch_datasize[index]);
            s->tx_cur_len0[index] = 0;
        }
        break;
    case MODE_CIRCULAR:
        s->tx_cur_len0[index]++;
        s->tx_cur_addr[index] += 1 << s->tx_ch_datasize[index];
        if (s->tx_cur_len0[index] == s->tx_ch_len0[index]) {
            s->tx_cur_len1[index]++;
            if (s->tx_cur_len1[index] == s->tx_ch_len1[index]) {
                s->tx_done[index] = true;
                break;
            }
            s->tx_cur_addr[index] = s->tx_ch_addr[index];
            s->tx_cur_len0[index] = 0;
        }
        break;
    case MODE_2D:
        s->tx_cur_len0[index]++;
        s->tx_cur_addr[index] += 1 << s->tx_ch_datasize[index];
        if (s->tx_cur_len0[index] == s->tx_ch_len0[index]) {
            s->tx_cur_len1[index]++;
            if (s->tx_cur_len1[index] == s->tx_ch_len1[index]) {
                s->tx_done[index] = true;
                break;
            }
            s->tx_cur_addr[index] = s->tx_ch_addr[index] +
                                    s->tx_cur_len1[index] *
                                    s->tx_cur_len2[index];
            s->tx_cur_len0[index] = 0;
        }
        break;
    }
    return data;
}

/* TODO: cannot get filter data input currently */
static uint32_t get_filter_input(CoreVUDMAFilterState *s) { return 0; }

static void handle_filter_cmd(CoreVUDMAFilterState *s)
{
    uint32_t srcA, srcB, ret;
    s->bincu_val = 0;
    s->acc = 0;
    s->tx_cur_len0[0] = 0;
    s->tx_cur_len1[0] = 0;
    s->tx_cur_len2[0] = 0;
    s->tx_cur_len0[1] = 0;
    s->tx_cur_len1[1] = 0;
    s->tx_cur_len2[1] = 0;
    s->rx_cur_len0 = 0;
    s->rx_cur_len1 = 0;
    s->rx_cur_len2 = 0;

    do {
        switch (s->filt_mode) {
        case 0:
            srcA = dma_tx_transfer(s, 0);
            srcB = dma_tx_transfer(s, 1);
            ret = caculate_au_once(s, srcA, srcB);
            dma_rx_transfer(s, ret);
            break;
        case 1:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            srcB = dma_tx_transfer(s, 1);
            ret = caculate_au_once(s, srcA, srcB);
            dma_rx_transfer(s, ret);
            break;
        case 2:
            srcA = dma_tx_transfer(s, 0);
            s->tx_done[1] = true;
            ret = caculate_au_once(s, srcA, 0);
            dma_rx_transfer(s, ret);
            break;
        case 3:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            s->tx_done[1] = true;
            ret = caculate_au_once(s, srcA, 0);
            dma_rx_transfer(s, ret);
            break;
        case 4:
            srcA = dma_tx_transfer(s, 0);
            srcB = dma_tx_transfer(s, 1);
            ret = caculate_au_once(s, srcA, srcB);
            ret = update_bincu(s, ret);
            s->rx_done = true;
            break;
        case 5:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            srcB = dma_tx_transfer(s, 1);
            ret = caculate_au_once(s, srcA, srcB);
            ret = update_bincu(s, ret);
            s->rx_done = true;
            break;
        case 6:
            srcA = dma_tx_transfer(s, 0);
            s->tx_done[1] = true;
            ret = caculate_au_once(s, srcA, 0);
            ret = update_bincu(s, ret);
            s->rx_done = true;
            break;
        case 7:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            s->tx_done[1] = true;
            ret = caculate_au_once(s, srcA, 0);
            ret = update_bincu(s, ret);
            s->rx_done = true;
            break;
        case 8:
            srcA = dma_tx_transfer(s, 0);
            srcB = dma_tx_transfer(s, 1);
            ret = caculate_au_once(s, srcA, srcB);
            ret = update_bincu(s, ret);
            dma_rx_transfer(s, ret);
            break;
        case 9:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            srcB = dma_tx_transfer(s, 1);
            ret = caculate_au_once(s, srcA, srcB);
            ret = update_bincu(s, ret);
            dma_rx_transfer(s, ret);
            break;
        case 10:
            srcA = dma_tx_transfer(s, 0);
            s->tx_done[1] = true;
            ret = caculate_au_once(s, srcA, 0);
            ret = update_bincu(s, ret);
            dma_rx_transfer(s, ret);
            break;
        case 11:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            s->tx_done[1] = true;
            ret = caculate_au_once(s, srcA, 0);
            ret = update_bincu(s, ret);
            dma_rx_transfer(s, ret);
            break;
        case 12:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            s->tx_done[1] = true;
            ret = update_bincu(s, srcA);
            s->rx_done = true;
            break;
        case 13:
            srcA = get_filter_input(s);
            s->tx_done[0] = true;
            s->tx_done[1] = true;
            ret = update_bincu(s, srcA);
            dma_rx_transfer(s, ret);
            break;
        }

        if (s->tx_done[0] && s->tx_done[1] && s->rx_done) {
            s->status |= 1;
            qemu_set_irq(s->irq[FILTER_IRQ_EOT], 1);
            break;
        } else {
            qemu_set_irq(s->irq[FILTER_IRQ_EOT], 0);
        }
    } while (1);
}

static uint64_t udma_filter_read(void *opaque, hwaddr addr,
                               unsigned int size)
{
    CoreVUDMAFilterState *s = opaque;
    uint64_t ret = 0;

    if (!s->enable) {
        return 0;
    }

    switch (addr >> 2) {
    case R_TX_CH0_ADD:
        ret = s->tx_ch_addr[0];
        break;
    case R_TX_CH0_CFG:
        ret = s->tx_ch_datasize[0] | (s->tx_ch_mode[0] <<
                                      R_TX_CH1_CFG_MODE_SHIFT);
        break;
    case R_TX_CH0_LEN0:
        ret = s->tx_ch_len0[0];
        break;
    case R_TX_CH0_LEN1:
        ret = s->tx_ch_len1[0];
        break;
    case R_TX_CH0_LEN2:
        ret = s->tx_ch_len2[0];
        break;
    case R_TX_CH1_ADD:
        ret = s->tx_ch_addr[1];
        break;
    case R_TX_CH1_CFG:
        ret = s->tx_ch_datasize[1] | (s->tx_ch_mode[1] <<
                                      R_TX_CH1_CFG_MODE_SHIFT);
        break;
    case R_TX_CH1_LEN0:
        ret = s->tx_ch_len0[1];
        break;
    case R_TX_CH1_LEN1:
        ret = s->tx_ch_len1[1];
        break;
    case R_TX_CH1_LEN2:
        ret = s->tx_ch_len2[1];
        break;
    case R_RX_CH_ADD:
        ret = s->rx_ch_addr;
        break;
    case R_RX_CH_CFG:
        ret = s->rx_ch_datasize | (s->rx_ch_mode <<
                                   R_RX_CH_CFG_MODE_SHIFT);
        break;
    case R_RX_CH_LEN0:
        ret = s->rx_ch_len0;
        break;
    case R_RX_CH_LEN1:
        ret = s->rx_ch_len1;
        break;
    case R_RX_CH_LEN2:
        ret = s->rx_ch_len2;
        break;
    case R_AU_CFG:
        ret = s->au_cfg;
        break;
    case R_AU_REG0:
        ret = s->au_reg0;
        break;
    case R_AU_REG1:
        ret = s->au_reg1;
        break;
    case R_BINCU_TH:
        ret = s->bincu_th;
        break;
    case R_BINCU_SETUP:
        ret = s->bincu_datasize;
        break;
    case R_BINCU_CNT:
        ret = s->bincu_cnt;
        break;
    case R_BINCU_VAL:
        ret = s->bincu_val;
        break;
    case R_FILT:
        ret = s->filt_mode;
        break;
    case R_STATUS:
        ret = s->status;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }

    return ret;
}

static void udma_filter_write(void *opaque, hwaddr addr,
                            uint64_t val64, unsigned int size)
{
    CoreVUDMAFilterState *s = opaque;
    uint32_t value = val64;

    if (!s->enable) {
        return;
    }

    switch (addr >> 2) {
    case R_TX_CH0_ADD:
        s->tx_ch_addr[0] = value;
        break;
    case R_TX_CH0_CFG:
        s->tx_ch_datasize[0] = value & R_TX_CH0_CFG_DATASIZE_MASK;
        s->tx_ch_mode[0] = (value & R_TX_CH0_CFG_MODE_MASK) >>
                           R_TX_CH0_CFG_MODE_SHIFT;
        break;
    case R_TX_CH0_LEN0:
        s->tx_ch_len0[0] = value;
        break;
    case R_TX_CH0_LEN1:
        s->tx_ch_len1[0] = value;
        break;
    case R_TX_CH0_LEN2:
        s->tx_ch_len2[0] = value;
        break;
    case R_TX_CH1_ADD:
        s->tx_ch_addr[1] = value;
        break;
    case R_TX_CH1_CFG:
        s->tx_ch_datasize[1] = value & R_TX_CH1_CFG_DATASIZE_MASK;
        s->tx_ch_mode[1] = (value & R_TX_CH1_CFG_MODE_MASK) >>
                           R_TX_CH1_CFG_MODE_SHIFT;
        break;
    case R_TX_CH1_LEN0:
        s->tx_ch_len0[1] = value;
        break;
    case R_TX_CH1_LEN1:
        s->tx_ch_len1[1] = value;
        break;
    case R_TX_CH1_LEN2:
        s->tx_ch_len2[1] = value;
        break;
    case R_RX_CH_ADD:
        s->rx_ch_addr = value;
        break;
    case R_RX_CH_CFG:
        s->rx_ch_datasize = value & R_RX_CH_CFG_DATASIZE_MASK;
        s->rx_ch_mode = (value & R_RX_CH_CFG_MODE_MASK) >>
                        R_RX_CH_CFG_MODE_SHIFT;
        break;
    case R_RX_CH_LEN0:
        s->rx_ch_len0 = value;
        break;
    case R_RX_CH_LEN1:
        s->rx_ch_len1 = value;
        break;
    case R_RX_CH_LEN2:
        s->rx_ch_len2 = value;
        break;
    case R_AU_CFG:
        s->au_cfg = value;
        s->is_signed = s->au_cfg & R_AU_CFG_SIGNED_MASK;
        break;
    case R_AU_REG0:
        s->au_reg0 = value;
        break;
    case R_AU_REG1:
        s->au_reg1 = value;
        break;
    case R_BINCU_TH:
        s->bincu_th = value;
        break;
    case R_BINCU_SETUP:
        s->bincu_datasize = value & 3;
        break;
    case R_BINCU_CNT:
        s->bincu_cnt = value;
        break;
    case R_FILT:
        s->filt_mode = value & 0xF;
        break;
    case R_FILT_CMD:
        if (value & 1) {
            handle_filter_cmd(s);
        }
        break;
    case R_STATUS:
        s->status &= ~(value & 1);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static const MemoryRegionOps udma_filter_ops = {
    .read = udma_filter_read,
    .write = udma_filter_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_udma_filter = {
    .name = TYPE_CORE_V_UDMA_FILTER,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enable, CoreVUDMAFilterState),
        VMSTATE_UINT32_ARRAY(tx_ch_addr, CoreVUDMAFilterState, 2),
        VMSTATE_UINT32_ARRAY(tx_cur_addr, CoreVUDMAFilterState, 2),
        VMSTATE_UINT8_ARRAY(tx_ch_datasize, CoreVUDMAFilterState, 2),
        VMSTATE_UINT8_ARRAY(tx_ch_mode, CoreVUDMAFilterState, 2),
        VMSTATE_UINT32_ARRAY(tx_ch_len0, CoreVUDMAFilterState, 2),
        VMSTATE_UINT32_ARRAY(tx_ch_len1, CoreVUDMAFilterState, 2),
        VMSTATE_UINT32_ARRAY(tx_ch_len2, CoreVUDMAFilterState, 2),
        VMSTATE_BOOL_ARRAY(tx_done, CoreVUDMAFilterState, 2),
        VMSTATE_UINT32(rx_ch_addr, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_cur_addr, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_ch_datasize, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_ch_mode, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_ch_len0, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_ch_len1, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_ch_len2, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_cur_len0, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_cur_len1, CoreVUDMAFilterState),
        VMSTATE_UINT32(rx_cur_len2, CoreVUDMAFilterState),
        VMSTATE_BOOL(rx_done, CoreVUDMAFilterState),
        VMSTATE_UINT32(au_cfg, CoreVUDMAFilterState),
        VMSTATE_BOOL(is_signed, CoreVUDMAFilterState),
        VMSTATE_INT32(au_reg0, CoreVUDMAFilterState),
        VMSTATE_INT32(au_reg1, CoreVUDMAFilterState),
        VMSTATE_INT32(acc, CoreVUDMAFilterState),
        VMSTATE_INT32(bincu_th, CoreVUDMAFilterState),
        VMSTATE_UINT32(bincu_cnt, CoreVUDMAFilterState),
        VMSTATE_UINT32(bincu_val, CoreVUDMAFilterState),
        VMSTATE_UINT8(bincu_datasize, CoreVUDMAFilterState),
        VMSTATE_UINT8(filt_mode, CoreVUDMAFilterState),
        VMSTATE_UINT32(status, CoreVUDMAFilterState),
        VMSTATE_END_OF_LIST()
    }
};

static void core_v_udma_ctrl_event(void *opaque, int num, int level)
{
    CoreVUDMAFilterState *s = opaque;
    s->enable = level;
}

static void udma_filter_init(Object *obj)
{
    CoreVUDMAFilterState *s = CORE_V_UDMA_FILTER(obj);

    memory_region_init_io(&s->iomem, obj, &udma_filter_ops, s,
                          TYPE_CORE_V_UDMA_FILTER, UDMA_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq, UDMA_PER_EVENT_NUM);
    qdev_init_gpio_in(DEVICE(obj), core_v_udma_ctrl_event, 1);
}

static void udma_filter_realize(DeviceState *dev, Error **errp)
{
}

static void udma_filter_reset(DeviceState *dev)
{
    CoreVUDMAFilterState *s = CORE_V_UDMA_FILTER(dev);
    s->status = 0;
    s->enable = false;
}

static void udma_filter_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = udma_filter_reset;
    dc->realize = udma_filter_realize;
    dc->vmsd = &vmstate_udma_filter;
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo udma_filter_info = {
    .name          = TYPE_CORE_V_UDMA_FILTER,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVUDMAFilterState),
    .instance_init = udma_filter_init,
    .class_init    = udma_filter_class_init,
};

static void udma_filter_register_types(void)
{
    type_register_static(&udma_filter_info);
}

type_init(udma_filter_register_types)
