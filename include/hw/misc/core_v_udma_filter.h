/*
 * CORE-V MCU UDMA SPI Device
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

#ifndef HW_CORE_V_UDMA_FILTER_H
#define HW_CORE_V_UDMA_FILTER_H

#include "hw/dma/core_v_udma.h"

enum {
    MODE_LINEAR = 0,
    MODE_SLIDING = 1,
    MODE_2D_ROW = 1,
    MODE_CIRCULAR = 2,
    MODE_2D_COL = 2,
    MODE_2D = 3,
};

typedef struct CoreVUDMAFilterState {
    SysBusDevice parent;
    MemoryRegion iomem;
    bool enable;
    uint32_t tx_ch_addr[2];
    uint32_t tx_cur_addr[2];
    uint8_t tx_ch_datasize[2];
    uint8_t tx_ch_mode[2];
    uint32_t tx_ch_len0[2];
    uint32_t tx_ch_len1[2];
    uint32_t tx_ch_len2[2];
    uint32_t tx_cur_len0[2];
    uint32_t tx_cur_len1[2];
    uint32_t tx_cur_len2[2];
    bool tx_done[2];

    uint32_t rx_ch_addr;
    uint32_t rx_cur_addr;
    uint32_t rx_ch_datasize;
    uint32_t rx_ch_mode;
    uint32_t rx_ch_len0;
    uint32_t rx_ch_len1;
    uint32_t rx_ch_len2;
    uint32_t rx_cur_len0;
    uint32_t rx_cur_len1;
    uint32_t rx_cur_len2;
    bool rx_done;

    uint32_t au_cfg;
    bool is_signed;
    int32_t au_reg0;
    int32_t au_reg1;
    int32_t acc;
    int32_t bincu_th;
    uint32_t bincu_cnt;
    uint8_t bincu_datasize;
    uint32_t bincu_val;

    uint8_t filt_mode;
    uint32_t status;

    qemu_irq irq[UDMA_PER_EVENT_NUM];
} CoreVUDMAFilterState;

#define FILTER_IRQ_EOT   0
#define FILTER_IRQ_ACT   1

#define TYPE_CORE_V_UDMA_FILTER    "core-v.udma.filter"

#define CORE_V_UDMA_FILTER(obj) \
    OBJECT_CHECK(CoreVUDMAFilterState, (obj), TYPE_CORE_V_UDMA_FILTER)

#endif /* HW_CORE_V_UDMA_FILTER_H */
