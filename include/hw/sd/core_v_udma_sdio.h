/*
 * CORE-V MCU UDMA SDIO
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

#ifndef HW_CORE_V_UDMA_SDIO_H
#define HW_CORE_V_UDMA_SDIO_H

#include "hw/dma/core_v_udma.h"
#include "hw/sd/sd.h"
#include "sysemu/blockdev.h"

enum {
  RSP_TYPE_NULL = 0,
  RSP_TYPE_48_CRC = 1,
  RSP_TYPE_48_NOCRC = 2,
  RSP_TYPE_136 = 3,
  RSP_TYPE_48_BSY = 4
};

typedef struct CoreVUDMASdioState {
    SysBusDevice parent;
    MemoryRegion iomem;
    bool enable;
    uint32_t rx_saddr;
    uint32_t rx_cur_saddr;
    uint32_t rx_size;
    uint32_t rx_cur_size;
    uint32_t rx_cfg;

    uint32_t tx_saddr;
    uint32_t tx_cur_saddr;
    uint32_t tx_size;
    uint32_t tx_cur_size;
    uint32_t tx_cfg;

    uint8_t cmd_op;
    uint8_t cmd_rsp;
    uint32_t cmd_arg;
    uint32_t data_setup;
    uint32_t block_num;
    uint32_t block_size;

    uint32_t resp[4];
    uint32_t clk_div;
    uint32_t status;

    SDBus bus;
    qemu_irq irq[UDMA_PER_EVENT_NUM];
} CoreVUDMASdioState;

#define SDIO_IRQ_CH_RX 0
#define SDIO_IRQ_CH_TX 1
#define SDIO_IRQ_EOT   2
#define SDIO_IRQ_ERR   3

#define TYPE_CORE_V_UDMA_SDIO    "core-v.udma.sdio"

#define CORE_V_UDMA_SDIO(obj) \
    OBJECT_CHECK(CoreVUDMASdioState, (obj), TYPE_CORE_V_UDMA_SDIO)

#define TYPE_CORE_V_UDMA_SDBUS   "core-v.udma.sdbus"
DECLARE_INSTANCE_CHECKER(SDBus, CORE_V_UDMA_SDBUS,
                         TYPE_CORE_V_UDMA_SDBUS)

#endif /* HW_CORE_V_UDMA_SDIO_H */
