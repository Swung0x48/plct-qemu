/*
 * CORE-V MCU UDMA UART
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

#ifndef HW_CORE_V_UDMA_UART_H
#define HW_CORE_V_UDMA_UART_H

#include "hw/dma/core_v_udma.h"
#include "chardev/char-fe.h"

#define UDMA_UART_RX_FIFO_NUM 4

typedef struct CoreVUDMAUartState {
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

    uint32_t status;
    uint32_t setup;
    uint32_t error;
    uint32_t irq_en;
    uint32_t valid;
    uint8_t rx_data;
    uint8_t rx_fifo[4];
    uint8_t rx_fifo_level;
    CharBackend chr;
    qemu_irq irq[UDMA_PER_EVENT_NUM];
} CoreVUDMAUartState;

#define TYPE_CORE_V_UDMA_UART    "core-v.udma.uart"

#define CORE_V_UDMA_UART(obj) \
    OBJECT_CHECK(CoreVUDMAUartState, (obj), TYPE_CORE_V_UDMA_UART)

#define UART_IRQ_CH_RX 0
#define UART_IRQ_CH_TX 1
#define UART_IRQ_RX    2
#define UART_IRQ_ERR   3

#endif /* HW_CORE_V_UDMA_UART_H */
