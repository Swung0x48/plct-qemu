/*
 * CORE-V MCU UDMA SPI
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

#ifndef HW_CORE_V_UDMA_SPI_H
#define HW_CORE_V_UDMA_SPI_H

#include "hw/dma/core_v_udma.h"
#include "hw/ssi/ssi.h"

#define SPI_CMD_FIFO_LEN 5

enum {
    SPI_CMD_CFG = 0,
    SPI_CMD_SOT = 1,
    SPI_CMD_SEND_CMD = 2,
    SPI_CMD_DUMMY = 4,
    SPI_CMD_WAIT = 5,
    SPI_CMD_TX_DATA = 6,
    SPI_CMD_RX_DATA = 7,
    SPI_CMD_RPT = 8,
    SPI_CMD_EOT = 9,
    SPI_CMD_RPT_END = 10,
    SPI_CMD_RX_CHECK = 11,
    SPI_CMD_FULL_DUPL = 12,
    SPI_CMD_SETUP_UCA = 13,
    SPI_CMD_SETUP_UCS = 14,
};

enum {
    SPI_WAIT_EVT = 0,
    SPI_WAIT_CYC = 1,
    SPI_WAIT_GP = 2,
};

enum {
    SPI_IDLE,
    SPI_WAIT_DONE,
    SPI_WAIT_CHECK,
    SPI_WAIT_EVENT,
    SPI_DO_REPEAT,
    SPI_WAIT_CYCLE,
    SPI_CLEAR_CS,
    SPI_SEND,
    SPI_RECV,
    SPI_TRANSFER
};

enum {
    SPI_ST_NONE,
    SPI_ST_CHECK,
    SPI_ST_EOL
};

typedef struct CoreVUDMASpiState {
    SysBusDevice parent;
    MemoryRegion iomem;
    bool enable;
    uint32_t rx_saddr;
    uint32_t rx_cur_saddr;
    uint32_t rx_size;
    uint8_t rx_data_size;
    uint32_t rx_cur_size;
    uint32_t rx_cfg;

    uint32_t tx_saddr;
    uint32_t tx_cur_saddr;
    uint32_t tx_size;
    uint8_t tx_data_size;
    uint32_t tx_cur_size;
    uint32_t tx_cfg;

    uint32_t cmd_saddr;
    uint32_t cmd_cur_saddr;
    uint32_t cmd_size;
    uint32_t cmd_cur_size;
    uint32_t cmd_cfg;

    int32_t transfer_size;

    uint32_t status;
    bool is_replay;
    bool first_replay;
    uint8_t cmd_status;
    uint8_t event_select;
    uint32_t recv_data;
    uint8_t recv_data_size;
    bool check_result;
    uint8_t repeat_cmd;
    uint32_t cur_cmd;
    uint8_t bitsword;
    uint64_t cmd_fifo[SPI_CMD_FIFO_LEN];
    uint8_t fifo_pos;
    uint8_t fifo_len;
    bool trigger_event[UDMA_CTRL_EVENT_NUM];

    SSIBus *bus;
    qemu_irq irq[UDMA_PER_EVENT_NUM];
    qemu_irq irq_cs;
} CoreVUDMASpiState;

#define SPI_IRQ_CH_RX 0
#define SPI_IRQ_CH_TX 1
#define SPI_IRQ_CH_CMD 2
#define SPI_IRQ_EOT 3

#define TYPE_CORE_V_UDMA_SPI    "core-v.udma.spi"

#define CORE_V_UDMA_SPI(obj) \
    OBJECT_CHECK(CoreVUDMASpiState, (obj), TYPE_CORE_V_UDMA_SPI)

#endif /* HW_CORE_V_UDMA_SPI_H */
