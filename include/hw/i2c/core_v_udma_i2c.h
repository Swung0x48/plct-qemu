/*
 * CORE-V MCU UDMA I2C
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

#ifndef HW_CORE_V_UDMA_I2C_H
#define HW_CORE_V_UDMA_I2C_H

#include "hw/dma/core_v_udma.h"
#include "hw/i2c/i2c.h"
#include "qemu/timer.h"

enum {
    I2C_CMD_START = 0,
    I2C_CMD_STOP = 2,
    I2C_CMD_RD_ACK = 4,
    I2C_CMD_RD_NACK = 6,
    I2C_CMD_WR = 8,
    I2C_CMD_WAIT = 10,
    I2C_CMD_RPT = 12,
    I2C_CMD_CFG = 14,
    I2C_CMD_WAIT_EV = 1
};

enum {
    I2C_ST_WAIT_IN_CMD,
    I2C_ST_WAIT_EV,
    I2C_ST_CMD_DONE,
    I2C_ST_READ,
    I2C_ST_WRITE,
    I2C_ST_STORE_DATA,
    I2C_ST_SKIP_CMD,
    I2C_ST_GET_DATA,
    I2C_ST_GET_WAIT,
    I2C_ST_GET_WAIT_EV,
    I2C_ST_GET_RPT,
    I2C_ST_GET_CFG_MSB,
    I2C_ST_GET_CFG_LSB
};

typedef struct CoreVUDMAI2cState {
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
    bool is_nack;
    uint8_t cmd_status;
    uint8_t event_select;
    uint8_t recv_data;
    uint8_t repeat_cmd;
    bool start_transfer;
    bool trigger_event[UDMA_CTRL_EVENT_NUM];

    I2CBus *bus;
    QEMUTimer *dma_handle;
    qemu_irq irq[UDMA_PER_EVENT_NUM];
} CoreVUDMAI2cState;

#define I2C_IRQ_CH_RX 0
#define I2C_IRQ_CH_TX 1

#define TYPE_CORE_V_UDMA_I2C    "core-v.udma.i2c"

#define CORE_V_UDMA_I2C(obj) \
    OBJECT_CHECK(CoreVUDMAI2cState, (obj), TYPE_CORE_V_UDMA_I2C)

#endif /* HW_CORE_V_UDMA_I2C_H */
