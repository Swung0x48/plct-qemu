/*
 * CORE-V MCU UDMA subsystem emulation
 *
 * Copyright (c) 2022 PLCT Lab
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 or
 * (at your option) version 3 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef HW_CORE_V_UDMA_H
#define HW_CORE_V_UDMA_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define CLOCK_FREQ 50000000
enum {
    UDMA_CTRL_OFFSET = 0,
    UDMA_UART_ID = 0,
    UDMA_UART_OFFSET = 1 << 7,
    UDMA_N_UART = 2,
    UDMA_SPI_ID = UDMA_UART_ID + UDMA_N_UART,
    UDMA_SPI_OFFSET = UDMA_UART_OFFSET + (UDMA_N_UART << 7),
    UDMA_N_SPI = 2,
    UDMA_I2CM_ID = UDMA_SPI_ID + UDMA_N_SPI,
    UDMA_I2CM_OFFSET = UDMA_SPI_OFFSET + (UDMA_N_SPI << 7),
    UDMA_N_I2C = 2,
    UDMA_SDIO_ID = UDMA_I2CM_ID + UDMA_N_I2C,
    UDMA_SDIO_OFFSET = UDMA_I2CM_OFFSET + (UDMA_N_I2C << 7),
    UDMA_N_SDIO = 1,
    UDMA_I2S_ID = UDMA_SDIO_ID + UDMA_N_SDIO,
    UDMA_I2S_OFFSET = UDMA_SDIO_OFFSET + (UDMA_N_SDIO << 7),
    UDMA_N_I2S = 0,
    UDMA_CAM_ID = UDMA_I2S_ID + UDMA_N_I2S,
    UDMA_CAM_OFFSET = UDMA_I2S_OFFSET + (UDMA_N_I2S << 7),
    UDMA_N_CAM = 0,
    UDMA_FILTER_ID = UDMA_CAM_ID + UDMA_N_CAM,
    UDMA_FILTER_OFFSET = UDMA_CAM_OFFSET + (UDMA_N_CAM << 7),
    UDMA_N_FILTER = 1,
    UDMA_FPGA_ID = UDMA_FILTER_ID + UDMA_N_FILTER,
    UDMA_FPGA_OFFSET = UDMA_FILTER_OFFSET + (UDMA_N_FILTER << 7),
    UDMA_N_FPGA = 0,
    UDMA_PER_EXT_ID = UDMA_FPGA_ID + UDMA_N_FPGA,
    UDMA_PER_EXT_OFFSET = UDMA_FPGA_OFFSET + (UDMA_N_FPGA << 7),
    UDMA_N_PER_EXT = 0,
};

#define N_PERIPHS (UDMA_N_UART + UDMA_N_SPI + UDMA_N_I2C + UDMA_N_SDIO + \
                   UDMA_N_I2S + UDMA_N_CAM + UDMA_N_FILTER + UDMA_N_FPGA + \
                   UDMA_N_PER_EXT)

#define UDMA_PER_EVENT_NUM 4
#define UDMA_PER_SIZE (1 << 7)

#define UDMA_CTRL_EVENT_NUM 4

#define UDMA_ADDR_MASK 0xFFF
#define UDMA_TRANS_SIZE_MASK 0xFFFF

typedef struct CoreVUDMACtrlState {
    SysBusDevice parent;
    MemoryRegion iomem;
    uint32_t cg;
    uint32_t cfg_evt;
    uint32_t rst;
    qemu_irq irq[UDMA_CTRL_EVENT_NUM];
    qemu_irq gate[N_PERIPHS];
} CoreVUDMACtrlState;

#define TYPE_CORE_V_UDMA_CTRL    "core-v.udma.ctrl"

#define CORE_V_UDMA_CTRL(obj) \
    OBJECT_CHECK(CoreVUDMACtrlState, (obj), TYPE_CORE_V_UDMA_CTRL)


#endif /* HW_CORE_V_UDMA_H */
