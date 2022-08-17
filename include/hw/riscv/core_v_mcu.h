/*
 * QEMU RISC-V Board Compatible with CORE-V-MCU platform
 *
 * Copyright (c) 2022 PLCT Lab
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

#ifndef HW_CORE_V_MCU_H
#define HW_CORE_V_MCU_H

#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/core_v_cpu.h"
#include "hw/gpio/core_v_apb_gpio.h"
#include "hw/timer/core_v_timer.h"
#include "hw/timer/core_v_apb_adv_timer.h"
#include "hw/intc/core_v_evt_gen.h"
#include "hw/dma/core_v_udma.h"
#include "hw/char/core_v_udma_uart.h"
#include "hw/i2c/core_v_udma_i2c.h"
#include "hw/ssi/core_v_udma_spi.h"
#include "hw/sd/core_v_udma_sdio.h"
#include "hw/i2c/core_v_i2cs.h"
#include "qom/object.h"

#define TYPE_CORE_V_MCU_SOC  "core-v.mcu.soc"
#define CORE_V_MCU_SOC(obj) \
    OBJECT_CHECK(COREVMCUSoCState, (obj), TYPE_CORE_V_MCU_SOC)

typedef struct COREVMCUSoCState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    RISCVHartArrayState cpus;
    CoreVEvtGenState evt_gen;
    CoreVTimerState timer;
    CoreVApbAdvTimerState adv_timer;
    CoreVUDMACtrlState udma_ctrl;
    CoreVUDMAUartState udma_uart[UDMA_N_UART];
    CoreVUDMASpiState udma_spi[UDMA_N_SPI];
    CoreVUDMAI2cState udma_i2c[UDMA_N_I2C];
    CoreVUDMASdioState udma_sdio[UDMA_N_SDIO];
    CoreVGpioState gpio;
    CoreVI2csState i2cs;
    MemoryRegion ram;
    MemoryRegion rom;
} COREVMCUSoCState;

typedef struct COREVMCUState {
    /*< private >*/
    SysBusDevice parent_obj;

    /*< public >*/
    COREVMCUSoCState soc;

} COREVMCUState;

#define TYPE_CORE_V_MCU_MACHINE MACHINE_TYPE_NAME("core_v_mcu")
#define CORE_V_MCU_MACHINE(obj) \
    OBJECT_CHECK(COREVMCUState, (obj), TYPE_CORE_V_MCU_MACHINE)

enum {
    CORE_V_DEV_ROM,
    CORE_V_DEV_FLL,
    CORE_V_DEV_RAM,
    CORE_V_DEV_GPIO,
    CORE_V_DEV_UDMA,
    CORE_V_DEV_SOC_CTR,
    CORE_V_DEV_ATIMER,
    CORE_V_DEV_EVT_GEN,
    CORE_V_DEV_I2CS,
    CORE_V_DEV_SPI,
    CORE_V_DEV_TIMER,
    CORE_V_DEV_STDOUT,
    CORE_V_DEV_DEBUG
};

enum {
    IRQ_LOCAL_TIMER_LO = 16,
    IRQ_LOCAL_TIMER_HI = 17,
    IRQ_LOCAL_REF_RISE = 18,
    IRQ_LOCAL_REF_FALL = 19,
    IRQ_LOCAL_I2CS = 20,
    IRQ_LOCAL_ATIMER_0 = 21,
    IRQ_LOCAL_ATIMER_1 = 22,
    IRQ_LOCAL_ATIMER_2 = 23,
    IRQ_LOCAL_ATIMER_3 = 24,
    IRQ_LOCAL_EFPGA_0 = 25,
    IRQ_LOCAL_EFPGA_1 = 26,
    IRQ_LOCAL_EFPGA_2 = 27,
    IRQ_LOCAL_EFPGA_3 = 28,
    IRQ_LOCAL_EFPGA_4 = 29,
    IRQ_LOCAL_EFPGA_5 = 30,
    IRQ_LOCAL_FC_ERR = 31
};

#ifdef ENABLE_BOOTROM
#define CORE_V_RESET_VECTOR    0x1A000080
#else
#define CORE_V_RESET_VECTOR    0x1C008080
#endif

#define D(X)
#define ENABLE_UDMA

#define DEBUG_I2C_SLAVE
#define DEBUG_SPI_FLASH

#endif /* HW_CORE_V_MCU_H */
