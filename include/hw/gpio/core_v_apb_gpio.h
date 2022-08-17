/*
 * QEMU CORE-V MCU APB GPIO Controller
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

#ifndef HW_COREV_GPIO_H
#define HW_COREV_GPIO_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define COREV_N_GPIO 32

struct CoreVGpioState {
    SysBusDevice parent_obj;

    MemoryRegion iomem;

    uint8_t select;
    bool inten[COREV_N_GPIO];
    uint8_t inttype[COREV_N_GPIO];
    bool out[COREV_N_GPIO];
    uint8_t dir[COREV_N_GPIO];
    bool in[COREV_N_GPIO];
    bool rise[COREV_N_GPIO];
    bool fall[COREV_N_GPIO];
    bool block_int[COREV_N_GPIO];

    qemu_irq irq[COREV_N_GPIO];
};

#define TYPE_CORE_V_GPIO "core_v_apb_gpio"
OBJECT_DECLARE_SIMPLE_TYPE(CoreVGpioState, CORE_V_GPIO)

#endif
