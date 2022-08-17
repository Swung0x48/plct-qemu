/*
 * CORE-V Event Generator
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

#ifndef HW_CORE_V_EVT_GEN_H
#define HW_CORE_V_EVT_GEN_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define EVENT_NUM 168
#define TYPE_CORE_V_EVT_GEN "core_v_evt_gen"
OBJECT_DECLARE_SIMPLE_TYPE(CoreVEvtGenState, CORE_V_EVT_GEN)

struct CoreVEvtGenState {
    SysBusDevice parent_obj;

    MemoryRegion mem;
    uint8_t sw_event;
    uint32_t fc_mask[8];
    uint32_t pr_mask[8];
    uint32_t cl_mask[8];
    uint32_t err_reg[8];
    uint8_t timer_lo;
    uint8_t timer_hi;
    uint8_t round;
    uint8_t events[EVENT_NUM];
    bool fc_ready;

    QEMUTimer r_timer;

    /*
     * IRQ 0 and 1 are for FC
     * IRQ 2 and 3 are for timer
     * IRQ 4 are for PR
     */
    qemu_irq irq[5];
};

#endif /* HW_CORE_V_EVT_GEN_H */
