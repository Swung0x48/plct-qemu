/*
 * CORE-V MCU APB Timer
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

#ifndef CORE_V_TIMER_H
#define CORE_V_TIMER_H

#include "hw/sysbus.h"
#include "qom/object.h"

#define TYPE_CORE_V_TIMER "core-v.mcu.timer"
OBJECT_DECLARE_SIMPLE_TYPE(CoreVTimerState, CORE_V_TIMER)


struct CoreVTimerState {
    /*< private >*/
    SysBusDevice parent_obj;
    /*< public >*/

    MemoryRegion iomem;

    QEMUTimer timer[2];

    uint32_t cfg[2];
    uint32_t cnt[2];
    uint32_t cmp[2];

    uint32_t freq[2];
    uint64_t start_time[2];

    qemu_irq irq[2];
};

#endif /* CORE_V_TIMER_H */
