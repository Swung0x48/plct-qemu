/*
 * QEMU RISC-V Board Compatible with CORE-V-MCU platform
 *
 * Copyright (c) 2022 PLCT Lab
 *
 * Provides a board compatible with the CORE-V-MCU platform:
 *
 * 0) APB Timer Device
 * 1) Event Generator Device
 * 2) UDMA subsystem: contain ctrl, uart, i2cm, spi, sdio, filter
 * 3) APB Gpio Device
 * 4) APB Advanced Timer Device
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

#include "qemu/osdep.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/char/serial.h"
#include "hw/misc/unimp.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/core_v_mcu.h"
#include "hw/intc/core_v_evt_gen.h"
#include "hw/riscv/boot.h"
#include "chardev/char.h"
#include "sysemu/sysemu.h"

static const MemMapEntry core_v_mcu_memmap[] = {
    [CORE_V_DEV_ROM] =       { 0x1A000000,    0x40000 },
    [CORE_V_DEV_FLL] =       { 0x1A100000,     0x1000 },
    [CORE_V_DEV_GPIO] =      { 0x1A101000,     0x1000 },
    [CORE_V_DEV_UDMA] =      { 0x1A102000,     0x2000 },
    [CORE_V_DEV_SOC_CTR] =   { 0x1A104000,     0x1000 },
    [CORE_V_DEV_ATIMER] =    { 0x1A105000,     0x1000 },
    [CORE_V_DEV_EVT_GEN] =   { 0x1A106000,     0x1000 },
    [CORE_V_DEV_I2CS] =      { 0x1A107000,     0x1000 },
    [CORE_V_DEV_TIMER] =     { 0x1A10B000,     0x1000 },
    [CORE_V_DEV_STDOUT] =    { 0x1A10F000,     0x1000 },
    [CORE_V_DEV_DEBUG] =     { 0x1A110000,    0x10000 },
    [CORE_V_DEV_RAM] =       { 0x1C000000,    0x90000 }
};

static void core_v_mcu_machine_init(MachineState *machine)
{
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    COREVMCUState *s = CORE_V_MCU_MACHINE(machine);
    MemoryRegion *sys_mem = get_system_memory();

    if (machine->ram_size != mc->default_ram_size) {
        char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    /* Initialize SoC */
    object_initialize_child(OBJECT(machine), "soc", &s->soc,
                            TYPE_CORE_V_MCU_SOC);
    qdev_realize(DEVICE(&s->soc), NULL, &error_fatal);

    memory_region_add_subregion(sys_mem,
                                core_v_mcu_memmap[CORE_V_DEV_RAM].base,
                                machine->ram);

    if (machine->kernel_filename) {
        riscv_load_kernel(machine->kernel_filename,
                          core_v_mcu_memmap[CORE_V_DEV_RAM].base, NULL);
    }
}

static void core_v_mcu_machine_instance_init(Object *obj)
{
}

static void core_v_mcu_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "CORE-V MCU board";
    mc->init = core_v_mcu_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = CORE_V_CV32E40P_CPU;
    mc->default_ram_id = "core-v.mcu.ram";
    mc->default_ram_size = core_v_mcu_memmap[CORE_V_DEV_RAM].size;
}

static const TypeInfo core_v_mcu_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("core_v_mcu"),
    .parent     = TYPE_MACHINE,
    .class_init = core_v_mcu_machine_class_init,
    .instance_init = core_v_mcu_machine_instance_init,
    .instance_size = sizeof(COREVMCUState),
};

static void core_v_mcu_machine_init_register_types(void)
{
    type_register_static(&core_v_mcu_machine_typeinfo);
}

type_init(core_v_mcu_machine_init_register_types)

static void core_v_mcu_soc_init(Object *obj)
{
    COREVMCUSoCState *s = CORE_V_MCU_SOC(obj);
    int i;

    object_initialize_child(obj, "cpus", &s->cpus, TYPE_RISCV_HART_ARRAY);

    object_initialize_child(obj, "timer", &s->timer, TYPE_CORE_V_TIMER);

    object_initialize_child(obj, "apb-adv-timer", &s->adv_timer,
                            TYPE_CORE_V_APB_ADV_TIMER);

    object_initialize_child(obj, "evt_gen", &s->evt_gen, TYPE_CORE_V_EVT_GEN);

    object_initialize_child(obj, "udma_ctrl", &s->udma_ctrl,
                            TYPE_CORE_V_UDMA_CTRL);

    for (i = 0; i < UDMA_N_UART; i++) {
        char name[32];
        sprintf(name, "udma_uart%d", i);
        object_initialize_child(obj, name, &s->udma_uart[i],
                                TYPE_CORE_V_UDMA_UART);
    }

    for (i = 0; i < UDMA_N_SPI; i++) {
        char name[32];
        sprintf(name, "udma_spi%d", i);
        object_initialize_child(obj, name, &s->udma_spi[i],
                                TYPE_CORE_V_UDMA_SPI);
    }

    for (i = 0; i < UDMA_N_I2C; i++) {
        char name[32];
        sprintf(name, "udma_i2c%d", i);
        object_initialize_child(obj, name, &s->udma_i2c[i],
                                TYPE_CORE_V_UDMA_I2C);
    }

    for (i = 0; i < UDMA_N_SDIO; i++) {
        char name[32];
        sprintf(name, "udma_sdio%d", i);
        object_initialize_child(obj, name, &s->udma_sdio[i],
                                TYPE_CORE_V_UDMA_SDIO);
    }

    object_initialize_child(obj, "gpio", &s->gpio, TYPE_CORE_V_GPIO);
    object_initialize_child(obj, "i2cs", &s->i2cs, TYPE_CORE_V_I2CS);
}

static void core_v_mcu_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    const MemMapEntry *memmap = core_v_mcu_memmap;
    COREVMCUSoCState *s = CORE_V_MCU_SOC(dev);
    MemoryRegion *sys_mem = get_system_memory();
    int i, j;

    object_property_set_str(OBJECT(&s->cpus), "cpu-type", ms->cpu_type,
                            &error_abort);
    object_property_set_int(OBJECT(&s->cpus), "num-harts", 1,
                            &error_abort);
    object_property_set_int(OBJECT(&s->cpus), "resetvec", CORE_V_RESET_VECTOR,
                            &error_abort);

    sysbus_realize(SYS_BUS_DEVICE(&s->cpus), &error_abort);

    /* ROM */
    memory_region_init_rom(&s->rom, OBJECT(dev), "core-v.mcu.rom",
                           memmap[CORE_V_DEV_ROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[CORE_V_DEV_ROM].base,
                                &s->rom);

    /* Timer */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->timer), &error_abort)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->timer), 0,
                    memmap[CORE_V_DEV_TIMER].base);
    qdev_connect_gpio_out(DEVICE(&s->timer), 0,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_M_TIMER));
    qdev_connect_gpio_out(DEVICE(&s->timer), 0,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_TIMER_LO));
    qdev_connect_gpio_out(DEVICE(&s->timer), 1,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_TIMER_HI));

    /* APB Adv Timer */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->adv_timer), &error_abort)) {
        return;
    }
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->adv_timer), 0,
                    memmap[CORE_V_DEV_ATIMER].base);
    qdev_connect_gpio_out(DEVICE(&s->adv_timer), 0,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_ATIMER_0));
    qdev_connect_gpio_out(DEVICE(&s->adv_timer), 1,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_ATIMER_1));
    qdev_connect_gpio_out(DEVICE(&s->adv_timer), 2,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_ATIMER_2));
    qdev_connect_gpio_out(DEVICE(&s->adv_timer), 3,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_ATIMER_3));

    /* Event Generator */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->evt_gen), &error_abort)) {
        return;
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(&s->evt_gen), 0,
                    memmap[CORE_V_DEV_EVT_GEN].base);
    qdev_connect_gpio_out(DEVICE(&s->evt_gen), 0,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_M_EXT));
    qdev_connect_gpio_out(DEVICE(&s->evt_gen), 1,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_FC_ERR));
    qdev_connect_gpio_out(DEVICE(&s->evt_gen), 2,
                          qdev_get_gpio_in(DEVICE(&s->timer), 0));
    qdev_connect_gpio_out(DEVICE(&s->evt_gen), 3,
                          qdev_get_gpio_in(DEVICE(&s->timer), 1));

    /* UDMA Subsystem */
    /* UDMA Ctrl */
    if (!sysbus_realize(SYS_BUS_DEVICE(&s->udma_ctrl), &error_abort)) {
        return;
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(&s->udma_ctrl), 0,
                    memmap[CORE_V_DEV_UDMA].base);

    qdev_connect_gpio_out(DEVICE(&s->evt_gen), 4,
                          qdev_get_gpio_in(DEVICE(&s->udma_ctrl), 0));

    /* UDMA UART */
    for (i = 0; i < UDMA_N_UART; i++) {
        qdev_prop_set_chr(DEVICE(&s->udma_uart[i]), "chardev", serial_hd(i));

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->udma_uart[i]), &error_abort)) {
            return;
        }

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->udma_uart[i]), 0,
                        memmap[CORE_V_DEV_UDMA].base + UDMA_UART_OFFSET +
                        i * UDMA_PER_SIZE);
        for (j = 0; j < UDMA_PER_EVENT_NUM; j++) {
            qdev_connect_gpio_out(DEVICE(&s->udma_uart[i]), j,
                                  qdev_get_gpio_in(DEVICE(&s->evt_gen),
                                                   4 * (UDMA_UART_ID + i) + j)
                                 );
        }

        qdev_connect_gpio_out_named(DEVICE(&s->udma_ctrl), "gate",
                                    UDMA_UART_ID + i,
                                    qdev_get_gpio_in(DEVICE(&s->udma_uart[i]),
                                                     0));
    }

    /* UDMA SPI */
    for (i = 0; i < UDMA_N_SPI; i++) {

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->udma_spi[i]), &error_abort)) {
            return;
        }

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->udma_spi[i]), 0,
                        memmap[CORE_V_DEV_UDMA].base + UDMA_SPI_OFFSET +
                        i * UDMA_PER_SIZE);
        for (j = 0; j < UDMA_PER_EVENT_NUM; j++) {
            qdev_connect_gpio_out(DEVICE(&s->udma_spi[i]), j,
                                  qdev_get_gpio_in(DEVICE(&s->evt_gen),
                                                   4 * (UDMA_SPI_ID + i) + j)
                                 );
        }

        for (j = 0; j < UDMA_CTRL_EVENT_NUM; j++) {
            qdev_connect_gpio_out_named(DEVICE(&s->udma_ctrl), "event", j,
                                        qdev_get_gpio_in(
                                            DEVICE(&s->udma_spi[i]), j));
        }

        qdev_connect_gpio_out_named(DEVICE(&s->udma_ctrl), "gate",
                                    UDMA_SPI_ID + i,
                                    qdev_get_gpio_in(
                                        DEVICE(&s->udma_spi[i]),
                                        UDMA_CTRL_EVENT_NUM));
    }

    /* UDMA I2C */
    for (i = 0; i < UDMA_N_I2C; i++) {

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->udma_i2c[i]), &error_abort)) {
            return;
        }

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->udma_i2c[i]), 0,
                        memmap[CORE_V_DEV_UDMA].base + UDMA_I2CM_OFFSET +
                        i * UDMA_PER_SIZE);
        for (j = 0; j < UDMA_PER_EVENT_NUM; j++) {
            qdev_connect_gpio_out(DEVICE(&s->udma_i2c[i]), j,
                                  qdev_get_gpio_in(DEVICE(&s->evt_gen),
                                                   4 * (UDMA_I2CM_ID + i) + j)
                                 );
        }

        for (j = 0; j < UDMA_CTRL_EVENT_NUM; j++) {
            qdev_connect_gpio_out_named(DEVICE(&s->udma_ctrl), "event", j,
                                        qdev_get_gpio_in(
                                            DEVICE(&s->udma_i2c[i]), j));
        }

        qdev_connect_gpio_out_named(DEVICE(&s->udma_ctrl), "gate",
                                    UDMA_I2CM_ID + i,
                                    qdev_get_gpio_in(DEVICE(&s->udma_i2c[i]),
                                                     UDMA_CTRL_EVENT_NUM));
    }

    /* UDMA SDIO */
    for (i = 0; i < UDMA_N_SDIO; i++) {

        if (!sysbus_realize(SYS_BUS_DEVICE(&s->udma_sdio[i]), &error_abort)) {
            return;
        }

        sysbus_mmio_map(SYS_BUS_DEVICE(&s->udma_sdio[i]), 0,
                        memmap[CORE_V_DEV_UDMA].base + UDMA_SDIO_OFFSET +
                        i * UDMA_PER_SIZE);
        for (j = 0; j < UDMA_PER_EVENT_NUM; j++) {
            qdev_connect_gpio_out(DEVICE(&s->udma_sdio[i]), j,
                                  qdev_get_gpio_in(DEVICE(&s->evt_gen),
                                                   4 * (UDMA_SDIO_ID + i) + j)
                                 );
        }

        qdev_connect_gpio_out_named(DEVICE(&s->udma_ctrl), "gate",
                                    UDMA_SDIO_ID + i,
                                    qdev_get_gpio_in(DEVICE(&s->udma_sdio[i]),
                                                     0));
    }

    create_unimplemented_device("core-v.udma.extra",
                                memmap[CORE_V_DEV_UDMA].base +
                                UDMA_SDIO_OFFSET +
                                UDMA_N_SDIO * UDMA_PER_SIZE,
                                memmap[CORE_V_DEV_UDMA].size -
                                UDMA_SDIO_OFFSET -
                                UDMA_N_SDIO * UDMA_PER_SIZE);

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), &error_abort)) {
        return;
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0,
                    memmap[CORE_V_DEV_GPIO].base);
    for (j = 0; j < COREV_N_GPIO; j++) {
        qdev_connect_gpio_out(DEVICE(&s->gpio), j,
                              qdev_get_gpio_in(DEVICE(&s->evt_gen), 128 + j));
    }

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->i2cs), &error_abort)) {
        return;
    }

    sysbus_mmio_map(SYS_BUS_DEVICE(&s->i2cs), 0,
                    memmap[CORE_V_DEV_I2CS].base);

    corev_i2cs_init_one(s->udma_i2c[0].bus, DEFAULT_I2CS_ADDRESS, &s->i2cs);

    qdev_connect_gpio_out(DEVICE(&s->i2cs), 0,
                          qdev_get_gpio_in(DEVICE(&s->evt_gen), 128 + 18));

    qdev_connect_gpio_out(DEVICE(&s->i2cs), 1,
                          qdev_get_gpio_in(DEVICE(qemu_get_cpu(0)),
                                           IRQ_LOCAL_I2CS));

    create_unimplemented_device("core-v.stdout",
                                memmap[CORE_V_DEV_STDOUT].base,
                                memmap[CORE_V_DEV_STDOUT].size);
    create_unimplemented_device("core-v.fll",
                                memmap[CORE_V_DEV_FLL].base,
                                memmap[CORE_V_DEV_FLL].size);
    create_unimplemented_device("core-v.soc.ctl",
                                memmap[CORE_V_DEV_SOC_CTR].base,
                                memmap[CORE_V_DEV_SOC_CTR].size);
    create_unimplemented_device("core-v.debug",
                                memmap[CORE_V_DEV_DEBUG].base,
                                memmap[CORE_V_DEV_DEBUG].size);
}

static void core_v_mcu_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = core_v_mcu_soc_realize;
    /* Reason: Uses serial_hds in realize function, thus can't be used twice */
    dc->user_creatable = false;
}

static const TypeInfo core_v_mcu_soc_type_info = {
    .name = TYPE_CORE_V_MCU_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(COREVMCUSoCState),
    .instance_init = core_v_mcu_soc_init,
    .class_init = core_v_mcu_soc_class_init,
};

static void core_v_mcu_soc_register_types(void)
{
    type_register_static(&core_v_mcu_soc_type_info);
}

type_init(core_v_mcu_soc_register_types)
