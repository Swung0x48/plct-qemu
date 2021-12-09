/*
 * QEMU RISC-V VirtIO Board
 *
 * Copyright (c) 2021 PLCT, lab.
 * Copyright (c) 2017 SiFive, Inc.
 *
 * RISC-V machine with 16550a UART and VirtIO MMIO
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
#include "qemu/units.h"
#include "qemu/cutils.h"
#include "qemu/error-report.h"
#include "qapi/error.h"
#include "hw/boards.h"
#include "hw/loader.h"
#include "hw/sysbus.h"
#include "hw/qdev-properties.h"
#include "hw/char/serial.h"
#include "hw/misc/unimp.h"
#include "target/riscv/cpu.h"
#include "hw/riscv/riscv_hart.h"
#include "hw/riscv/plct_stone.h"
#include "hw/riscv/boot.h"
#include "hw/riscv/numa.h"
#include "hw/intc/riscv_aclint.h"
#include "hw/intc/sifive_plic.h"
#include "hw/misc/sifive_test.h"
#include "hw/char/sifive_uart.h"
#include "hw/misc/sifive_e_prci.h"
#include "chardev/char.h"
#include "sysemu/device_tree.h"
#include "sysemu/sysemu.h"
#include "hw/pci/pci.h"
#include "hw/pci-host/gpex.h"
#include "hw/display/ramfb.h"

static const MemMapEntry plct_stone_memmap[] = {
    [PLCT_STONE_DEBUG] =       {        0x0,         0x100 },
    [PLCT_STONE_MROM] =        {     0x1000,        0xf000 },
    [PLCT_STONE_TEST] =        {   0x100000,        0x1000 },
    [PLCT_STONE_RTC] =         {   0x101000,        0x1000 },
    [PLCT_STONE_CLINT] =       {  0x2000000,       0x10000 },
    [PLCT_STONE_ACLINT_SSWI] = {  0x2F00000,        0x4000 },
    [PLCT_STONE_PCIE_PIO] =    {  0x3000000,       0x10000 },
    [PLCT_STONE_PLIC] =        {  0xc000000, PLCT_STONE_PLIC_SIZE(PLCT_STONE_CPUS_MAX * 2) },
    [PLCT_STONE_UART0] =       { 0x10000000,         0x100 },
    [PLCT_STONE_VIRTIO] =      { 0x10001000,        0x1000 },
    [PLCT_STONE_FW_CFG] =      { 0x10100000,          0x18 },
    [PLCT_STONE_FLASH] =       { 0x20000000,     0x4000000 },
    [PLCT_STONE_PCIE_ECAM] =   { 0x30000000,    0x10000000 },
    [PLCT_STONE_PCIE_MMIO] =   { 0x40000000,    0x40000000 },
    [PLCT_STONE_DRAM] =        { 0x80000000,           0x0 },
};

/* PCIe high mmio is fixed for RV32 */
#define PLCT_STONE32_HIGH_PCIE_MMIO_BASE  0x300000000ULL
#define PLCT_STONE32_HIGH_PCIE_MMIO_SIZE  (4 * GiB)

/* PCIe high mmio for RV64, size is fixed but base depends on top of RAM */
#define PLCT_STONE64_HIGH_PCIE_MMIO_SIZE  (16 * GiB)

static MemMapEntry plct_stone_high_pcie_memmap;

#define PLCT_STONE_FLASH_SECTOR_SIZE (256 * KiB)

static PFlashCFI01 *plct_stone_flash_create1(RISCVStoneState *s,
                                       const char *name,
                                       const char *alias_prop_name)
{
    /*
     * Create a single flash device.  We use the same parameters as
     * the flash devices on the ARM virt board.
     */
    DeviceState *dev = qdev_new(TYPE_PFLASH_CFI01);

    qdev_prop_set_uint64(dev, "sector-length", PLCT_STONE_FLASH_SECTOR_SIZE);
    qdev_prop_set_uint8(dev, "width", 4);
    qdev_prop_set_uint8(dev, "device-width", 2);
    qdev_prop_set_bit(dev, "big-endian", false);
    qdev_prop_set_uint16(dev, "id0", 0x89);
    qdev_prop_set_uint16(dev, "id1", 0x18);
    qdev_prop_set_uint16(dev, "id2", 0x00);
    qdev_prop_set_uint16(dev, "id3", 0x00);
    qdev_prop_set_string(dev, "name", name);

    object_property_add_child(OBJECT(s), name, OBJECT(dev));
    object_property_add_alias(OBJECT(s), alias_prop_name,
                              OBJECT(dev), "drive");

    return PFLASH_CFI01(dev);
}

static void plct_stone_flash_create(RISCVStoneState *s)
{
    s->flash[0] = plct_stone_flash_create1(s, "stone.flash0", "pflash0");
    s->flash[1] = plct_stone_flash_create1(s, "stone.flash1", "pflash1");
}

static void plct_stone_flash_map1(PFlashCFI01 *flash,
                            hwaddr base, hwaddr size,
                            MemoryRegion *sysmem)
{
    DeviceState *dev = DEVICE(flash);

    assert(QEMU_IS_ALIGNED(size, PLCT_STONE_FLASH_SECTOR_SIZE));
    assert(size / PLCT_STONE_FLASH_SECTOR_SIZE <= UINT32_MAX);
    qdev_prop_set_uint32(dev, "num-blocks", size / PLCT_STONE_FLASH_SECTOR_SIZE);
    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    memory_region_add_subregion(sysmem, base,
                                sysbus_mmio_get_region(SYS_BUS_DEVICE(dev),
                                                       0));
}

static void plct_stone_flash_map(RISCVStoneState *s,
                           MemoryRegion *sysmem)
{
    hwaddr flashsize = plct_stone_memmap[PLCT_STONE_FLASH].size / 2;
    hwaddr flashbase = plct_stone_memmap[PLCT_STONE_FLASH].base;

    plct_stone_flash_map1(s->flash[0], flashbase, flashsize,
                    sysmem);
    plct_stone_flash_map1(s->flash[1], flashbase + flashsize, flashsize,
                    sysmem);
}

static void create_pcie_irq_map(void *fdt, char *nodename,
                                uint32_t plic_phandle)
{
    int pin, dev;
    uint32_t
        full_irq_map[GPEX_NUM_IRQS * GPEX_NUM_IRQS * FDT_INT_MAP_WIDTH] = {};
    uint32_t *irq_map = full_irq_map;

    /* This code creates a standard swizzle of interrupts such that
     * each device's first interrupt is based on it's PCI_SLOT number.
     * (See pci_swizzle_map_irq_fn())
     *
     * We only need one entry per interrupt in the table (not one per
     * possible slot) seeing the interrupt-map-mask will allow the table
     * to wrap to any number of devices.
     */
    for (dev = 0; dev < GPEX_NUM_IRQS; dev++) {
        int devfn = dev * 0x8;

        for (pin = 0; pin < GPEX_NUM_IRQS; pin++) {
            int irq_nr = PCIE_IRQ + ((pin + PCI_SLOT(devfn)) % GPEX_NUM_IRQS);
            int i = 0;

            irq_map[i] = cpu_to_be32(devfn << 8);

            i += FDT_PCI_ADDR_CELLS;
            irq_map[i] = cpu_to_be32(pin + 1);

            i += FDT_PCI_INT_CELLS;
            irq_map[i++] = cpu_to_be32(plic_phandle);

            i += FDT_PLIC_ADDR_CELLS;
            irq_map[i] = cpu_to_be32(irq_nr);

            irq_map += FDT_INT_MAP_WIDTH;
        }
    }

    qemu_fdt_setprop(fdt, nodename, "interrupt-map",
                     full_irq_map, sizeof(full_irq_map));

    qemu_fdt_setprop_cells(fdt, nodename, "interrupt-map-mask",
                           0x1800, 0, 0, 0x7);
}

static void create_fdt_socket_cpus(RISCVStoneState *s, int socket,
                                   char *clust_name, uint32_t *phandle,
                                   bool is_32_bit, uint32_t *intc_phandles)
{
    int cpu;
    uint32_t cpu_phandle;
    MachineState *mc = MACHINE(s);
    char *name, *cpu_name, *core_name, *intc_name;

    for (cpu = s->soc[socket].num_harts - 1; cpu >= 0; cpu--) {
        cpu_phandle = (*phandle)++;

        cpu_name = g_strdup_printf("/cpus/cpu@%d",
            s->soc[socket].hartid_base + cpu);
        qemu_fdt_add_subnode(mc->fdt, cpu_name);
        qemu_fdt_setprop_string(mc->fdt, cpu_name, "mmu-type",
            (is_32_bit) ? "riscv,sv32" : "riscv,sv48");
        name = riscv_isa_string(&s->soc[socket].harts[cpu]);
        qemu_fdt_setprop_string(mc->fdt, cpu_name, "riscv,isa", name);
        g_free(name);
        qemu_fdt_setprop_string(mc->fdt, cpu_name, "compatible", "riscv");
        qemu_fdt_setprop_string(mc->fdt, cpu_name, "status", "okay");
        qemu_fdt_setprop_cell(mc->fdt, cpu_name, "reg",
            s->soc[socket].hartid_base + cpu);
        qemu_fdt_setprop_string(mc->fdt, cpu_name, "device_type", "cpu");
        riscv_socket_fdt_write_id(mc, mc->fdt, cpu_name, socket);
        qemu_fdt_setprop_cell(mc->fdt, cpu_name, "phandle", cpu_phandle);

        intc_phandles[cpu] = (*phandle)++;

        intc_name = g_strdup_printf("%s/interrupt-controller", cpu_name);
        qemu_fdt_add_subnode(mc->fdt, intc_name);
        qemu_fdt_setprop_cell(mc->fdt, intc_name, "phandle",
            intc_phandles[cpu]);
        qemu_fdt_setprop_string(mc->fdt, intc_name, "compatible",
            "riscv,cpu-intc");
        qemu_fdt_setprop(mc->fdt, intc_name, "interrupt-controller", NULL, 0);
        qemu_fdt_setprop_cell(mc->fdt, intc_name, "#interrupt-cells", 1);

        core_name = g_strdup_printf("%s/core%d", clust_name, cpu);
        qemu_fdt_add_subnode(mc->fdt, core_name);
        qemu_fdt_setprop_cell(mc->fdt, core_name, "cpu", cpu_phandle);

        g_free(core_name);
        g_free(intc_name);
        g_free(cpu_name);
    }
}

static void create_fdt_socket_memory(RISCVStoneState *s,
                                     const MemMapEntry *memmap, int socket)
{
    char *mem_name;
    uint64_t addr, size;
    MachineState *mc = MACHINE(s);

    addr = memmap[PLCT_STONE_DRAM].base + riscv_socket_mem_offset(mc, socket);
    size = riscv_socket_mem_size(mc, socket);
    mem_name = g_strdup_printf("/memory@%lx", (long)addr);
    qemu_fdt_add_subnode(mc->fdt, mem_name);
    qemu_fdt_setprop_cells(mc->fdt, mem_name, "reg",
        addr >> 32, addr, size >> 32, size);
    qemu_fdt_setprop_string(mc->fdt, mem_name, "device_type", "memory");
    riscv_socket_fdt_write_id(mc, mc->fdt, mem_name, socket);
    g_free(mem_name);
}

static void create_fdt_socket_clint(RISCVStoneState *s,
                                    const MemMapEntry *memmap, int socket,
                                    uint32_t *intc_phandles)
{
    int cpu;
    char *clint_name;
    uint32_t *clint_cells;
    unsigned long clint_addr;
    MachineState *mc = MACHINE(s);
    static const char * const clint_compat[2] = {
        "sifive,clint0", "riscv,clint0"
    };

    clint_cells = g_new0(uint32_t, s->soc[socket].num_harts * 4);

    for (cpu = 0; cpu < s->soc[socket].num_harts; cpu++) {
        clint_cells[cpu * 4 + 0] = cpu_to_be32(intc_phandles[cpu]);
        clint_cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_SOFT);
        clint_cells[cpu * 4 + 2] = cpu_to_be32(intc_phandles[cpu]);
        clint_cells[cpu * 4 + 3] = cpu_to_be32(IRQ_M_TIMER);
    }

    clint_addr = memmap[PLCT_STONE_CLINT].base + (memmap[PLCT_STONE_CLINT].size * socket);
    clint_name = g_strdup_printf("/soc/clint@%lx", clint_addr);
    qemu_fdt_add_subnode(mc->fdt, clint_name);
    qemu_fdt_setprop_string_array(mc->fdt, clint_name, "compatible",
                                  (char **)&clint_compat,
                                  ARRAY_SIZE(clint_compat));
    qemu_fdt_setprop_cells(mc->fdt, clint_name, "reg",
        0x0, clint_addr, 0x0, memmap[PLCT_STONE_CLINT].size);
    qemu_fdt_setprop(mc->fdt, clint_name, "interrupts-extended",
        clint_cells, s->soc[socket].num_harts * sizeof(uint32_t) * 4);
    riscv_socket_fdt_write_id(mc, mc->fdt, clint_name, socket);
    g_free(clint_name);

    g_free(clint_cells);
}

static void create_fdt_socket_aclint(RISCVStoneState *s,
                                     const MemMapEntry *memmap, int socket,
                                     uint32_t *intc_phandles)
{
    int cpu;
    char *name;
    unsigned long addr;
    uint32_t aclint_cells_size;
    uint32_t *aclint_mswi_cells;
    uint32_t *aclint_sswi_cells;
    uint32_t *aclint_mtimer_cells;
    MachineState *mc = MACHINE(s);

    aclint_mswi_cells = g_new0(uint32_t, s->soc[socket].num_harts * 2);
    aclint_mtimer_cells = g_new0(uint32_t, s->soc[socket].num_harts * 2);
    aclint_sswi_cells = g_new0(uint32_t, s->soc[socket].num_harts * 2);

    for (cpu = 0; cpu < s->soc[socket].num_harts; cpu++) {
        aclint_mswi_cells[cpu * 2 + 0] = cpu_to_be32(intc_phandles[cpu]);
        aclint_mswi_cells[cpu * 2 + 1] = cpu_to_be32(IRQ_M_SOFT);
        aclint_mtimer_cells[cpu * 2 + 0] = cpu_to_be32(intc_phandles[cpu]);
        aclint_mtimer_cells[cpu * 2 + 1] = cpu_to_be32(IRQ_M_TIMER);
        aclint_sswi_cells[cpu * 2 + 0] = cpu_to_be32(intc_phandles[cpu]);
        aclint_sswi_cells[cpu * 2 + 1] = cpu_to_be32(IRQ_S_SOFT);
    }
    aclint_cells_size = s->soc[socket].num_harts * sizeof(uint32_t) * 2;

    addr = memmap[PLCT_STONE_CLINT].base + (memmap[PLCT_STONE_CLINT].size * socket);
    name = g_strdup_printf("/soc/mswi@%lx", addr);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible", "riscv,aclint-mswi");
    qemu_fdt_setprop_cells(mc->fdt, name, "reg",
        0x0, addr, 0x0, RISCV_ACLINT_SWI_SIZE);
    qemu_fdt_setprop(mc->fdt, name, "interrupts-extended",
        aclint_mswi_cells, aclint_cells_size);
    qemu_fdt_setprop(mc->fdt, name, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cell(mc->fdt, name, "#interrupt-cells", 0);
    riscv_socket_fdt_write_id(mc, mc->fdt, name, socket);
    g_free(name);

    addr = memmap[PLCT_STONE_CLINT].base + RISCV_ACLINT_SWI_SIZE +
        (memmap[PLCT_STONE_CLINT].size * socket);
    name = g_strdup_printf("/soc/mtimer@%lx", addr);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible",
        "riscv,aclint-mtimer");
    qemu_fdt_setprop_cells(mc->fdt, name, "reg",
        0x0, addr + RISCV_ACLINT_DEFAULT_MTIME,
        0x0, memmap[PLCT_STONE_CLINT].size - RISCV_ACLINT_SWI_SIZE -
             RISCV_ACLINT_DEFAULT_MTIME,
        0x0, addr + RISCV_ACLINT_DEFAULT_MTIMECMP,
        0x0, RISCV_ACLINT_DEFAULT_MTIME);
    qemu_fdt_setprop(mc->fdt, name, "interrupts-extended",
        aclint_mtimer_cells, aclint_cells_size);
    riscv_socket_fdt_write_id(mc, mc->fdt, name, socket);
    g_free(name);

    addr = memmap[PLCT_STONE_ACLINT_SSWI].base +
        (memmap[PLCT_STONE_ACLINT_SSWI].size * socket);
    name = g_strdup_printf("/soc/sswi@%lx", addr);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible", "riscv,aclint-sswi");
    qemu_fdt_setprop_cells(mc->fdt, name, "reg",
        0x0, addr, 0x0, memmap[PLCT_STONE_ACLINT_SSWI].size);
    qemu_fdt_setprop(mc->fdt, name, "interrupts-extended",
        aclint_sswi_cells, aclint_cells_size);
    qemu_fdt_setprop(mc->fdt, name, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop_cell(mc->fdt, name, "#interrupt-cells", 0);
    riscv_socket_fdt_write_id(mc, mc->fdt, name, socket);
    g_free(name);

    g_free(aclint_mswi_cells);
    g_free(aclint_mtimer_cells);
    g_free(aclint_sswi_cells);
}

static void create_fdt_socket_plic(RISCVStoneState *s,
                                   const MemMapEntry *memmap, int socket,
                                   uint32_t *phandle, uint32_t *intc_phandles,
                                   uint32_t *plic_phandles)
{
    int cpu;
    char *plic_name;
    uint32_t *plic_cells;
    unsigned long plic_addr;
    MachineState *mc = MACHINE(s);
    static const char * const plic_compat[2] = {
        "sifive,plic-1.0.0", "riscv,plic0"
    };

    plic_cells = g_new0(uint32_t, s->soc[socket].num_harts * 4);

    for (cpu = 0; cpu < s->soc[socket].num_harts; cpu++) {
        plic_cells[cpu * 4 + 0] = cpu_to_be32(intc_phandles[cpu]);
        plic_cells[cpu * 4 + 1] = cpu_to_be32(IRQ_M_EXT);
        plic_cells[cpu * 4 + 2] = cpu_to_be32(intc_phandles[cpu]);
        plic_cells[cpu * 4 + 3] = cpu_to_be32(IRQ_S_EXT);
    }

    plic_phandles[socket] = (*phandle)++;
    plic_addr = memmap[PLCT_STONE_PLIC].base + (memmap[PLCT_STONE_PLIC].size * socket);
    plic_name = g_strdup_printf("/soc/plic@%lx", plic_addr);
    qemu_fdt_add_subnode(mc->fdt, plic_name);
    qemu_fdt_setprop_cell(mc->fdt, plic_name,
        "#address-cells", FDT_PLIC_ADDR_CELLS);
    qemu_fdt_setprop_cell(mc->fdt, plic_name,
        "#interrupt-cells", FDT_PLIC_INT_CELLS);
    qemu_fdt_setprop_string_array(mc->fdt, plic_name, "compatible",
                                  (char **)&plic_compat,
                                  ARRAY_SIZE(plic_compat));
    qemu_fdt_setprop(mc->fdt, plic_name, "interrupt-controller", NULL, 0);
    qemu_fdt_setprop(mc->fdt, plic_name, "interrupts-extended",
        plic_cells, s->soc[socket].num_harts * sizeof(uint32_t) * 4);
    qemu_fdt_setprop_cells(mc->fdt, plic_name, "reg",
        0x0, plic_addr, 0x0, memmap[PLCT_STONE_PLIC].size);
    qemu_fdt_setprop_cell(mc->fdt, plic_name, "riscv,ndev", VIRTIO_NDEV);
    riscv_socket_fdt_write_id(mc, mc->fdt, plic_name, socket);
    qemu_fdt_setprop_cell(mc->fdt, plic_name, "phandle",
        plic_phandles[socket]);
    g_free(plic_name);

    g_free(plic_cells);
}

static void create_fdt_sockets(RISCVStoneState *s, const MemMapEntry *memmap,
                               bool is_32_bit, uint32_t *phandle,
                               uint32_t *irq_mmio_phandle,
                               uint32_t *irq_pcie_phandle,
                               uint32_t *irq_virtio_phandle)
{
    int socket;
    char *clust_name;
    uint32_t *intc_phandles;
    MachineState *mc = MACHINE(s);
    uint32_t xplic_phandles[MAX_NODES];

    qemu_fdt_add_subnode(mc->fdt, "/cpus");
    qemu_fdt_setprop_cell(mc->fdt, "/cpus", "timebase-frequency",
                          RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ);
    qemu_fdt_setprop_cell(mc->fdt, "/cpus", "#size-cells", 0x0);
    qemu_fdt_setprop_cell(mc->fdt, "/cpus", "#address-cells", 0x1);
    qemu_fdt_add_subnode(mc->fdt, "/cpus/cpu-map");

    for (socket = (riscv_socket_count(mc) - 1); socket >= 0; socket--) {
        clust_name = g_strdup_printf("/cpus/cpu-map/cluster%d", socket);
        qemu_fdt_add_subnode(mc->fdt, clust_name);

        intc_phandles = g_new0(uint32_t, s->soc[socket].num_harts);

        create_fdt_socket_cpus(s, socket, clust_name, phandle,
            is_32_bit, intc_phandles);

        create_fdt_socket_memory(s, memmap, socket);

        if (s->have_aclint) {
            create_fdt_socket_aclint(s, memmap, socket, intc_phandles);
        } else {
            create_fdt_socket_clint(s, memmap, socket, intc_phandles);
        }

        create_fdt_socket_plic(s, memmap, socket, phandle,
            intc_phandles, xplic_phandles);

        g_free(intc_phandles);
        g_free(clust_name);
    }

    for (socket = 0; socket < riscv_socket_count(mc); socket++) {
        if (socket == 0) {
            *irq_mmio_phandle = xplic_phandles[socket];
            *irq_virtio_phandle = xplic_phandles[socket];
            *irq_pcie_phandle = xplic_phandles[socket];
        }
        if (socket == 1) {
            *irq_virtio_phandle = xplic_phandles[socket];
            *irq_pcie_phandle = xplic_phandles[socket];
        }
        if (socket == 2) {
            *irq_pcie_phandle = xplic_phandles[socket];
        }
    }

    riscv_socket_fdt_write_distance_matrix(mc, mc->fdt);
}

static void create_fdt_virtio(RISCVStoneState *s, const MemMapEntry *memmap,
                              uint32_t irq_virtio_phandle)
{
    int i;
    char *name;
    MachineState *mc = MACHINE(s);

    for (i = 0; i < VIRTIO_COUNT; i++) {
        name = g_strdup_printf("/soc/virtio_mmio@%lx",
            (long)(memmap[PLCT_STONE_VIRTIO].base + i * memmap[PLCT_STONE_VIRTIO].size));
        qemu_fdt_add_subnode(mc->fdt, name);
        qemu_fdt_setprop_string(mc->fdt, name, "compatible", "virtio,mmio");
        qemu_fdt_setprop_cells(mc->fdt, name, "reg",
            0x0, memmap[PLCT_STONE_VIRTIO].base + i * memmap[PLCT_STONE_VIRTIO].size,
            0x0, memmap[PLCT_STONE_VIRTIO].size);
        qemu_fdt_setprop_cell(mc->fdt, name, "interrupt-parent",
            irq_virtio_phandle);
        qemu_fdt_setprop_cell(mc->fdt, name, "interrupts", VIRTIO_IRQ + i);
        g_free(name);
    }
}

static void create_fdt_pcie(RISCVStoneState *s, const MemMapEntry *memmap,
                            uint32_t irq_pcie_phandle)
{
    char *name;
    MachineState *mc = MACHINE(s);

    name = g_strdup_printf("/soc/pci@%lx",
        (long) memmap[PLCT_STONE_PCIE_ECAM].base);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_cell(mc->fdt, name, "#address-cells",
        FDT_PCI_ADDR_CELLS);
    qemu_fdt_setprop_cell(mc->fdt, name, "#interrupt-cells",
        FDT_PCI_INT_CELLS);
    qemu_fdt_setprop_cell(mc->fdt, name, "#size-cells", 0x2);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible",
        "pci-host-ecam-generic");
    qemu_fdt_setprop_string(mc->fdt, name, "device_type", "pci");
    qemu_fdt_setprop_cell(mc->fdt, name, "linux,pci-domain", 0);
    qemu_fdt_setprop_cells(mc->fdt, name, "bus-range", 0,
        memmap[PLCT_STONE_PCIE_ECAM].size / PCIE_MMCFG_SIZE_MIN - 1);
    qemu_fdt_setprop(mc->fdt, name, "dma-coherent", NULL, 0);
    qemu_fdt_setprop_cells(mc->fdt, name, "reg", 0,
        memmap[PLCT_STONE_PCIE_ECAM].base, 0, memmap[PLCT_STONE_PCIE_ECAM].size);
    qemu_fdt_setprop_sized_cells(mc->fdt, name, "ranges",
        1, FDT_PCI_RANGE_IOPORT, 2, 0,
        2, memmap[PLCT_STONE_PCIE_PIO].base, 2, memmap[PLCT_STONE_PCIE_PIO].size,
        1, FDT_PCI_RANGE_MMIO,
        2, memmap[PLCT_STONE_PCIE_MMIO].base,
        2, memmap[PLCT_STONE_PCIE_MMIO].base, 2, memmap[PLCT_STONE_PCIE_MMIO].size,
        1, FDT_PCI_RANGE_MMIO_64BIT,
        2, plct_stone_high_pcie_memmap.base,
        2, plct_stone_high_pcie_memmap.base, 2, plct_stone_high_pcie_memmap.size);

    create_pcie_irq_map(mc->fdt, name, irq_pcie_phandle);
    g_free(name);
}

static void create_fdt_reset(RISCVStoneState *s, const MemMapEntry *memmap,
                             uint32_t *phandle)
{
    char *name;
    uint32_t test_phandle;
    MachineState *mc = MACHINE(s);

    test_phandle = (*phandle)++;
    name = g_strdup_printf("/soc/test@%lx",
        (long)memmap[PLCT_STONE_TEST].base);
    qemu_fdt_add_subnode(mc->fdt, name);
    {
        static const char * const compat[3] = {
            "sifive,test1", "sifive,test0", "syscon"
        };
        qemu_fdt_setprop_string_array(mc->fdt, name, "compatible",
                                      (char **)&compat, ARRAY_SIZE(compat));
    }
    qemu_fdt_setprop_cells(mc->fdt, name, "reg",
        0x0, memmap[PLCT_STONE_TEST].base, 0x0, memmap[PLCT_STONE_TEST].size);
    qemu_fdt_setprop_cell(mc->fdt, name, "phandle", test_phandle);
    test_phandle = qemu_fdt_get_phandle(mc->fdt, name);
    g_free(name);

    name = g_strdup_printf("/soc/reboot");
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible", "syscon-reboot");
    qemu_fdt_setprop_cell(mc->fdt, name, "regmap", test_phandle);
    qemu_fdt_setprop_cell(mc->fdt, name, "offset", 0x0);
    qemu_fdt_setprop_cell(mc->fdt, name, "value", FINISHER_RESET);
    g_free(name);

    name = g_strdup_printf("/soc/poweroff");
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible", "syscon-poweroff");
    qemu_fdt_setprop_cell(mc->fdt, name, "regmap", test_phandle);
    qemu_fdt_setprop_cell(mc->fdt, name, "offset", 0x0);
    qemu_fdt_setprop_cell(mc->fdt, name, "value", FINISHER_PASS);
    g_free(name);
}

static void create_fdt_uart(RISCVStoneState *s, const MemMapEntry *memmap,
                            uint32_t irq_mmio_phandle)
{
    char *name;
    MachineState *mc = MACHINE(s);

    name = g_strdup_printf("/soc/uart@%lx", (long)memmap[PLCT_STONE_UART0].base);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible", "ns16550a");
    qemu_fdt_setprop_cells(mc->fdt, name, "reg",
        0x0, memmap[PLCT_STONE_UART0].base,
        0x0, memmap[PLCT_STONE_UART0].size);
    qemu_fdt_setprop_cell(mc->fdt, name, "clock-frequency", 3686400);
    qemu_fdt_setprop_cell(mc->fdt, name, "interrupt-parent", irq_mmio_phandle);
    qemu_fdt_setprop_cell(mc->fdt, name, "interrupts", UART0_IRQ);

    qemu_fdt_add_subnode(mc->fdt, "/chosen");
    qemu_fdt_setprop_string(mc->fdt, "/chosen", "stdout-path", name);
    g_free(name);
}

static void create_fdt_rtc(RISCVStoneState *s, const MemMapEntry *memmap,
                           uint32_t irq_mmio_phandle)
{
    char *name;
    MachineState *mc = MACHINE(s);

    name = g_strdup_printf("/soc/rtc@%lx", (long)memmap[PLCT_STONE_RTC].base);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible",
        "google,goldfish-rtc");
    qemu_fdt_setprop_cells(mc->fdt, name, "reg",
        0x0, memmap[PLCT_STONE_RTC].base, 0x0, memmap[PLCT_STONE_RTC].size);
    qemu_fdt_setprop_cell(mc->fdt, name, "interrupt-parent",
        irq_mmio_phandle);
    qemu_fdt_setprop_cell(mc->fdt, name, "interrupts", RTC_IRQ);
    g_free(name);
}

static void create_fdt_flash(RISCVStoneState *s, const MemMapEntry *memmap)
{
    char *name;
    MachineState *mc = MACHINE(s);
    hwaddr flashsize = plct_stone_memmap[PLCT_STONE_FLASH].size / 2;
    hwaddr flashbase = plct_stone_memmap[PLCT_STONE_FLASH].base;

    name = g_strdup_printf("/flash@%" PRIx64, flashbase);
    qemu_fdt_add_subnode(mc->fdt, name);
    qemu_fdt_setprop_string(mc->fdt, name, "compatible", "cfi-flash");
    qemu_fdt_setprop_sized_cells(mc->fdt, name, "reg",
                                 2, flashbase, 2, flashsize,
                                 2, flashbase + flashsize, 2, flashsize);
    qemu_fdt_setprop_cell(mc->fdt, name, "bank-width", 4);
    g_free(name);
}

static void create_fdt(RISCVStoneState *s, const MemMapEntry *memmap,
                       uint64_t mem_size, const char *cmdline, bool is_32_bit)
{
    MachineState *mc = MACHINE(s);
    uint32_t phandle = 1, irq_mmio_phandle = 1;
    uint32_t irq_pcie_phandle = 1, irq_virtio_phandle = 1;

    if (mc->dtb) {
        mc->fdt = load_device_tree(mc->dtb, &s->fdt_size);
        if (!mc->fdt) {
            error_report("load_device_tree() failed");
            exit(1);
        }
        goto update_bootargs;
    } else {
        mc->fdt = create_device_tree(&s->fdt_size);
        if (!mc->fdt) {
            error_report("create_device_tree() failed");
            exit(1);
        }
    }

    qemu_fdt_setprop_string(mc->fdt, "/", "model", "riscv-virtio,qemu");
    qemu_fdt_setprop_string(mc->fdt, "/", "compatible", "riscv-virtio");
    qemu_fdt_setprop_cell(mc->fdt, "/", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(mc->fdt, "/", "#address-cells", 0x2);

    qemu_fdt_add_subnode(mc->fdt, "/soc");
    qemu_fdt_setprop(mc->fdt, "/soc", "ranges", NULL, 0);
    qemu_fdt_setprop_string(mc->fdt, "/soc", "compatible", "simple-bus");
    qemu_fdt_setprop_cell(mc->fdt, "/soc", "#size-cells", 0x2);
    qemu_fdt_setprop_cell(mc->fdt, "/soc", "#address-cells", 0x2);

    create_fdt_sockets(s, memmap, is_32_bit, &phandle,
        &irq_mmio_phandle, &irq_pcie_phandle, &irq_virtio_phandle);

    create_fdt_virtio(s, memmap, irq_virtio_phandle);

    create_fdt_pcie(s, memmap, irq_pcie_phandle);

    create_fdt_reset(s, memmap, &phandle);

    create_fdt_uart(s, memmap, irq_mmio_phandle);

    create_fdt_rtc(s, memmap, irq_mmio_phandle);

    create_fdt_flash(s, memmap);

update_bootargs:
    if (cmdline) {
        qemu_fdt_setprop_string(mc->fdt, "/chosen", "bootargs", cmdline);
    }
}

static inline DeviceState *gpex_pcie_init(MemoryRegion *sys_mem,
                                          hwaddr ecam_base, hwaddr ecam_size,
                                          hwaddr mmio_base, hwaddr mmio_size,
                                          hwaddr high_mmio_base,
                                          hwaddr high_mmio_size,
                                          hwaddr pio_base,
                                          DeviceState *plic)
{
    DeviceState *dev;
    MemoryRegion *ecam_alias, *ecam_reg;
    MemoryRegion *mmio_alias, *high_mmio_alias, *mmio_reg;
    qemu_irq irq;
    int i;

    dev = qdev_new(TYPE_GPEX_HOST);

    sysbus_realize_and_unref(SYS_BUS_DEVICE(dev), &error_fatal);

    ecam_alias = g_new0(MemoryRegion, 1);
    ecam_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 0);
    memory_region_init_alias(ecam_alias, OBJECT(dev), "pcie-ecam",
                             ecam_reg, 0, ecam_size);
    memory_region_add_subregion(get_system_memory(), ecam_base, ecam_alias);

    mmio_alias = g_new0(MemoryRegion, 1);
    mmio_reg = sysbus_mmio_get_region(SYS_BUS_DEVICE(dev), 1);
    memory_region_init_alias(mmio_alias, OBJECT(dev), "pcie-mmio",
                             mmio_reg, mmio_base, mmio_size);
    memory_region_add_subregion(get_system_memory(), mmio_base, mmio_alias);

    /* Map high MMIO space */
    high_mmio_alias = g_new0(MemoryRegion, 1);
    memory_region_init_alias(high_mmio_alias, OBJECT(dev), "pcie-mmio-high",
                             mmio_reg, high_mmio_base, high_mmio_size);
    memory_region_add_subregion(get_system_memory(), high_mmio_base,
                                high_mmio_alias);

    sysbus_mmio_map(SYS_BUS_DEVICE(dev), 2, pio_base);

    for (i = 0; i < GPEX_NUM_IRQS; i++) {
        irq = qdev_get_gpio_in(plic, PCIE_IRQ + i);

        sysbus_connect_irq(SYS_BUS_DEVICE(dev), i, irq);
        gpex_set_irq_num(GPEX_HOST(dev), i, PCIE_IRQ + i);
    }

    return dev;
}

static FWCfgState *create_fw_cfg(const MachineState *mc)
{
    hwaddr base = plct_stone_memmap[PLCT_STONE_FW_CFG].base;
    hwaddr size = plct_stone_memmap[PLCT_STONE_FW_CFG].size;
    FWCfgState *fw_cfg;
    char *nodename;

    fw_cfg = fw_cfg_init_mem_wide(base + 8, base, 8, base + 16,
                                  &address_space_memory);
    fw_cfg_add_i16(fw_cfg, FW_CFG_NB_CPUS, (uint16_t)mc->smp.cpus);

    nodename = g_strdup_printf("/fw-cfg@%" PRIx64, base);
    qemu_fdt_add_subnode(mc->fdt, nodename);
    qemu_fdt_setprop_string(mc->fdt, nodename,
                            "compatible", "qemu,fw-cfg-mmio");
    qemu_fdt_setprop_sized_cells(mc->fdt, nodename, "reg",
                                 2, base, 2, size);
    qemu_fdt_setprop(mc->fdt, nodename, "dma-coherent", NULL, 0);
    g_free(nodename);
    return fw_cfg;
}

static void plct_stone_machine_init(MachineState *machine)
{
    const MemMapEntry *memmap = plct_stone_memmap;
    RISCVStoneState *s = RISCV_PLCT_STONE_MACHINE(machine);
    MemoryRegion *system_memory = get_system_memory();
    MemoryRegion *mask_rom = g_new(MemoryRegion, 1);
    char *plic_hart_config, *soc_name;
    target_ulong start_addr = memmap[PLCT_STONE_DRAM].base;
    target_ulong firmware_end_addr, kernel_start_addr;
    uint32_t fdt_load_addr;
    uint64_t kernel_entry;
    DeviceState *mmio_plic, *virtio_plic, *pcie_plic;
    int i, base_hartid, hart_count;

    /* Check socket count limit */
    if (PLCT_STONE_SOCKETS_MAX < riscv_socket_count(machine)) {
        error_report("number of sockets/nodes should be less than %d",
            PLCT_STONE_SOCKETS_MAX);
        exit(1);
    }

    /* Initialize sockets */
    mmio_plic = virtio_plic = pcie_plic = NULL;
    for (i = 0; i < riscv_socket_count(machine); i++) {
        if (!riscv_socket_check_hartids(machine, i)) {
            error_report("discontinuous hartids in socket%d", i);
            exit(1);
        }

        base_hartid = riscv_socket_first_hartid(machine, i);
        if (base_hartid < 0) {
            error_report("can't find hartid base for socket%d", i);
            exit(1);
        }

        hart_count = riscv_socket_hart_count(machine, i);
        if (hart_count < 0) {
            error_report("can't find hart count for socket%d", i);
            exit(1);
        }

        soc_name = g_strdup_printf("soc%d", i);
        object_initialize_child(OBJECT(machine), soc_name, &s->soc[i],
                                TYPE_RISCV_HART_ARRAY);
        g_free(soc_name);
        object_property_set_str(OBJECT(&s->soc[i]), "cpu-type",
                                machine->cpu_type, &error_abort);
        object_property_set_int(OBJECT(&s->soc[i]), "hartid-base",
                                base_hartid, &error_abort);
        object_property_set_int(OBJECT(&s->soc[i]), "num-harts",
                                hart_count, &error_abort);
        sysbus_realize(SYS_BUS_DEVICE(&s->soc[i]), &error_abort);

        /* Per-socket CLINT */
        riscv_aclint_swi_create(
            memmap[PLCT_STONE_CLINT].base + i * memmap[PLCT_STONE_CLINT].size,
            base_hartid, hart_count, false);
        riscv_aclint_mtimer_create(
            memmap[PLCT_STONE_CLINT].base + i * memmap[PLCT_STONE_CLINT].size +
                RISCV_ACLINT_SWI_SIZE,
            RISCV_ACLINT_DEFAULT_MTIMER_SIZE, base_hartid, hart_count,
            RISCV_ACLINT_DEFAULT_MTIMECMP, RISCV_ACLINT_DEFAULT_MTIME,
            RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, true);

        /* Per-socket ACLINT SSWI */
        if (s->have_aclint) {
            riscv_aclint_swi_create(
                memmap[PLCT_STONE_ACLINT_SSWI].base +
                    i * memmap[PLCT_STONE_ACLINT_SSWI].size,
                base_hartid, hart_count, true);
        }

        /* Per-socket PLIC hart topology configuration string */
        plic_hart_config = riscv_plic_hart_config_string(hart_count);

        /* Per-socket PLIC */
        s->plic[i] = sifive_plic_create(
            memmap[PLCT_STONE_PLIC].base + i * memmap[PLCT_STONE_PLIC].size,
            plic_hart_config, hart_count, base_hartid,
            PLCT_STONE_PLIC_NUM_SOURCES,
            PLCT_STONE_PLIC_NUM_PRIORITIES,
            PLCT_STONE_PLIC_PRIORITY_BASE,
            PLCT_STONE_PLIC_PENDING_BASE,
            PLCT_STONE_PLIC_ENABLE_BASE,
            PLCT_STONE_PLIC_ENABLE_STRIDE,
            PLCT_STONE_PLIC_CONTEXT_BASE,
            PLCT_STONE_PLIC_CONTEXT_STRIDE,
            memmap[PLCT_STONE_PLIC].size);
        g_free(plic_hart_config);

        /* Try to use different PLIC instance based device type */
        if (i == 0) {
            mmio_plic = s->plic[i];
            virtio_plic = s->plic[i];
            pcie_plic = s->plic[i];
        }
        if (i == 1) {
            virtio_plic = s->plic[i];
            pcie_plic = s->plic[i];
        }
        if (i == 2) {
            pcie_plic = s->plic[i];
        }
    }

    if (riscv_is_32bit(&s->soc[0])) {
#if HOST_LONG_BITS == 64
        /* limit RAM size in a 32-bit system */
        if (machine->ram_size > 10 * GiB) {
            machine->ram_size = 10 * GiB;
            error_report("Limiting RAM size to 10 GiB");
        }
#endif
        plct_stone_high_pcie_memmap.base = PLCT_STONE32_HIGH_PCIE_MMIO_BASE;
        plct_stone_high_pcie_memmap.size = PLCT_STONE32_HIGH_PCIE_MMIO_SIZE;
    } else {
        plct_stone_high_pcie_memmap.size = PLCT_STONE64_HIGH_PCIE_MMIO_SIZE;
        plct_stone_high_pcie_memmap.base = memmap[PLCT_STONE_DRAM].base + machine->ram_size;
        plct_stone_high_pcie_memmap.base =
            ROUND_UP(plct_stone_high_pcie_memmap.base, plct_stone_high_pcie_memmap.size);
    }

    /* register system main memory (actual RAM) */
    memory_region_add_subregion(system_memory, memmap[PLCT_STONE_DRAM].base,
        machine->ram);

    /* create device tree */
    create_fdt(s, memmap, machine->ram_size, machine->kernel_cmdline,
               riscv_is_32bit(&s->soc[0]));

    /* boot rom */
    memory_region_init_rom(mask_rom, NULL, "riscv_plct_stone_board.mrom",
                           memmap[PLCT_STONE_MROM].size, &error_fatal);
    memory_region_add_subregion(system_memory, memmap[PLCT_STONE_MROM].base,
                                mask_rom);

    if (riscv_is_32bit(&s->soc[0])) {
        firmware_end_addr = riscv_find_and_load_firmware(machine,
                                    RISCV32_BIOS_BIN, start_addr, NULL);
    } else {
        firmware_end_addr = riscv_find_and_load_firmware(machine,
                                    RISCV64_BIOS_BIN, start_addr, NULL);
    }

    if (machine->kernel_filename) {
        kernel_start_addr = riscv_calc_kernel_start_addr(&s->soc[0],
                                                         firmware_end_addr);

        kernel_entry = riscv_load_kernel(machine->kernel_filename,
                                         kernel_start_addr, NULL);

        if (machine->initrd_filename) {
            hwaddr start;
            hwaddr end = riscv_load_initrd(machine->initrd_filename,
                                           machine->ram_size, kernel_entry,
                                           &start);
            qemu_fdt_setprop_cell(machine->fdt, "/chosen",
                                  "linux,initrd-start", start);
            qemu_fdt_setprop_cell(machine->fdt, "/chosen", "linux,initrd-end",
                                  end);
        }
    } else {
       /*
        * If dynamic firmware is used, it doesn't know where is the next mode
        * if kernel argument is not set.
        */
        kernel_entry = 0;
    }

    if (drive_get(IF_PFLASH, 0, 0)) {
        /*
         * Pflash was supplied, let's overwrite the address we jump to after
         * reset to the base of the flash.
         */
        start_addr = plct_stone_memmap[PLCT_STONE_FLASH].base;
    }

    /*
     * Init fw_cfg.  Must be done before riscv_load_fdt, otherwise the device
     * tree cannot be altered and we get FDT_ERR_NOSPACE.
     */
    s->fw_cfg = create_fw_cfg(machine);
    rom_set_fw(s->fw_cfg);

    /* Compute the fdt load address in dram */
    fdt_load_addr = riscv_load_fdt(memmap[PLCT_STONE_DRAM].base,
                                   machine->ram_size, machine->fdt);
    /* load the reset vector */
    riscv_setup_rom_reset_vec(machine, &s->soc[0], start_addr,
                              plct_stone_memmap[PLCT_STONE_MROM].base,
                              plct_stone_memmap[PLCT_STONE_MROM].size, kernel_entry,
                              fdt_load_addr, machine->fdt);

    /* SiFive Test MMIO device */
    sifive_test_create(memmap[PLCT_STONE_TEST].base);

    /* VirtIO MMIO devices */
    for (i = 0; i < VIRTIO_COUNT; i++) {
        sysbus_create_simple("virtio-mmio",
            memmap[PLCT_STONE_VIRTIO].base + i * memmap[PLCT_STONE_VIRTIO].size,
            qdev_get_gpio_in(DEVICE(virtio_plic), VIRTIO_IRQ + i));
    }

    gpex_pcie_init(system_memory,
                   memmap[PLCT_STONE_PCIE_ECAM].base,
                   memmap[PLCT_STONE_PCIE_ECAM].size,
                   memmap[PLCT_STONE_PCIE_MMIO].base,
                   memmap[PLCT_STONE_PCIE_MMIO].size,
                   plct_stone_high_pcie_memmap.base,
                   plct_stone_high_pcie_memmap.size,
                   memmap[PLCT_STONE_PCIE_PIO].base,
                   DEVICE(pcie_plic));

    serial_mm_init(system_memory, memmap[PLCT_STONE_UART0].base,
        0, qdev_get_gpio_in(DEVICE(mmio_plic), UART0_IRQ), 399193,
        serial_hd(0), DEVICE_LITTLE_ENDIAN);

    sysbus_create_simple("goldfish_rtc", memmap[PLCT_STONE_RTC].base,
        qdev_get_gpio_in(DEVICE(mmio_plic), RTC_IRQ));

    plct_stone_flash_create(s);

    for (i = 0; i < ARRAY_SIZE(s->flash); i++) {
        /* Map legacy -drive if=pflash to machine properties */
        pflash_cfi01_legacy_drive(s->flash[i],
                                  drive_get(IF_PFLASH, 0, i));
    }
    plct_stone_flash_map(s, system_memory);
}

static void plct_stone_machine_instance_init(Object *obj)
{
}

static bool plct_stone_get_aclint(Object *obj, Error **errp)
{
    MachineState *ms = MACHINE(obj);
    RISCVStoneState *s = RISCV_PLCT_STONE_MACHINE(ms);

    return s->have_aclint;
}

static void plct_stone_set_aclint(Object *obj, bool value, Error **errp)
{
    MachineState *ms = MACHINE(obj);
    RISCVStoneState *s = RISCV_PLCT_STONE_MACHINE(ms);

    s->have_aclint = value;
}

static void plct_stone_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "PLCT RISC-V Stone board";
    mc->init = plct_stone_machine_init;
    mc->max_cpus = PLCT_STONE_CPUS_MAX;
    mc->default_cpu_type = PLCT_U_CPU;
    mc->pci_allow_0_address = true;
    mc->possible_cpu_arch_ids = riscv_numa_possible_cpu_arch_ids;
    mc->cpu_index_to_instance_props = riscv_numa_cpu_index_to_props;
    mc->get_default_cpu_node_id = riscv_numa_get_default_cpu_node_id;
    mc->numa_mem_supported = true;
    mc->default_ram_id = "riscv_plct_stone_board.ram";

    machine_class_allow_dynamic_sysbus_dev(mc, TYPE_RAMFB_DEVICE);

    object_class_property_add_bool(oc, "aclint", plct_stone_get_aclint,
                                   plct_stone_set_aclint);
    object_class_property_set_description(oc, "aclint",
                                          "Set on/off to enable/disable "
                                          "emulating ACLINT devices");
}

static const TypeInfo plct_stone_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("plct_stone"),
    .parent     = TYPE_MACHINE,
    .class_init = plct_stone_machine_class_init,
    .instance_init = plct_stone_machine_instance_init,
    .instance_size = sizeof(RISCVStoneState),
};

static void plct_stone_machine_init_register_types(void)
{
    type_register_static(&plct_stone_machine_typeinfo);
}

type_init(plct_stone_machine_init_register_types)

static const MemMapEntry plct_machine_memmap[] = {
    [PLCT_MACHINE_DEV_DEBUG] =    {        0x0,     0x1000 },
    [PLCT_MACHINE_DEV_MROM] =     {     0x1000,     0x2000 },
    [PLCT_MACHINE_DEV_OTP] =      {    0x20000,     0x2000 },
    [PLCT_MACHINE_DEV_CLINT] =    {  0x2000000,    0x10000 },
    [PLCT_MACHINE_DEV_PLIC] =     {  0xc000000,  0x4000000 },
    [PLCT_MACHINE_DEV_AON] =      { 0x10000000,     0x8000 },
    [PLCT_MACHINE_DEV_PRCI] =     { 0x10008000,     0x8000 },
    [PLCT_MACHINE_DEV_OTP_CTRL] = { 0x10010000,     0x1000 },
    [PLCT_MACHINE_DEV_GPIO0] =    { 0x10012000,     0x1000 },
    [PLCT_MACHINE_DEV_UART0] =    { 0x10013000,     0x1000 },
    [PLCT_MACHINE_DEV_QSPI0] =    { 0x10014000,     0x1000 },
    [PLCT_MACHINE_DEV_PWM0] =     { 0x10015000,     0x1000 },
    [PLCT_MACHINE_DEV_UART1] =    { 0x10023000,     0x1000 },
    [PLCT_MACHINE_DEV_QSPI1] =    { 0x10024000,     0x1000 },
    [PLCT_MACHINE_DEV_PWM1] =     { 0x10025000,     0x1000 },
    [PLCT_MACHINE_DEV_QSPI2] =    { 0x10034000,     0x1000 },
    [PLCT_MACHINE_DEV_PWM2] =     { 0x10035000,     0x1000 },
    [PLCT_MACHINE_DEV_XIP] =      { 0x20000000, 0x20000000 },
    [PLCT_MACHINE_DEV_DTIM] =     { 0x80000000,     0x4000 }
};

static void plct_machine_machine_init(MachineState *machine)
{
    MachineClass *mc = MACHINE_GET_CLASS(machine);
    const MemMapEntry *memmap = plct_machine_memmap;

    PLCTMachineState *s = RISCV_PLCT_MACHINE_MACHINE(machine);
    MemoryRegion *sys_mem = get_system_memory();
    int i;

    if (machine->ram_size != mc->default_ram_size) {
        char *sz = size_to_str(mc->default_ram_size);
        error_report("Invalid RAM size, should be %s", sz);
        g_free(sz);
        exit(EXIT_FAILURE);
    }

    /* Initialize SoC */
    object_initialize_child(OBJECT(machine), "soc", &s->soc, TYPE_RISCV_PLCT_MACHINE_SOC);
    qdev_realize(DEVICE(&s->soc), NULL, &error_abort);

    /* Data Tightly Integrated Memory */
    memory_region_add_subregion(sys_mem,
        memmap[PLCT_MACHINE_DEV_DTIM].base, machine->ram);

    /* Mask ROM reset vector */
    uint32_t reset_vec[4];

    if (s->revb) {
        reset_vec[1] = 0x200102b7;  /* 0x1004: lui     t0,0x20010 */
    } else {
        reset_vec[1] = 0x204002b7;  /* 0x1004: lui     t0,0x20400 */
    }
    reset_vec[2] = 0x00028067;      /* 0x1008: jr      t0 */

    reset_vec[0] = reset_vec[3] = 0;

    /* copy in the reset vector in little_endian byte order */
    for (i = 0; i < sizeof(reset_vec) >> 2; i++) {
        reset_vec[i] = cpu_to_le32(reset_vec[i]);
    }
    rom_add_blob_fixed_as("mrom.reset", reset_vec, sizeof(reset_vec),
                          memmap[PLCT_MACHINE_DEV_MROM].base, &address_space_memory);

    if (machine->kernel_filename) {
        riscv_load_kernel(machine->kernel_filename,
                          memmap[PLCT_MACHINE_DEV_DTIM].base, NULL);
    }
}

static bool plct_machine_machine_get_revb(Object *obj, Error **errp)
{
    PLCTMachineState *s = RISCV_PLCT_MACHINE_MACHINE(obj);

    return s->revb;
}

static void plct_machine_machine_set_revb(Object *obj, bool value, Error **errp)
{
    PLCTMachineState *s = RISCV_PLCT_MACHINE_MACHINE(obj);

    s->revb = value;
}

static void plct_machine_machine_instance_init(Object *obj)
{
    PLCTMachineState *s = RISCV_PLCT_MACHINE_MACHINE(obj);

    s->revb = false;
}

static void plct_machine_machine_class_init(ObjectClass *oc, void *data)
{
    MachineClass *mc = MACHINE_CLASS(oc);

    mc->desc = "PLCT RISC-V Board";
    mc->init = plct_machine_machine_init;
    mc->max_cpus = 1;
    mc->default_cpu_type = PLCT_N_CPU;
    mc->default_ram_id = "riscv.plct.machine.ram";
    mc->default_ram_size = plct_machine_memmap[PLCT_MACHINE_DEV_DTIM].size;

    object_class_property_add_bool(oc, "revb", plct_machine_machine_get_revb,
                                   plct_machine_machine_set_revb);
    object_class_property_set_description(oc, "revb",
                                          "Set on to tell QEMU that it should model "
                                          "the revB HiFive1 board");
}

static const TypeInfo plct_machine_machine_typeinfo = {
    .name       = MACHINE_TYPE_NAME("plct_machine"),
    .parent     = TYPE_MACHINE,
    .class_init = plct_machine_machine_class_init,
    .instance_init = plct_machine_machine_instance_init,
    .instance_size = sizeof(PLCTMachineState),
};

static void plct_machine_machine_init_register_types(void)
{
    type_register_static(&plct_machine_machine_typeinfo);
}

type_init(plct_machine_machine_init_register_types)

static void plct_machine_soc_init(Object *obj)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    PLCTMachineSoCState *s = RISCV_PLCT_MACHINE_SOC(obj);

    object_initialize_child(obj, "cpus", &s->cpus, TYPE_RISCV_HART_ARRAY);
    object_property_set_int(OBJECT(&s->cpus), "num-harts", ms->smp.cpus,
                            &error_abort);
    object_property_set_int(OBJECT(&s->cpus), "resetvec", 0x1004, &error_abort);
    object_initialize_child(obj, "riscv.plct.machine.gpio0", &s->gpio,
                            TYPE_SIFIVE_GPIO);
}

static void plct_machine_soc_realize(DeviceState *dev, Error **errp)
{
    MachineState *ms = MACHINE(qdev_get_machine());
    const MemMapEntry *memmap = plct_machine_memmap;
    PLCTMachineSoCState *s = RISCV_PLCT_MACHINE_SOC(dev);
    MemoryRegion *sys_mem = get_system_memory();

    object_property_set_str(OBJECT(&s->cpus), "cpu-type", ms->cpu_type,
                            &error_abort);
    sysbus_realize(SYS_BUS_DEVICE(&s->cpus), &error_abort);

    /* Mask ROM */
    memory_region_init_rom(&s->mask_rom, OBJECT(dev), "riscv.plct.machine.mrom",
                           memmap[PLCT_MACHINE_DEV_MROM].size, &error_fatal);
    memory_region_add_subregion(sys_mem,
        memmap[PLCT_MACHINE_DEV_MROM].base, &s->mask_rom);

    /* MMIO */
    s->plic = sifive_plic_create(memmap[PLCT_MACHINE_DEV_PLIC].base,
        (char *)PLCT_MACHINE_PLIC_HART_CONFIG, ms->smp.cpus, 0,
        PLCT_MACHINE_PLIC_NUM_SOURCES,
        PLCT_MACHINE_PLIC_NUM_PRIORITIES,
        PLCT_MACHINE_PLIC_PRIORITY_BASE,
        PLCT_MACHINE_PLIC_PENDING_BASE,
        PLCT_MACHINE_PLIC_ENABLE_BASE,
        PLCT_MACHINE_PLIC_ENABLE_STRIDE,
        PLCT_MACHINE_PLIC_CONTEXT_BASE,
        PLCT_MACHINE_PLIC_CONTEXT_STRIDE,
        memmap[PLCT_MACHINE_DEV_PLIC].size);
    riscv_aclint_swi_create(memmap[PLCT_MACHINE_DEV_CLINT].base,
        0, ms->smp.cpus, false);
    riscv_aclint_mtimer_create(memmap[PLCT_MACHINE_DEV_CLINT].base +
            RISCV_ACLINT_SWI_SIZE,
        RISCV_ACLINT_DEFAULT_MTIMER_SIZE, 0, ms->smp.cpus,
        RISCV_ACLINT_DEFAULT_MTIMECMP, RISCV_ACLINT_DEFAULT_MTIME,
        RISCV_ACLINT_DEFAULT_TIMEBASE_FREQ, false);
    create_unimplemented_device("riscv.plct.machine.aon",
        memmap[PLCT_MACHINE_DEV_AON].base, memmap[PLCT_MACHINE_DEV_AON].size);
    sifive_e_prci_create(memmap[PLCT_MACHINE_DEV_PRCI].base);

    /* GPIO */

    if (!sysbus_realize(SYS_BUS_DEVICE(&s->gpio), errp)) {
        return;
    }

    /* Map GPIO registers */
    sysbus_mmio_map(SYS_BUS_DEVICE(&s->gpio), 0, memmap[PLCT_MACHINE_DEV_GPIO0].base);

    /* Pass all GPIOs to the SOC layer so they are available to the board */
    qdev_pass_gpios(DEVICE(&s->gpio), dev, NULL);

    /* Connect GPIO interrupts to the PLIC */
    for (int i = 0; i < 32; i++) {
        sysbus_connect_irq(SYS_BUS_DEVICE(&s->gpio), i,
                           qdev_get_gpio_in(DEVICE(s->plic),
                                            PLCT_MACHINE_GPIO0_IRQ0 + i));
    }

    sifive_uart_create(sys_mem, memmap[PLCT_MACHINE_DEV_UART0].base,
        serial_hd(0), qdev_get_gpio_in(DEVICE(s->plic), PLCT_MACHINE_UART0_IRQ));
    create_unimplemented_device("riscv.plct.machine.qspi0",
        memmap[PLCT_MACHINE_DEV_QSPI0].base, memmap[PLCT_MACHINE_DEV_QSPI0].size);
    create_unimplemented_device("riscv.plct.machine.pwm0",
        memmap[PLCT_MACHINE_DEV_PWM0].base, memmap[PLCT_MACHINE_DEV_PWM0].size);
    sifive_uart_create(sys_mem, memmap[PLCT_MACHINE_DEV_UART1].base,
        serial_hd(1), qdev_get_gpio_in(DEVICE(s->plic), PLCT_MACHINE_UART1_IRQ));
    create_unimplemented_device("riscv.plct.machine.qspi1",
        memmap[PLCT_MACHINE_DEV_QSPI1].base, memmap[PLCT_MACHINE_DEV_QSPI1].size);
    create_unimplemented_device("riscv.plct.machine.pwm1",
        memmap[PLCT_MACHINE_DEV_PWM1].base, memmap[PLCT_MACHINE_DEV_PWM1].size);
    create_unimplemented_device("riscv.plct.machine.qspi2",
        memmap[PLCT_MACHINE_DEV_QSPI2].base, memmap[PLCT_MACHINE_DEV_QSPI2].size);
    create_unimplemented_device("riscv.plct.machine.pwm2",
        memmap[PLCT_MACHINE_DEV_PWM2].base, memmap[PLCT_MACHINE_DEV_PWM2].size);

    /* Flash memory */
    memory_region_init_rom(&s->xip_mem, OBJECT(dev), "riscv.plct.machine.xip",
                           memmap[PLCT_MACHINE_DEV_XIP].size, &error_fatal);
    memory_region_add_subregion(sys_mem, memmap[PLCT_MACHINE_DEV_XIP].base,
        &s->xip_mem);
}

static void plct_machine_soc_class_init(ObjectClass *oc, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(oc);

    dc->realize = plct_machine_soc_realize;
    /* Reason: Uses serial_hds in realize function, thus can't be used twice */
    dc->user_creatable = false;
}

static const TypeInfo plct_machine_soc_type_info = {
    .name = TYPE_RISCV_PLCT_MACHINE_SOC,
    .parent = TYPE_DEVICE,
    .instance_size = sizeof(PLCTMachineSoCState),
    .instance_init = plct_machine_soc_init,
    .class_init = plct_machine_soc_class_init,
};

static void plct_machine_soc_register_types(void)
{
    type_register_static(&plct_machine_soc_type_info);
}

type_init(plct_machine_soc_register_types)