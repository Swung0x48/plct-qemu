/*
 * QEMU Core-V MCU UDMA UART
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

#include "qemu/osdep.h"
#include "hw/sysbus.h"
#include "hw/registerfields.h"
#include "hw/char/core_v_udma_uart.h"
#include "qom/object.h"
#include "hw/irq.h"
#include "hw/qdev-clock.h"
#include "hw/qdev-properties.h"
#include "hw/qdev-properties-system.h"
#include "migration/vmstate.h"
#include "qemu/log.h"
#include "qemu/module.h"

#define DEBUG_UART_READ(X)
#define DEBUG_UART_WRITE(X)

REG32(RX_SADDR, 0x00)
REG32(RX_SIZE, 0x04)
REG32(RX_CFG, 0x08)
    FIELD(RX_CFG, CONTINUE, 0, 1)
    FIELD(RX_CFG, EN, 4, 1)
    FIELD(RX_CFG, PENDING, 5, 1)
    FIELD(RX_CFG, CLR, 6, 1)
REG32(TX_SADDR, 0x10)
REG32(TX_SIZE, 0x14)
REG32(TX_CFG, 0x18)
    FIELD(TX_CFG, CONTINUE, 0, 1)
    FIELD(TX_CFG, EN, 4, 1)
    FIELD(TX_CFG, PENDING, 5, 1)
    FIELD(TX_CFG, CLR, 6, 1)
REG32(STATUS, 0x20)
    FIELD(STATUS, TX_BUSY, 0, 1)
    FIELD(STATUS, RX_BUSY, 1, 1)
REG32(SETUP, 0x24)
    FIELD(SETUP, PARITY_ENA, 0, 1)
    FIELD(SETUP, BIT_LENGTH, 1, 2)
    FIELD(SETUP, STOP_BITS, 3, 1)
    FIELD(SETUP, CLEAN_FIFO, 5, 1)
    FIELD(SETUP, POLLING_EN, 4, 1)
    FIELD(SETUP, TX_ENA, 8, 1)
    FIELD(SETUP, RX_ENA, 9, 1)
    FIELD(SETUP, CLKDIV, 16, 16)
REG32(ERROR, 0x28)
    FIELD(ERROR, RX_ERR_OVERFLOW, 0, 1)
    FIELD(ERROR, RX_ERR_PARITY, 1, 1)
REG32(IRQ_EN, 0x2C)
    FIELD(IRQ_EN, RX, 0, 1)
    FIELD(IRQ_EN, ERROR, 1, 1)
REG32(VALID, 0x30)
    FIELD(VALID, READY, 0, 1)
REG32(DATA, 0x34)

static void udma_uart_update_irqs(CoreVUDMAUartState *s)
{
    if (s->rx_fifo_level && ((s->irq_en & R_IRQ_EN_RX_MASK) ||
                             (s->setup & R_SETUP_POLLING_EN_MASK))) {
        s->valid = 1;
    } else {
        s->valid = 0;
    }

    if (s->rx_fifo_level && (s->irq_en & R_IRQ_EN_RX_MASK) &&
        !(s->setup & R_SETUP_POLLING_EN_MASK)) {
        qemu_irq_raise(s->irq[UART_IRQ_RX]);
    } else {
        qemu_irq_lower(s->irq[UART_IRQ_RX]);
    }

    if (s->error && (s->irq_en & R_IRQ_EN_ERROR_MASK)) {
        qemu_irq_raise(s->irq[UART_IRQ_ERR]);
    } else {
        qemu_irq_lower(s->irq[UART_IRQ_ERR]);
    }
}

static int udma_uart_can_receive(void *opaque)
{
    CoreVUDMAUartState *s = opaque;

    if (s->enable && ((s->rx_cfg & R_SETUP_RX_ENA_MASK) ||
                      (s->irq_en & R_IRQ_EN_RX_MASK) ||
                      (s->setup & R_SETUP_POLLING_EN_MASK))) {
        return s->rx_fifo_level < sizeof(s->rx_fifo);
    }

    return 0;
}

static void dma_rx_transfer(CoreVUDMAUartState *s)
{
    if (!(s->setup & R_SETUP_RX_ENA_MASK) ||
        !(s->rx_cfg & R_RX_CFG_EN_MASK)) {
        return;
    }

    if (s->rx_fifo_level >= s->rx_cur_size) {
        cpu_physical_memory_write(s->rx_saddr, &s->rx_fifo[0],
                                  s->rx_cur_size);
        s->rx_cur_size = 0;
        s->rx_fifo_level -= s->rx_cur_size;
        memmove(s->rx_fifo, s->rx_fifo + s->rx_cur_size, s->rx_fifo_level);
        qemu_irq_raise(s->irq[UART_IRQ_CH_RX]);
        if (s->rx_cfg & R_RX_CFG_CONTINUE_MASK) {
            s->rx_cur_size = s->rx_size;
            s->rx_cur_saddr = s->rx_saddr;
        } else {
            s->rx_cfg &= ~R_RX_CFG_EN_MASK;
            s->status &= ~R_STATUS_RX_BUSY_MASK;
        }
    } else {
        if (s->rx_fifo_level) {
            cpu_physical_memory_write(s->rx_saddr, &s->rx_fifo[0],
                                    s->rx_fifo_level);
            s->rx_saddr += s->rx_fifo_level;
            s->rx_size -= s->rx_fifo_level;
            s->rx_fifo_level = 0;
            qemu_irq_lower(s->irq[UART_IRQ_CH_RX]);
        }

        qemu_chr_fe_accept_input(&s->chr);
    }
}

static void dma_tx_transfer(CoreVUDMAUartState *s)
{
    if ((s->tx_cfg & R_TX_CFG_EN_MASK) && (s->setup & R_SETUP_TX_ENA_MASK)) {
        s->status |= R_STATUS_TX_BUSY_MASK;
        unsigned char *buf = g_malloc(s->tx_size);
        cpu_physical_memory_read(s->tx_saddr, buf, s->tx_size);
        qemu_chr_fe_write_all(&s->chr, buf, s->tx_size);
        g_free(buf);
        s->status &= ~R_STATUS_TX_BUSY_MASK;
        qemu_irq_raise(s->irq[UART_IRQ_CH_TX]);
        if (s->tx_cfg & R_RX_CFG_CONTINUE_MASK) {
            s->tx_cur_size = s->tx_size;
            s->tx_cur_saddr = s->tx_saddr;
        } else {
            s->tx_cfg &= ~R_TX_CFG_EN_MASK;
            s->status &= ~R_STATUS_TX_BUSY_MASK;
        }
    }
}

static void update_timer(CoreVUDMAUartState *s)
{
    if (s->enable && ((s->irq_en & R_IRQ_EN_RX_MASK) ||
                      (s->setup & R_SETUP_POLLING_EN_MASK))) {
        qemu_chr_fe_accept_input(&s->chr);
    }

    if (s->enable && (((s->tx_cfg & R_TX_CFG_EN_MASK) &&
                       (s->setup & R_SETUP_TX_ENA_MASK)) ||
                      ((s->rx_cfg & R_RX_CFG_EN_MASK) &&
                       (s->setup & R_SETUP_RX_ENA_MASK)))) {
        if (!s->timer_start) {
            dma_rx_transfer(s);
            dma_tx_transfer(s);
            if ((s->tx_cfg & R_TX_CFG_EN_MASK) ||
                (s->rx_cfg & R_RX_CFG_EN_MASK)) {
                s->timer_start = true;
                timer_mod(s->dma_handle,
                          qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                          (NANOSECONDS_PER_SECOND / 230400) * 10);
            } else {
                timer_del(s->dma_handle);
            }
        } else {
            timer_mod(s->dma_handle, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL) +
                                     (NANOSECONDS_PER_SECOND / 230400) * 10);
        }
    } else if (s->timer_start) {
        timer_del(s->dma_handle);
        s->timer_start = false;
    }
}

static void udma_uart_receive(void *opaque, const uint8_t *buf, int size)
{
    CoreVUDMAUartState *s = opaque;

    if (s->rx_fifo_level >= sizeof(s->rx_fifo)) {
        s->error |= R_ERROR_RX_ERR_OVERFLOW_MASK;
    } else {
        s->status |= R_STATUS_RX_BUSY_MASK;
        s->rx_fifo[s->rx_fifo_level++] = *buf;
        s->status |= ~R_STATUS_RX_BUSY_MASK;
    }

    udma_uart_update_irqs(s);
}

static uint64_t udma_uart_read(void *opaque, hwaddr addr, unsigned int size)
{
    CoreVUDMAUartState *s = opaque;
    uint64_t ret = 0;

    if (!s->enable) {
        return 0;
    }

    switch (addr >> 2) {
    case R_RX_SADDR:
        ret = s->rx_cur_saddr;
        break;
    case R_RX_SIZE:
        ret = s->rx_cur_size;
        break;
    case R_RX_CFG:
        ret = s->rx_cfg;
        break;
    case R_TX_SADDR:
        ret = s->tx_cur_saddr;
        break;
    case R_TX_SIZE:
        ret = s->tx_cur_size;
        break;
    case R_TX_CFG:
        ret = s->tx_cfg;
        break;
    case R_SETUP:
        ret = s->setup;
        break;
    case R_ERROR:
        ret = s->error;
        s->error = 0;
        break;
    case R_IRQ_EN:
        ret = s->irq_en;
        break;
    case R_VALID:
        ret = s->valid;
        break;
    case R_DATA:
        if ((s->irq_en & R_IRQ_EN_RX_MASK) ||
            (s->setup & R_SETUP_POLLING_EN_MASK)) {
            if (s->rx_fifo_level) {
                s->rx_data = s->rx_fifo[0];
                memmove(s->rx_fifo, s->rx_fifo + 1, s->rx_fifo_level - 1);
                s->rx_fifo_level--;
                qemu_chr_fe_accept_input(&s->chr);
                udma_uart_update_irqs(s);
            }
        }
        ret = s->rx_data;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
        return 0;
    }
    DEBUG_UART_READ(printf("udma uart read %d, %x\n", (uint32_t)addr >> 2,
                           (uint32_t)ret);)
    return ret;
}

static void udma_uart_write(void *opaque, hwaddr addr, uint64_t val64,
                            unsigned int size)
{
    CoreVUDMAUartState *s = opaque;
    uint32_t value = val64;
    DEBUG_UART_WRITE(printf("udma uart write %d, %x\n", (uint32_t)addr >> 2,
                            value);)
    if (!s->enable) {
        return;
    }

    switch (addr >> 2) {
    case R_RX_SADDR:
        s->rx_saddr = value;
        break;
    case R_RX_SIZE:
        s->rx_size = value;
        break;
    case R_RX_CFG:
        if (!(s->rx_cfg & R_RX_CFG_EN_MASK) && (value & R_RX_CFG_EN_MASK)) {
            s->rx_cur_saddr = s->rx_saddr;
            s->rx_cur_size = s->rx_size;
        }
        if (s->rx_cfg & R_RX_CFG_CLR_MASK) {
            s->rx_cur_saddr = 0;
            s->rx_cur_size = 0;
            s->rx_cfg = value & R_RX_CFG_CONTINUE_MASK;
        } else {
            s->rx_cfg = value & ~R_RX_CFG_CLR_MASK;
        }
        update_timer(s);
        break;
    case R_TX_SADDR:
        s->tx_saddr = value;
        break;
    case R_TX_SIZE:
        s->tx_size = value;
        break;
    case R_TX_CFG:
        if (!(s->tx_cfg & R_TX_CFG_EN_MASK) && (value & R_TX_CFG_EN_MASK)) {
            s->tx_cur_saddr = s->tx_saddr;
            s->tx_cur_size = s->tx_size;
        }
        if (s->tx_cfg & R_TX_CFG_CLR_MASK) {
            s->tx_cur_saddr = 0;
            s->tx_cur_size = 0;
            s->tx_cfg = value & R_TX_CFG_CONTINUE_MASK;
        } else {
            s->tx_cfg = value & ~R_TX_CFG_CLR_MASK;
        }
        update_timer(s);
        break;
    case R_SETUP:
        if (value & R_SETUP_CLEAN_FIFO_MASK) {
            value &= ~R_SETUP_CLEAN_FIFO_MASK;
            s->rx_fifo_level = 0;
        }
        s->setup = value;
        if (value & R_SETUP_PARITY_ENA_MASK) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: UART_SETUP_PARITY_EN is not supported\n",
                          __func__);
        }

        if (value & R_SETUP_BIT_LENGTH_MASK) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: UART_SETUP_BIT_LENGTH is not supported\n",
                          __func__);
        }

        if (value & R_SETUP_STOP_BITS_MASK) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: UART_SETUP_STOP_BITS is not supported\n",
                          __func__);
        }

        if (value & R_SETUP_CLKDIV_MASK) {
            qemu_log_mask(LOG_UNIMP,
                          "%s: UART_SETUP_CLKDIV is not supported\n",
                          __func__);
        }
        update_timer(s);
        break;
    case R_IRQ_EN:
        s->irq_en = value;
        update_timer(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: Bad offset 0x%"HWADDR_PRIx"\n", __func__, addr);
    }
}

static void trigger_dma_transfer(void *opaque)
{
    CoreVUDMAUartState *s = opaque;
    dma_rx_transfer(s);
    dma_tx_transfer(s);
    update_timer(s);
}

static const MemoryRegionOps udma_uart_ops = {
    .read = udma_uart_read,
    .write = udma_uart_write,
    .endianness = DEVICE_NATIVE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

static const VMStateDescription vmstate_udma_uart = {
    .name = TYPE_CORE_V_UDMA_UART,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (VMStateField[]) {
        VMSTATE_BOOL(enable, CoreVUDMAUartState),
        VMSTATE_TIMER_PTR(dma_handle, CoreVUDMAUartState),
        VMSTATE_BOOL(timer_start, CoreVUDMAUartState),
        VMSTATE_UINT32(rx_saddr, CoreVUDMAUartState),
        VMSTATE_UINT32(rx_size, CoreVUDMAUartState),
        VMSTATE_UINT32(rx_cfg, CoreVUDMAUartState),
        VMSTATE_UINT32(tx_saddr, CoreVUDMAUartState),
        VMSTATE_UINT32(tx_size, CoreVUDMAUartState),
        VMSTATE_UINT32(tx_cfg, CoreVUDMAUartState),
        VMSTATE_UINT32(status, CoreVUDMAUartState),
        VMSTATE_UINT32(setup, CoreVUDMAUartState),
        VMSTATE_UINT32(error, CoreVUDMAUartState),
        VMSTATE_UINT32(irq_en, CoreVUDMAUartState),
        VMSTATE_UINT32(valid, CoreVUDMAUartState),
        VMSTATE_UINT8(rx_data, CoreVUDMAUartState),
        VMSTATE_UINT8_ARRAY(rx_fifo, CoreVUDMAUartState, 4),
        VMSTATE_UINT8(rx_fifo_level, CoreVUDMAUartState),
        VMSTATE_END_OF_LIST()
    }
};

static Property udma_uart_properties[] = {
    DEFINE_PROP_CHR("chardev", CoreVUDMAUartState, chr),
    DEFINE_PROP_END_OF_LIST(),
};

static void core_v_udma_ctrl_event(void *opaque, int num, int level)
{
    CoreVUDMAUartState *s = opaque;
    s->enable = level;
    if (s->enable) {
        update_timer(s);
    }
}

static void udma_uart_init(Object *obj)
{
    CoreVUDMAUartState *s = CORE_V_UDMA_UART(obj);

    memory_region_init_io(&s->iomem, obj, &udma_uart_ops, s,
                          TYPE_CORE_V_UDMA_UART, UDMA_PER_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(obj), &s->iomem);

    qdev_init_gpio_out(DEVICE(obj), s->irq, UDMA_PER_EVENT_NUM);
    qdev_init_gpio_in(DEVICE(obj), core_v_udma_ctrl_event, 1);
}

static void udma_uart_realize(DeviceState *dev, Error **errp)
{
    CoreVUDMAUartState *s = CORE_V_UDMA_UART(dev);

    qemu_chr_fe_set_handlers(&s->chr, udma_uart_can_receive,
                             udma_uart_receive, NULL, NULL,
                             s, NULL, true);

    s->dma_handle = timer_new_ns(QEMU_CLOCK_VIRTUAL,
                                 trigger_dma_transfer, s);
    s->timer_start = false;
}

static void udma_uart_reset(DeviceState *dev)
{
    CoreVUDMAUartState *s = CORE_V_UDMA_UART(dev);
    s->rx_saddr = 0;
    s->rx_size = 0;
    s->rx_cfg = 0;
    s->tx_saddr = 0;
    s->tx_size = 0;
    s->tx_cfg = 0;
    s->status = 0;
    s->setup = 0;
    s->error = 0;
    s->irq_en = 0;
    s->valid = 0;
    s->rx_data = 0;
    s->rx_fifo_level = 0;
    s->rx_cur_saddr = 0;
    s->rx_cur_size = 0;
    s->tx_cur_saddr = 0;
    s->tx_cur_size = 0;
    s->enable = false;
}

static void udma_uart_class_init(ObjectClass *klass, void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->reset = udma_uart_reset;
    dc->realize = udma_uart_realize;
    dc->vmsd = &vmstate_udma_uart;
    device_class_set_props(dc, udma_uart_properties);
    set_bit(DEVICE_CATEGORY_INPUT, dc->categories);
}

static const TypeInfo udma_uart_info = {
    .name          = TYPE_CORE_V_UDMA_UART,
    .parent        = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(CoreVUDMAUartState),
    .instance_init = udma_uart_init,
    .class_init    = udma_uart_class_init,
};

static void udma_uart_register_types(void)
{
    type_register_static(&udma_uart_info);
}

type_init(udma_uart_register_types)
