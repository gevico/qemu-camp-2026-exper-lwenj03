/*
 * G233 SPI Controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * SPI master controller for the G233 SoC (§9).
 * Supports 4 chip-select lines, each connected to a NOR Flash device
 * (W25X16 on CS0, W25X32 on CS1).
 *
 * Register map (base 0x1001_8000):
 *   0x00  SPI_CR1  – SPE / MSTR / ERRIE / RXNEIE / TXEIE
 *   0x04  SPI_CR2  – CS_SEL[1:0]
 *   0x08  SPI_SR   – RXNE / TXE / OVERRUN (W1C)
 *   0x0C  SPI_DR   – data register (8-bit)
 *
 * Transfers are instant (completed within the MMIO write handler).
 * Writing DR sends the byte to the selected flash and latches the
 * response in rx_data; RXNE is set and can be read from DR.
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/core/irq.h"
#include "hw/ssi/gevico_spi.h"
#include "migration/vmstate.h"

/* ================================================================== */
/*  IRQ                                                               */
/* ================================================================== */

static void spi_update_irq(GEVICOSPIState *s)
{
    bool irq = false;

    /* TXE interrupt */
    if ((s->cr1 & SPI_CR1_TXEIE) && true) {  /* TXE is always 1 (empty) */
        irq = true;
    }

    /* RXNE interrupt */
    if ((s->cr1 & SPI_CR1_RXNEIE) && s->rxne) {
        irq = true;
    }

    /* Error interrupt */
    if ((s->cr1 & SPI_CR1_ERRIE) && s->overrun) {
        irq = true;
    }

    qemu_set_irq(s->irq, irq);
}

/* ================================================================== */
/*  Helpers                                                            */
/* ================================================================== */

static int spi_get_cs(GEVICOSPIState *s)
{
    return s->cr2 & 0x3;
}

/* ================================================================== */
/*  MMIO read / write                                                  */
/* ================================================================== */

static uint64_t gevico_spi_read(void *opaque, hwaddr offset,
                                 unsigned int size)
{
    GEVICOSPIState *s = GEVICO_SPI(opaque);
    uint64_t r = 0;

    switch (offset) {
    case 0x00: /* SPI_CR1 */
        r = s->cr1;
        break;
    case 0x04: /* SPI_CR2 */
        r = s->cr2;
        break;
    case 0x08: /* SPI_SR */
        r = 0;
        if (s->rxne) {
            r |= SPI_SR_RXNE;
        }
        /* TXE is always 1 (transmit buffer empty) for instant transfer */
        r |= SPI_SR_TXE;
        if (s->overrun) {
            r |= SPI_SR_OVERRUN;
        }
        break;
    case 0x0C: /* SPI_DR */
        r = s->rx_data;
        s->rxne = false;
        spi_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        break;
    }

    return r;
}

static void gevico_spi_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    GEVICOSPIState *s = GEVICO_SPI(opaque);

    switch (offset) {
    case 0x00: /* SPI_CR1 */
        s->cr1 = value & 0xE5;  /* SPE, MSTR, ERRIE, RXNEIE, TXEIE */
        spi_update_irq(s);
        break;

    case 0x04: /* SPI_CR2 — CS select */
        {
            int old_cs = spi_get_cs(s);
            int new_cs = value & 0x3;
            if (old_cs != new_cs) {
                /* CS toggled: de-assert old flash (terminates any
                 * in-progress command, commits page program). */
                gevico_spi_flash_cs_deassert(&s->flash[old_cs]);
            }
            s->cr2 = new_cs;
        }
        break;

    case 0x08: /* SPI_SR — only OVERRUN is W1C */
        if (value & SPI_SR_OVERRUN) {
            s->overrun = false;
        }
        spi_update_irq(s);
        break;

    case 0x0C: /* SPI_DR — write: initiate transfer */
        {
            /* Check if controller is enabled and in master mode */
            if (!(s->cr1 & SPI_CR1_SPE) || !(s->cr1 & SPI_CR1_MSTR)) {
                break;
            }

            /* Overrun: previous RXNE not cleared before new transfer */
            if (s->rxne) {
                s->overrun = true;
            }

            /* Transfer byte to selected flash */
            int cs = spi_get_cs(s);
            uint8_t tx_byte = (uint8_t)(value & 0xFF);
            s->rx_data = gevico_spi_flash_transfer(&s->flash[cs], tx_byte);
            s->rxne = true;

            spi_update_irq(s);
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        break;
    }
}

static const MemoryRegionOps gevico_spi_ops = {
    .read  = gevico_spi_read,
    .write = gevico_spi_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ================================================================== */
/*  Device lifecycle                                                   */
/* ================================================================== */

static void gevico_spi_reset(DeviceState *dev)
{
    GEVICOSPIState *s = GEVICO_SPI(dev);

    s->cr1     = 0;
    s->cr2     = 0;
    s->rx_data = 0;
    s->rxne    = false;
    s->overrun = false;

    for (int i = 0; i < GEVICO_SPI_NUM_CS; i++) {
        gevico_spi_flash_reset(&s->flash[i]);
    }
}

static void gevico_spi_realize(DeviceState *dev, Error **errp)
{
    GEVICOSPIState *s = GEVICO_SPI(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &gevico_spi_ops, s,
                          TYPE_GEVICO_SPI, GEVICO_SPI_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    /* Initialize flash devices:
     * CS0: W25X16 (2 MB, JEDEC 0xEF3015)
     * CS1: W25X32 (4 MB, JEDEC 0xEF3016)
     * CS2-CS3: unpopulated (2 MB dummy, JEDEC 0)
     */
    gevico_spi_flash_init(&s->flash[0], 2, 0xEF3015);
    gevico_spi_flash_init(&s->flash[1], 4, 0xEF3016);
    gevico_spi_flash_init(&s->flash[2], 2, 0);
    gevico_spi_flash_init(&s->flash[3], 2, 0);
}

/* VMState: only SPI controller state is migrated; flash content
 * is not preserved across migration for this simple model. */
static const VMStateDescription vmstate_gevico_spi = {
    .name = TYPE_GEVICO_SPI,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(cr1,     GEVICOSPIState),
        VMSTATE_UINT32(cr2,     GEVICOSPIState),
        VMSTATE_UINT8(rx_data,  GEVICOSPIState),
        VMSTATE_BOOL(rxne,      GEVICOSPIState),
        VMSTATE_BOOL(overrun,   GEVICOSPIState),
        VMSTATE_END_OF_LIST()
    }
};

static void gevico_spi_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_gevico_spi;
    dc->realize = gevico_spi_realize;
    device_class_set_legacy_reset(dc, gevico_spi_reset);
    dc->desc = "G233 SPI Controller";
}

static const TypeInfo gevico_spi_info = {
    .name = TYPE_GEVICO_SPI,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GEVICOSPIState),
    .class_init = gevico_spi_class_init
};

static void gevico_spi_register_types(void)
{
    type_register_static(&gevico_spi_info);
}

type_init(gevico_spi_register_types)
