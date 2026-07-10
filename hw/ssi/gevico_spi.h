/*
 * G233 SPI Controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * SPI master controller with 4 chip-select lines.  Connects to
 * SPI NOR Flash devices (W25X16 on CS0, W25X32 on CS1).
 */

#ifndef HW_SSI_GEVICO_SPI_H
#define HW_SSI_GEVICO_SPI_H

#include "hw/core/sysbus.h"
#include "hw/ssi/gevico_spi_flash.h"

#define TYPE_GEVICO_SPI "gevico-spi"
#define GEVICO_SPI(obj) OBJECT_CHECK(GEVICOSPIState, (obj), TYPE_GEVICO_SPI)

#define GEVICO_SPI_SIZE   0x10
#define GEVICO_SPI_NUM_CS 4

/* SPI_CR1 bits */
#define SPI_CR1_SPE    (1u << 0)
#define SPI_CR1_MSTR   (1u << 2)
#define SPI_CR1_ERRIE  (1u << 5)
#define SPI_CR1_RXNEIE (1u << 6)
#define SPI_CR1_TXEIE  (1u << 7)

/* SPI_SR bits */
#define SPI_SR_RXNE    (1u << 0)
#define SPI_SR_TXE     (1u << 1)
#define SPI_SR_OVERRUN (1u << 4)

typedef struct GEVICOSPIState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t cr1;   /* Control Register 1 */
    uint32_t cr2;   /* Control Register 2 (CS select) */

    uint8_t  rx_data;    /* received data byte */
    bool     rxne;       /* RXNE flag */
    bool     overrun;    /* OVERRUN flag */

    GEVICOSPIFlash flash[GEVICO_SPI_NUM_CS];

    qemu_irq irq;   /* PLIC IRQ 5 */
} GEVICOSPIState;

#endif
