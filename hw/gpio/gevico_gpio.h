/*
 * G233 GPIO Controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 32-pin GPIO with:
 *   - Direction / output / input registers
 *   - Edge-triggered (sticky) and level-triggered (transient) interrupt per pin
 *   - Single IRQ output (aggregated) to PLIC IRQ 2
 *   - External pin input support (for pin-level interrupt detection)
 */

#ifndef HW_GEVICO_GPIO_H
#define HW_GEVICO_GPIO_H

#include "hw/core/sysbus.h"
#include "hw/core/irq.h"

#define TYPE_GEVICO_GPIO "gevico-gpio"
#define GEVICO_GPIO(obj) OBJECT_CHECK(GEVICOGPIOState, (obj), TYPE_GEVICO_GPIO)

#define GEVICO_GPIO_PINS 32
#define GEVICO_GPIO_SIZE  0x100

typedef struct GEVICOGPIOState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t dir;     /* 0x00 - Direction (0=input, 1=output) */
    uint32_t out;     /* 0x04 - Output data */
    uint32_t in;      /* external pin input values */
    uint32_t ie;      /* 0x0C - Interrupt enable */
    uint32_t is;      /* 0x10 - Interrupt status (write-1-to-clear) */
    uint32_t trig;    /* 0x14 - Trigger type (0=edge, 1=level) */
    uint32_t pol;     /* 0x18 - Polarity (0=low/falling, 1=high/rising) */

    uint32_t prev_level; /* previous pin_level for edge detection */

    qemu_irq irq;     /* single IRQ output to PLIC */
    qemu_irq input[GEVICO_GPIO_PINS]; /* external pin input lines */
} GEVICOGPIOState;

#endif
