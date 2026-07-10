/*
 * G233 Watchdog Timer
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 32-bit down-counter watchdog with feed/lock/timeout/interrupt.
 */

#ifndef HW_GEVICO_WDT_H
#define HW_GEVICO_WDT_H

#include "hw/core/sysbus.h"

#define TYPE_GEVICO_WDT "gevico-wdt"
#define GEVICO_WDT(obj) OBJECT_CHECK(GEVICOWDTState, (obj), TYPE_GEVICO_WDT)

#define GEVICO_WDT_SIZE  0x14

/* WDT clock tick: 1 us (1 MHz) — same as PWM */
#define WDT_TICK_NS 1000

/* WDT_CTRL bits */
#define WDT_CTRL_EN     (1u << 0)
#define WDT_CTRL_INTEN  (1u << 1)
#define WDT_CTRL_RSTEN  (1u << 2)
#define WDT_CTRL_LOCK   (1u << 3)

/* WDT_SR bits */
#define WDT_SR_TIMEOUT  (1u << 0)

/* WDT_KEY magic values */
#define WDT_KEY_FEED    0x5A5A5A5Au
#define WDT_KEY_LOCK    0x1ACCE551u

typedef struct GEVICOWDTState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    uint32_t ctrl;   /* 0x00 — Control register */
    uint32_t load;   /* 0x04 — Load/reload value */
    uint32_t val;    /* frozen counter value (when disabled) */
    uint32_t sr;     /* 0x0C — Status register (TIMEOUT) */

    bool locked;     /* CTRL is read-only when locked */
    bool timeout_acked;   /* timeout event already acknowledged */

    int64_t start_ns;       /* virtual-clock time when counter started */
    bool    timeout_latched; /* sticky TIMEOUT */

    QEMUTimer *timer;
    qemu_irq   irq;         /* PLIC IRQ 4 */
} GEVICOWDTState;

#endif
