/*
 * G233 PWM Controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 4-channel PWM with 32-bit period/duty/counter, polarity control,
 * and period-completion interrupt.
 */

#ifndef HW_GEVICO_PWM_H
#define HW_GEVICO_PWM_H

#include "hw/core/sysbus.h"

#define TYPE_GEVICO_PWM "gevico-pwm"
#define GEVICO_PWM(obj) OBJECT_CHECK(GEVICOPWMState, (obj), TYPE_GEVICO_PWM)

#define GEVICO_PWM_NUM_CHANNELS 4
#define GEVICO_PWM_SIZE         0x50  /* 0x00 + 4 * 0x10 + 0x10 */

/* PWM tick period: 1 us (1 MHz counter clock) */
#define PWM_TICK_NS 1000

/* PWM_GLB bit definitions */
#define PWM_GLB_CH_EN(n)   (1u << (n))
#define PWM_GLB_CH_DONE(n) (1u << (4 + (n)))

/* PWM_CHn_CTRL bit definitions */
#define PWM_CTRL_EN    (1u << 0)
#define PWM_CTRL_POL   (1u << 1)
#define PWM_CTRL_INTIE (1u << 2)

typedef struct PWMChannel {
    uint32_t ctrl;    /* CTRL register */
    uint32_t period;  /* PERIOD register */
    uint32_t duty;    /* DUTY register */
    uint32_t cnt;     /* frozen counter value (when disabled) */

    /* Internal state */
    int64_t  start_ns;     /* virtual-clock time when last enabled */
    uint64_t wraps_acked;  /* number of wraps already acknowledged */
    bool     done_latched; /* sticky DONE flag (set when wraps > wraps_acked) */
} PWMChannel;

typedef struct GEVICOPWMState {
    SysBusDevice parent_obj;

    MemoryRegion mmio;

    PWMChannel ch[GEVICO_PWM_NUM_CHANNELS];

    QEMUTimer *timer;
    qemu_irq   irq;       /* PLIC IRQ 3 */
} GEVICOPWMState;

#endif
