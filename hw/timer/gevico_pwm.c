/*
 * G233 PWM Controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 4-channel PWM for the G233 SoC (§8).  Each channel is independently
 * configured with a 32-bit period, 32-bit duty cycle, and polarity.
 *
 * The counter increments at 1 MHz (1 tick per microsecond) driven by the
 * QEMU virtual clock.  Counters are computed on-the-fly from elapsed
 * virtual time so that qtest_clock_step() works efficiently.
 *
 * Register map (base 0x1001_5000):
 *   0x00  PWM_GLB        – bits[3:0] CHn_EN mirror (RO),
 *                          bits[7:4] CHn_DONE (W1C)
 *   Per-channel (CHn at 0x10 + n*0x10):
 *     0x00  CHn_CTRL     – EN / POL / INTIE
 *     0x04  CHn_PERIOD   – period value
 *     0x08  CHn_DUTY     – duty cycle value
 *     0x0C  CHn_CNT      – current counter (RO)
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/timer/gevico_pwm.h"
#include "migration/vmstate.h"

/* ================================================================== */
/*  Counter computation — on-the-fly from virtual clock                */
/* ================================================================== */

/* Return the current counter value for channel ch.  When disabled the
 * counter is frozen at the last-computed value. */
static uint32_t pwm_ch_get_cnt(GEVICOPWMState *s, int ch)
{
    PWMChannel *c = &s->ch[ch];

    if (!(c->ctrl & PWM_CTRL_EN)) {
        return c->cnt;
    }

    if (c->period == 0) {
        return 0;
    }

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now - c->start_ns;
    if (elapsed_ns <= 0) {
        return 0;
    }

    uint64_t ticks     = (uint64_t)elapsed_ns / PWM_TICK_NS;
    uint64_t period_ticks = (uint64_t)c->period + 1;

    return (uint32_t)(ticks % period_ticks);
}

/* Return the number of full period wraps that have occurred since the
 * channel was enabled.  Used to set the latched DONE flag. */
static uint64_t pwm_ch_wraps(GEVICOPWMState *s, int ch)
{
    PWMChannel *c = &s->ch[ch];

    if (!(c->ctrl & PWM_CTRL_EN) || c->period == 0) {
        return 0;
    }

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now - c->start_ns;
    if (elapsed_ns <= 0) {
        return 0;
    }

    uint64_t ticks = (uint64_t)elapsed_ns / PWM_TICK_NS;
    uint64_t period_ticks = (uint64_t)c->period + 1;

    return ticks / period_ticks;
}

/* ================================================================== */
/*  IRQ                                                               */
/* ================================================================== */

static void pwm_update_irq(GEVICOPWMState *s)
{
    bool irq = false;

    for (int ch = 0; ch < GEVICO_PWM_NUM_CHANNELS; ch++) {
        PWMChannel *c = &s->ch[ch];

        /* Latch DONE when new (unacknowledged) wraps have occurred */
        uint64_t wraps = pwm_ch_wraps(s, ch);
        if (wraps > c->wraps_acked) {
            c->done_latched = true;
        }

        if (c->done_latched && (c->ctrl & PWM_CTRL_INTIE)) {
            irq = true;
        }
    }

    qemu_set_irq(s->irq, irq);
}

/* ================================================================== */
/*  Timer — fires at the next DONE event for IRQ delivery              */
/* ================================================================== */

static void pwm_timer_cb(void *opaque)
{
    GEVICOPWMState *s = GEVICO_PWM(opaque);
    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t next = INT64_MAX;

    for (int ch = 0; ch < GEVICO_PWM_NUM_CHANNELS; ch++) {
        PWMChannel *c = &s->ch[ch];

        if (!(c->ctrl & PWM_CTRL_EN) || c->period == 0) {
            continue;
        }

        uint32_t cnt = pwm_ch_get_cnt(s, ch);
        uint64_t remaining = (uint64_t)(c->period - cnt) + 1;
        int64_t ch_next = now + (int64_t)remaining * PWM_TICK_NS;

        if (ch_next < next) {
            next = ch_next;
        }
    }

    pwm_update_irq(s);

    if (next != INT64_MAX && next > now) {
        timer_mod(s->timer, next);
    }
}

static void pwm_schedule_timer(GEVICOPWMState *s)
{
    /* Fire immediately to catch any pending events */
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* ================================================================== */
/*  Channel enable / disable helpers                                   */
/* ================================================================== */

static void pwm_ch_enable(GEVICOPWMState *s, int ch)
{
    PWMChannel *c = &s->ch[ch];

    if (c->ctrl & PWM_CTRL_EN) {
        return; /* already enabled */
    }

    c->ctrl |= PWM_CTRL_EN;
    c->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    c->cnt = 0;
    c->wraps_acked = 0;
    c->done_latched = false;

    pwm_schedule_timer(s);
    pwm_update_irq(s);
}

static void pwm_ch_disable(GEVICOPWMState *s, int ch)
{
    PWMChannel *c = &s->ch[ch];

    if (!(c->ctrl & PWM_CTRL_EN)) {
        return; /* already disabled */
    }

    /* Freeze counter at current value */
    c->cnt = pwm_ch_get_cnt(s, ch);
    c->ctrl &= ~PWM_CTRL_EN;

    pwm_schedule_timer(s);
    pwm_update_irq(s);
}

/* ================================================================== */
/*  MMIO read / write                                                  */
/* ================================================================== */

static uint64_t gevico_pwm_read(void *opaque, hwaddr offset,
                                 unsigned int size)
{
    GEVICOPWMState *s = GEVICO_PWM(opaque);
    uint64_t r = 0;

    switch (offset) {
    /* ── Global register ─────────────────────────────────────── */
    case 0x00: /* PWM_GLB */
        /* EN mirror bits (RO) */
        for (int ch = 0; ch < GEVICO_PWM_NUM_CHANNELS; ch++) {
            if (s->ch[ch].ctrl & PWM_CTRL_EN) {
                r |= PWM_GLB_CH_EN(ch);
            }
        }
        /* DONE bits — latch any new wraps first */
        for (int ch = 0; ch < GEVICO_PWM_NUM_CHANNELS; ch++) {
            uint64_t wraps = pwm_ch_wraps(s, ch);
            if (wraps > s->ch[ch].wraps_acked) {
                s->ch[ch].done_latched = true;
            }
            if (s->ch[ch].done_latched) {
                r |= PWM_GLB_CH_DONE(ch);
            }
        }
        break;

    default:
        if (offset < 0x10 || offset >= GEVICO_PWM_SIZE) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                          __func__, offset);
            r = 0;
            break;
        }
        {
            /* ── Channel register ───────────────────────────── */
            int ch = (offset - 0x10) / 0x10;
            uint32_t ch_off = offset - 0x10 - ch * 0x10;

            if (ch >= GEVICO_PWM_NUM_CHANNELS) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: bad channel %d\n", __func__, ch);
                break;
            }

            switch (ch_off) {
            case 0x00: /* CHn_CTRL */
                r = s->ch[ch].ctrl;
                break;
            case 0x04: /* CHn_PERIOD */
                r = s->ch[ch].period;
                break;
            case 0x08: /* CHn_DUTY */
                r = s->ch[ch].duty;
                break;
            case 0x0C: /* CHn_CNT — on-the-fly from virtual clock */
                r = pwm_ch_get_cnt(s, ch);
                break;
            default:
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: bad channel offset 0x%x\n",
                              __func__, ch_off);
                break;
            }
        }
        break;
    }

    return r;
}

static void gevico_pwm_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    GEVICOPWMState *s = GEVICO_PWM(opaque);

    switch (offset) {
    /* ── Global register ─────────────────────────────────────── */
    case 0x00: /* PWM_GLB — DONE bits are W1C */
        {
            uint32_t w1c_mask = value & 0xF0;  /* DONE bits [7:4] */
            for (int ch = 0; ch < GEVICO_PWM_NUM_CHANNELS; ch++) {
                if (w1c_mask & PWM_GLB_CH_DONE(ch)) {
                    s->ch[ch].done_latched = false;
                    /* Acknowledge all wraps that have occurred so far,
                     * so DONE only re-asserts on a future wrap. */
                    s->ch[ch].wraps_acked = pwm_ch_wraps(s, ch);
                }
            }
        }
        pwm_update_irq(s);
        break;

    default:
        if (offset < 0x10 || offset >= GEVICO_PWM_SIZE) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                          __func__, offset);
            break;
        }
        {
            /* ── Channel register ───────────────────────────── */
            int ch = (offset - 0x10) / 0x10;
            uint32_t ch_off = offset - 0x10 - ch * 0x10;

            if (ch >= GEVICO_PWM_NUM_CHANNELS) {
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: bad channel %d\n", __func__, ch);
                break;
            }

            switch (ch_off) {
            case 0x00: /* CHn_CTRL */
                {
                    uint32_t old_val = s->ch[ch].ctrl;
                    /* Only EN, POL, INTIE are writable */
                    s->ch[ch].ctrl = value & (PWM_CTRL_EN | PWM_CTRL_POL |
                                              PWM_CTRL_INTIE);

                    /* Handle EN transitions */
                    bool was_enabled = (old_val & PWM_CTRL_EN) != 0;
                    bool now_enabled = (s->ch[ch].ctrl & PWM_CTRL_EN) != 0;

                    if (!was_enabled && now_enabled) {
                        pwm_ch_enable(s, ch);
                    } else if (was_enabled && !now_enabled) {
                        pwm_ch_disable(s, ch);
                    }
                }
                pwm_update_irq(s);
                pwm_schedule_timer(s);
                break;
            case 0x04: /* CHn_PERIOD */
                s->ch[ch].period = value;
                pwm_update_irq(s);
                pwm_schedule_timer(s);
                break;
            case 0x08: /* CHn_DUTY */
                s->ch[ch].duty = value;
                break;
            case 0x0C: /* CHn_CNT — read-only, ignore writes */
                break;
            default:
                qemu_log_mask(LOG_GUEST_ERROR,
                              "%s: bad channel offset 0x%x\n",
                              __func__, ch_off);
                break;
            }
        }
        break;
    }
}

static const MemoryRegionOps gevico_pwm_ops = {
    .read  = gevico_pwm_read,
    .write = gevico_pwm_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ================================================================== */
/*  Device lifecycle                                                   */
/* ================================================================== */

static void gevico_pwm_reset(DeviceState *dev)
{
    GEVICOPWMState *s = GEVICO_PWM(dev);

    for (int ch = 0; ch < GEVICO_PWM_NUM_CHANNELS; ch++) {
        s->ch[ch].ctrl   = 0;
        s->ch[ch].period = 0;
        s->ch[ch].duty   = 0;
        s->ch[ch].cnt    = 0;
        s->ch[ch].start_ns = 0;
        s->ch[ch].wraps_acked = 0;
        s->ch[ch].done_latched = false;
    }

    timer_del(s->timer);
}

static void gevico_pwm_realize(DeviceState *dev, Error **errp)
{
    GEVICOPWMState *s = GEVICO_PWM(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &gevico_pwm_ops, s,
                          TYPE_GEVICO_PWM, GEVICO_PWM_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    /* Single aggregated IRQ output → PLIC IRQ 3 */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, pwm_timer_cb, s);
}

static const VMStateDescription vmstate_pwm_channel = {
    .name = "gevico-pwm-channel",
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl,         PWMChannel),
        VMSTATE_UINT32(period,       PWMChannel),
        VMSTATE_UINT32(duty,         PWMChannel),
        VMSTATE_UINT32(cnt,          PWMChannel),
        VMSTATE_INT64(start_ns,      PWMChannel),
        VMSTATE_UINT64(wraps_acked,  PWMChannel),
        VMSTATE_BOOL(done_latched,   PWMChannel),
        VMSTATE_END_OF_LIST()
    }
};

static const VMStateDescription vmstate_gevico_pwm = {
    .name = TYPE_GEVICO_PWM,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_STRUCT_ARRAY(ch, GEVICOPWMState,
                             GEVICO_PWM_NUM_CHANNELS, 1,
                             vmstate_pwm_channel, PWMChannel),
        VMSTATE_END_OF_LIST()
    }
};

static void gevico_pwm_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_gevico_pwm;
    dc->realize = gevico_pwm_realize;
    device_class_set_legacy_reset(dc, gevico_pwm_reset);
    dc->desc = "G233 PWM Controller";
}

static const TypeInfo gevico_pwm_info = {
    .name = TYPE_GEVICO_PWM,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GEVICOPWMState),
    .class_init = gevico_pwm_class_init
};

static void gevico_pwm_register_types(void)
{
    type_register_static(&gevico_pwm_info);
}

type_init(gevico_pwm_register_types)
