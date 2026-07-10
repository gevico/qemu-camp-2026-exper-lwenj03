/*
 * G233 Watchdog Timer
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 32-bit down-counter watchdog for the G233 SoC (§6).
 *
 * The counter decrements at 1 MHz (1 tick per microsecond) driven by the
 * QEMU virtual clock.  Counter values are computed on-the-fly from elapsed
 * time so that qtest_clock_step() works efficiently.
 *
 * Register map (base 0x1001_0000):
 *   0x00  WDT_CTRL  – EN / INTEN / RSTEN / LOCK(RO)
 *   0x04  WDT_LOAD  – reload value (reset 0xFFFF)
 *   0x08  WDT_VAL   – current counter (RO)
 *   0x0C  WDT_SR    – TIMEOUT flag (W1C)
 *   0x10  WDT_KEY   – key register (WO): 0x5A5A5A5A=feed, 0x1ACCE551=lock
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "qemu/timer.h"
#include "hw/core/irq.h"
#include "hw/timer/gevico_wdt.h"
#include "migration/vmstate.h"

/* ================================================================== */
/*  Counter computation — on-the-fly from virtual clock               */
/* ================================================================== */

/* Return current counter value.  When disabled the counter is frozen. */
static uint32_t wdt_get_val(GEVICOWDTState *s)
{
    if (!(s->ctrl & WDT_CTRL_EN)) {
        return s->val;
    }

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now - s->start_ns;
    if (elapsed_ns <= 0) {
        return s->load;
    }

    uint64_t elapsed_ticks = (uint64_t)elapsed_ns / WDT_TICK_NS;
    if (elapsed_ticks >= s->load) {
        return 0;  /* timed out, counter saturates at 0 */
    }
    return (uint32_t)(s->load - elapsed_ticks);
}

/* Return true if the counter has reached zero (timed out). */
static bool wdt_has_timed_out(GEVICOWDTState *s)
{
    if (!(s->ctrl & WDT_CTRL_EN)) {
        return false;
    }
    if (s->load == 0) {
        return false;  /* load=0 means no timeout possible? Or immediate? */
    }

    int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    int64_t elapsed_ns = now - s->start_ns;
    if (elapsed_ns <= 0) {
        return false;
    }

    uint64_t elapsed_ticks = (uint64_t)elapsed_ns / WDT_TICK_NS;
    return elapsed_ticks >= s->load;
}

/* ================================================================== */
/*  IRQ                                                               */
/* ================================================================== */

static void wdt_update_irq(GEVICOWDTState *s)
{
    /* Latch timeout only if not yet acknowledged for this run */
    if (!s->timeout_acked && !s->timeout_latched && wdt_has_timed_out(s)) {
        s->timeout_latched = true;
    }

    bool irq = s->timeout_latched && (s->ctrl & WDT_CTRL_INTEN);
    qemu_set_irq(s->irq, irq);
}

/* ================================================================== */
/*  Timer — fires at the timeout moment for IRQ delivery              */
/* ================================================================== */

static void wdt_timer_cb(void *opaque)
{
    GEVICOWDTState *s = GEVICO_WDT(opaque);

    wdt_update_irq(s);

    /* Reschedule: fire at next timeout (if counter is running) */
    if ((s->ctrl & WDT_CTRL_EN) && s->load > 0) {
        uint32_t val = wdt_get_val(s);
        if (val > 0) {
            int64_t now = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
            int64_t next = now + (int64_t)val * WDT_TICK_NS;
            timer_mod(s->timer, next);
        }
    } else {
        timer_del(s->timer);
    }
}

static void wdt_schedule_timer(GEVICOWDTState *s)
{
    timer_mod(s->timer, qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL));
}

/* ================================================================== */
/*  Feed / lock helpers                                               */
/* ================================================================== */

static void wdt_feed(GEVICOWDTState *s)
{
    /* Reload counter from LOAD */
    s->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
    s->val = s->load;
    s->timeout_latched = false;
    s->timeout_acked = false;  /* re-arm timeout */

    wdt_update_irq(s);
    wdt_schedule_timer(s);
}

static void wdt_lock(GEVICOWDTState *s)
{
    s->locked = true;
}

/* ================================================================== */
/*  MMIO read / write                                                  */
/* ================================================================== */

static uint64_t gevico_wdt_read(void *opaque, hwaddr offset,
                                 unsigned int size)
{
    GEVICOWDTState *s = GEVICO_WDT(opaque);
    uint64_t r = 0;

    switch (offset) {
    case 0x00: /* WDT_CTRL */
        r = s->ctrl;
        if (s->locked) {
            r |= WDT_CTRL_LOCK;
        }
        break;
    case 0x04: /* WDT_LOAD */
        r = s->load;
        break;
    case 0x08: /* WDT_VAL — on-the-fly from virtual clock */
        /* Freeze the counter value in case we're running */
        s->val = wdt_get_val(s);
        r = s->val;
        break;
    case 0x0C: /* WDT_SR */
        /* Latch any new timeout (only if not already acked) */
        if (!s->timeout_acked && !s->timeout_latched &&
            wdt_has_timed_out(s)) {
            s->timeout_latched = true;
        }
        r = s->sr;
        if (s->timeout_latched) {
            r |= WDT_SR_TIMEOUT;
        }
        break;
    case 0x10: /* WDT_KEY — write-only, reads return 0 */
        r = 0;
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad read offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        r = 0;
        break;
    }

    return r;
}

static void gevico_wdt_write(void *opaque, hwaddr offset,
                              uint64_t value, unsigned int size)
{
    GEVICOWDTState *s = GEVICO_WDT(opaque);

    switch (offset) {
    case 0x00: /* WDT_CTRL */
        if (s->locked) {
            /* CTRL is read-only when locked (§6.3.1) */
            break;
        }
        {
            bool was_enabled = (s->ctrl & WDT_CTRL_EN) != 0;
            /* Only EN, INTEN, RSTEN are writable; LOCK is RO */
            s->ctrl = value & (WDT_CTRL_EN | WDT_CTRL_INTEN | WDT_CTRL_RSTEN);

            bool now_enabled = (s->ctrl & WDT_CTRL_EN) != 0;
            if (!was_enabled && now_enabled) {
                /* Start counting from LOAD */
                s->start_ns = qemu_clock_get_ns(QEMU_CLOCK_VIRTUAL);
                s->val = s->load;
                s->timeout_latched = false;
                s->timeout_acked = false;
                wdt_schedule_timer(s);
            } else if (was_enabled && !now_enabled) {
                /* Freeze counter */
                s->val = wdt_get_val(s);
            }
        }
        wdt_update_irq(s);
        break;

    case 0x04: /* WDT_LOAD */
        s->load = value;
        break;

    case 0x0C: /* WDT_SR — TIMEOUT W1C */
        if (value & WDT_SR_TIMEOUT) {
            s->timeout_latched = false;
            s->timeout_acked = true;  /* acknowledge current timeout */
        }
        wdt_update_irq(s);
        break;

    case 0x10: /* WDT_KEY */
        if (value == WDT_KEY_FEED) {
            wdt_feed(s);
        } else if (value == WDT_KEY_LOCK) {
            wdt_lock(s);
        }
        /* Other values: undefined behavior, ignored */
        break;

    case 0x08: /* WDT_VAL — read-only, ignore writes */
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        break;
    }
}

static const MemoryRegionOps gevico_wdt_ops = {
    .read  = gevico_wdt_read,
    .write = gevico_wdt_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ================================================================== */
/*  Device lifecycle                                                   */
/* ================================================================== */

static void gevico_wdt_reset(DeviceState *dev)
{
    GEVICOWDTState *s = GEVICO_WDT(dev);

    s->ctrl   = 0;
    s->load   = 0xFFFF;
    s->val    = 0xFFFF;
    s->sr     = 0;
    s->locked = false;
    s->timeout_acked = false;
    s->start_ns = 0;
    s->timeout_latched = false;

    timer_del(s->timer);
}

static void gevico_wdt_realize(DeviceState *dev, Error **errp)
{
    GEVICOWDTState *s = GEVICO_WDT(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &gevico_wdt_ops, s,
                          TYPE_GEVICO_WDT, GEVICO_WDT_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    s->timer = timer_new_ns(QEMU_CLOCK_VIRTUAL, wdt_timer_cb, s);
}

static const VMStateDescription vmstate_gevico_wdt = {
    .name = TYPE_GEVICO_WDT,
    .version_id = 1,
    .minimum_version_id = 1,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(ctrl,   GEVICOWDTState),
        VMSTATE_UINT32(load,   GEVICOWDTState),
        VMSTATE_UINT32(val,    GEVICOWDTState),
        VMSTATE_UINT32(sr,     GEVICOWDTState),
        VMSTATE_BOOL(locked,   GEVICOWDTState),
        VMSTATE_BOOL(timeout_acked, GEVICOWDTState),
        VMSTATE_INT64(start_ns, GEVICOWDTState),
        VMSTATE_BOOL(timeout_latched, GEVICOWDTState),
        VMSTATE_END_OF_LIST()
    }
};

static void gevico_wdt_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_gevico_wdt;
    dc->realize = gevico_wdt_realize;
    device_class_set_legacy_reset(dc, gevico_wdt_reset);
    dc->desc = "G233 Watchdog Timer";
}

static const TypeInfo gevico_wdt_info = {
    .name = TYPE_GEVICO_WDT,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GEVICOWDTState),
    .class_init = gevico_wdt_class_init
};

static void gevico_wdt_register_types(void)
{
    type_register_static(&gevico_wdt_info);
}

type_init(gevico_wdt_register_types)
