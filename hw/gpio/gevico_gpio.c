/*
 * G233 GPIO Controller
 *
 * Copyright (c) 2025 Chao Liu <chao.liu@yeah.net>
 * SPDX-License-Identifier: GPL-2.0-or-later
 *
 * 32-pin GPIO controller for the G233 SoC.  Each pin can be independently
 * configured as input or output.  Interrupts support edge-triggered (sticky)
 * and level-triggered (transient) modes with configurable polarity.
 *
 * Register map (base 0x1001_2000):
 *   0x00  GPIO_DIR   R/W   Direction (0=input, 1=output)
 *   0x04  GPIO_OUT   R/W   Output data
 *   0x08  GPIO_IN    R     Input data (actual pin level)
 *   0x0C  GPIO_IE    R/W   Interrupt enable
 *   0x10  GPIO_IS    R/W   Interrupt status (write-1-to-clear)
 *   0x14  GPIO_TRIG  R/W   Trigger type (0=edge, 1=level)
 *   0x18  GPIO_POL   R/W   Polarity (0=low/falling, 1=high/rising)
 *
 * Interrupt logic per pin (see §7.3.4–§7.3.7):
 *
 *   The "pin level" is the actual electrical level:
 *     pin_level = (DIR & OUT) | (~DIR & ext_in)
 *   i.e., output mode reads OUT; input mode reads the external pin.
 *
 *   Edge-triggered (TRIG=0):
 *     IS latches on a pin_level transition toward POL.
 *     Rising  (POL=1): IS set on 0→1 transition (if IE=1).
 *     Falling (POL=0): IS set on 1→0 transition (if IE=1).
 *     IS is sticky — write 1 to clear.
 *
 *   Level-triggered (TRIG=1):
 *     IS = IE[N] && (pin_level == POL).  Not sticky — IS follows the
 *     condition directly.  Software writing 1 to clear a level-triggered
 *     IS bit has no lasting effect if the condition is still active.
 *
 *   IRQ output = (IE & IS) != 0   (single line → PLIC IRQ 2).
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include "hw/gpio/gevico_gpio.h"
#include "migration/vmstate.h"

/* ------------------------------------------------------------------ */
/* Compute the effective pin level for each pin.                       */
/*   Output mode (DIR=1): pin driven by OUT.                           */
/*   Input  mode (DIR=0): pin driven by external input.                */
/* ------------------------------------------------------------------ */
static uint32_t gevico_gpio_pin_level(const GEVICOGPIOState *s)
{
    return (s->dir & s->out) | (~s->dir & s->in);
}

/* ------------------------------------------------------------------ */
/* IRQ output — asserted when any enabled pin has IS set.             */
/* ------------------------------------------------------------------ */
static void gevico_gpio_update_irq(GEVICOGPIOState *s)
{
    bool asserted = (s->ie & s->is) != 0;
    qemu_set_irq(s->irq, asserted);
}

/* ------------------------------------------------------------------ */
/* Update IS based on pin_level transitions and level conditions.      */
/* prev_level / new_level are the effective pin levels (see above).    */
/* ------------------------------------------------------------------ */
static void gevico_gpio_update_is(GEVICOGPIOState *s,
                                   uint32_t prev_level,
                                   uint32_t new_level)
{
    uint32_t edge_set = 0;
    uint32_t level_is = 0;
    uint32_t pin_mask;

    for (int i = 0; i < GEVICO_GPIO_PINS; i++) {
        pin_mask = 1u << i;

        if (s->trig & pin_mask) {
            /* ── Level-triggered ─────────────────────────────── */
            /* IS follows the level condition directly, gated by IE. */
            if ((s->ie & pin_mask) == 0) {
                continue;
            }

            bool lvl_bit = (new_level & pin_mask) != 0;
            bool pol_bit = (s->pol & pin_mask) != 0;

            if (lvl_bit == pol_bit) {
                level_is |= pin_mask;
            }
        } else {
            /* ── Edge-triggered ──────────────────────────────── */
            /* Edge detection requires IE to be set. */
            if ((s->ie & pin_mask) == 0) {
                continue;
            }

            bool prev_bit = (prev_level & pin_mask) != 0;
            bool new_bit  = (new_level & pin_mask) != 0;

            if (prev_bit == new_bit) {
                continue; /* no edge */
            }

            if (s->pol & pin_mask) {
                /* Rising edge: 0→1 */
                if (!prev_bit && new_bit) {
                    edge_set |= pin_mask;
                }
            } else {
                /* Falling edge: 1→0 */
                if (prev_bit && !new_bit) {
                    edge_set |= pin_mask;
                }
            }
        }
    }

    /* Edge bits are sticky — OR into existing IS. */
    s->is |= edge_set;

    /* Level bits are transient — they replace the existing IS for
     * level-configured pins.  (If the level condition is active, the bit
     * stays 1 even after a W1C; if inactive, it stays 0.) */
    uint32_t level_mask = s->trig;   /* bits configured as level-triggered */
    s->is = (s->is & ~level_mask) | level_is;
}

/* ================================================================== */
/*  MMIO read / write                                                  */
/* ================================================================== */

static uint64_t gevico_gpio_read(void *opaque, hwaddr offset,
                                  unsigned int size)
{
    GEVICOGPIOState *s = GEVICO_GPIO(opaque);
    uint64_t r = 0;

    switch (offset) {
    case 0x00: /* GPIO_DIR */
        r = s->dir;
        break;
    case 0x04: /* GPIO_OUT */
        r = s->out;
        break;
    case 0x08: /* GPIO_IN — actual pin level (§7.3.3) */
        r = gevico_gpio_pin_level(s);
        break;
    case 0x0C: /* GPIO_IE */
        r = s->ie;
        break;
    case 0x10: /* GPIO_IS */
        r = s->is;
        break;
    case 0x14: /* GPIO_TRIG */
        r = s->trig;
        break;
    case 0x18: /* GPIO_POL */
        r = s->pol;
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

static void gevico_gpio_write(void *opaque, hwaddr offset,
                               uint64_t value, unsigned int size)
{
    GEVICOGPIOState *s = GEVICO_GPIO(opaque);
    uint32_t prev_level;

    switch (offset) {
    case 0x00: /* GPIO_DIR */
        s->dir = value;
        /* Direction change may change pin_level → re-evaluate IS */
        prev_level = s->prev_level;
        s->prev_level = gevico_gpio_pin_level(s);
        gevico_gpio_update_is(s, prev_level, s->prev_level);
        gevico_gpio_update_irq(s);
        break;
    case 0x04: /* GPIO_OUT */
        prev_level = gevico_gpio_pin_level(s);
        s->out = value;
        s->prev_level = gevico_gpio_pin_level(s);
        gevico_gpio_update_is(s, prev_level, s->prev_level);
        gevico_gpio_update_irq(s);
        break;
    case 0x0C: /* GPIO_IE */
        s->ie = value;
        /* Re-evaluate IS: enabling IE while level condition is active
         * should immediately set the level-triggered IS bits. */
        gevico_gpio_update_is(s, s->prev_level, s->prev_level);
        gevico_gpio_update_irq(s);
        break;
    case 0x10: /* GPIO_IS — write-1-to-clear */
        s->is &= ~value;
        /* Re-evaluate level-triggered pins.  If the level condition is
         * still active, the IS bit re-asserts immediately (§7.3.5 note). */
        gevico_gpio_update_is(s, s->prev_level, s->prev_level);
        gevico_gpio_update_irq(s);
        break;
    case 0x14: /* GPIO_TRIG */
        {
            /* Clear IS for pins whose trigger type changed, before
             * re-evaluating with the new TRIG settings.  This prevents
             * stale level-IS from surviving a level→edge transition, and
             * stale edge-IS from surviving an edge→level transition. */
            uint32_t changed = s->trig ^ value;
            s->is &= ~changed;
            s->trig = value;
            gevico_gpio_update_is(s, s->prev_level, s->prev_level);
            gevico_gpio_update_irq(s);
        }
        break;
    case 0x18: /* GPIO_POL */
        s->pol = value;
        /* Polarity change re-evaluates IS */
        gevico_gpio_update_is(s, s->prev_level, s->prev_level);
        gevico_gpio_update_irq(s);
        break;
    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "%s: bad write offset 0x%" HWADDR_PRIx "\n",
                      __func__, offset);
        break;
    }
}

static const MemoryRegionOps gevico_gpio_ops = {
    .read  = gevico_gpio_read,
    .write = gevico_gpio_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
    .impl.min_access_size = 4,
    .impl.max_access_size = 4,
};

/* ------------------------------------------------------------------ */
/* External pin input handler                                          */
/* ------------------------------------------------------------------ */
static void gevico_gpio_set(void *opaque, int line, int value)
{
    GEVICOGPIOState *s = GEVICO_GPIO(opaque);

    assert(line >= 0 && line < GEVICO_GPIO_PINS);

    uint32_t prev_level = gevico_gpio_pin_level(s);
    s->in = deposit32(s->in, line, 1, value != 0);
    uint32_t new_level = gevico_gpio_pin_level(s);
    s->prev_level = new_level;

    gevico_gpio_update_is(s, prev_level, new_level);
    gevico_gpio_update_irq(s);
}

/* ================================================================== */
/*  Device lifecycle                                                   */
/* ================================================================== */

static void gevico_gpio_reset(DeviceState *dev)
{
    GEVICOGPIOState *s = GEVICO_GPIO(dev);

    s->dir  = 0;
    s->out  = 0;
    s->in   = 0;
    s->ie   = 0;
    s->is   = 0;
    s->trig = 0;
    s->pol  = 0;
    s->prev_level = 0;
}

static void gevico_gpio_realize(DeviceState *dev, Error **errp)
{
    GEVICOGPIOState *s = GEVICO_GPIO(dev);

    memory_region_init_io(&s->mmio, OBJECT(dev), &gevico_gpio_ops, s,
                          TYPE_GEVICO_GPIO, GEVICO_GPIO_SIZE);
    sysbus_init_mmio(SYS_BUS_DEVICE(dev), &s->mmio);

    /* Single aggregated IRQ output → PLIC */
    sysbus_init_irq(SYS_BUS_DEVICE(dev), &s->irq);

    /* 32 external input pins */
    qdev_init_gpio_in(DEVICE(s), gevico_gpio_set, GEVICO_GPIO_PINS);
}

static const VMStateDescription vmstate_gevico_gpio = {
    .name = TYPE_GEVICO_GPIO,
    .version_id = 2,
    .minimum_version_id = 2,
    .fields = (const VMStateField[]) {
        VMSTATE_UINT32(dir,        GEVICOGPIOState),
        VMSTATE_UINT32(out,        GEVICOGPIOState),
        VMSTATE_UINT32(in,         GEVICOGPIOState),
        VMSTATE_UINT32(ie,         GEVICOGPIOState),
        VMSTATE_UINT32(is,         GEVICOGPIOState),
        VMSTATE_UINT32(trig,       GEVICOGPIOState),
        VMSTATE_UINT32(pol,        GEVICOGPIOState),
        VMSTATE_UINT32(prev_level, GEVICOGPIOState),
        VMSTATE_END_OF_LIST()
    }
};

static void gevico_gpio_class_init(ObjectClass *klass, const void *data)
{
    DeviceClass *dc = DEVICE_CLASS(klass);

    dc->vmsd = &vmstate_gevico_gpio;
    dc->realize = gevico_gpio_realize;
    device_class_set_legacy_reset(dc, gevico_gpio_reset);
    dc->desc = "G233 GPIO Controller";
}

static const TypeInfo gevico_gpio_info = {
    .name = TYPE_GEVICO_GPIO,
    .parent = TYPE_SYS_BUS_DEVICE,
    .instance_size = sizeof(GEVICOGPIOState),
    .class_init = gevico_gpio_class_init
};

static void gevico_gpio_register_types(void)
{
    type_register_static(&gevico_gpio_info);
}

type_init(gevico_gpio_register_types)
