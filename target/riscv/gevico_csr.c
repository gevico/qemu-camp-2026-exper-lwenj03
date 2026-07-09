/*
 * Gevico-specific CSRs.
 *
 * Copyright (c) 2025
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"

#define CSR_GEVICO_ACCEL_ENABLE  0x301

static uint64_t gevico_accel_enable;

static RISCVException any(CPURISCVState *env, int csrno)
{
    return RISCV_EXCP_NONE;
}

static RISCVException read_gevico_accel(CPURISCVState *env, int csrno,
                                         target_ulong *val)
{
    *val = gevico_accel_enable;
    return RISCV_EXCP_NONE;
}

static RISCVException write_gevico_accel(CPURISCVState *env, int csrno,
                                          target_ulong val, uintptr_t ra)
{
    gevico_accel_enable = val;
    return RISCV_EXCP_NONE;
}

const RISCVCSR gevico_csr_list[] = {
    {
        .csrno = CSR_GEVICO_ACCEL_ENABLE,
        .csr_ops = { "gevico_accel", any,
                     read_gevico_accel, write_gevico_accel }
    },
    { },
};
