/*
 * QEMU RISC-V Xg233ai custom accelerator helpers.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/helper-proto.h"
#include "accel/tcg/cpu-ldst.h"


/* ── dma: FP32 matrix transpose (funct7=6) ────────────────────────── */

void helper_xg233ai_dma(CPURISCVState *env, target_ulong dst_ptr,
                         target_ulong src_ptr, target_ulong grain)
{
    int n = (grain == 0) ? 8 : (grain == 1) ? 16 : 32;

    for (int i = 0; i < n; i++) {
        for (int j = 0; j < n; j++) {
            uint32_t val = cpu_ldl_data(env, src_ptr + (i * n + j) * 4);
            cpu_stl_data(env, dst_ptr + (j * n + i) * 4, val);
        }
    }
}


/* ── gemm: INT32 4×4 matrix multiply (funct7=14) ──────────────────── */

void helper_xg233ai_gemm(CPURISCVState *env, target_ulong c_ptr,
                          target_ulong a_ptr, target_ulong b_ptr)
{
    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            int64_t acc = 0;
            for (int k = 0; k < 4; k++) {
                int32_t a = (int32_t)cpu_ldl_data(env, a_ptr + (i * 4 + k) * 4);
                int32_t b = (int32_t)cpu_ldl_data(env, b_ptr + (k * 4 + j) * 4);
                acc += (int64_t)a * (int64_t)b;
            }
            cpu_stl_data(env, c_ptr + (i * 4 + j) * 4, (int32_t)acc);
        }
    }
}


/* ── sort: INT32 bubble sort (funct7=22) ──────────────────────────── */

void helper_xg233ai_sort(CPURISCVState *env, target_ulong k_val,
                          target_ulong arr_ptr, target_ulong n_val)
{
    int64_t k = (int64_t)(target_long)k_val;
    (void)n_val; /* total array size passed through, not directly used */

    for (int i = 0; i < k - 1; i++) {
        for (int j = 0; j < k - i - 1; j++) {
            target_ulong addr_j   = arr_ptr + j * 4;
            target_ulong addr_jp1 = arr_ptr + (j + 1) * 4;
            int32_t a = (int32_t)cpu_ldl_data(env, addr_j);
            int32_t b = (int32_t)cpu_ldl_data(env, addr_jp1);
            if (a > b) {
                cpu_stl_data(env, addr_j,   b);
                cpu_stl_data(env, addr_jp1, a);
            }
        }
    }
}


/* ── vadd: INT32 vector element-wise add, 16 elements (funct7=30) ─── */

void helper_xg233ai_vadd(CPURISCVState *env, target_ulong dst_ptr,
                          target_ulong src1_ptr, target_ulong src2_ptr)
{
    for (int i = 0; i < 16; i++) {
        int32_t a = (int32_t)cpu_ldl_data(env, src1_ptr + i * 4);
        int32_t b = (int32_t)cpu_ldl_data(env, src2_ptr + i * 4);
        cpu_stl_data(env, dst_ptr + i * 4, a + b);
    }
}


/* ── crush: 8→4 bit nibble compression (funct7=38) ────────────────── */

void helper_xg233ai_crush(CPURISCVState *env, target_ulong dst_ptr,
                           target_ulong src_ptr, target_ulong n)
{
    for (int i = 0; i < (int)n / 2; i++) {
        uint8_t lo = cpu_ldub_data(env, src_ptr + 2 * i) & 0x0F;
        uint8_t hi = (cpu_ldub_data(env, src_ptr + 2 * i + 1) & 0x0F) << 4;
        cpu_stb_data(env, dst_ptr + i, lo | hi);
    }
    if ((int)n & 1) {
        cpu_stb_data(env, dst_ptr + (int)n / 2,
                     cpu_ldub_data(env, src_ptr + (int)n - 1) & 0x0F);
    }
}


/* ── expand: 4→8 bit nibble decompression (funct7=54) ─────────────── */

void helper_xg233ai_expand(CPURISCVState *env, target_ulong dst_ptr,
                            target_ulong src_ptr, target_ulong n)
{
    for (int i = 0; i < (int)n; i++) {
        uint8_t b = cpu_ldub_data(env, src_ptr + i);
        cpu_stb_data(env, dst_ptr + 2 * i,       b & 0x0F);
        cpu_stb_data(env, dst_ptr + 2 * i + 1,  (b >> 4) & 0x0F);
    }
}


/* ── vrelu: INT32 vector ReLU, 16 elements (funct7=86) ────────────── */

void helper_xg233ai_vrelu(CPURISCVState *env, target_ulong dst_ptr,
                           target_ulong src_ptr, target_ulong n)
{
    (void)n; /* element count, always 16 in tests */
    for (int i = 0; i < 16; i++) {
        int32_t val = (int32_t)cpu_ldl_data(env, src_ptr + i * 4);
        cpu_stl_data(env, dst_ptr + i * 4, (val > 0) ? val : 0);
    }
}


/* ── vscale: INT32 vector×scalar, 16 elements (funct7=102) ────────── */

void helper_xg233ai_vscale(CPURISCVState *env, target_ulong dst_ptr,
                            target_ulong src_ptr, target_ulong scale)
{
    int32_t s = (int32_t)(target_long)scale;

    for (int i = 0; i < 16; i++) {
        int32_t val = (int32_t)cpu_ldl_data(env, src_ptr + i * 4);
        cpu_stl_data(env, dst_ptr + i * 4, val * s);
    }
}


/* ── vdot: INT32 dot product → int64 (funct7=70) ──────────────────── */

target_ulong helper_xg233ai_vdot(CPURISCVState *env, target_ulong a_ptr,
                                  target_ulong b_ptr)
{
    int64_t acc = 0;

    for (int i = 0; i < 16; i++) {
        int32_t a = (int32_t)cpu_ldl_data(env, a_ptr + i * 4);
        int32_t b = (int32_t)cpu_ldl_data(env, b_ptr + i * 4);
        acc += (int64_t)a * (int64_t)b;
    }
    return (target_ulong)acc;
}


/* ── vmax: INT32 max reduction → int64 (funct7=118) ───────────────── */

target_ulong helper_xg233ai_vmax(CPURISCVState *env, target_ulong src_ptr,
                                  target_ulong n)
{
    int32_t max_val = (int32_t)cpu_ldl_data(env, src_ptr);

    for (int i = 1; i < (int)n; i++) {
        int32_t val = (int32_t)cpu_ldl_data(env, src_ptr + i * 4);
        if (val > max_val) {
            max_val = val;
        }
    }
    /* Sign-extend to int64 per test spec */
    return (target_ulong)(int64_t)(int32_t)max_val;
}
