/*
 * QEMU GPGPU - RISC-V SIMT Core Implementation
 *
 * Copyright (c) 2024-2025
 *
 * This work is licensed under the terms of the GNU GPL, version 2 or later.
 * See the COPYING file in the top-level directory.
 *
 * 简化的 RV32I + RV32F + LP-Float SIMT 指令解释器。
 * 每个 Lane 拥有独立的寄存器文件 (x0-x31, f0-f31, pc, fcsr)，
 * Warp 内所有活跃 Lane 锁步执行同一条指令 (§12.4)。
 */

#include "qemu/osdep.h"
#include "qemu/log.h"
#include <math.h>
#include "gpgpu.h"
#include "gpgpu_core.h"

/* ============================================================================
 * 指令解码宏
 * ============================================================================ */
#define OPCODE(inst)    ((inst) & 0x7F)
#define RD(inst)        (((inst) >> 7) & 0x1F)
#define FUNCT3(inst)    (((inst) >> 12) & 0x7)
#define RS1(inst)       (((inst) >> 15) & 0x1F)
#define RS2(inst)       (((inst) >> 20) & 0x1F)
#define FUNCT7(inst)    (((inst) >> 25) & 0x7F)
#define FUNCT5(inst)    (((inst) >> 27) & 0x1F)  /* for opcode 0x53 (OP-FP) */

#define IMM_I(inst)     ((int32_t)(inst) >> 20)
#define IMM_S(inst)     ((int32_t)((((inst) >> 25) << 5) | \
                                   (((inst) >> 7) & 0x1F)))
#define IMM_B(inst)     ((int32_t)((((inst) >> 31) << 12) | \
                                   ((((inst) >> 7) & 1) << 11) | \
                                   ((((inst) >> 25) & 0x3F) << 5) | \
                                   ((((inst) >> 8) & 0xF) << 1)))
#define IMM_U(inst)     ((int32_t)(inst) & 0xFFFFF000U)
#define IMM_J(inst)     ((int32_t)((((inst) >> 31) << 20) | \
                                   ((((inst) >> 12) & 0xFF) << 12) | \
                                   ((((inst) >> 20) & 1) << 11) | \
                                   ((((inst) >> 21) & 0x3FF) << 1)))

/* ============================================================================
 * GPU 内存访问 (§12.5)
 *
 *   0x0000_0000 – VRAM_SIZE-1    → VRAM 读写
 *   0x8000_0000 + offset          → CTRL 寄存器 (只读)
 * ============================================================================ */

static uint8_t gpu_mem_read8(GPGPUState *s, uint32_t addr)
{
    if (addr < s->vram_size) {
        return s->vram_ptr[addr];
    }
    return 0;
}

static uint16_t gpu_mem_read16(GPGPUState *s, uint32_t addr)
{
    if (addr + 1 < s->vram_size) {
        uint16_t val;
        memcpy(&val, s->vram_ptr + addr, 2);
        return val;
    }
    return 0;
}

static uint32_t gpu_mem_read32(GPGPUState *s, uint32_t addr)
{
    /* CTRL 地址空间 §12.5: GPU 只读获取 thread/block/grid 信息 */
    if (addr >= GPGPU_CORE_CTRL_BASE) {
        uint32_t ctrl_off = addr - GPGPU_CORE_CTRL_BASE;
        switch (ctrl_off) {
        case 0x00: return s->simt.thread_id[0];   /* THREAD_ID_X */
        case 0x04: return s->simt.thread_id[1];   /* THREAD_ID_Y */
        case 0x08: return s->simt.thread_id[2];   /* THREAD_ID_Z */
        case 0x10: return s->simt.block_id[0];    /* BLOCK_ID_X */
        case 0x14: return s->simt.block_id[1];    /* BLOCK_ID_Y */
        case 0x18: return s->simt.block_id[2];    /* BLOCK_ID_Z */
        case 0x20: return s->kernel.block_dim[0]; /* BLOCK_DIM_X */
        case 0x24: return s->kernel.block_dim[1]; /* BLOCK_DIM_Y */
        case 0x28: return s->kernel.block_dim[2]; /* BLOCK_DIM_Z */
        case 0x30: return s->kernel.grid_dim[0];  /* GRID_DIM_X */
        case 0x34: return s->kernel.grid_dim[1];  /* GRID_DIM_Y */
        case 0x38: return s->kernel.grid_dim[2];  /* GRID_DIM_Z */
        default:   return 0;
        }
    }
    if (addr + 3 < s->vram_size) {
        uint32_t val;
        memcpy(&val, s->vram_ptr + addr, 4);
        return val;
    }
    return 0;
}

static void gpu_mem_write8(GPGPUState *s, uint32_t addr, uint8_t val)
{
    if (addr < s->vram_size) {
        s->vram_ptr[addr] = val;
    }
}

static void gpu_mem_write16(GPGPUState *s, uint32_t addr, uint16_t val)
{
    if (addr + 1 < s->vram_size) {
        memcpy(s->vram_ptr + addr, &val, 2);
    }
}

static void gpu_mem_write32(GPGPUState *s, uint32_t addr, uint32_t val)
{
    if (addr + 3 < s->vram_size) {
        memcpy(s->vram_ptr + addr, &val, 4);
    }
}

/* ============================================================================
 * CSR 读写辅助
 * ============================================================================ */

static uint32_t csr_read(GPGPULane *lane, uint32_t csr_addr)
{
    switch (csr_addr) {
    case CSR_MHARTID:
        return lane->mhartid;
    case CSR_FCSR:
        return lane->fcsr;
    case CSR_FFLAGS:
        return lane->fcsr & 0x1F;
    case CSR_FRM:
        return (lane->fcsr >> 5) & 0x7;
    default:
        return 0;
    }
}

static void csr_write(GPGPULane *lane, uint32_t csr_addr, uint32_t val)
{
    switch (csr_addr) {
    case CSR_MHARTID:
        /* mhartid 只读，忽略写入 */
        break;
    case CSR_FCSR:
        lane->fcsr = val & 0xFF;
        break;
    case CSR_FFLAGS:
        lane->fcsr = (lane->fcsr & ~0x1F) | (val & 0x1F);
        break;
    case CSR_FRM:
        lane->fcsr = (lane->fcsr & ~0xE0) | ((val & 0x7) << 5);
        break;
    default:
        break;
    }
}

/* ============================================================================
 * 浮点辅助函数
 * ============================================================================ */

/* 设置 softfloat 舍入模式 (§13.2 rm 字段) */
static void set_float_rm(GPGPULane *lane, uint32_t rm)
{
    if (rm == 7) { /* dynamic: 使用 fcsr.frm */
        rm = (lane->fcsr >> 5) & 0x7;
    }
    switch (rm) {
    case 0: /* RNE */
        lane->fp_status.float_rounding_mode = float_round_nearest_even;
        break;
    case 1: /* RTZ */
        lane->fp_status.float_rounding_mode = float_round_to_zero;
        break;
    case 2: /* RDN */
        lane->fp_status.float_rounding_mode = float_round_down;
        break;
    case 3: /* RUP */
        lane->fp_status.float_rounding_mode = float_round_up;
        break;
    case 4: /* RMM */
        lane->fp_status.float_rounding_mode = float_round_ties_away;
        break;
    default:
        break;
    }
}

/* 将 softfloat 累积的异常标志合并到 fcsr.fflags */
static void update_fcsr_flags(GPGPULane *lane)
{
    uint8_t sf_flags = get_float_exception_flags(&lane->fp_status);
    lane->fcsr |= (sf_flags & 0x1F);
    set_float_exception_flags(0, &lane->fp_status);
}

/* ============================================================================
 * 低精度浮点转换 (§13.3)
 *
 * BF16:    sign(1) + exp(8, bias=127) + mantissa(7) = 16 bit
 * E4M3:    sign(1) + exp(4, bias=7)   + mantissa(3) = 8 bit, max ±448
 * E5M2:    sign(1) + exp(5, bias=15)  + mantissa(2) = 8 bit
 * E2M1:    sign(1) + exp(2, bias=1)   + mantissa(1) = 4 bit (3 magnitude bits)
 *
 * 所有转换经 BF16 中转 (§13.3.2 描述)。
 * ============================================================================ */

/* ── BF16 ↔ FP32 ─────────────────────────────────────────────────── */

static uint16_t f32_to_bf16(uint32_t f32_bits)
{
    uint32_t sign = (f32_bits >> 31) & 1;
    int32_t exp   = (int32_t)((f32_bits >> 23) & 0xFF);
    uint32_t mant = f32_bits & 0x7FFFFF;

    if (exp == 0xFF) {          /* Inf / NaN */
        return (uint16_t)((sign << 15) | (0xFF << 7) |
                          ((mant ? (mant >> 16) | 1 : 0) & 0x7F));
    }
    if (exp == 0) {             /* zero / denormal → flush to zero */
        return (uint16_t)(sign << 15);
    }

    /* RNE: 23-bit mantissa → 7-bit */
    uint32_t bf_mant = mant >> 16;
    uint32_t round   = (mant >> 15) & 1;
    uint32_t sticky  = (mant & 0x7FFF) ? 1 : 0;

    if (round && (sticky || (bf_mant & 1))) {
        bf_mant++;
        if (bf_mant > 0x7F) { bf_mant = 0; exp++; }
    }
    if (exp >= 0xFF) {          /* overflow → Inf */
        return (uint16_t)((sign << 15) | (0xFF << 7));
    }

    return (uint16_t)((sign << 15) | ((uint32_t)exp << 7) | bf_mant);
}

static uint32_t bf16_to_f32(uint16_t bf16)
{
    return ((uint32_t)(bf16 & 0x8000) << 16) |
           ((uint32_t)(bf16 & 0x7F80) << 16) |
           ((uint32_t)(bf16 & 0x007F) << 16);
}

/* ── E4M3 ↔ FP32 (经 BF16 中转) ──────────────────────────────────── */

static uint32_t float_to_f32_bits(float f)
{
    uint32_t bits;
    memcpy(&bits, &f, 4);
    return bits;
}

static uint8_t f32_to_e4m3_via_bf16(uint32_t f32_bits)
{
    uint16_t bf = f32_to_bf16(f32_bits);
    uint32_t sign = (bf >> 15) & 1;
    int32_t bf_exp = (int32_t)((bf >> 7) & 0xFF);
    uint32_t bf_mant = bf & 0x7F;

    /* NaN / Inf → 饱和到最大值 */
    if (bf_exp == 0xFF) {
        return (uint8_t)((sign << 7) | 0x7E);  /* exp=15, mant=6 → 448 */
    }
    if (bf_exp == 0 && bf_mant == 0) {
        return (uint8_t)(sign << 7);
    }

    /* 计算实际浮点值 */
    float val;
    if (bf_exp == 0) {
        val = (float)bf_mant / 128.0f * powf(2.0f, -126.0f);
    } else {
        val = (1.0f + (float)bf_mant / 128.0f) * powf(2.0f, (float)(bf_exp - 127));
    }
    if (sign) val = -val;

    float abs_val = fabsf(val);

    /* 饱和检查 */
    if (abs_val >= 448.0f) {
        return (uint8_t)((sign << 7) | 0x7E);
    }

    /* zero / 下溢 */
    if (abs_val == 0.0f || abs_val < powf(2.0f, -9.0f)) {
        return (uint8_t)(sign << 7);
    }

    /*
     * 编码为 E4M3: val = 2^(exp-7) * (1 + mant/8)
     * 通过反复缩放找到 exp，使得 (1 + mant/8) ∈ [1.0, 2.0)
     */
    float scaled = abs_val;
    int32_t e4_exp = 0;

    while (scaled >= 2.0f)  { scaled /= 2.0f;  e4_exp++; }
    while (scaled < 1.0f)   { scaled *= 2.0f;  e4_exp--; }

    e4_exp += 7;  /* 转换为 bias=7 */

    /* denormal: e4_exp < 1 */
    if (e4_exp < 1) {
        float denorm = abs_val / (powf(2.0f, -6.0f) / 8.0f);  /* abs / (2^(-6)/8) */
        int32_t e4_mant = (int32_t)(denorm + 0.5f);
        if (e4_mant < 0) e4_mant = 0;
        if (e4_mant > 7) e4_mant = 7;
        if (e4_mant == 0) return (uint8_t)(sign << 7);
        return (uint8_t)((sign << 7) | (uint32_t)e4_mant);
    }

    /* overflow */
    if (e4_exp > 15) {
        return (uint8_t)((sign << 7) | 0x7E);
    }

    /* 舍入 mantissa: scaled ∈ [1.0, 2.0), mant = round((scaled-1)*8) */
    int32_t e4_mant = (int32_t)((scaled - 1.0f) * 8.0f + 0.5f);
    if (e4_mant >= 8) { e4_mant = 0; e4_exp++; }
    if (e4_exp > 15) {
        return (uint8_t)((sign << 7) | 0x7E);
    }

    /* exp=15 且 mant≥6 → 饱和到 448 */
    if (e4_exp == 15 && e4_mant >= 6) {
        return (uint8_t)((sign << 7) | 0x7E);
    }

    return (uint8_t)((sign << 7) | ((uint32_t)e4_exp << 3) | (uint32_t)e4_mant);
}

static uint32_t e4m3_to_f32_via_bf16(uint8_t e4)
{
    uint32_t sign = (e4 >> 7) & 1;
    uint32_t exp  = (e4 >> 3) & 0xF;
    uint32_t mant = e4 & 0x7;

    float val;
    if (exp == 0) {
        if (mant == 0) {
            val = 0.0f;
        } else {
            val = ldexpf((float)mant / 8.0f, -6);
        }
    } else if (exp == 15 && mant == 7) {
        /* NaN encoding → return NaN */
        return 0x7FFFFFFF;
    } else {
        val = ldexpf(1.0f + (float)mant / 8.0f, (int32_t)exp - 7);
    }
    if (sign) val = -val;

    /* BF16 中转: 编码为 BF16, 再转为 FP32 */
    uint16_t bf = f32_to_bf16(float_to_f32_bits(val));
    return bf16_to_f32(bf);
}

/* ── E5M2 ↔ FP32 (经 BF16 中转) ──────────────────────────────────── */

static uint8_t f32_to_e5m2_via_bf16(uint32_t f32_bits)
{
    uint16_t bf = f32_to_bf16(f32_bits);
    uint32_t sign = (bf >> 15) & 1;
    int32_t bf_exp = (int32_t)((bf >> 7) & 0xFF);
    uint32_t bf_mant = bf & 0x7F;

    /* Inf → Inf */
    if (bf_exp == 0xFF && bf_mant == 0) {
        return (uint8_t)((sign << 7) | (0x1F << 2));
    }
    /* NaN → NaN */
    if (bf_exp == 0xFF) {
        return (uint8_t)((sign << 7) | (0x1F << 2) | 1);
    }
    /* zero */
    if (bf_exp == 0 && bf_mant == 0) {
        return (uint8_t)(sign << 7);
    }

    /* 计算实际浮点值 */
    float val;
    if (bf_exp == 0) {
        val = (float)bf_mant / 128.0f * powf(2.0f, -126.0f);
    } else {
        val = (1.0f + (float)bf_mant / 128.0f) * powf(2.0f, (float)(bf_exp - 127));
    }
    if (sign) val = -val;

    float abs_val = fabsf(val);

    if (abs_val == 0.0f) {
        return (uint8_t)(sign << 7);
    }

    /*
     * 编码为 E5M2: val = 2^(exp-15) * (1 + mant/4)
     * 规格化: scaled ∈ [1.0, 2.0)
     */
    float scaled = abs_val;
    int32_t e5_exp = 0;

    while (scaled >= 2.0f)  { scaled /= 2.0f;  e5_exp++; }
    while (scaled < 1.0f)   { scaled *= 2.0f;  e5_exp--; }

    e5_exp += 15;  /* 转换为 bias=15 */

    /* overflow → Inf */
    if (e5_exp > 30) {
        return (uint8_t)((sign << 7) | (0x1F << 2));
    }

    /* denormal */
    if (e5_exp < 1) {
        float denorm = abs_val / (powf(2.0f, -14.0f) / 4.0f);
        int32_t e5_mant = (int32_t)(denorm + 0.5f);
        if (e5_mant < 0) e5_mant = 0;
        if (e5_mant > 3) e5_mant = 3;
        if (e5_mant == 0) return (uint8_t)(sign << 7);
        return (uint8_t)((sign << 7) | (uint32_t)e5_mant);
    }

    /* 舍入 mantissa: scaled ∈ [1.0, 2.0), mant = round((scaled-1)*4) */
    int32_t e5_mant = (int32_t)((scaled - 1.0f) * 4.0f + 0.5f);
    if (e5_mant >= 4) { e5_mant = 0; e5_exp++; }
    if (e5_exp > 30) {
        return (uint8_t)((sign << 7) | (0x1F << 2));
    }

    return (uint8_t)((sign << 7) | ((uint32_t)e5_exp << 2) | (uint32_t)e5_mant);
}

static uint32_t e5m2_to_f32_via_bf16(uint8_t e5)
{
    uint32_t sign = (e5 >> 7) & 1;
    uint32_t exp  = (e5 >> 2) & 0x1F;
    uint32_t mant = e5 & 0x3;

    float val;
    if (exp == 0) {
        if (mant == 0) {
            val = 0.0f;
        } else {
            val = ldexpf((float)mant / 4.0f, -14);
        }
    } else if (exp == 0x1F) {
        /* Inf or NaN */
        if (mant == 0) {
            return sign ? 0xFF800000 : 0x7F800000;  /* Inf */
        } else {
            return 0x7FFFFFFF;  /* NaN */
        }
    } else {
        val = ldexpf(1.0f + (float)mant / 4.0f, (int32_t)exp - 15);
    }
    if (sign) val = -val;

    uint16_t bf = f32_to_bf16(float_to_f32_bits(val));
    return bf16_to_f32(bf);
}

/* ── E2M1 ↔ FP32 (经 E4M3 → BF16 中转, §13.3.3) ────────────────── */

/* E2M1 正数编码表 (3-bit magnitude) */
static const float e2m1_val_table[8] = {
    0.0f, 0.5f, 1.0f, 1.5f, 2.0f, 3.0f, 4.0f, 6.0f
};

static uint8_t f32_to_e2m1(uint32_t f32_bits)
{
    /* FP32 → BF16 → E4M3 → E2M1 */
    uint16_t bf = f32_to_bf16(f32_bits);
    float val;
    uint32_t sign = (bf >> 15) & 1;
    int32_t bf_exp = (int32_t)((bf >> 7) & 0xFF);
    uint32_t bf_mant = bf & 0x7F;

    if (bf_exp == 0xFF) {
        /* Inf / NaN → 饱和到 ±6.0 */
        return (uint8_t)((sign << 7) | 7);
    }
    if (bf_exp == 0 && bf_mant == 0) {
        return (uint8_t)(sign << 7);  /* zero */
    }
    if (bf_exp == 0) {
        val = ldexpf((float)bf_mant / 128.0f, -126);
    } else {
        val = ldexpf(1.0f + (float)bf_mant / 128.0f, bf_exp - 127);
    }
    if (sign) val = -val;

    float abs_val = fabsf(val);

    /* 饱和到 ±6.0 */
    if (abs_val >= 6.0f) {
        return (uint8_t)((sign << 7) | 7);
    }

    /* §13.3.3: 手写阈值舍入 — 找最近的可表示值 */
    /* 阈值 = (e2m1_val_table[i] + e2m1_val_table[i+1]) / 2 */
    uint8_t best_idx = 0;
    for (int i = 7; i >= 0; i--) {
        if (abs_val >= e2m1_val_table[i]) {
            if (i < 7) {
                float midpoint = (e2m1_val_table[i] + e2m1_val_table[i + 1]) * 0.5f;
                best_idx = (abs_val >= midpoint) ? (uint8_t)(i + 1) : (uint8_t)i;
            } else {
                best_idx = 7;
            }
            break;
        }
    }

    return (uint8_t)((sign << 7) | best_idx);
}

static uint32_t e2m1_to_f32(uint8_t e2)
{
    /* §13.3.3: E2M1 → E4M3 → BF16 → FP32 */
    uint32_t sign = (e2 >> 7) & 1;
    uint32_t mag  = e2 & 0x7;
    float val = e2m1_val_table[mag];
    if (sign) val = -val;

    /* 先经 E4M3 */
    uint8_t e4 = f32_to_e4m3_via_bf16(float_to_f32_bits(val));
    /* 再经 BF16 → FP32 */
    return e4m3_to_f32_via_bf16(e4);
}

/* ============================================================================
 * 单条指令执行 — 针对单个 Lane
 *
 * 返回: true 该 lane 保持活跃，false 该 lane 退出 (ebreak / ret)
 * ============================================================================ */

static bool exec_one_inst(GPGPUState *s, GPGPULane *lane,
                          uint32_t inst, uint32_t *next_pc)
{
    uint32_t opcode   = OPCODE(inst);
    uint32_t rd       = RD(inst);
    uint32_t rs1      = RS1(inst);
    uint32_t rs2      = RS2(inst);
    uint32_t funct3   = FUNCT3(inst);
    uint32_t funct7   = FUNCT7(inst);
    int32_t  imm_i    = IMM_I(inst);
    int32_t  imm_s    = IMM_S(inst);
    int32_t  imm_b    = IMM_B(inst);
    int32_t  imm_u    = IMM_U(inst);
    int32_t  imm_j    = IMM_J(inst);

    uint32_t *gpr = lane->gpr;
    int32_t  rs1_val = (int32_t)gpr[rs1];
    int32_t  rs2_val = (int32_t)gpr[rs2];
    uint32_t addr;
    uint32_t shamt;

    *next_pc = lane->pc + 4;

    switch (opcode) {
    /* ── LUI ─────────────────────────────────────────────────────── */
    case 0x37:
        gpr[rd] = (uint32_t)imm_u;
        break;

    /* ── AUIPC ───────────────────────────────────────────────────── */
    case 0x17:
        gpr[rd] = lane->pc + (uint32_t)imm_u;
        break;

    /* ── JAL ─────────────────────────────────────────────────────── */
    case 0x6F:
        gpr[rd] = lane->pc + 4;
        *next_pc = lane->pc + (uint32_t)imm_j;
        break;

    /* ── JALR ────────────────────────────────────────────────────── */
    case 0x67:
        {
            uint32_t target = ((uint32_t)(rs1_val + imm_i)) & ~1u;
            gpr[rd] = lane->pc + 4;
            /*
             * §12.4: ret (JALR x0, ra, 0) 当 ra==0 时 lane 退出
             * JALR x0, x1, 0 → rd=0, rs1=1, ra==0
             */
            if (rd == 0 && rs1 == 1 && gpr[1] == 0) {
                return false;
            }
            *next_pc = target;
        }
        break;

    /* ── 分支 (BEQ / BNE / BLT / BGE / BLTU / BGEU) ────────────── */
    case 0x63:
        {
            bool taken = false;
            switch (funct3) {
            case 0x0: taken = (rs1_val == rs2_val); break;
            case 0x1: taken = (rs1_val != rs2_val); break;
            case 0x4: taken = (rs1_val < rs2_val); break;
            case 0x5: taken = (rs1_val >= rs2_val); break;
            case 0x6: taken = ((uint32_t)rs1_val < (uint32_t)rs2_val); break;
            case 0x7: taken = ((uint32_t)rs1_val >= (uint32_t)rs2_val); break;
            }
            if (taken) {
                *next_pc = lane->pc + (uint32_t)imm_b;
            }
        }
        break;

    /* ── Load (LB / LH / LW / LBU / LHU) ─────────────────────────── */
    case 0x03:
        addr = (uint32_t)(rs1_val + imm_i);
        switch (funct3) {
        case 0x0: gpr[rd] = (int32_t)(int8_t)gpu_mem_read8(s, addr);  break;
        case 0x1: gpr[rd] = (int32_t)(int16_t)gpu_mem_read16(s, addr); break;
        case 0x2: gpr[rd] = gpu_mem_read32(s, addr);                   break;
        case 0x4: gpr[rd] = (uint32_t)gpu_mem_read8(s, addr);          break;
        case 0x5: gpr[rd] = (uint32_t)gpu_mem_read16(s, addr);         break;
        }
        break;

    /* ── Store (SB / SH / SW) ────────────────────────────────────── */
    case 0x23:
        addr = (uint32_t)(rs1_val + imm_s);
        switch (funct3) {
        case 0x0: gpu_mem_write8(s, addr, (uint8_t)gpr[rs2]);  break;
        case 0x1: gpu_mem_write16(s, addr, (uint16_t)gpr[rs2]); break;
        case 0x2: gpu_mem_write32(s, addr, gpr[rs2]);          break;
        }
        break;

    /* ── FLW (float load) ─────────────────────────────────────────── */
    case 0x07:
        if (funct3 == 0x2) {
            addr = (uint32_t)(rs1_val + imm_i);
            lane->fpr[rd] = gpu_mem_read32(s, addr);
        }
        break;

    /* ── FSW (float store) ────────────────────────────────────────── */
    case 0x27:
        if (funct3 == 0x2) {
            addr = (uint32_t)(rs1_val + imm_s);
            gpu_mem_write32(s, addr, lane->fpr[rs2]);
        }
        break;

    /* ── I 型 ALU (ADDI / SLTI / SLTIU / XORI / ORI / ANDI /
     *               SLLI / SRLI / SRAI) ───────────────────────────── */
    case 0x13:
        shamt = (uint32_t)imm_i & 0x1F;
        switch (funct3) {
        case 0x0: gpr[rd] = (uint32_t)(rs1_val + imm_i); break;
        case 0x2: gpr[rd] = (rs1_val < imm_i) ? 1 : 0; break;
        case 0x3: gpr[rd] = ((uint32_t)rs1_val < (uint32_t)imm_i) ? 1 : 0; break;
        case 0x4: gpr[rd] = (uint32_t)(rs1_val ^ imm_i); break;
        case 0x6: gpr[rd] = (uint32_t)(rs1_val | imm_i); break;
        case 0x7: gpr[rd] = (uint32_t)(rs1_val & imm_i); break;
        case 0x1: gpr[rd] = (uint32_t)rs1_val << shamt; break;
        case 0x5:
            if (funct7 == 0x00) {
                gpr[rd] = (uint32_t)rs1_val >> shamt;
            } else if (funct7 == 0x20) {
                gpr[rd] = (uint32_t)(rs1_val >> shamt);
            }
            break;
        }
        break;

    /* ── R 型 ALU (ADD / SUB / SLL / SLT / SLTU /
     *              XOR / SRL / SRA / OR / AND) ────────────────────── */
    case 0x33:
        shamt = rs2_val & 0x1F;
        switch (funct3) {
        case 0x0:
            if (funct7 == 0x00) {
                gpr[rd] = (uint32_t)(rs1_val + rs2_val);
            } else if (funct7 == 0x20) {
                gpr[rd] = (uint32_t)(rs1_val - rs2_val);
            }
            break;
        case 0x1: gpr[rd] = (uint32_t)rs1_val << shamt; break;
        case 0x2: gpr[rd] = (rs1_val < rs2_val) ? 1 : 0; break;
        case 0x3: gpr[rd] = ((uint32_t)rs1_val < (uint32_t)rs2_val) ? 1 : 0; break;
        case 0x4: gpr[rd] = (uint32_t)(rs1_val ^ rs2_val); break;
        case 0x5:
            if (funct7 == 0x00) {
                gpr[rd] = (uint32_t)rs1_val >> shamt;
            } else if (funct7 == 0x20) {
                gpr[rd] = (uint32_t)(rs1_val >> shamt);
            }
            break;
        case 0x6: gpr[rd] = (uint32_t)(rs1_val | rs2_val); break;
        case 0x7: gpr[rd] = (uint32_t)(rs1_val & rs2_val); break;
        }
        break;

    /* ── FENCE / FENCE.I — NOP ───────────────────────────────────── */
    case 0x0F:
        break;

    /* ── SYSTEM (ECALL / EBREAK / CSRRx) ─────────────────────────── */
    case 0x73:
        if (funct3 == 0x0) {
            /* ECALL (imm_i == 0) / EBREAK (imm_i == 1) */
            if (imm_i == 1) {
                return false;  /* §12.4: ebreak 导致 lane 退出 */
            }
            /* ECALL: 当前不做系统调用，视为 NOP */
        } else {
            uint32_t csr_addr = (uint32_t)imm_i & 0xFFF;
            uint32_t csr_val = csr_read(lane, csr_addr);
            uint32_t new_csr = csr_val;

            switch (funct3) {
            case 0x1: /* CSRRW */
                gpr[rd] = csr_val;
                new_csr = gpr[rs1];
                break;
            case 0x2: /* CSRRS */
                gpr[rd] = csr_val;
                if (rs1 != 0) new_csr = csr_val | gpr[rs1];
                break;
            case 0x3: /* CSRRC */
                gpr[rd] = csr_val;
                if (rs1 != 0) new_csr = csr_val & ~gpr[rs1];
                break;
            case 0x5: /* CSRRWI */
                gpr[rd] = csr_val;
                new_csr = (uint32_t)rs1;  /* zimm[4:0] = rs1 field */
                break;
            case 0x6: /* CSRRSI */
                gpr[rd] = csr_val;
                if (rs1 != 0) new_csr = csr_val | (uint32_t)rs1;
                break;
            case 0x7: /* CSRRCI */
                gpr[rd] = csr_val;
                if (rs1 != 0) new_csr = csr_val & ~(uint32_t)rs1;
                break;
            }
            csr_write(lane, csr_addr, new_csr);
        }
        break;

    /* ── FMA (Fused Multiply-Add, opcodes 0x43/0x47/0x4B/0x4F) ───── */
    case 0x43: /* FMADD.S: rd = (rs1 * rs2) + rs3 */
    case 0x47: /* FMSUB.S: rd = (rs1 * rs2) - rs3 */
    case 0x4B: /* FNMSUB.S: rd = -(rs1 * rs2) + rs3 */
    case 0x4F: /* FNMADD.S: rd = -(rs1 * rs2) - rs3 */
        {
            uint32_t rs3_idx = FUNCT5(inst); /* rs3 in bits 31:27 */
            uint32_t fmt = (inst >> 25) & 0x3;
            uint32_t rm  = funct3;
            if (fmt == 0) { /* single-precision, 无 NaN boxing */
                float32 f1 = lane->fpr[rs1];
                float32 f2 = lane->fpr[rs2];
                float32 f3 = lane->fpr[rs3_idx];
                float32 result;
                set_float_rm(lane, rm);
                if (opcode == 0x43) {
                    result = float32_muladd(f1, f2, f3, 0, &lane->fp_status);
                } else if (opcode == 0x47) {
                    result = float32_muladd(f1, f2, f3,
                                            float_muladd_negate_c,
                                            &lane->fp_status);
                } else if (opcode == 0x4B) {
                    result = float32_muladd(f1, f2, f3,
                                            float_muladd_negate_product,
                                            &lane->fp_status);
                } else {
                    result = float32_muladd(f1, f2, f3,
                                            float_muladd_negate_product |
                                            float_muladd_negate_c,
                                            &lane->fp_status);
                }
                lane->fpr[rd] = result;
                update_fcsr_flags(lane);
            }
        }
        break;

    /* ── OP-FP (§13.2: 浮点 + 低精度转换, opcode 0x53) ───────────── */
    case 0x53:
        {
            uint32_t funct5 = FUNCT5(inst);
            uint32_t funct7_full = FUNCT7(inst);
            uint32_t rm     = funct3;
            uint32_t inst_rs2 = RS2(inst);  /* LP-Float 方向选择 */

            /*
             * §13.3 LP-Float 自定义扩展: 按完整 funct7 分发
             * funct7=0x22 (BF16), 0x24 (FP8), 0x26 (FP4/E2M1)
             */
            if (funct7_full == 0x22) {
                /* BF16: FCVT.S.BF16 (rs2=0), FCVT.BF16.S (rs2=1) */
                if (inst_rs2 == 0) {
                    uint16_t bf_val = (uint16_t)(lane->fpr[rs1] & 0xFFFF);
                    lane->fpr[rd] = bf16_to_f32(bf_val);
                } else {
                    uint16_t bf_val = f32_to_bf16(lane->fpr[rs1]);
                    lane->fpr[rd] = (uint32_t)bf_val;
                }
                break;
            }
            if (funct7_full == 0x24) {
                /* FP8: rs2=0:E4M3→FP32, 1:FP32→E4M3, 2:E5M2→FP32, 3:FP32→E5M2 */
                if (inst_rs2 == 0) {
                    uint8_t e4 = (uint8_t)(lane->fpr[rs1] & 0xFF);
                    lane->fpr[rd] = e4m3_to_f32_via_bf16(e4);
                } else if (inst_rs2 == 1) {
                    uint8_t e4 = f32_to_e4m3_via_bf16(lane->fpr[rs1]);
                    lane->fpr[rd] = (uint32_t)e4;
                } else if (inst_rs2 == 2) {
                    uint8_t e5 = (uint8_t)(lane->fpr[rs1] & 0xFF);
                    lane->fpr[rd] = e5m2_to_f32_via_bf16(e5);
                } else {
                    uint8_t e5 = f32_to_e5m2_via_bf16(lane->fpr[rs1]);
                    lane->fpr[rd] = (uint32_t)e5;
                }
                break;
            }
            if (funct7_full == 0x26) {
                /* E2M1: rs2=0:E2M1→FP32, rs2=1:FP32→E2M1 */
                if (inst_rs2 == 0) {
                    uint8_t e2 = (uint8_t)(lane->fpr[rs1] & 0xF);
                    lane->fpr[rd] = e2m1_to_f32(e2);
                } else {
                    uint8_t e2 = f32_to_e2m1(lane->fpr[rs1]);
                    lane->fpr[rd] = (uint32_t)e2;
                }
                break;
            }

            /* 标准 RV32F: 仅 fmt=0 (单精度) */
            {
                uint32_t fmt = (inst >> 25) & 0x3;
                if (fmt != 0) {
                    break;
                }
            }

            switch (funct5) {
            /* ── FADD.S / FSUB.S / FMUL.S / FDIV.S ────────────────── */
            case 0x00: /* FADD.S */
                set_float_rm(lane, rm);
                lane->fpr[rd] = float32_add(lane->fpr[rs1],
                                             lane->fpr[rs2],
                                             &lane->fp_status);
                update_fcsr_flags(lane);
                break;
            case 0x01: /* FSUB.S */
                set_float_rm(lane, rm);
                lane->fpr[rd] = float32_sub(lane->fpr[rs1],
                                             lane->fpr[rs2],
                                             &lane->fp_status);
                update_fcsr_flags(lane);
                break;
            case 0x02: /* FMUL.S */
                set_float_rm(lane, rm);
                lane->fpr[rd] = float32_mul(lane->fpr[rs1],
                                             lane->fpr[rs2],
                                             &lane->fp_status);
                update_fcsr_flags(lane);
                break;
            case 0x03: /* FDIV.S */
                set_float_rm(lane, rm);
                lane->fpr[rd] = float32_div(lane->fpr[rs1],
                                             lane->fpr[rs2],
                                             &lane->fp_status);
                update_fcsr_flags(lane);
                break;

            /* ── FSQRT.S ──────────────────────────────────────────── */
            case 0x0B: /* FSQRT.S */
                set_float_rm(lane, rm);
                lane->fpr[rd] = float32_sqrt(lane->fpr[rs1],
                                              &lane->fp_status);
                update_fcsr_flags(lane);
                break;

            /* ── FSGNJ.S / FSGNJN.S / FSGNJX.S ─────────────────────── */
            case 0x04:
                {
                    uint32_t s1 = lane->fpr[rs1];
                    uint32_t s2 = lane->fpr[rs2];
                    uint32_t sign1 = s1 >> 31;
                    uint32_t sign2 = s2 >> 31;
                    uint32_t result;
                    if (rm == 0) {      /* FSGNJ.S: 取 rs2 符号 */
                        result = (s1 & 0x7FFFFFFFu) | (sign2 << 31);
                    } else if (rm == 1) { /* FSGNJN.S: 取 rs2 符号的反 */
                        result = (s1 & 0x7FFFFFFFu) | ((sign2 ^ 1) << 31);
                    } else {              /* FSGNJX.S: XOR 符号 */
                        result = (s1 & 0x7FFFFFFFu) | ((sign1 ^ sign2) << 31);
                    }
                    lane->fpr[rd] = result;
                }
                break;

            /* ── FMIN.S / FMAX.S ──────────────────────────────────── */
            case 0x05:
                {
                    float32 f1 = lane->fpr[rs1];
                    float32 f2 = lane->fpr[rs2];
                    if (rm == 0) { /* FMIN.S */
                        lane->fpr[rd] = float32_minnum(f1, f2,
                                                        &lane->fp_status);
                    } else { /* FMAX.S */
                        lane->fpr[rd] = float32_maxnum(f1, f2,
                                                        &lane->fp_status);
                    }
                    update_fcsr_flags(lane);
                }
                break;

            /* ── FEQ.S / FLT.S / FLE.S ────────────────────────────── */
            case 0x14:
                {
                    float32 f1 = lane->fpr[rs1];
                    float32 f2 = lane->fpr[rs2];
                    if (rm == 2) {      /* FEQ.S */
                        gpr[rd] = float32_eq(f1, f2, &lane->fp_status) ? 1 : 0;
                    } else if (rm == 1) { /* FLT.S */
                        gpr[rd] = float32_lt(f1, f2, &lane->fp_status) ? 1 : 0;
                    } else {              /* FLE.S */
                        gpr[rd] = float32_le(f1, f2, &lane->fp_status) ? 1 : 0;
                    }
                    update_fcsr_flags(lane);
                }
                break;

            /* ── FCVT.W.S / FCVT.WU.S ──────────────────────────────── */
            case 0x18: /* FCVT.W.S: float → signed int32 */
                set_float_rm(lane, rm);
                gpr[rd] = float32_to_int32(lane->fpr[rs1],
                                            &lane->fp_status);
                update_fcsr_flags(lane);
                break;
            case 0x19: /* FCVT.WU.S: float → unsigned int32 */
                set_float_rm(lane, rm);
                gpr[rd] = (int32_t)float32_to_uint32(lane->fpr[rs1],
                                                       &lane->fp_status);
                update_fcsr_flags(lane);
                break;

            /* ── FCVT.S.W / FCVT.S.WU ──────────────────────────────── */
            case 0x1A: /* FCVT.S.W: signed int32 → float */
                set_float_rm(lane, rm);
                lane->fpr[rd] = int32_to_float32((int32_t)gpr[rs1],
                                                   &lane->fp_status);
                update_fcsr_flags(lane);
                break;
            case 0x1B: /* FCVT.S.WU: unsigned int32 → float */
                set_float_rm(lane, rm);
                lane->fpr[rd] = uint32_to_float32(gpr[rs1],
                                                    &lane->fp_status);
                update_fcsr_flags(lane);
                break;

            /* ── FMV.X.W / FCLASS.S ────────────────────────────────── */
            case 0x1C:
                if (rm == 0) { /* FMV.X.W */
                    gpr[rd] = lane->fpr[rs1];
                } else { /* FCLASS.S */
                    float32 f = lane->fpr[rs1];
                    uint32_t bits = f;
                    uint32_t sign  = (bits >> 31) & 1;
                    uint32_t exp   = (bits >> 23) & 0xFF;
                    uint32_t mant  = bits & 0x7FFFFF;
                    uint32_t class_out = 0;
                    if (exp == 0 && mant == 0) {
                        class_out = (1u << 3) | (1u << 4); /* ±0 */
                    } else if (exp == 0 && mant != 0) {
                        class_out = (1u << 5) | (1u << 6); /* subnormal */
                    } else if (exp == 0xFF && mant == 0) {
                        class_out = (1u << 7) | (1u << 8); /* ±∞ */
                    } else if (exp == 0xFF && mant != 0) {
                        if (mant & 0x400000) {
                            class_out = (1u << 9); /* signaling NaN */
                        } else {
                            class_out = (1u << 0); /* quiet NaN */
                        }
                    } else {
                        class_out = (1u << 1) | (1u << 2); /* normal */
                    }
                    gpr[rd] = sign ? (class_out >> 1) : class_out;
                }
                break;

            /* ── FMV.W.X ──────────────────────────────────────────── */
            case 0x1E:
                if (rm == 0) {
                    lane->fpr[rd] = gpr[rs1];
                }
                break;

            default:
                qemu_log_mask(LOG_GUEST_ERROR,
                              "GPGPU: unknown fp op funct5=0x%02x"
                              " inst 0x%08x at pc 0x%08x\n",
                              funct5, inst, lane->pc);
                break;
            }
        }
        break;

    default:
        qemu_log_mask(LOG_GUEST_ERROR,
                      "GPGPU: unknown opcode 0x%02x inst 0x%08x at pc 0x%08x\n",
                      opcode, inst, lane->pc);
        return false;
    }

    /* RISC-V 规范: x0 恒为 0 */
    gpr[0] = 0;
    return true;
}

/* ============================================================================
 * gpgpu_core_init_warp — 初始化 Warp
 *
 * §12.2: 每 Lane 有独立的 x0-x31, f0-f31, pc, fcsr, mhartid
 * §12.3: mhartid = (block_id_linear << 13) | (warp_id << 5) | lane_id
 * ============================================================================ */

void gpgpu_core_init_warp(GPGPUWarp *warp, uint32_t pc,
                          uint32_t thread_id_base, const uint32_t block_id[3],
                          uint32_t num_threads,
                          uint32_t warp_id, uint32_t block_id_linear)
{
    memset(warp, 0, sizeof(*warp));

    warp->active_mask = (num_threads >= GPGPU_WARP_SIZE)
                        ? 0xFFFFFFFFu
                        : (1u << num_threads) - 1;
    warp->thread_id_base = thread_id_base;
    warp->warp_id = warp_id;
    warp->block_id[0] = block_id[0];
    warp->block_id[1] = block_id[1];
    warp->block_id[2] = block_id[2];

    for (uint32_t i = 0; i < GPGPU_WARP_SIZE; i++) {
        GPGPULane *lane = &warp->lanes[i];
        lane->pc = pc;
        lane->mhartid = MHARTID_ENCODE(block_id_linear, warp_id,
                                       thread_id_base + i);
        lane->gpr[0] = 0;
        lane->active = (i < num_threads);
        /* 初始化 softfloat 状态: 默认 RNE 舍入, 清零异常标志 */
        set_float_rounding_mode(float_round_nearest_even, &lane->fp_status);
        set_float_exception_flags(0, &lane->fp_status);
    }
}

/* ============================================================================
 * gpgpu_core_exec_warp — SIMT 锁步执行
 *
 * §12.4: 所有活跃 Lane 执行同一条指令；ebreak 或 ret(ra==0) 退出
 *        每 Warp 最大 100,000 周期
 * ============================================================================ */

int gpgpu_core_exec_warp(GPGPUState *s, GPGPUWarp *warp, uint32_t max_cycles)
{
    uint32_t cycles = 0;

    while (warp->active_mask != 0) {
        uint32_t warp_pc;
        uint32_t inst;
        bool pc_found = false;

        /* 找到第一个活跃 lane 的 PC 作为 warp 当前 PC */
        for (int li = 0; li < GPGPU_WARP_SIZE; li++) {
            if (warp->active_mask & (1u << li)) {
                warp_pc = warp->lanes[li].pc;
                pc_found = true;
                break;
            }
        }
        if (!pc_found) break;

        /* 从 VRAM 取指 */
        if (warp_pc + 3 >= s->vram_size) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "GPGPU: warp PC 0x%08x out of VRAM bounds\n",
                          warp_pc);
            return -1;
        }
        memcpy(&inst, s->vram_ptr + warp_pc, 4);

        /* 对所有活跃 lane 执行该指令 */
        for (int li = 0; li < GPGPU_WARP_SIZE; li++) {
            if (!(warp->active_mask & (1u << li))) continue;

            GPGPULane *lane = &warp->lanes[li];
            uint32_t next_pc;

            bool still_active = exec_one_inst(s, lane, inst, &next_pc);

            lane->pc = next_pc;

            if (!still_active) {
                warp->active_mask &= ~(1u << li);
            }
        }

        cycles++;
        if (cycles > max_cycles) {
            qemu_log_mask(LOG_GUEST_ERROR,
                          "GPGPU: warp exceeded max cycles %u\n", max_cycles);
            return -1;
        }
    }

    return 0;
}

/* ============================================================================
 * gpgpu_core_exec_kernel — 完整内核分发与执行
 *
 * §12.1: Grid 包含 grid_x × grid_y × grid_z 个 Block
 *        每个 Block 包含 block_x × block_y × block_z 个线程
 *        线程以 Warp (最多 32 lane) 为单位执行
 * ============================================================================ */

int gpgpu_core_exec_kernel(GPGPUState *s)
{
    GPGPUKernelParams *k = &s->kernel;
    GPGPUWarp warp;
    uint32_t block_idx[3];
    uint32_t block_id_linear = 0;
    int ret;

    /*
     * 遍历所有 Block (row-major: z 最内层? 实际上是 x 最内层)
     * 顺序对所有 Block 的所有 Warp 进行执行。
     */
    for (uint32_t bz = 0; bz < k->grid_dim[2]; bz++) {
        block_idx[2] = bz;
        for (uint32_t by = 0; by < k->grid_dim[1]; by++) {
            block_idx[1] = by;
            for (uint32_t bx = 0; bx < k->grid_dim[0]; bx++) {
                block_idx[0] = bx;

                uint32_t total_threads = k->block_dim[0] *
                                         k->block_dim[1] *
                                         k->block_dim[2];
                uint32_t warps_needed = (total_threads + GPGPU_WARP_SIZE - 1)
                                        / GPGPU_WARP_SIZE;

                for (uint32_t wid = 0; wid < warps_needed; wid++) {
                    uint32_t thread_base = wid * GPGPU_WARP_SIZE;
                    uint32_t threads_in_warp = total_threads - thread_base;
                    if (threads_in_warp > GPGPU_WARP_SIZE) {
                        threads_in_warp = GPGPU_WARP_SIZE;
                    }

                    gpgpu_core_init_warp(&warp,
                                         (uint32_t)k->kernel_addr,
                                         thread_base,
                                         block_idx,
                                         threads_in_warp,
                                         wid,
                                         block_id_linear);

                    ret = gpgpu_core_exec_warp(s, &warp, 100000);
                    if (ret != 0) {
                        return ret;
                    }
                }

                block_id_linear++;
            }
        }
    }

    return 0;
}
