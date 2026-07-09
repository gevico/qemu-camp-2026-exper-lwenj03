/*
 * RISC-V translation routines for the Xg233ai custom accelerator extension.
 *
 * SPDX-License-Identifier: GPL-2.0-or-later
 */

#define REQUIRE_XG233AI(ctx) do {              \
    if (!ctx->cfg_ptr->ext_xg233ai) {          \
        return false;                          \
    }                                          \
} while (0)

/* ── Category A: memory-to-memory ─────────────────────────────────── */

static bool trans_dma(DisasContext *ctx, arg_dma *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv dst_ptr = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src_ptr = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv grain   = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_dma(tcg_env, dst_ptr, src_ptr, grain);
    return true;
}

static bool trans_gemm(DisasContext *ctx, arg_gemm *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv c_ptr = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv a_ptr = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv b_ptr = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_gemm(tcg_env, c_ptr, a_ptr, b_ptr);
    return true;
}

static bool trans_sort(DisasContext *ctx, arg_sort *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv k   = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv arr = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv n   = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_sort(tcg_env, k, arr, n);
    return true;
}

static bool trans_vadd(DisasContext *ctx, arg_vadd *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv dst  = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv src2 = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_vadd(tcg_env, dst, src1, src2);
    return true;
}

static bool trans_crush(DisasContext *ctx, arg_crush *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv n   = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_crush(tcg_env, dst, src, n);
    return true;
}

static bool trans_expand(DisasContext *ctx, arg_expand *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv n   = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_expand(tcg_env, dst, src, n);
    return true;
}

static bool trans_vrelu(DisasContext *ctx, arg_vrelu *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv dst = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv n   = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_vrelu(tcg_env, dst, src, n);
    return true;
}

static bool trans_vscale(DisasContext *ctx, arg_vscale *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv dst   = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv src   = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv scale = get_gpr(ctx, a->rs2, EXT_NONE);
    gen_helper_xg233ai_vscale(tcg_env, dst, src, scale);
    return true;
}

/* ── Category B: scalar return to rd register ─────────────────────── */

static bool trans_vdot(DisasContext *ctx, arg_vdot *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv src2 = get_gpr(ctx, a->rs2, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rd);
    gen_helper_xg233ai_vdot(dest, tcg_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_vmax(DisasContext *ctx, arg_vmax *a)
{
    REQUIRE_XG233AI(ctx);
    TCGv src = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv n   = get_gpr(ctx, a->rs2, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rd);
    gen_helper_xg233ai_vmax(dest, tcg_env, src, n);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}
