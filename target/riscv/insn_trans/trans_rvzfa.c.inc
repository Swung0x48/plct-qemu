/*
 * RISC-V translation routines for the RISC-V Zfa Standard Extension.
 *
 * Copyright (c) 2020 Chih-Min Chao, chihmin.chao@sifive.com
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2 or later, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#define REQUIRE_ZFA(ctx) do { \
    if (!ctx->cfg_ptr->ext_zfa) {      \
        return false;         \
    }                         \
} while (0)

static int8_t exp_const[] = {
    0,  0, -16, -15,
   -8, -7,  -4,  -3,
   -2, -2,  -2,  -2,
   -1, -1,  -1,  -1,
    0,  0,   0,   0,
    1,  1,   1,   2,
    3,  4,   7,   8,
   15, 16,   0,   0
};

static uint64_t sig2_const[] = {
    0, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 0, 1, 2, 3,
    0, 1, 2, 3, 0, 1, 2, 0,
    0, 0, 0, 0, 0, 0, 0, 2
};

#define F16_MAX_EXP 0x1F
#define F32_MAX_EXP 0xFF
#define F64_MAX_EXP 0x3FF
//#define F128_MAX_EXP 0x3FFF

#define F16_SIG_BITS 10
#define F32_SIG_BITS 23
#define F64_SIG_BITS 52
//#define F128_SIG_BITS 48

#define FLI(value, index, flen) do { \
  uint64_t exp_bias = F##flen##_MAX_EXP >> 1; \
  uint64_t exp = exp_const[index] + exp_bias; \
  value = sig2_const[index] << (F##flen##_SIG_BITS - 2); \
  bool sign = false; \
  \
  switch (index) { \
  case 0: \
    sign = true; \
    exp = exp_bias; \
    break; \
  case 1: \
    exp = 1; \
    break; \
  case 30: \
  case 31: \
    exp = F##flen##_MAX_EXP; \
    break; \
  default: \
    break; \
  } \
  \
  value |= ((uint64_t)sign << (flen - 1)) | (exp << F##flen##_SIG_BITS); \
} while (0)

static bool trans_fli_s(DisasContext *ctx, arg_fli_s *a)
{
    uint16_t value;
    TCGv_i64 dest;

    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    FLI(value, a->rs1, 32);
    dest = tcg_constant_i64(value);
    gen_nanbox_s(dest, dest);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fli_d(DisasContext *ctx, arg_fli_d *a)
{
    uint64_t value;
    TCGv_i64 dest;

    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    FLI(value, a->rs1, 64);
    dest = tcg_constant_i64(value);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fli_h(DisasContext *ctx, arg_fli_h *a)
{
    uint16_t value;
    TCGv_i64 dest;

    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    FLI(value, a->rs1, 16);
    dest = tcg_constant_i64(value);
    gen_nanbox_h(dest, dest);
     tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fminm_s(DisasContext *ctx, arg_fminm_s *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fminm_s(dest, cpu_env, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fmaxm_s(DisasContext *ctx, arg_fmaxm_s *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fmaxm_s(dest, cpu_env, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fminm_d(DisasContext *ctx, arg_fminm_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_d(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_d(ctx, a->rs2);

    gen_helper_fminm_d(dest, cpu_env, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fmaxm_d(DisasContext *ctx, arg_fmaxm_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_d(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_d(ctx, a->rs2);

    gen_helper_fmaxm_d(dest, cpu_env, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fminm_h(DisasContext *ctx, arg_fminm_h *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fminm_h(dest, cpu_env, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fmaxm_h(DisasContext *ctx, arg_fmaxm_h *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fmaxm_h(dest, cpu_env, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fround_s(DisasContext *ctx, arg_fround_s *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);

    gen_set_rm(ctx, a->rm);
    gen_helper_fround_s(dest, cpu_env, src1);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_froundnx_s(DisasContext *ctx, arg_froundnx_s *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);

    gen_set_rm(ctx, a->rm);
    gen_helper_froundnx_s(dest, cpu_env, src1);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fround_d(DisasContext *ctx, arg_fround_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_d(ctx, a->rs1);

    gen_set_rm(ctx, a->rm);
    gen_helper_fround_d(dest, cpu_env, src1);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_froundnx_d(DisasContext *ctx, arg_froundnx_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);

    gen_set_rm(ctx, a->rm);
    gen_helper_froundnx_d(dest, cpu_env, src1);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fround_h(DisasContext *ctx, arg_fround_h *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);

    gen_set_rm(ctx, a->rm);
    gen_helper_fround_h(dest, cpu_env, src1);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_froundnx_h(DisasContext *ctx, arg_froundnx_h *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);

    gen_set_rm(ctx, a->rm);
    gen_helper_froundnx_h(dest, cpu_env, src1);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    mark_fs_dirty(ctx);
    return true;
}

static bool trans_fcvtmod_w_d(DisasContext *ctx, arg_fcvtmod_w_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_d(ctx, a->rs1);

    gen_set_rm(ctx, 1);
    gen_helper_fcvtmod_w_d(dest, cpu_env, src1);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fmvh_x_d(DisasContext *ctx, arg_fmvh_x_d *a)
{
    REQUIRE_32BIT(ctx);
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_d(ctx, a->rs1);

    tcg_gen_extr_i64_tl(dest, dest, src1);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fmvp_d_x(DisasContext *ctx, arg_fmvp_d_x *a)
{
    REQUIRE_32BIT(ctx);
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv_i64 dest = dest_fpr(ctx, a->rd);
    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv src2 = get_gpr(ctx, a->rs2, EXT_NONE);

    tcg_gen_concat_tl_i64(dest, src1, src2);
    tcg_gen_mov_i64(cpu_fpr[a->rd], dest);
    return true;
}

static bool trans_fleq_s(DisasContext *ctx, arg_fleq_s *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fleq_s(dest, cpu_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fltq_s(DisasContext *ctx, arg_fltq_s *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fltq_s(dest, cpu_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fleq_h(DisasContext *ctx, arg_fleq_h *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fleq_h(dest, cpu_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fltq_h(DisasContext *ctx, arg_fltq_h *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_ZFH(ctx);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fltq_h(dest, cpu_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fleq_d(DisasContext *ctx, arg_fleq_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fleq_d(dest, cpu_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool trans_fltq_d(DisasContext *ctx, arg_fltq_d *a)
{
    REQUIRE_FPU;
    REQUIRE_ZFA(ctx);
    REQUIRE_EXT(ctx, RVD);

    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv_i64 src1 = get_fpr_hs(ctx, a->rs1);
    TCGv_i64 src2 = get_fpr_hs(ctx, a->rs2);

    gen_helper_fltq_d(dest, cpu_env, src1, src2);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}
