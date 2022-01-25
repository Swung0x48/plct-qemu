/*
 * RISC-V translation routines for the RV64M Standard Extension.
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2018 Peer Adelt, peer.adelt@hni.uni-paderborn.de
 *                    Bastian Koppelmann, kbastian@mail.uni-paderborn.de
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

#define REQUIRE_ZCA do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zca) \
        return false;                     \
} while (0)

#define REQUIRE_ZCB do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcb) \
        return false;                     \
} while (0)

#define REQUIRE_ZCF do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcf) \
        return false;                     \
} while (0)

#define REQUIRE_ZCMB do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcmb) \
        return false;                      \
} while (0)

#define REQUIRE_ZCMP do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcmp) \
        return false;                      \
} while (0)

#define REQUIRE_ZCMPE do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcmpe) \
        return false;                       \
} while (0)

#define REQUIRE_ZCMT do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcmt) \
        return false;                      \
} while (0)

static bool trans_c_zext_b(DisasContext *ctx, arg_c_zext_b *a)
{
    REQUIRE_ZCB;
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext8u_tl);
}

static bool trans_c_zext_h(DisasContext *ctx, arg_c_zext_h *a)
{
    REQUIRE_ZCB;
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext16u_tl);
}

static bool trans_c_sext_b(DisasContext *ctx, arg_c_sext_b *a)
{
    REQUIRE_ZCB;
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext8s_tl);
}

static bool trans_c_sext_h(DisasContext *ctx, arg_c_sext_h *a)
{
    REQUIRE_ZCB;
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext16s_tl);
}

// RV64 Only
static bool trans_c_zext_w(DisasContext *ctx, arg_c_zext_w *a)
{
    REQUIRE_64BIT(ctx);
    REQUIRE_ZCB;
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext32u_tl);
}

static bool trans_c_not(DisasContext *ctx, arg_c_not *a)
{
    REQUIRE_ZCB;
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_not_tl);
}

static bool trans_c_mul(DisasContext *ctx, arg_c_mul *a)
{
    REQUIRE_ZCB;
    REQUIRE_EXT(ctx, RVM);
    return gen_arith(ctx, a, EXT_NONE, tcg_gen_mul_tl, NULL);
}

static bool gen_zce_load(DisasContext *ctx, arg_i *a, MemOp memop)
{
    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv t0 = tcg_temp_new();

    tcg_gen_addi_tl(t0, src1, a->imm);

    tcg_gen_qemu_ld_tl(dest, t0, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rd, dest);

    tcg_temp_free(t0);
    return true;
}

static bool trans_c_lbu(DisasContext *ctx, arg_c_lbu *a)
{
    REQUIRE_ZCB;
    MemOp memop = MO_UB; // 8u-bit
    return gen_zce_load(ctx, a, memop);
}

static bool trans_c_lhu(DisasContext *ctx, arg_c_lhu *a)
{
    REQUIRE_ZCB;
    MemOp memop = MO_UW; // 16u-bit
    return gen_zce_load(ctx, a, memop);
}

static bool trans_c_lh(DisasContext *ctx, arg_c_lh *a)
{
    REQUIRE_ZCB;
    MemOp memop = MO_SW; // 16s-bit
    return gen_zce_load(ctx, a, memop);
}

static bool gen_zce_store(DisasContext *ctx, arg_i *a, MemOp memop)
{
    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv src2 = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv t0 = tcg_temp_new();

    tcg_gen_addi_tl(t0, src1, a->imm);
    tcg_gen_qemu_st_tl(src2, t0, ctx->mem_idx, memop);

    tcg_temp_free(t0);
    return true;
}

static bool trans_c_sb(DisasContext *ctx, arg_c_sb *a)
{
    REQUIRE_ZCB;
    MemOp memop = MO_UB; // 8u-bit
    return gen_zce_store(ctx, a, memop);
}

static bool trans_c_sh(DisasContext *ctx, arg_c_sh *a)
{
    REQUIRE_ZCB;
    MemOp memop = MO_UW; // 16u-bit
    return gen_zce_store(ctx, a, memop);
}

static bool trans_cm_lbu(DisasContext *ctx, arg_cm_lbu *a)
{
    REQUIRE_ZCMB;
    MemOp memop = MO_UB; // 8u-bit
    return gen_zce_load(ctx, a, memop);
}

static bool trans_cm_lhu(DisasContext *ctx, arg_cm_lhu *a)
{
    REQUIRE_ZCMB;
    MemOp memop = MO_UW; // 16u-bit
    return gen_zce_load(ctx, a, memop);
}

static bool trans_cm_lb(DisasContext *ctx, arg_cm_lb *a)
{
    REQUIRE_ZCMB;
    MemOp memop = MO_SB; // 8u-bit
    return gen_zce_load(ctx, a, memop);
}

static bool trans_cm_lh(DisasContext *ctx, arg_cm_lh *a)
{
    REQUIRE_ZCMB;
    MemOp memop = MO_SW; // 16s-bit
    return gen_zce_load(ctx, a, memop);
}

static bool trans_cm_sb(DisasContext *ctx, arg_cm_sb *a)
{
    REQUIRE_ZCMB;
    MemOp memop = MO_UB; // 8u-bit
    return gen_zce_store(ctx, a, memop);;
}

static bool trans_cm_sh(DisasContext *ctx, arg_cm_sh *a)
{
    REQUIRE_ZCMB;
    MemOp memop = MO_UW; // 16u-bit
    return gen_zce_store(ctx, a, memop);;
}

static bool gen_zcmp(DisasContext *ctx, arg_zcmp *a, void (*func)(TCGv_env, TCGv, TCGv, TCGv))
{
    REQUIRE_ZCMP;

    /* rlist 0 to 3 are reserved for a future EABI variant called cm.popret.e */
    if (a->zcmp_rlist >= 4) {
        return false;
    }

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv rlist = tcg_const_tl(a->zcmp_rlist);
    TCGv spimm = tcg_const_tl(a->zcmp_spimm);

    func(cpu_env, sp, spimm, rlist);

    tcg_temp_free(spimm);
    tcg_temp_free(rlist);
    return true;
}

static bool trans_cm_push(DisasContext *ctx, arg_cm_push *a)
{
    return gen_zcmp(ctx, a, gen_helper_cm_push);
}

static bool trans_cm_pop(DisasContext *ctx, arg_cm_pop *a)
{
    return gen_zcmp(ctx, a, gen_helper_cm_pop);
}

static bool trans_cm_popret(DisasContext *ctx, arg_cm_popret *a)
{
    return gen_zcmp(ctx, a, gen_helper_cm_popret);
}

static bool trans_cm_popretz(DisasContext *ctx, arg_cm_popret *a)
{
    return gen_zcmp(ctx, a, gen_helper_cm_popretz);
}

static bool trans_cm_mva01s(DisasContext *ctx, arg_cm_mva01s *a)
{
    REQUIRE_ZCMP;

    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv src2 = get_gpr(ctx, a->rs2, EXT_NONE);

    gen_set_gpr(ctx, xA0, src1);
    gen_set_gpr(ctx, xA1, src2);

    return true;
}

static bool trans_cm_mvsa01(DisasContext *ctx, arg_cm_mvsa01 *a)
{
    REQUIRE_ZCMP;

    TCGv a0 = get_gpr(ctx, xA0, EXT_NONE);
    TCGv a1 = get_gpr(ctx, xA1, EXT_NONE);

    gen_set_gpr(ctx, a->rd, a0);
    gen_set_gpr(ctx, a->rs2, a1);

    return true;
}

static bool gen_zcmt(DisasContext *ctx, arg_zcmt *a)
{
    REQUIRE_ZCMT;

    TCGv index = tcg_const_tl(a->zcmt_index);
    TCGv next_pc = tcg_const_tl(ctx->pc_succ_insn);

    gen_helper_cm_jt_all(cpu_pc, cpu_env, index, next_pc);

    tcg_gen_lookup_and_goto_ptr();

    ctx->base.is_jmp = DISAS_NORETURN;

    tcg_temp_free(index);
    tcg_temp_free(next_pc);
    return true;
}

static bool trans_cm_jt(DisasContext *ctx, arg_cm_jt *a)
{
    if (a->zcmt_index >= 64) {
        return false;
    }

    return gen_zcmt(ctx, a);
}

static bool trans_cm_jalt(DisasContext *ctx, arg_cm_jalt *a)
{
    if (a->zcmt_index < 64) {
        return false;
    }

    return gen_zcmt(ctx, a);
}
