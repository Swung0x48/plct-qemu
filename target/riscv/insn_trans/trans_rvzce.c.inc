/*
 * RISC-V translation routines for the Zc[b,mp,mt] Standard Extension.
 *
 * Copyright (c) 2021-2022 PLCT Lab
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

#define REQUIRE_ZCB(ctx) do {   \
    if (!ctx->cfg_ptr->ext_zcb) \
        return false;           \
} while (0)

#define REQUIRE_ZCMP(ctx) do {   \
    if (!ctx->cfg_ptr->ext_zcmp) \
        return false;            \
} while (0)

#define REQUIRE_ZCMT(ctx) do {                       \
    if (!ctx->cfg_ptr->ext_zcmt) {                   \
        return false;                                \
    }                                                \
    if (!smstateen_check(ctx, 0, SMSTATEEN0_JVT)) {  \
        return false;                                \
    }                                                \
} while (0)

static bool trans_c_zext_b(DisasContext *ctx, arg_c_zext_b *a)
{
    REQUIRE_ZCB(ctx);
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext8u_tl);
}

static bool trans_c_zext_h(DisasContext *ctx, arg_c_zext_h *a)
{
    REQUIRE_ZCB(ctx);
    REQUIRE_ZBB(ctx);
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext16u_tl);
}

static bool trans_c_sext_b(DisasContext *ctx, arg_c_sext_b *a)
{
    REQUIRE_ZCB(ctx);
    REQUIRE_ZBB(ctx);
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext8s_tl);
}

static bool trans_c_sext_h(DisasContext *ctx, arg_c_sext_h *a)
{
    REQUIRE_ZCB(ctx);
    REQUIRE_ZBB(ctx);
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext16s_tl);
}

static bool trans_c_zext_w(DisasContext *ctx, arg_c_zext_w *a)
{
    REQUIRE_64BIT(ctx);
    REQUIRE_ZCB(ctx);
    REQUIRE_ZBA(ctx);
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_ext32u_tl);
}

static bool trans_c_not(DisasContext *ctx, arg_c_not *a)
{
    REQUIRE_ZCB(ctx);
    return gen_unary(ctx, a, EXT_NONE, tcg_gen_not_tl);
}

static bool trans_c_mul(DisasContext *ctx, arg_c_mul *a)
{
    REQUIRE_ZCB(ctx);
    REQUIRE_M_OR_ZMMUL(ctx);
    return gen_arith(ctx, a, EXT_NONE, tcg_gen_mul_tl, NULL);
}

static bool trans_c_lbu(DisasContext *ctx, arg_c_lbu *a)
{
    REQUIRE_ZCB(ctx);
    return gen_load(ctx, a, MO_UB);
}

static bool trans_c_lhu(DisasContext *ctx, arg_c_lhu *a)
{
    REQUIRE_ZCB(ctx);
    return gen_load(ctx, a, MO_UW);
}

static bool trans_c_lh(DisasContext *ctx, arg_c_lh *a)
{
    REQUIRE_ZCB(ctx);
    return gen_load(ctx, a, MO_SW);
}

static bool trans_c_sb(DisasContext *ctx, arg_c_sb *a)
{
    REQUIRE_ZCB(ctx);
    return gen_store(ctx, a, MO_UB);
}

static bool trans_c_sh(DisasContext *ctx, arg_c_sh *a)
{
    REQUIRE_ZCB(ctx);
    return gen_store(ctx, a, MO_UW);
}

static bool gen_zcmp_check(DisasContext *ctx, arg_zcmp *a)
{
    /* rlist 0 to 3 are reserved for future EABI variant */
    if (a->zcmp_rlist < 4) {
        return false;
    }

    /* rlist <= 6 when RV32E/RV64E */
    if (ctx->cfg_ptr->ext_e && a->zcmp_rlist > 6) {
        return false;
    }

    return true;
}

#define X_S0    8
#define X_S1    9
#define X_Sn    16

static inline void update_push_pop_list(target_ulong rlist, bool *xreg_list)
{
    switch (rlist) {
    case 15:
        xreg_list[X_Sn + 11] = true;
        xreg_list[X_Sn + 10] = true;
        /* FALL THROUGH */
    case 14:
        xreg_list[X_Sn + 9] = true;
        /* FALL THROUGH */
    case 13:
        xreg_list[X_Sn + 8] = true;
        /* FALL THROUGH */
    case 12:
        xreg_list[X_Sn + 7] = true;
        /* FALL THROUGH */
    case 11:
        xreg_list[X_Sn + 6] = true;
        /* FALL THROUGH */
    case 10:
        xreg_list[X_Sn + 5] = true;
        /* FALL THROUGH */
    case 9:
        xreg_list[X_Sn + 4] = true;
        /* FALL THROUGH */
    case 8:
        xreg_list[X_Sn + 3] = true;
        /* FALL THROUGH */
    case 7:
        xreg_list[X_Sn + 2] = true;
        /* FALL THROUGH */
    case 6:
        xreg_list[X_S1] = true;
        /* FALL THROUGH */
    case 5:
        xreg_list[X_S0] = true;
        /* FALL THROUGH */
    case 4:
        xreg_list[xRA] = true;
        break;
    }
}

static inline target_ulong caculate_stack_adj(int bytes, target_ulong rlist,
                                              target_ulong spimm)
{
    target_ulong stack_adj_base = 0;
    switch (rlist) {
    case 15:
        stack_adj_base = bytes == 4 ? 64 : 112;
        break;
    case 14:
        stack_adj_base = bytes == 4 ? 48 : 96;
        break;
    case 13:
    case 12:
        stack_adj_base = bytes == 4 ? 48 : 80;
        break;
    case 11:
    case 10:
        stack_adj_base = bytes == 4 ? 32 : 64;
        break;
    case 9:
    case 8:
        stack_adj_base = bytes == 4 ? 32 : 48;
        break;
    case 7:
    case 6:
        stack_adj_base = bytes == 4 ? 16 : 32;
        break;
    case 5:
    case 4:
        stack_adj_base = 16;
        break;
    }

    return stack_adj_base + spimm;
}

static bool gen_pop(DisasContext *ctx, arg_zcmp *a, bool ret, bool ret_val)
{
    REQUIRE_ZCMP(ctx);

    if (!gen_zcmp_check(ctx, a)) {
        return false;
    }

    bool xreg_list[32] = {false};
    int bytes = get_ol(ctx) == MXL_RV32 ? 4 : 8;
    target_ulong stack_adj = caculate_stack_adj(bytes, a->zcmp_rlist,
                                                a->zcmp_spimm);
    TCGv sp = dest_gpr(ctx, xSP);
    TCGv addr = tcg_temp_new();
    int i;

    update_push_pop_list(a->zcmp_rlist, xreg_list);
    tcg_gen_addi_tl(addr, sp, stack_adj - bytes);

    for (i = X_Sn + 11; i >= 0; i--) {
        if (xreg_list[i]) {
            TCGv dest = dest_gpr(ctx, i);
            switch (bytes) {
            case 4:
                tcg_gen_qemu_ld_tl(dest, addr, ctx->mem_idx, MO_32);
                break;
            case 8:
                tcg_gen_qemu_ld_tl(dest, addr, ctx->mem_idx, MO_64);
                break;
            default:
                break;
            }
            gen_set_gpr(ctx, i, dest);
            tcg_gen_subi_tl(addr, addr, bytes);
        }
    }

    tcg_gen_addi_tl(sp, sp, stack_adj);
    gen_set_gpr(ctx, xSP, sp);

    if (ret_val) {
        gen_set_gpr(ctx, xA0, ctx->zero);
    }

    if (ret) {
        TCGv ret_addr = get_gpr(ctx, xRA, EXT_NONE);
        gen_set_pc(ctx, ret_addr);
        tcg_gen_lookup_and_goto_ptr();
        ctx->base.is_jmp = DISAS_NORETURN;
    }

    tcg_temp_free(addr);
    return true;
}

static bool trans_cm_push(DisasContext *ctx, arg_cm_push *a)
{
    REQUIRE_ZCMP(ctx);

    if (!gen_zcmp_check(ctx, a)) {
        return false;
    }

    bool xreg_list[32] = {false};
    int bytes = get_ol(ctx) == MXL_RV32 ? 4 : 8;
    target_ulong stack_adj = caculate_stack_adj(bytes, a->zcmp_rlist,
                                                a->zcmp_spimm);
    TCGv sp = dest_gpr(ctx, xSP);
    TCGv addr = tcg_temp_new();
    int i;

    update_push_pop_list(a->zcmp_rlist, xreg_list);
    tcg_gen_subi_tl(addr, sp, bytes);

    for (i = X_Sn + 11; i >= 0; i--) {
        if (xreg_list[i]) {
            TCGv val = get_gpr(ctx, i, EXT_NONE);
            switch (bytes) {
            case 4:
                tcg_gen_qemu_st_tl(val, addr, ctx->mem_idx, MO_32);
                break;
            case 8:
                tcg_gen_qemu_st_tl(val, addr, ctx->mem_idx, MO_64);
                break;
            default:
                break;
            }
            tcg_gen_subi_tl(addr, addr, bytes);
        }
    }

    tcg_gen_subi_tl(sp, sp, stack_adj);
    gen_set_gpr(ctx, xSP, sp);
    return true;
}

static bool trans_cm_pop(DisasContext *ctx, arg_cm_pop *a)
{
    return gen_pop(ctx, a, false, false);
}

static bool trans_cm_popret(DisasContext *ctx, arg_cm_popret *a)
{
    return gen_pop(ctx, a, true, false);
}

static bool trans_cm_popretz(DisasContext *ctx, arg_cm_popret *a)
{
    return gen_pop(ctx, a, true, true);
}

static bool trans_cm_mva01s(DisasContext *ctx, arg_cm_mva01s *a)
{
    REQUIRE_ZCMP(ctx);

    TCGv src1 = get_gpr(ctx, a->rs1, EXT_NONE);
    TCGv src2 = get_gpr(ctx, a->rs2, EXT_NONE);

    gen_set_gpr(ctx, xA0, src1);
    gen_set_gpr(ctx, xA1, src2);

    return true;
}

static bool trans_cm_mvsa01(DisasContext *ctx, arg_cm_mvsa01 *a)
{
    REQUIRE_ZCMP(ctx);

    TCGv a0 = get_gpr(ctx, xA0, EXT_NONE);
    TCGv a1 = get_gpr(ctx, xA1, EXT_NONE);

    gen_set_gpr(ctx, a->rs1, a0);
    gen_set_gpr(ctx, a->rs2, a1);

    return true;
}

static bool trans_cm_jalt(DisasContext *ctx, arg_cm_jalt *a)
{
    REQUIRE_ZCMT(ctx);

    TCGv index = tcg_const_tl(a->zcmt_index);
    TCGv next_pc = tcg_const_tl(ctx->pc_succ_insn);

    gen_helper_cm_jalt(cpu_pc, cpu_env, index, next_pc);

    tcg_gen_lookup_and_goto_ptr();

    ctx->base.is_jmp = DISAS_NORETURN;

    tcg_temp_free(index);
    tcg_temp_free(next_pc);
    return true;
}
