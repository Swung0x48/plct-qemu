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

#define REQUIRE_ZCEA do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcea) \
        return false;                       \
} while (0)

#define REQUIRE_ZCEB do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zceb) \
        return false;                       \
} while (0)

#define REQUIRE_ZCEE do {\
    if (!RISCV_CPU(ctx->cs)->cfg.ext_zcee) \
        return false;                       \
} while (0)

static bool gen_zce_unary(DisasContext *ctx, arg_zcea_unary *a, void (*func)(TCGv, TCGv))
{
    TCGv source = get_gpr(ctx, a->rs1_3, EXT_NONE);

    (*func)(source, source);
    gen_set_gpr(ctx, a->rs1_3, source);
    tcg_temp_free(source);
    return true;
}

static bool trans_c_zext_b(DisasContext *ctx, arg_c_zext_b *a)
{
    REQUIRE_ZCEE;
    return gen_zce_unary(ctx, a, tcg_gen_ext8u_tl);
}

static bool trans_c_zext_h(DisasContext *ctx, arg_c_zext_h *a)
{
    REQUIRE_ZCEE;
    return gen_zce_unary(ctx, a, tcg_gen_ext16u_tl);
}

static bool trans_c_sext_b(DisasContext *ctx, arg_c_sext_b *a)
{
    REQUIRE_ZCEE;
    return gen_zce_unary(ctx, a, tcg_gen_ext8s_tl);
}

static bool trans_c_sext_h(DisasContext *ctx, arg_c_sext_h *a)
{
    REQUIRE_ZCEE;
    return gen_zce_unary(ctx, a, tcg_gen_ext16s_tl);
}

static bool trans_c_not(DisasContext *ctx, arg_c_not *a)
{
    REQUIRE_ZCEA;
    return gen_zce_unary(ctx, a, tcg_gen_not_tl);
}

static bool trans_c_neg(DisasContext *ctx, arg_c_neg *a)
{
    REQUIRE_ZCEA;
    return gen_zce_unary(ctx, a, tcg_gen_neg_tl);
}

static bool trans_c_mul(DisasContext *ctx, arg_c_mul *a)
{
    REQUIRE_ZCEA;
    REQUIRE_EXT(ctx, RVM);
    TCGv source1 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv source2 = get_gpr(ctx, a->rs2_3, EXT_NONE);
    tcg_gen_mul_tl(source1, source1, source2);
    gen_set_gpr(ctx, a->rs1_3, source1);
    return true;
}

static bool trans_c_mva01s07(DisasContext *ctx, arg_c_mva01s07 *a)
{
    REQUIRE_ZCEA;
    TCGv source1 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv source2 = get_gpr(ctx, a->rs2_3, EXT_NONE);
    gen_set_gpr(ctx, xA0, source1);
    gen_set_gpr(ctx, xA1, source2);
    return true;
}

static bool trans_muli(DisasContext *ctx, arg_muli *a)
{
    REQUIRE_ZCEA;
    REQUIRE_EXT(ctx, RVM);
    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv source1 = get_gpr(ctx, a->rs1, EXT_NONE);
    tcg_gen_muli_tl(dest, source1, a->imm_i);
    gen_set_gpr(ctx, a->rd, dest);
    return true;
}

static bool gen_branch_zce(DisasContext *ctx, arg_zcea_b *a, TCGCond cond)
{
    REQUIRE_ZCEA;
    TCGLabel *l = gen_new_label();
    TCGv source = get_gpr(ctx, a->rs2, EXT_NONE);
    TCGv uimm = tcg_const_tl(a->zce_uimm);

    tcg_gen_brcond_tl(cond, source, uimm, l);
    gen_goto_tb(ctx, 1, ctx->pc_succ_insn);
    gen_set_label(l); /* branch taken */

    if ((ctx->base.pc_next + a->zce_offset) & 0x3)
    {
        /* misaligned */
        gen_exception_inst_addr_mis(ctx);
    }
    else
    {
        gen_goto_tb(ctx, 0, ctx->base.pc_next + a->zce_offset);
    }
    ctx->base.is_jmp = DISAS_NORETURN;

    tcg_temp_free(uimm);

    return true;
}

static bool trans_beqi(DisasContext *ctx, arg_beqi *a)
{
    return gen_branch_zce(ctx, a, TCG_COND_EQ);
}

static bool trans_bnei(DisasContext *ctx, arg_bnei *a)
{
    return gen_branch_zce(ctx, a, TCG_COND_NE);
}

static bool trans_decbnez(DisasContext *ctx, arg_decbnez *a)
{
    REQUIRE_ZCEB;
    TCGv source = get_gpr(ctx, a->rd, EXT_NONE);
    TCGv scale = tcg_const_tl(1 << a->zce_scale);
    tcg_gen_sub_tl(source, source, scale);

    // branch
    TCGLabel *l = gen_new_label();
    TCGCond cond = TCG_COND_NE;
    tcg_gen_brcond_tl(cond, source, 0, l);
    gen_goto_tb(ctx, 1, ctx->pc_succ_insn);
    gen_set_label(l); /* branch taken */

    if ((ctx->base.pc_next + a->zce_b_imm_dec) & 0x3)
    {
        /* misaligned */
        gen_exception_inst_addr_mis(ctx);
    }
    else
    {
        gen_goto_tb(ctx, 0, ctx->base.pc_next + a->zce_b_imm_dec);
    }
    ctx->base.is_jmp = DISAS_NORETURN;

    tcg_temp_free(scale);

    return true;
}

static bool trans_lwgp(DisasContext *ctx, arg_lwgp *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, xGP, EXT_NONE);
    TCGv t1 = dest_gpr(ctx, a->rd);

    tcg_gen_addi_tl(t0, t0, a->zce_b_imm_lw);

    MemOp memop = MO_TESL; // 32s-bit
    tcg_gen_qemu_ld_tl(t1, t0, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rd, t1);
    return true;
}

static bool trans_swgp(DisasContext *ctx, arg_swgp *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, xGP, EXT_NONE);
    TCGv dat = get_gpr(ctx, a->rs2, EXT_NONE);

    tcg_gen_addi_tl(t0, t0, a->zce_b_imm_sw);

    MemOp memop = MO_TESL; // 32s-bit
    tcg_gen_qemu_st_tl(dat, t0, ctx->mem_idx, memop);

    return true;
}

static bool trans_c_tblj_all(DisasContext *ctx, arg_c_tblj_all *a)
{
    REQUIRE_ZCEA;
//    TCGLabel *misaligned = NULL;
    TCGv t0 = tcg_temp_new();
    TCGv index = tcg_const_tl(a->zce_table_jump_index);
    TCGv csr_store = tcg_temp_new();
    TCGv next_pc = tcg_const_tl(ctx->pc_succ_insn);

    gen_helper_c_tblj_all(cpu_pc, cpu_env, csr_store, index, next_pc);

//    if (!has_ext(ctx, RVC)) {
//        misaligned = gen_new_label();
//        tcg_gen_andi_tl(t0, cpu_pc, 0x2);
//        tcg_gen_brcondi_tl(TCG_COND_NE, t0, 0x0, misaligned);
//    }
    tcg_gen_lookup_and_goto_ptr();

//    if (misaligned) {
//        gen_set_label(misaligned);
//        gen_exception_inst_addr_mis(ctx);
//    }
    ctx->base.is_jmp = DISAS_NORETURN;
    tcg_temp_free(index);
    tcg_temp_free(csr_store);
    tcg_temp_free(next_pc);
    tcg_temp_free(t0);
    return true;
}

static bool trans_c_decbnez(DisasContext *ctx, arg_c_decbnez *a)
{
    REQUIRE_ZCEB;
    TCGv source = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv scale = tcg_const_tl(1 << a->zce_scale);
    tcg_gen_sub_tl(source, source, scale);

    // branch
    TCGLabel *l = gen_new_label();
    TCGCond cond = TCG_COND_NE;
    tcg_gen_brcond_tl(cond, source, 0, l);
    gen_goto_tb(ctx, 1, ctx->pc_succ_insn);
    gen_set_label(l); /* branch taken */

//    if ((ctx->base.pc_next - a->zce_nzuimm) & 0x3)
//    {
//        /* misaligned */
//        gen_exception_inst_addr_mis(ctx);
//    }
//    else
//    {
        gen_goto_tb(ctx, 0, ctx->base.pc_next - a->zce_nzuimm);
//    }
    ctx->base.is_jmp = DISAS_NORETURN;

    tcg_temp_free(scale);
    return true;
}

static bool trans_c_lbu(DisasContext *ctx, arg_c_lbu *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rs2_3);
    TCGv t1 = tcg_temp_new();

    tcg_gen_addi_tl(t1, t0, a->zce_b_uimm);
    MemOp memop = MO_UB; // 8u-bit
    tcg_gen_qemu_ld_tl(dest, t1, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rs2_3, dest);
    tcg_temp_free(t1);
    return true;
}

static bool trans_c_lhu(DisasContext *ctx, arg_c_lhu *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rs2_3);
    TCGv t1 = tcg_temp_new();
    tcg_gen_addi_tl(t1, t0, a->zce_h_uimm);

    MemOp memop = MO_UW; // 16u-bit
    tcg_gen_qemu_ld_tl(dest, t1, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rs2_3, dest);
    tcg_temp_free(t1);
    return true;
}

static bool trans_c_lb(DisasContext *ctx, arg_c_lb *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rs2_3);
    TCGv t1 = tcg_temp_new();
    tcg_gen_addi_tl(t1, t0, a->zce_b_uimm);

    MemOp memop = MO_SB; // 8s-bit
    tcg_gen_qemu_ld_tl(dest, t1, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rs2_3, dest);
    tcg_temp_free(t1);
    return true;
}

static bool trans_c_lh(DisasContext *ctx, arg_c_lh *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rs2_3);
    TCGv t1 = tcg_temp_new();
    tcg_gen_addi_tl(t1, t0, a->zce_h_uimm);

    MemOp memop = MO_SW; // 16s-bit
    tcg_gen_qemu_ld_tl(dest, t1, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rs2_3, dest);
    tcg_temp_free(t1);
    return true;
}

static bool trans_c_sb(DisasContext *ctx, arg_c_sb *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv dat = get_gpr(ctx, a->rs2_3, EXT_NONE);

    tcg_gen_addi_tl(t0, t0, a->zce_b_uimm);
    MemOp memop = MO_UB; // 8u-bit
    tcg_gen_qemu_st_tl(dat, t0, ctx->mem_idx, memop);

    return true;
}

static bool trans_c_sh(DisasContext *ctx, arg_c_sh *a)
{
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, a->rs1_3, EXT_NONE);
    TCGv dat = get_gpr(ctx, a->rs2_3, EXT_NONE);

    tcg_gen_addi_tl(t0, t0, a->zce_h_uimm);

    MemOp memop = MO_UW; // 16u-bit
    tcg_gen_qemu_st_tl(dat, t0, ctx->mem_idx, memop);
    tcg_temp_free(t0);
    tcg_temp_free(dat);
    return true;
}

static bool trans_c_popret(DisasContext *ctx, arg_c_popret *a)
{
    REQUIRE_ZCEA;

    if (a->zce_spimm0_5 > (5 << 4)) {
        return false;
    }

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv ret0_1 = tcg_const_tl(a->zce_ret0_1);
    TCGv rlist3 = tcg_const_tl(a->zce_rlist3);
    TCGv spimm0_5 = tcg_const_tl(a->zce_spimm0_5);

    gen_helper_c_popret(cpu_env, sp, spimm0_5, rlist3, ret0_1);
    tcg_temp_free(spimm0_5);
    tcg_temp_free(ret0_1);
    tcg_temp_free(rlist3);
    return true;
}

static bool trans_c_popret_e(DisasContext *ctx, arg_c_popret_e *a)
{
    REQUIRE_ZCEA;

    if (a->zce_spimm0_5 > (5 << 4)) {
        return false;
    }

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv ret0_2 = tcg_const_tl(a->zce_ret0_2);
    TCGv rlist2 = tcg_const_tl(a->zce_rlist2);
    TCGv spimm0_5 = tcg_const_tl(a->zce_spimm0_5);

    gen_helper_c_popret_e(cpu_env, sp, spimm0_5, rlist2, ret0_2);

    tcg_temp_free(spimm0_5);
    tcg_temp_free(ret0_2);
    tcg_temp_free(rlist2);
    return true;
}

static bool trans_c_pop(DisasContext *ctx, arg_c_pop *a)
{
    REQUIRE_ZCEA;

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv spimm = tcg_const_tl(a->zce_spimm);
    TCGv rlist3 = tcg_const_tl(a->zce_rlist3);

    gen_helper_c_pop(cpu_env, sp, spimm, rlist3);
    tcg_temp_free(spimm);
    tcg_temp_free(rlist3);
    return true;
}

static bool trans_c_pop_e(DisasContext *ctx, arg_c_pop_e *a)
{
    REQUIRE_ZCEA;

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv spimm = tcg_const_tl(a->zce_spimm);
    TCGv rlist2 = tcg_const_tl(a->zce_rlist2);

    gen_helper_c_pop_e(cpu_env, sp, spimm, rlist2);

    tcg_temp_free(spimm);
    tcg_temp_free(rlist2);
    return true;
}

static bool trans_c_push(DisasContext *ctx, arg_c_push *a)
{
    REQUIRE_ZCEA;

    if (a->zce_spimm0_5 > (5 << 4)) {
        return false;
    }

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv rlist = tcg_const_tl(a->zce_rlist3);
    TCGv spimm = tcg_const_tl(a->zce_spimm0_5);

    gen_helper_c_push(cpu_env, sp, spimm, rlist);

    tcg_temp_free(rlist);
    tcg_temp_free(spimm);
    return true;
}

static bool trans_c_push_e(DisasContext *ctx, arg_c_push_e *a)
{
    REQUIRE_ZCEA;

    if (a->zce_spimm_pe > (5 << 4)) {
        return false;
    }

    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);

    TCGv rlist = tcg_const_tl(a->zce_rlist2);
    TCGv spimm = tcg_const_tl(a->zce_spimm_pe);

    gen_helper_c_push_e(cpu_env, sp, spimm, rlist);

    tcg_temp_free(rlist);
    tcg_temp_free(spimm);
    return true;
}

static bool trans_push(DisasContext *ctx, arg_push *a)
{
    REQUIRE_ZCEA;
    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);
    int zce_rlist, zce_alist;
    zce_rlist = a->zce_rlist;

    zce_alist = a->areg? zce_rlist <= 4 ? zce_rlist : 4 : 0;
    TCGv rlist = tcg_const_tl(zce_rlist);
    TCGv spimm = tcg_const_tl(a->zce_spimm);
    TCGv alist = tcg_const_tl(zce_alist);   
    gen_helper_push(cpu_env, sp, spimm, rlist, alist);

    tcg_temp_free(rlist);
    tcg_temp_free(spimm);
    tcg_temp_free(alist);
    return true;
}

static bool trans_pop(DisasContext *ctx, arg_pop *a)
{
    REQUIRE_ZCEA;
    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);
    int zce_rlist;
    zce_rlist = a->zce_rlist;

    TCGv rlist = tcg_const_tl(zce_rlist);
    TCGv spimm = tcg_const_tl(a->zce_spimm);
    TCGv ret = tcg_const_tl(a->ret_val);   
    gen_helper_pop(cpu_env, sp, spimm, rlist, ret);

    tcg_temp_free(rlist);
    tcg_temp_free(spimm);
    tcg_temp_free(ret);
    return true;
}

static bool trans_popret(DisasContext *ctx, arg_popret *a)
{
    REQUIRE_ZCEA;
    TCGv sp = get_gpr(ctx, xSP, EXT_NONE);
    int zce_rlist;
    zce_rlist = a->zce_rlist;

    TCGv rlist = tcg_const_tl(zce_rlist);
    TCGv spimm = tcg_const_tl(a->zce_spimm);
    TCGv ret = tcg_const_tl(a->ret_val);   
    gen_helper_popret(cpu_env, sp, spimm, rlist, ret);

    tcg_temp_free(rlist);
    tcg_temp_free(spimm);
    tcg_temp_free(ret);
    return true;
}

// RV64 Only
static bool trans_c_zext_w(DisasContext *ctx, arg_c_zext_w *a)
{
    REQUIRE_64BIT(ctx);
    REQUIRE_ZCEE;
    return gen_zce_unary(ctx, a, tcg_gen_ext32u_tl);
}

static bool trans_ldgp(DisasContext *ctx, arg_ldgp *a)
{
    REQUIRE_64BIT(ctx);
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, xSP, EXT_NONE);
    TCGv dest = dest_gpr(ctx, a->rd);
    TCGv t1 = tcg_temp_new();

    tcg_gen_addi_tl(t1, t0, a->zce_b_imm_ld);

    MemOp memop = MO_TEQ; // 64s-bit
    tcg_gen_qemu_ld_tl(dest, t1, ctx->mem_idx, memop);
    gen_set_gpr(ctx, a->rd, dest);

    tcg_temp_free(t1);
    return true;
}

static bool trans_sdgp(DisasContext *ctx, arg_sdgp *a)
{
    REQUIRE_64BIT(ctx);
    REQUIRE_ZCEB;
    TCGv t0 = get_gpr(ctx, xGP, EXT_NONE);
    TCGv dat = get_gpr(ctx, a->rs2, EXT_NONE);

    tcg_gen_addi_tl(t0, t0, a->zce_b_imm_sd);

    MemOp memop = MO_TEQ; // 64s-bit
    tcg_gen_qemu_st_tl(dat, t0, ctx->mem_idx, memop);

    return true;
}

#undef REQUIRE_ZCE