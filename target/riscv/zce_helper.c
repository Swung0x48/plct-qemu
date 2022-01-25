/*
 * RISC-V Emulation Helpers for QEMU.
 *
 * Copyright (c) 2016-2017 Sagar Karandikar, sagark@eecs.berkeley.edu
 * Copyright (c) 2017-2018 SiFive, Inc.
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

#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"
// #include "exec/memop.h"
// #include "exec/memattrs.h"
// #include "tcg/tcg-internal.h"
#include "exec/cpu_ldst.h"

#define X_S0    8
#define X_S1    9
#define X_Sn    16
#define X_RA    1
#define X_A0    10
#define X_S4_E  7
#define X_S3_E  6
#define X_S2_E  14
#define XLEN (8 * sizeof(target_ulong))
#define align16(x) (((x) + 15) & ~0xf)
#define sext_xlen(x) (((int64_t)(x) << (64 - XLEN)) >> (64 - XLEN))

#define ZCE_POP(env, sp, bytes, rlist, spimm, ret_val, ret, eabi) \
{                                                           \
    target_ulong stack_adjust = rlist * (XLEN >> 3);        \
    stack_adjust += XLEN >> 3;                              \
    stack_adjust = align16(stack_adjust) + spimm;           \
    target_ulong addr = sp + stack_adjust;                  \
    addr -= bytes;                                      \
    switch (bytes) {                                    \
    case 4:                                             \
        env->gpr[X_RA] = cpu_ldl_le_data(env, addr);    \
        break;                                          \
    case 8:                                             \
        env->gpr[X_RA] = cpu_ldq_le_data(env, addr);    \
        break;                                          \
    default:                                            \
        break;                                          \
    }                                                   \
    target_ulong xreg_list[32] = {0};                       \
    switch (rlist) {                                        \
    case 12: xreg_list[X_Sn + 11] = 1;                      \
            /* FALL THROUGH */                              \
    case 11: xreg_list[X_Sn + 10] = 1;                      \
            /* FALL THROUGH */                              \
    case 10: xreg_list[X_Sn + 9] = 1;                       \
            /* FALL THROUGH */                              \
    case 9: xreg_list[X_Sn + 8] = 1;                        \
            /* FALL THROUGH */                              \
    case 8: xreg_list[X_Sn + 7] = 1;                        \
            /* FALL THROUGH */                              \
    case 7: xreg_list[X_Sn + 6] = 1;                        \
            /* FALL THROUGH */                              \
    case 6: xreg_list[X_Sn + 5] = 1;                        \
            /* FALL THROUGH */                              \
    case 5: if (eabi) xreg_list[X_S4_E] = 1;                \
            else xreg_list[X_Sn + 4] = 1;                   \
            /* FALL THROUGH */                              \
    case 4: if (eabi) xreg_list[X_S3_E] = 1;                \
            else xreg_list[X_Sn + 3] = 1;                   \
            /* FALL THROUGH */                              \
    case 3: if (eabi) xreg_list[X_S2_E] = 1;                \
            else xreg_list[X_Sn + 2] = 1;                   \
            /* FALL THROUGH */                              \
    case 2: xreg_list[X_S1] = 1;                            \
            /* FALL THROUGH */                              \
    case 1: xreg_list[X_S0] = 1;                            \
            /* FALL THROUGH */                              \
    case 0: xreg_list[X_RA] = 1;                            \
        break;                                              \
    }                                                       \
    \
    for(int i = 31; i >= 0; i--) {                          \
        if (xreg_list[i] == 0) continue;                    \
        addr -= bytes;                                      \
        switch (bytes)                                      \
        {                                                   \
        case 4:                                             \
            env->gpr[i] = cpu_ldl_le_data(env, addr);       \
            break;                                          \
        case 8:                                             \
            env->gpr[i] = cpu_ldq_le_data(env, addr);       \
            break;                                          \
        default:                                            \
            break;                                          \
        }                                                   \
    }                                                       \
    switch (ret_val) {                                      \
    case 1:                                                 \
        env->gpr[xA0] = 0;                                  \
        break;                                              \
    case 2:                                                 \
        env->gpr[xA0] = 1;                                  \
        break;                                              \
    case 3:                                                 \
        env->gpr[xA0] = -1;                                 \
        break;                                              \
    default:                                                \
        break;                                              \
    }                                                       \
    env->gpr[xSP] = sp + stack_adjust;                      \
    if (ret) {                                              \
        env->pc = env->gpr[xRA];                            \
    }                                                       \
}

#define ZCE_PUSH(env, sp, bytes, rlist, spimm, alist, eabi)                \
{                                                                          \
    target_ulong addr = sp;                                                \
    target_ulong dat;                                                      \
    addr -= bytes;                                                     \
    dat = env->gpr[X_RA];                                               \
    switch (bytes) {                                                    \
    case 4:                                                            \
        cpu_stl_le_data(env, addr, dat);                               \
        break;                                                         \
    case 8:                                                            \
        cpu_stq_le_data(env, addr, dat);                               \
        break;                                                         \
    default:                                                           \
        break;                                                         \
    }                                                                  \
    target_ulong xreg_list[32] = {0};                                      \
    target_ulong xareg_list[32] = {0};                                     \
    switch (rlist) {                                                       \
    case 12: xreg_list[X_Sn + 11] = 1;                                     \
            /* FALL THROUGH */                                             \
    case 11: xreg_list[X_Sn + 10] = 1;                                     \
            /* FALL THROUGH */                                             \
    case 10: xreg_list[X_Sn + 9] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 9: xreg_list[X_Sn + 8] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 8: xreg_list[X_Sn + 7] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 7: xreg_list[X_Sn + 6] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 6: xreg_list[X_Sn + 5] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 5: if (eabi) xreg_list[X_S4_E] = 1;                               \
            else xreg_list[X_Sn + 4] = 1;                                  \
            /* FALL THROUGH */                                             \
    case 4: if (eabi) xreg_list[X_S3_E] = 1;                               \
            else xreg_list[X_Sn + 3] = 1;                                  \
            xareg_list[X_A0 + 3] = alist;                                  \
            /* FALL THROUGH */                                             \
    case 3: if (eabi) xreg_list[X_S2_E] = 1;                               \
            else xreg_list[X_Sn + 2] = 1;                                  \
            xareg_list[X_A0 + 2] = alist;                                  \
            /* FALL THROUGH */                                             \
    case 2: xreg_list[X_S1] = 1;                                           \
            xareg_list[X_A0 + 1] = alist;                                  \
            /* FALL THROUGH */                                             \
    case 1: xreg_list[X_S0] = 1;                                           \
            xareg_list[X_A0] = alist;                                      \
        break;                                                             \
    }                                                                      \
    target_ulong data;                                                     \
    for (int i = 31; i >= 0; i--) {                                        \
        if (xreg_list[i] == 0) continue;                                   \
        addr -= bytes;                                                     \
        data = env->gpr[i];                                                \
        dat = data;                                                        \
        switch (bytes)                                                     \
        {                                                                  \
        case 4:                                                            \
            cpu_stl_le_data(env, addr, dat);                               \
            break;                                                         \
        case 8:                                                            \
            cpu_stq_le_data(env, addr, dat);                               \
            break;                                                         \
        default:                                                           \
            break;                                                         \
        }                                                                  \
    }                                                                      \
    if (xareg_list[10]) {                                                  \
        env->gpr[8] = env->gpr[10];                                        \
    }                                                                      \
    if (xareg_list[11]) {                                                  \
        env->gpr[9] = env->gpr[11];                                        \
    }                                                                      \
    if (xareg_list[12]) {                                                  \
        env->gpr[eabi? 14 : 18] = env->gpr[12];                            \
    }                                                                      \
    if (xareg_list[13]) {                                                  \
        env->gpr[eabi? 6 : 19] = env->gpr[13];                             \
    }                                                                      \
    target_ulong stack_adjust = align16(addr - sp) - spimm;                \
    env->gpr[xSP] = sp + stack_adjust;                                     \
}

target_ulong HELPER(c_tblj_all)(CPURISCVState *env, target_ulong csr, target_ulong index, target_ulong next_pc)
{
    target_ulong target = next_pc;
#ifndef CONFIG_USER_ONLY
    target_ulong val = 0;
    val = env->tbljalvec;

    uint8_t config = get_field(val, TBLJALVEC_CONFIG);
    target_ulong base = get_field(val, TBLJALVEC_BASE);
    target_ulong t0;
    switch (config) {
    case 0:  //jump table mode
        if (XLEN == 32) {
            t0 = base + (index << 2);
            target = cpu_ldl_le_data(env, t0);
        } else {  // XLEN = 8
            t0 = base + (index << 3);
            target = cpu_ldq_le_data(env, t0);
        }

        if (index < 8) {          // C.TBLJALM
            env->gpr[5] = next_pc;
        } else if (index < 64) {  // C.TBLJ
            env->gpr[0] = next_pc;
        } else {                  // C.TBLJAL
            env->gpr[1] = next_pc;
        }
        break;
    default:
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
        break;
    }
#endif
    return target;
}

void HELPER(c_pop)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, 0, false, false);
}

void HELPER(c_pop_e)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    if (rlist == 3) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, 0, false, true);
}

void HELPER(c_popret)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist, target_ulong ret)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, ret, true, false);
}

void HELPER(c_popret_e)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist, target_ulong ret)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    if (rlist == 3) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, ret, true, true);
}

void HELPER(c_push)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    target_ulong bytes = XLEN >> 3;
    target_ulong alist = rlist <= 4 ? rlist : 4;
    ZCE_PUSH(env, sp, bytes, rlist, spimm, alist, false);
}

void HELPER(c_push_e)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    target_ulong bytes = XLEN >> 3;

    if (rlist == 3) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    ZCE_PUSH(env, sp, bytes, rlist, spimm, 0, true);
}

void HELPER(pop)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist, target_ulong ret)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    if (rlist > 12)  //pop.e
        rlist -= 10;
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, ret, false, (rlist > 12));
}

void HELPER(popret)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist, target_ulong ret)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    if (rlist > 12) {  //popret.e
        rlist -= 10;
    }
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, ret, true, (rlist > 12));
}

void HELPER(push)(CPURISCVState *env, target_ulong sp, target_ulong spimm, target_ulong rlist, target_ulong alist)
{
    if (((XLEN / 4 - 1) & sp) != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
    if (rlist > 12) { //push.e
        rlist -= 10;
    }
    target_ulong bytes = XLEN >> 3;
    ZCE_PUSH(env, sp, bytes, rlist, spimm, alist, (rlist > 12));
}

#undef X_S0
#undef X_Sn
#undef align16
#undef XLEN
#undef ZCE_POP
#undef sext_xlen

#undef load32
#undef loadu32
#undef load64

#undef store32
#undef store64