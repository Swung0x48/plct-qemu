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
#include "exec/cpu_ldst.h"

#define X_S0    8
#define X_S1    9
#define X_Sn    16
#define X_RA    1
#define X_A0    10
#define X_S4_E  7
#define X_S3_E  6
#define X_S2_E  14
#define XLEN (8 * sizeof(target_ulong))  // to fix
#define sext_xlen(x) (((int64_t)(x) << (64 - XLEN)) >> (64 - XLEN))

#define ZCE_POP(env, sp, bytes, rlist, spimm, ret_val, ret) {              \
    target_ulong xreg_list[32] = {0};                                      \
    target_ulong addr;                                                     \
    target_ulong stack_adj;                                                \
    \
    switch (rlist) {                                                       \
    case 15:                                                               \
        xreg_list[X_Sn + 11] = 1;                                          \
        xreg_list[X_Sn + 10] = 1;                                          \
            /* FALL THROUGH */                                             \
    case 14: xreg_list[X_Sn + 9] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 13: xreg_list[X_Sn + 8] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 12: xreg_list[X_Sn + 7] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 11: xreg_list[X_Sn + 6] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 10: xreg_list[X_Sn + 5] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 9: xreg_list[X_Sn + 4] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 8: xreg_list[X_Sn + 3] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 7: xreg_list[X_Sn + 2] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 6: xreg_list[X_S1] = 1;                                           \
            /* FALL THROUGH */                                             \
    case 5: xreg_list[X_S0] = 1;                                           \
            /* FALL THROUGH */                                             \
    case 4: xreg_list[X_RA] = 1;                                           \
        break;                                                             \
    default:                                                               \
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());      \
    }                                                                      \
    stack_adj = caculate_stack_adj(rlist, spimm);                          \
    addr = sp + stack_adj - bytes;                                         \
    for(int i = 31; i >= 0; i--) {                                         \
        if (xreg_list[i]) {                                           \
            switch (bytes) {                                               \
            case 4:                                                        \
                env->gpr[i] = cpu_ldl_le_data(env, addr);                  \
                break;                                                     \
            case 8:                                                        \
                env->gpr[i] = cpu_ldq_le_data(env, addr);                  \
                break;                                                     \
            default:                                                       \
                break;                                                     \
            }                                                              \
            addr -= bytes;                                                 \
        }                                                                  \
    }                                                                      \
    \
    if (ret_val) {                                                         \
        env->gpr[xA0] = 0;                                                 \
    }                                                                      \
    \
    env->gpr[xSP] = sp + stack_adj;                                        \
    if (ret) {                                                             \
        return env->gpr[xRA];                                              \
    } else {                                                               \
        return env->pc;                                                    \
    }                                                                      \
}

#define ZCE_PUSH(env, sp, bytes, rlist, spimm) {                           \
    target_ulong addr = sp;                                                \
    target_ulong xreg_list[32] = {0};                                      \
    target_ulong stack_adj;                                                \
    \
    switch (rlist) {                                                       \
    case 15: xreg_list[X_Sn + 11] = 1;                                     \
        xreg_list[X_Sn + 10] = 1;                                          \
            /* FALL THROUGH */                                             \
    case 14: xreg_list[X_Sn + 9] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 13: xreg_list[X_Sn + 8] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 12: xreg_list[X_Sn + 7] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 11: xreg_list[X_Sn + 6] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 10: xreg_list[X_Sn + 5] = 1;                                      \
            /* FALL THROUGH */                                             \
    case 9: xreg_list[X_Sn + 4] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 8: xreg_list[X_Sn + 3] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 7: xreg_list[X_Sn + 2] = 1;                                       \
            /* FALL THROUGH */                                             \
    case 6: xreg_list[X_S1] = 1;                                           \
            /* FALL THROUGH */                                             \
    case 5: xreg_list[X_S0] = 1;                                           \
            /* FALL THROUGH */                                             \
    case 4: xreg_list[X_RA] = 1;                                           \
        break;                                                             \
    default:                                                               \
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());      \
    }                                                                      \
    \
    stack_adj = caculate_stack_adj(rlist, spimm);                          \
    addr -= bytes;                                                         \
    for (int i = 31; i >= 0; i--) {                                        \
        if (xreg_list[i]) {                                                \
            switch (bytes) {                                               \
            case 4:                                                        \
                cpu_stl_le_data(env, addr, env->gpr[i]);                   \
                break;                                                     \
            case 8:                                                        \
                cpu_stq_le_data(env, addr, env->gpr[i]);                   \
                break;                                                     \
            default:                                                       \
                break;                                                     \
            }                                                              \
            addr -= bytes;                                                 \
        }                                                                  \
    }                                                                      \
    env->gpr[xSP] = sp - stack_adj;                                        \
}

static target_ulong caculate_stack_adj(target_ulong rlist, target_ulong spimm)
{
    target_ulong stack_adj_base = 0;
#ifdef TARGET_RISCV64
    switch (rlist) {
    case 15:
        stack_adj_base = 112;
        break;
    case 14:
        stack_adj_base = 96;
        break;
    case 13:
    case 12:
        stack_adj_base = 80;
        break;
    case 11:
    case 10:
        stack_adj_base = 64;
        break;
    case 9:
    case 8:
        stack_adj_base = 48;
        break;
    case 7:
    case 6:
        stack_adj_base = 32;
        break;
    case 5:
    case 4:
        stack_adj_base = 16;
        break;
    }
#else
    switch (rlist) {
    case 15:
        stack_adj_base = 64;
        break;
    case 14:
    case 13:
    case 12:
        stack_adj_base = 48;
        break;
    case 11:
    case 10:
    case 9:
    case 8:
        stack_adj_base = 32;
        break;
    case 7:
    case 6:
    case 5:
    case 4:
        stack_adj_base = 16;
        break;
    }
#endif
    return stack_adj_base + spimm;
}

void HELPER(cm_push)(CPURISCVState *env, target_ulong sp, target_ulong spimm,
                     target_ulong rlist)
{
    target_ulong bytes = XLEN >> 3;
    ZCE_PUSH(env, sp, bytes, rlist, spimm);
}


target_ulong HELPER(cm_pop)(CPURISCVState *env, target_ulong sp,
                            target_ulong spimm, target_ulong rlist)
{
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, false, false);
}


target_ulong HELPER(cm_popret)(CPURISCVState *env, target_ulong sp,
                               target_ulong spimm, target_ulong rlist)
{
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, false, true);
}

target_ulong HELPER(cm_popretz)(CPURISCVState *env, target_ulong sp,
                                target_ulong spimm, target_ulong rlist)
{
    target_ulong bytes = XLEN >> 3;
    ZCE_POP(env, sp, bytes, rlist, spimm, true, true);
}

target_ulong HELPER(cm_jt_all)(CPURISCVState *env, target_ulong index, target_ulong next_pc)
{
    target_ulong target = next_pc;
    target_ulong val = 0;
    val = env->jvt;

    uint8_t mode = get_field(val, JVT_MODE);
    target_ulong base = get_field(val, JVT_BASE);
    target_ulong t0;
    bool link = false;

    if (mode != 0) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }

    if (index >= 64) {
        index -= 64;
        link = true;
    }

    if (XLEN == 32) {  // to fix
        t0 = base + (index << 2);
        target = cpu_ldl_le_data(env, t0);
    } else {  // XLEN = 64
        t0 = base + (index << 3);
        target = cpu_ldq_le_data(env, t0);
    }

    if (link) {  // ra for cm.jalt
        env->gpr[1] = next_pc;
    }

    return target;
}

#undef X_S0
#undef X_Sn
#undef XLEN
#undef ZCE_POP
#undef ZCE_PUSH
#undef sext_xlen