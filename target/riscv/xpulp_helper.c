
#include "qemu/osdep.h"
#include "cpu.h"
#include "exec/exec-all.h"
#include "exec/helper-proto.h"

void HELPER(check_hwlp_body)(CPURISCVState *env, target_ulong pc)
{
    if ((env->hwlp[0].valid && (pc >= env->hwlp[0].lpstart) &&
         (pc <= env->hwlp[0].lpend) && (env->hwlp[0].lpcount > 1)) ||
        (env->hwlp[1].valid && (pc >= env->hwlp[1].lpstart) &&
         (pc <= env->hwlp[1].lpend) && (env->hwlp[1].lpcount > 1))) {
        riscv_raise_exception(env, RISCV_EXCP_ILLEGAL_INST, GETPC());
    }
}
