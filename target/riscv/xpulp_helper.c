
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

typedef void SIMDArith(void *, void *, void *, uint8_t);

static inline target_ulong xpulp_simd(target_ulong a, target_ulong b,
                                      uint8_t size, SIMDArith *fn)
{
    int i, passes = sizeof(target_ulong) / size;
    target_ulong result = 0;

    for (i = 0; i < passes; i++) {
        fn(&result, &a, &b, i);
    }
    return result;
}

#define XPULP_SIMD(NAME, OPSIZE)                             \
target_ulong HELPER(NAME)(target_ulong a, target_ulong b)    \
{                                                            \
    return xpulp_simd(a, b, OPSIZE, (SIMDArith *)do_##NAME); \
}

static inline void do_min_h(void *vd, void *va, void *vb, uint8_t i)
{
    int16_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? b[i] : a[i];
}
XPULP_SIMD(min_h, 2);

static inline void do_min_b(void *vd, void *va,
                             void *vb, uint8_t i)
{
    int8_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? b[i] : a[i];
}
XPULP_SIMD(min_b, 1);

static inline void do_max_h(void *vd, void *va, void *vb, uint8_t i)
{
    int16_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? a[i] : b[i];
}
XPULP_SIMD(max_h, 2);

static inline void do_max_b(void *vd, void *va, void *vb, uint8_t i)
{
    int8_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? a[i] : b[i];
}
XPULP_SIMD(max_b, 1);

static inline void do_minu_h(void *vd, void *va, void *vb, uint8_t i)
{
    uint16_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? b[i] : a[i];
}
XPULP_SIMD(minu_h, 2);

static inline void do_minu_b(void *vd, void *va, void *vb, uint8_t i)
{
    uint8_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? b[i] : a[i];
}
XPULP_SIMD(minu_b, 1);

static inline void do_maxu_h(void *vd, void *va, void *vb, uint8_t i)
{
    uint16_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? a[i] : b[i];
}
XPULP_SIMD(maxu_h, 2);

static inline void do_maxu_b(void *vd, void *va, void *vb, uint8_t i)
{
    uint8_t *d = vd, *a = va, *b = vb;
    d[i] = a[i] > b[i] ? a[i] : b[i];
}
XPULP_SIMD(maxu_b, 1);
