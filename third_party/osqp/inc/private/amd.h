#ifndef AMD_H
#define AMD_H

#include "osqp_api_types.h"

#ifdef __cplusplus
extern "C" {
#endif

#define AMD_INFO 20

static inline OSQPInt amd_l_order(OSQPInt n,
                                  const OSQPInt* Ap,
                                  const OSQPInt* Ai,
                                  OSQPInt* P,
                                  OSQPFloat* Control,
                                  OSQPFloat* Info) {
    (void)Ap;
    (void)Ai;
    (void)Control;
    if (Info) {
        for (int k = 0; k < AMD_INFO; ++k) Info[k] = 0.0;
    }
    if (!P || n < 0) return -1;
    for (OSQPInt i = 0; i < n; ++i) {
        P[i] = i;
    }
    return 0;
}

static inline OSQPInt amd_order(OSQPInt n,
                                const OSQPInt* Ap,
                                const OSQPInt* Ai,
                                OSQPInt* P,
                                OSQPFloat* Control,
                                OSQPFloat* Info) {
    return amd_l_order(n, Ap, Ai, P, Control, Info);
}

#ifdef __cplusplus
}
#endif

#endif /* ifndef AMD_H */

