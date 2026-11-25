#ifndef POLISH_H
#define POLISH_H

#include "osqp.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

static inline OSQPInt polish(OSQPSolver* solver) {
    if (solver && solver->info) {
        solver->info->status_polish = OSQP_POLISH_NOT_PERFORMED;
    }
    return 0;
}

#ifdef __cplusplus
}
#endif

#endif /* ifndef POLISH_H */

