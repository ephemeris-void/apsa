#ifndef SAL_DIAG_H
#define SAL_DIAG_H

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * SAL DIAGNOSTIC — Root Cause Detector
 * ============================================================
 * Standalone diagnostic. Runs before sal_init_all().
 * No dependency on SAL core. Safe to remove after debugging.
 * ============================================================ */

void sal_diag_run(void);

#ifdef __cplusplus
}
#endif

#endif /* SAL_DIAG_H */