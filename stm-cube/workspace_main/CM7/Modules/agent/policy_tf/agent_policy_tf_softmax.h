#ifndef AGENT_POLICY_TF_SOFTMAX_H
#define AGENT_POLICY_TF_SOFTMAX_H

#include "agent_policy_tf.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY TF SOFTMAX — FROZEN
 * ============================================================
 * Numerically stable softmax (row + 2D).
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5 µs per row.
 * ============================================================ */

agent_status_t  agent_policy_tf_softmax_init(void);

void            agent_policy_tf_softmax_row(float       *row,
                                            uint32_t     len);

void            agent_policy_tf_softmax_2d(float        mat[AGENT_TF_SEQ_LEN][AGENT_TF_SEQ_LEN],
                                           uint32_t     rows,
                                           uint32_t     cols);

float           agent_policy_tf_softmax_max(const float *v, uint32_t len);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_TF_SOFTMAX_H */