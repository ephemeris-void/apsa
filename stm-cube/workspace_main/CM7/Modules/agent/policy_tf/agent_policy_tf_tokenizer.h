#ifndef AGENT_POLICY_TF_TOKENIZER_H
#define AGENT_POLICY_TF_TOKENIZER_H

#include "agent_policy_tf.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY TF TOKENIZER — FROZEN
 * ============================================================
 * Input token buffer + feature-to-embedding projection.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per embed.
 * ============================================================ */

agent_status_t  agent_policy_tf_tokenizer_init(void);
agent_status_t  agent_policy_tf_tokenizer_reset(void);

agent_status_t  agent_policy_tf_tokenizer_set(uint32_t t, uint32_t f, float value);
agent_status_t  agent_policy_tf_tokenizer_get(uint32_t t, uint32_t f, float *out);
agent_status_t  agent_policy_tf_tokenizer_clear(void);

agent_status_t  agent_policy_tf_tokenizer_embed(float tokens[AGENT_TF_SEQ_LEN][AGENT_TF_EMBED_DIM]);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_TF_TOKENIZER_H */