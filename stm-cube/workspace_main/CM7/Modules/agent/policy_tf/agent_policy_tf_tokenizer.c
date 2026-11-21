#include "agent_policy_tf_tokenizer.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY TF TOKENIZER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 20 µs per embed
 * ============================================================ */

/* PERF: token buffer in DTCM (zero wait state) */
static AGENT_DTCM float g_tokens[AGENT_TF_SEQ_LEN][AGENT_TF_FEATURE_DIM];
static bool            g_tokenizer_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_tf_tokenizer_init(void)
{
    memset(g_tokens, 0, sizeof(g_tokens));
    g_tokenizer_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_policy_tf_tokenizer_reset(void)
{
    return agent_policy_tf_tokenizer_init();
}

/* ---------- SET ---------- */
agent_status_t agent_policy_tf_tokenizer_set(uint32_t t, uint32_t f, float value)
{
    if (!g_tokenizer_initialized)    return AGENT_ERR_NOT_INIT;
    if (t >= AGENT_TF_SEQ_LEN)       return AGENT_ERR;
    if (f >= AGENT_TF_FEATURE_DIM)   return AGENT_ERR;

    g_tokens[t][f] = value;
    return AGENT_OK;
}

/* ---------- GET ---------- */
agent_status_t agent_policy_tf_tokenizer_get(uint32_t t, uint32_t f, float *out)
{
    if (!g_tokenizer_initialized)    return AGENT_ERR_NOT_INIT;
    if (t >= AGENT_TF_SEQ_LEN)       return AGENT_ERR;
    if (f >= AGENT_TF_FEATURE_DIM)   return AGENT_ERR;
    if (out == NULL)                 return AGENT_ERR;

    *out = g_tokens[t][f];
    return AGENT_OK;
}

/* ---------- CLEAR ---------- */
agent_status_t agent_policy_tf_tokenizer_clear(void)
{
    if (!g_tokenizer_initialized) return AGENT_ERR_NOT_INIT;

    memset(g_tokens, 0, sizeof(g_tokens));
    return AGENT_OK;
}

/* ---------- EMBED (feature → embedding dimension) ----------
 * Simple projection: embedding[d] = tokens[t][d % FEATURE_DIM]
 */
agent_status_t agent_policy_tf_tokenizer_embed(float tokens[AGENT_TF_SEQ_LEN][AGENT_TF_EMBED_DIM])
{
    if (tokens == NULL)              return AGENT_ERR;
    if (!g_tokenizer_initialized)    return AGENT_ERR_NOT_INIT;

    for (uint32_t t = 0U; t < AGENT_TF_SEQ_LEN; t++) {
        for (uint32_t e = 0U; e < AGENT_TF_EMBED_DIM; e++) {
            uint32_t f = e % AGENT_TF_FEATURE_DIM;
            tokens[t][e] = g_tokens[t][f];
        }
    }
    return AGENT_OK;
}