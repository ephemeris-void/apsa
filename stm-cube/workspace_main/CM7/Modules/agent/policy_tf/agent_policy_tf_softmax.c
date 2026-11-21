#include "agent_policy_tf_softmax.h"
#include "../core/agent_context.h"
#include <math.h>

/* ============================================================
 * AGENT POLICY TF SOFTMAX — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per row
 * ============================================================ */

static bool g_softmax_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_policy_tf_softmax_init(void)
{
    g_softmax_initialized = true;
    return AGENT_OK;
}

/* ---------- MAX ---------- */
float agent_policy_tf_softmax_max(const float *v, uint32_t len)
{
    if (v == NULL || len == 0U) {
        return 0.0f;
    }

    float m = v[0];

    for (uint32_t i = 1U; i < len; i++) {
        if (v[i] > m) {
            m = v[i];
        }
    }
    return m;
}

/* ---------- ROW SOFTMAX (numerically stable) ---------- */
void agent_policy_tf_softmax_row(float *row, uint32_t len)
{
    if (row == NULL || len == 0U) {
        return;
    }

    float m = agent_policy_tf_softmax_max(row, len);

    float sum = 0.0f;

    for (uint32_t i = 0U; i < len; i++) {
        row[i] = expf(row[i] - m);
        sum += row[i];
    }

    /* Guard against zero sum */
    if (sum < 1e-8f) {
        sum = 1e-8f;
    }

    float inv = 1.0f / sum;

    for (uint32_t i = 0U; i < len; i++) {
        row[i] *= inv;
    }
}

/* ---------- 2D SOFTMAX ---------- */
void agent_policy_tf_softmax_2d(float    mat[AGENT_TF_SEQ_LEN][AGENT_TF_SEQ_LEN],
                                uint32_t rows,
                                uint32_t cols)
{
    if (mat == NULL) {
        return;
    }

    for (uint32_t r = 0U; r < rows && r < AGENT_TF_SEQ_LEN; r++) {
        agent_policy_tf_softmax_row(mat[r], cols);
    }
}