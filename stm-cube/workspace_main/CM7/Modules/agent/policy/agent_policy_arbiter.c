#include "agent_policy_arbiter.h"
#include "agent_policy.h"
#include "agent_policy_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY ARBITER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs per run
 * ============================================================ */

/* PERF: arbitration context in DTCM (zero wait state) */
static AGENT_DTCM agent_policy_arbitration_t g_arb;

/* ---------- RESET ---------- */
agent_status_t agent_policy_arbiter_reset(void)
{
    memset(&g_arb, 0, sizeof(g_arb));    /* zero-init all including reserved[] */

    g_arb.winner_id    = AGENT_POLICY_ID_NONE;
    g_arb.timestamp_us = 0U;

    return AGENT_OK;
}

/* ---------- RUN (collect → filter → arbitrate) ---------- */
agent_status_t agent_policy_arbiter_run(agent_policy_candidate_t *winner_out)
{
    if (winner_out == NULL) {
        return AGENT_ERR;
    }

    agent_policy_arbiter_reset();

    uint32_t t0 = agent_time_now_us();

    /* 1. Collect candidates from all registered policies */
    agent_status_t st = agent_policy_collect(g_arb.candidates,
                                             AGENT_POLICY_REGISTRY_MAX,
                                             &g_arb.count);

    if (st != AGENT_OK || g_arb.count == 0U) {
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0U, 0U);
        return AGENT_ERR;
    }

    /* 2. Apply safety filter to each candidate */
    for (uint32_t i = 0U; i < g_arb.count; i++) {
        (void)agent_policy_safety_filter(&g_arb.candidates[i]);
    }

    /* 3. Deterministic arbitration */
    agent_policy_candidate_t winner;

    st = agent_policy_arbitrate(g_arb.candidates, g_arb.count, &winner);
    if (st != AGENT_OK) {
        return st;
    }

    uint32_t t1 = agent_time_now_us();

    g_arb.winner_id    = winner.id;
    g_arb.timestamp_us = t1;

    winner.wcet_us = t1 - t0;

    *winner_out = winner;
    return AGENT_OK;
}

/* ---------- LAST ---------- */
const agent_policy_arbitration_t *agent_policy_arbiter_last(void)
{
    return &g_arb;
}