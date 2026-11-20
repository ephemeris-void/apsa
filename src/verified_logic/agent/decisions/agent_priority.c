#include "agent_priority.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"

/* ============================================================
 * AGENT PRIORITY ARBITRATION — IMPLEMENTATION (FIXED)
 * ============================================================ */

agent_priority_t agent_priority_max(agent_priority_t a,
                                     agent_priority_t b)
{
    return (a > b) ? a : b;
}

agent_priority_t agent_priority_from_state(agent_state_t state)
{
    static const agent_priority_t prio_table[AGENT_STATE_MAX] = {
        AGENT_PRIO_NORMAL,    /* BOOT */
        AGENT_PRIO_NORMAL,    /* MONITOR */
        AGENT_PRIO_HIGH,      /* DEGRADED */
        AGENT_PRIO_HIGH,      /* VIRTUALIZE */
        AGENT_PRIO_HIGH,      /* FUSION */
        AGENT_PRIO_HIGH,      /* RECOVERY */
        AGENT_PRIO_CRITICAL,  /* EMERGENCY */
        AGENT_PRIO_CRITICAL   /* LANDING */
    };

    if (state >= AGENT_STATE_MAX) {
        return AGENT_PRIO_LOW;
    }
    return prio_table[state];
}

agent_priority_t agent_priority_from_health(agent_health_t health)
{
    static const agent_priority_t prio_table[4] = {
        AGENT_PRIO_LOW,       /* UNKNOWN */
        AGENT_PRIO_NORMAL,    /* OK */
        AGENT_PRIO_HIGH,      /* DEGRADED */
        AGENT_PRIO_CRITICAL   /* FAIL */
    };

    if ((uint32_t)health > 3U) {
        return AGENT_PRIO_LOW;
    }
    return prio_table[(uint32_t)health];
}

/* FIXED: no health fields in context. Only state-based priority. */
agent_priority_t agent_priority_evaluate(void)
{
    agent_context_full_t *ctx = agent_context_get();

    agent_priority_t p = agent_priority_from_state(ctx->current_state);

    /* EMERGENCY state implies CRITICAL */
    if (ctx->current_state == AGENT_STATE_EMERGENCY ||
        ctx->current_state == AGENT_STATE_LANDING) {
        p = AGENT_PRIO_CRITICAL;
    }

    ctx->current_priority = p;
    return p;
}

agent_priority_t agent_decide_priority(void)
{
    return agent_priority_evaluate();
}