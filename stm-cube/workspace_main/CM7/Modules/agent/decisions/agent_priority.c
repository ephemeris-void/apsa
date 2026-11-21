#include "agent_priority.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"

/* ============================================================
 * AGENT PRIORITY ARBITRATION — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs
 * ============================================================ */

/* ---------- Max of two priorities ---------- */
agent_priority_t agent_priority_max(agent_priority_t a,
                                                      agent_priority_t b)
{
    return (a > b) ? a : b;
}

/* ---------- Priority from state ----------
 * PERF: static const lookup table — O(1), no branch chain
 *       (was: switch-case chain, up to 8 comparisons)
 */
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

/* ---------- Priority from health ----------
 * PERF: static const lookup table — O(1)
 */
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

/* ---------- Evaluate combined priority ---------- */
agent_priority_t agent_priority_evaluate(void)
{
    agent_context_full_t *ctx = agent_context_get();

    agent_priority_t p = agent_priority_from_state(ctx->current_state);

    p = agent_priority_max(p, agent_priority_from_health(ctx->imu_health));
    p = agent_priority_max(p, agent_priority_from_health(ctx->ai_health));

    /* baro/gps only matter if they're worse than current */
    if (ctx->baro_health == AGENT_HEALTH_FAIL) {
        p = AGENT_PRIO_CRITICAL;
    }
    if (ctx->gps_health == AGENT_HEALTH_FAIL &&
        p < AGENT_PRIO_HIGH) {
        p = AGENT_PRIO_HIGH;
    }

    ctx->current_priority = p;
    return p;
}