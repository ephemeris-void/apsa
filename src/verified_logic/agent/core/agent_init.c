#include "agent_init.h"
#include "agent.h"
#include "agent_context.h"
#include "agent_config.h"
#include "../states/agent_states.h"
#include "../states/agent_transitions.h"
#include "../decisions/agent_decisions.h"
#include "../decisions/agent_priority.h"
#include "../actions/agent_actions.h"
#include "../safety/agent_safety.h"
#include "../safety/agent_watchdog.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include "../trace/agent_stats.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../interfaces/agent_control_if.h"

/* ============================================================
 * AGENT SYSTEM INITIALIZATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Order matters. Do not reorder.
 * WCET: ~2 ms total (runs once at boot, not in 1 kHz loop)
 * ============================================================ */

agent_status_t agent_system_init(void)
{
    agent_status_t st;

    /* 1. Trace (DWT) — must be first, used everywhere */
    st = agent_trace_init();
    if (st != AGENT_OK) return st;

    /* 2. Log ring buffer */
    st = agent_log_init();
    if (st != AGENT_OK) return st;

    /* 3. Stats */
    st = agent_stats_init();
    if (st != AGENT_OK) return st;

    /* 4. Context */
    agent_context_init();

    /* 5. Watchdog */
    st = agent_watchdog_init();
    if (st != AGENT_OK) return st;

    /* 6. Safety */
    st = agent_safety_init();
    if (st != AGENT_OK) return st;

    /* 7. External interfaces */
    st = imu_init();
    if (st != AGENT_OK) return st;

    st = ai_init();
    if (st != AGENT_OK) return st;

    st = ekf_init();
    if (st != AGENT_OK) return st;

    st = control_init();
    if (st != AGENT_OK) return st;

    /* 8. Agent core state machine */
    st = agent_init();
    if (st != AGENT_OK) return st;

    /* 9. Initial priority */
    agent_priority_evaluate();

    agent_log_event(AGENT_LOG_INIT, 0U, 0U);

    return AGENT_OK;
}