#ifndef AGENT_SIM_H
#define AGENT_SIM_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT SIM — FROZEN
 * ============================================================
 * Hardware-in-loop / fault-injection simulation runner.
 * Deterministic scenario replay for verification.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 1000 µs max per step.
 * ============================================================ */

#define AGENT_SIM_MAX_SCENARIOS     16U
#define AGENT_SIM_MAX_STEPS         1024U

/* ---------- Scenario IDs ---------- */
typedef enum {
    AGENT_SIM_SCENARIO_NONE     = 0,
    AGENT_SIM_SCENARIO_NOMINAL  = 1,
    AGENT_SIM_SCENARIO_IMU_FAIL = 2,
    AGENT_SIM_SCENARIO_AI_FAIL  = 3,
    AGENT_SIM_SCENARIO_BOTH     = 4,
    AGENT_SIM_SCENARIO_DIVERGE  = 5,
    AGENT_SIM_SCENARIO_WCET     = 6,
    AGENT_SIM_SCENARIO_CUSTOM   = 7
} agent_sim_scenario_t;

/* ---------- Context ---------- */
typedef struct {
    agent_sim_scenario_t scenario;
    uint32_t             step;
    uint32_t             total_steps;
    uint32_t             start_us;
    uint32_t             end_us;
    uint32_t             fail_at_step;
    bool                 running;
    bool                 completed;
    uint32_t             reserved[2];   /* ABI-safe future extension */
} agent_sim_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_sim_init(void);
agent_status_t              agent_sim_reset(void);

agent_status_t              agent_sim_start(agent_sim_scenario_t scenario,
                                            uint32_t             total_steps,
                                            uint32_t             fail_at_step);

agent_status_t              agent_sim_step(void);
agent_status_t              agent_sim_run_all(void);
agent_status_t              agent_sim_stop(void);

agent_sim_scenario_t        agent_sim_scenario(void);
uint32_t                    agent_sim_progress(void);
bool                        agent_sim_is_running(void);

const agent_sim_ctx_t      *agent_sim_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_SIM_H */