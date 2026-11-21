#include "agent_sim.h"
#include "../core/agent_main_hook.h"       /* ← এই লাইন যোগ করো */
#include "../core/agent.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../interfaces/agent_imu_if.h"
#include "../interfaces/agent_ai_if.h"
#include "../interfaces/agent_ekf_if.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT SIM — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 1000 µs per step
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_sim_ctx_t g_sim;
static bool                       g_sim_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_sim_init(void)
{
    memset(&g_sim, 0, sizeof(g_sim));    /* zero-init incl. reserved[] */

    g_sim.scenario     = AGENT_SIM_SCENARIO_NONE;
    g_sim.step         = 0U;
    g_sim.total_steps  = 0U;
    g_sim.start_us     = 0U;
    g_sim.end_us       = 0U;
    g_sim.fail_at_step = 0U;
    g_sim.running      = false;
    g_sim.completed    = false;

    g_sim_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_sim_reset(void)
{
    g_sim_initialized = false;
    return agent_sim_init();
}

/* ---------- START ---------- */
agent_status_t agent_sim_start(agent_sim_scenario_t scenario,
                               uint32_t             total_steps,
                               uint32_t             fail_at_step)
{
    if (!g_sim_initialized)                     return AGENT_ERR_NOT_INIT;
    if (total_steps == 0U)                      return AGENT_ERR;
    if (total_steps > AGENT_SIM_MAX_STEPS)      return AGENT_ERR;

    g_sim.scenario     = scenario;
    g_sim.step         = 0U;
    g_sim.total_steps  = total_steps;
    g_sim.fail_at_step = fail_at_step;
    g_sim.start_us     = agent_time_now_us();
    g_sim.running      = true;
    g_sim.completed    = false;

    /* Initialize agent system */
    (void)agent_main_init();

    return AGENT_OK;
}

/* ---------- STEP ---------- */
agent_status_t agent_sim_step(void)
{
    if (!g_sim_initialized) return AGENT_ERR_NOT_INIT;
    if (!g_sim.running)     return AGENT_ERR_INVALID_STATE;

    /* Inject fault at designated step */
    if (g_sim.step == g_sim.fail_at_step) {

        switch (g_sim.scenario) {
        case AGENT_SIM_SCENARIO_IMU_FAIL:
        case AGENT_SIM_SCENARIO_BOTH:
            imu_on_failure();
            break;

        case AGENT_SIM_SCENARIO_AI_FAIL:
            ai_on_failure();
            break;

        case AGENT_SIM_SCENARIO_DIVERGE:
            ekf_on_failure();
            break;

        default:
            break;
        }
    }

    /* Run one agent iteration */
    (void)agent_main_step();

    g_sim.step++;

    if (g_sim.step >= g_sim.total_steps) {
        g_sim.running   = false;
        g_sim.completed = true;
        g_sim.end_us    = agent_time_now_us();
    }

    return AGENT_OK;
}

/* ---------- RUN ALL ---------- */
agent_status_t agent_sim_run_all(void)
{
    if (!g_sim_initialized) return AGENT_ERR_NOT_INIT;

    while (g_sim.running) {
        (void)agent_sim_step();
    }

    return AGENT_OK;
}

/* ---------- STOP ---------- */
agent_status_t agent_sim_stop(void)
{
    g_sim.running = false;
    return AGENT_OK;
}

/* ---------- QUERIES ---------- */
agent_sim_scenario_t agent_sim_scenario(void)
{
    return g_sim.scenario;
}

uint32_t agent_sim_progress(void)
{
    if (g_sim.total_steps == 0U) {
        return 0U;
    }
    return (g_sim.step * 100U) / g_sim.total_steps;
}

bool agent_sim_is_running(void)
{
    return g_sim.running;
}

const agent_sim_ctx_t *agent_sim_ctx(void)
{
    return &g_sim;
}