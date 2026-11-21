#include "agent_cert.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT CERT — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs per check
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_cert_state_t g_cert;
static bool                          g_cert_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_cert_init(void)
{
    memset(&g_cert, 0, sizeof(g_cert));    /* zero-init incl. reserved[] */

    for (uint32_t i = 0U; i < AGENT_CERT_REQUIREMENT_COUNT; i++) {
        g_cert.requirements[i].name             = "unset";
        g_cert.requirements[i].check            = NULL;
        g_cert.requirements[i].status           = AGENT_CERT_STATUS_NOT_RUN;
        g_cert.requirements[i].last_run_us      = 0U;
        g_cert.requirements[i].last_duration_us = 0U;
        g_cert.requirements[i].run_count        = 0U;
        g_cert.requirements[i].fail_count       = 0U;
    }

    g_cert.count        = 0U;
    g_cert.passed       = 0U;
    g_cert.failed       = 0U;
    g_cert.total_runs   = 0U;
    g_cert.last_run_us  = 0U;
    g_cert.overall_pass = false;

    g_cert_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_cert_reset(void)
{
    g_cert_initialized = false;
    return agent_cert_init();
}

/* ---------- REGISTER ---------- */
agent_status_t agent_cert_register(const char *name, agent_cert_check_fn_t check)
{
    if (!g_cert_initialized)    return AGENT_ERR_NOT_INIT;
    if (check == NULL)          return AGENT_ERR;

    if (g_cert.count >= AGENT_CERT_REQUIREMENT_COUNT) {
        return AGENT_ERR;
    }

    agent_cert_requirement_t *r = &g_cert.requirements[g_cert.count];

    memset(r, 0, sizeof(*r));    /* zero-init incl. reserved[] */

    r->name             = (name != NULL) ? name : "unnamed";
    r->check            = check;
    r->status           = AGENT_CERT_STATUS_NOT_RUN;
    r->last_run_us      = 0U;
    r->last_duration_us = 0U;
    r->run_count        = 0U;
    r->fail_count       = 0U;

    g_cert.count++;
    return AGENT_OK;
}

/* ---------- RUN ONE ---------- */
agent_status_t agent_cert_run_one(uint32_t idx)
{
    if (!g_cert_initialized)    return AGENT_ERR_NOT_INIT;
    if (idx >= g_cert.count)    return AGENT_ERR;

    agent_cert_requirement_t *r = &g_cert.requirements[idx];

    if (r->check == NULL) {
        r->status = AGENT_CERT_STATUS_NOT_RUN;
        return AGENT_ERR;
    }

    uint32_t t0 = agent_time_now_us();

    agent_cert_status_t st = r->check();

    uint32_t t1 = agent_time_now_us();

    r->status           = st;
    r->last_run_us      = t1;
    r->last_duration_us = t1 - t0;
    r->run_count++;

    if (st == AGENT_CERT_STATUS_FAIL) {
        r->fail_count++;
        g_cert.failed++;
        agent_log_event(AGENT_LOG_SAFETY_ESCALATE, 0xF001U, idx);
    } else if (st == AGENT_CERT_STATUS_PASS) {
        g_cert.passed++;
    }

    return AGENT_OK;
}

/* ---------- RUN ALL ---------- */
agent_status_t agent_cert_run_all(void)
{
    if (!g_cert_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    g_cert.passed    = 0U;
    g_cert.failed    = 0U;
    g_cert.total_runs++;

    for (uint32_t i = 0U; i < g_cert.count; i++) {
        (void)agent_cert_run_one(i);
    }

    g_cert.overall_pass = (g_cert.failed == 0U) ? true : false;
    g_cert.last_run_us  = agent_time_now_us();

    agent_log_event(AGENT_LOG_SELFTEST_OK,
                    g_cert.passed,
                    g_cert.failed);

    return (g_cert.failed == 0U) ? AGENT_OK : AGENT_ERR;
}

/* ---------- OVERALL PASS ---------- */
bool agent_cert_overall_pass(void)
{
    return g_cert.overall_pass;
}

/* ---------- QUERIES ---------- */
const agent_cert_requirement_t *agent_cert_requirement(uint32_t idx)
{
    if (idx >= g_cert.count) {
        return NULL;
    }
    return &g_cert.requirements[idx];
}

const agent_cert_state_t *agent_cert_state(void)
{
    return &g_cert;
}