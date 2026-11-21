#include "agent_test.h"
#include "agent_test_integration.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT TEST — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5000 µs per run (boot only)
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_test_state_t g_test;
static bool                          g_test_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_test_init(void)
{
    memset(&g_test, 0, sizeof(g_test));    /* zero-init incl. reserved[] */

    for (uint32_t i = 0U; i < AGENT_TEST_MAX_CASES; i++) {
        g_test.cases[i].name        = "unset";
        g_test.cases[i].fn          = NULL;
        g_test.cases[i].result      = AGENT_TEST_RESULT_SKIP;
        g_test.cases[i].duration_us = 0U;
        g_test.cases[i].run_count   = 0U;
        g_test.cases[i].fail_count  = 0U;
    }

    g_test.count       = 0U;
    g_test.passed      = 0U;
    g_test.failed      = 0U;
    g_test.skipped     = 0U;
    g_test.total_runs  = 0U;
    g_test.last_run_us = 0U;

    agent_test_integration_init();

    g_test_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_test_reset(void)
{
    g_test_initialized = false;
    return agent_test_init();
}

/* ---------- REGISTER ---------- */
agent_status_t agent_test_register(const char *name, agent_test_fn_t fn)
{
    if (!g_test_initialized)    return AGENT_ERR_NOT_INIT;
    if (fn == NULL)             return AGENT_ERR;

    if (g_test.count >= AGENT_TEST_MAX_CASES) {
        return AGENT_ERR;
    }

    agent_test_case_t *c = &g_test.cases[g_test.count];

    memset(c, 0, sizeof(*c));    /* zero-init incl. reserved[] */

    c->name        = (name != NULL) ? name : "unnamed";
    c->fn          = fn;
    c->result      = AGENT_TEST_RESULT_SKIP;
    c->duration_us = 0U;
    c->run_count   = 0U;
    c->fail_count  = 0U;

    g_test.count++;
    return AGENT_OK;
}

/* ---------- RUN ONE ---------- */
agent_status_t agent_test_run_one(uint32_t idx)
{
    if (!g_test_initialized)    return AGENT_ERR_NOT_INIT;
    if (idx >= g_test.count)    return AGENT_ERR;

    agent_test_case_t *c = &g_test.cases[idx];

    if (c->fn == NULL) {
        c->result = AGENT_TEST_RESULT_SKIP;
        return AGENT_ERR;
    }

    uint32_t t0 = agent_time_now_us();
    agent_test_result_t r = c->fn();
    uint32_t t1 = agent_time_now_us();

    c->result      = r;
    c->duration_us = t1 - t0;
    c->run_count++;

    if (r == AGENT_TEST_RESULT_PASS) {
        g_test.passed++;
    } else if (r == AGENT_TEST_RESULT_FAIL) {
        g_test.failed++;
        c->fail_count++;
    } else if (r == AGENT_TEST_RESULT_SKIP) {
        g_test.skipped++;
    }

    return AGENT_OK;
}

/* ---------- RUN ALL ---------- */
agent_status_t agent_test_run_all(void)
{
    if (!g_test_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    g_test.passed    = 0U;
    g_test.failed    = 0U;
    g_test.skipped   = 0U;
    g_test.total_runs++;

    for (uint32_t i = 0U; i < g_test.count; i++) {
        (void)agent_test_run_one(i);
    }

    g_test.last_run_us = agent_time_now_us();

    agent_log_event(AGENT_LOG_SELFTEST_OK,
                    g_test.passed,
                    g_test.failed);

    return (g_test.failed == 0U) ? AGENT_OK : AGENT_ERR;
}

/* ---------- QUERIES ---------- */
uint32_t agent_test_passed(void) { return g_test.passed; }
uint32_t agent_test_failed(void) { return g_test.failed; }

bool agent_test_all_passed(void)
{
    return (g_test.failed == 0U && g_test.count > 0U) ? true : false;
}

const agent_test_case_t *agent_test_case(uint32_t idx)
{
    if (idx >= g_test.count) {
        return NULL;
    }
    return &g_test.cases[idx];
}

const agent_test_state_t *agent_test_state(void)
{
    return &g_test;
}