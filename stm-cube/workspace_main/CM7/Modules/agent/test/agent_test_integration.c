#include "agent_test_integration.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../health/agent_health.h"
#include "../fdir/agent_fdir.h"
#include "../confidence/agent_confidence.h"
#include "../crosscheck/agent_crosscheck.h"
#include "../voting/agent_voting.h"
#include "../fallback/agent_fallback.h"
#include "../wcet/agent_wcet.h"
#include "../modes/agent_modes.h"
#include "../policy/agent_policy.h"
#include "../policy/agent_policy_registry.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT TEST INTEGRATION — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 500 µs per test case
 * ============================================================ */

static bool g_test_integ_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_test_integration_init(void)
{
    g_test_integ_initialized = true;
    return agent_test_integration_register_all();
}

/* ---------- TC: context init ---------- */
agent_test_result_t agent_test_tc_context_init(void)
{
    agent_context_full_t *ctx = agent_context_get();
    if (ctx == NULL) return AGENT_TEST_RESULT_FAIL;

    agent_context_init();

    if (ctx->current_state != AGENT_STATE_BOOT)        return AGENT_TEST_RESULT_FAIL;
    if (ctx->system_health != AGENT_HEALTH_UNKNOWN)    return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: health init ---------- */
agent_test_result_t agent_test_tc_health_init(void)
{
    if (agent_health_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    if (agent_health_set(AGENT_HEALTH_SENSOR_IMU, AGENT_HEALTH_OK) != AGENT_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    if (agent_health_get(AGENT_HEALTH_SENSOR_IMU) != AGENT_HEALTH_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: FDIR init ---------- */
agent_test_result_t agent_test_tc_fdir_init(void)
{
    if (agent_fdir_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    if (agent_fdir_get_state(AGENT_FDIR_SENSOR_IMU) != AGENT_FDIR_STATE_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: FDIR isolate ---------- */
agent_test_result_t agent_test_tc_fdir_isolate(void)
{
    if (agent_fdir_isolate(AGENT_FDIR_SENSOR_IMU) != AGENT_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    if (!agent_fdir_is_isolated(AGENT_FDIR_SENSOR_IMU)) {
        return AGENT_TEST_RESULT_FAIL;
    }

    (void)agent_fdir_restore(AGENT_FDIR_SENSOR_IMU);
    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: confidence ok ---------- */
agent_test_result_t agent_test_tc_confidence_ok(void)
{
    if (agent_confidence_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    agent_confidence_set(0.95f);

    if (agent_confidence_get() < 0.90f) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: confidence low ---------- */
agent_test_result_t agent_test_tc_confidence_low(void)
{
    agent_confidence_set(0.30f);

    if (agent_confidence_get() > 0.50f) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: crosscheck ok ---------- */
agent_test_result_t agent_test_tc_crosscheck_ok(void)
{
    if (agent_crosscheck_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    agent_crosscheck_input_t in;
    memset(&in, 0, sizeof(in));    /* zero-init incl. reserved[] */

    in.count        = 3U;
    in.timestamp_us = agent_time_now_us();

    for (uint32_t i = 0U; i < 3U; i++) {
        in.ax[i] = 1000; in.ay[i] = 1000; in.az[i] = 1000;
        in.gx[i] = 100;  in.gy[i] = 100;  in.gz[i] = 100;
    }

    agent_crosscheck_output_t out;
    memset(&out, 0, sizeof(out));

    agent_crosscheck_result_t r = agent_crosscheck_run(&in, &out);

    if (r != AGENT_CROSSCHECK_OK) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: crosscheck diverge ---------- */
agent_test_result_t agent_test_tc_crosscheck_diverge(void)
{
    agent_crosscheck_input_t in;
    memset(&in, 0, sizeof(in));

    in.count        = 3U;
    in.timestamp_us = agent_time_now_us();

    in.ax[0] = 1000; in.ay[0] = 1000; in.az[0] = 1000;
    in.gx[0] = 100;  in.gy[0] = 100;  in.gz[0] = 100;

    in.ax[1] = 1000; in.ay[1] = 1000; in.az[1] = 1000;
    in.gx[1] = 100;  in.gy[1] = 100;  in.gz[1] = 100;

    in.ax[2] = 2000; in.ay[2] = 2000; in.az[2] = 2000;
    in.gx[2] = 200;  in.gy[2] = 200;  in.gz[2] = 200;

    agent_crosscheck_output_t out;
    memset(&out, 0, sizeof(out));

    agent_crosscheck_result_t r = agent_crosscheck_run(&in, &out);

    if (r != AGENT_CROSSCHECK_DIVERGED) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: voting unanimous ---------- */
agent_test_result_t agent_test_tc_voting_unanimous(void)
{
    if (agent_voting_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    agent_voting_input_t in;
    memset(&in, 0, sizeof(in));

    in.voter_count  = 3U;
    in.timestamp_us = agent_time_now_us();
    in.kind         = AGENT_VOTING_KIND_MAJORITY;

    for (uint32_t i = 0U; i < 3U; i++) {
        in.ballots[i].voter_id = i;
        in.ballots[i].value    = 100;
        in.ballots[i].weight   = 1.0f;
        in.ballots[i].valid    = true;
    }

    agent_voting_output_t out;
    memset(&out, 0, sizeof(out));

    agent_voting_result_t r = agent_voting_run(&in, &out);

    if (r != AGENT_VOTING_ALL_AGREE) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: voting majority ---------- */
agent_test_result_t agent_test_tc_voting_majority(void)
{
    agent_voting_input_t in;
    memset(&in, 0, sizeof(in));

    in.voter_count  = 3U;
    in.timestamp_us = agent_time_now_us();
    in.kind         = AGENT_VOTING_KIND_MAJORITY;

    in.ballots[0].voter_id = 0; in.ballots[0].value = 100;
    in.ballots[0].valid = true; in.ballots[0].weight = 1.0f;

    in.ballots[1].voter_id = 1; in.ballots[1].value = 100;
    in.ballots[1].valid = true; in.ballots[1].weight = 1.0f;

    in.ballots[2].voter_id = 2; in.ballots[2].value = 200;
    in.ballots[2].valid = true; in.ballots[2].weight = 1.0f;

    agent_voting_output_t out;
    memset(&out, 0, sizeof(out));

    agent_voting_result_t r = agent_voting_run(&in, &out);

    if (r != AGENT_VOTING_OK) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: fallback chain ---------- */
agent_test_result_t agent_test_tc_fallback_chain(void)
{
    if (agent_fallback_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    if (agent_fallback_active() != AGENT_FALLBACK_LEVEL_PRIMARY) {
        return AGENT_TEST_RESULT_FAIL;
    }

    if (agent_fallback_switch(AGENT_FALLBACK_LEVEL_SECONDARY) != AGENT_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    if (agent_fallback_active() != AGENT_FALLBACK_LEVEL_SECONDARY) {
        return AGENT_TEST_RESULT_FAIL;
    }

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: WCET ok ---------- */
agent_test_result_t agent_test_tc_wcet_ok(void)
{
    if (agent_wcet_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    int32_t id = agent_wcet_register("test", 1000U);
    if (id < 0) return AGENT_TEST_RESULT_FAIL;

    agent_wcet_result_t res;
    (void)agent_wcet_check(id, 500U, &res);

    if (res != AGENT_WCET_RESULT_OK) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: WCET violate ---------- */
agent_test_result_t agent_test_tc_wcet_violate(void)
{
    int32_t id = agent_wcet_register("test_violate", 100U);
    if (id < 0) return AGENT_TEST_RESULT_FAIL;

    agent_wcet_result_t res;
    (void)agent_wcet_check(id, 500U, &res);

    if (res != AGENT_WCET_RESULT_VIOLATE) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: modes switch ---------- */
agent_test_result_t agent_test_tc_modes_switch(void)
{
    if (agent_modes_init() != AGENT_OK) return AGENT_TEST_RESULT_FAIL;

    if (agent_modes_active() != AGENT_MODE_ID_NOMINAL) {
        return AGENT_TEST_RESULT_FAIL;
    }

    if (agent_modes_request(AGENT_MODE_ID_DEGRADED) != AGENT_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: policy arbitrate ---------- */
agent_test_result_t agent_test_tc_policy_arbitrate(void)
{
    agent_policy_candidate_t in[2];
    agent_policy_candidate_t winner;

    memset(in, 0, sizeof(in));
    memset(&winner, 0, sizeof(winner));

    in[0].id = AGENT_POLICY_ID_NOMINAL;
    in[0].priority = AGENT_PRIO_NORMAL;
    in[0].confidence_milli = 900U;
    in[0].target_state = AGENT_STATE_MONITOR;
    in[0].reason = AGENT_REASON_NONE;
    in[0].timestamp_us = 100U;
    in[0].wcet_us = 0U;

    in[1].id = AGENT_POLICY_ID_EMERGENCY;
    in[1].priority = AGENT_PRIO_CRITICAL;
    in[1].confidence_milli = 1000U;
    in[1].target_state = AGENT_STATE_EMERGENCY;
    in[1].reason = AGENT_REASON_ENVELOPE;
    in[1].timestamp_us = 200U;
    in[1].wcet_us = 0U;

    if (agent_policy_arbitrate(in, 2U, &winner) != AGENT_OK) {
        return AGENT_TEST_RESULT_FAIL;
    }

    if (winner.id != AGENT_POLICY_ID_EMERGENCY) {
        return AGENT_TEST_RESULT_FAIL;
    }

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- TC: policy registry ---------- */
agent_test_result_t agent_test_tc_policy_registry(void)
{
    agent_policy_registry_reset();

    if (agent_policy_registry_count() != 0U) return AGENT_TEST_RESULT_FAIL;

    return AGENT_TEST_RESULT_PASS;
}

/* ---------- REGISTER ALL ---------- */
agent_status_t agent_test_integration_register_all(void)
{
    (void)agent_test_register("tc_context_init",       agent_test_tc_context_init);
    (void)agent_test_register("tc_health_init",        agent_test_tc_health_init);
    (void)agent_test_register("tc_fdir_init",          agent_test_tc_fdir_init);
    (void)agent_test_register("tc_fdir_isolate",       agent_test_tc_fdir_isolate);
    (void)agent_test_register("tc_confidence_ok",      agent_test_tc_confidence_ok);
    (void)agent_test_register("tc_confidence_low",     agent_test_tc_confidence_low);
    (void)agent_test_register("tc_crosscheck_ok",      agent_test_tc_crosscheck_ok);
    (void)agent_test_register("tc_crosscheck_diverge", agent_test_tc_crosscheck_diverge);
    (void)agent_test_register("tc_voting_unanimous",   agent_test_tc_voting_unanimous);
    (void)agent_test_register("tc_voting_majority",    agent_test_tc_voting_majority);
    (void)agent_test_register("tc_fallback_chain",     agent_test_tc_fallback_chain);
    (void)agent_test_register("tc_wcet_ok",            agent_test_tc_wcet_ok);
    (void)agent_test_register("tc_wcet_violate",       agent_test_tc_wcet_violate);
    (void)agent_test_register("tc_modes_switch",       agent_test_tc_modes_switch);
    (void)agent_test_register("tc_policy_arbitrate",   agent_test_tc_policy_arbitrate);
    (void)agent_test_register("tc_policy_registry",    agent_test_tc_policy_registry);

    return AGENT_OK;
}