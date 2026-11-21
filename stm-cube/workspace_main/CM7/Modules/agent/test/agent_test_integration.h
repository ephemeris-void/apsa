#ifndef AGENT_TEST_INTEGRATION_H
#define AGENT_TEST_INTEGRATION_H

#include "agent_test.h"
#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT TEST INTEGRATION — FROZEN
 * ============================================================
 * Built-in integration test cases for core modules.
 * Registered automatically at agent_test_init().
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 500 µs max per case.
 * ============================================================ */

agent_status_t  agent_test_integration_init(void);
agent_status_t  agent_test_integration_register_all(void);

/* ---------- Individual test cases ---------- */
agent_test_result_t agent_test_tc_context_init(void);
agent_test_result_t agent_test_tc_health_init(void);
agent_test_result_t agent_test_tc_fdir_init(void);
agent_test_result_t agent_test_tc_fdir_isolate(void);
agent_test_result_t agent_test_tc_confidence_ok(void);
agent_test_result_t agent_test_tc_confidence_low(void);
agent_test_result_t agent_test_tc_crosscheck_ok(void);
agent_test_result_t agent_test_tc_crosscheck_diverge(void);
agent_test_result_t agent_test_tc_voting_unanimous(void);
agent_test_result_t agent_test_tc_voting_majority(void);
agent_test_result_t agent_test_tc_fallback_chain(void);
agent_test_result_t agent_test_tc_wcet_ok(void);
agent_test_result_t agent_test_tc_wcet_violate(void);
agent_test_result_t agent_test_tc_modes_switch(void);
agent_test_result_t agent_test_tc_policy_arbitrate(void);
agent_test_result_t agent_test_tc_policy_registry(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_TEST_INTEGRATION_H */