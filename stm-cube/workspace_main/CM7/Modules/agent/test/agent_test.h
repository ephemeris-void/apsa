#ifndef AGENT_TEST_H
#define AGENT_TEST_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT TEST — FROZEN
 * ============================================================
 * Unit + integration test harness for agent modules.
 * Runs on-target at boot or via debug command.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5000 µs max per run (boot only).
 * ============================================================ */

#define AGENT_TEST_MAX_CASES        32U
#define AGENT_TEST_NAME_LEN         32U

/* ---------- Result ---------- */
typedef enum {
    AGENT_TEST_RESULT_PASS      = 0,
    AGENT_TEST_RESULT_FAIL      = 1,
    AGENT_TEST_RESULT_SKIP      = 2,
    AGENT_TEST_RESULT_ERROR     = 3
} agent_test_result_t;

/* ---------- Test callback ---------- */
typedef agent_test_result_t (*agent_test_fn_t)(void);

/* ---------- Test case ---------- */
typedef struct {
    const char          *name;
    agent_test_fn_t      fn;
    agent_test_result_t  result;
    uint32_t             duration_us;
    uint32_t             run_count;
    uint32_t             fail_count;
    uint32_t             reserved[2];   /* ABI-safe future extension */
} agent_test_case_t;

/* ---------- State ---------- */
typedef struct {
    agent_test_case_t  cases[AGENT_TEST_MAX_CASES];
    uint32_t           count;
    uint32_t           passed;
    uint32_t           failed;
    uint32_t           skipped;
    uint32_t           total_runs;
    uint32_t           last_run_us;
    uint32_t           reserved[2];   /* ABI-safe future extension */
} agent_test_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_test_init(void);
agent_status_t              agent_test_reset(void);

agent_status_t              agent_test_register(const char     *name,
                                                agent_test_fn_t fn);

agent_status_t              agent_test_run_all(void);
agent_status_t              agent_test_run_one(uint32_t idx);

uint32_t                    agent_test_passed(void);
uint32_t                    agent_test_failed(void);
bool                        agent_test_all_passed(void);

const agent_test_case_t    *agent_test_case(uint32_t idx);
const agent_test_state_t   *agent_test_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_TEST_H */