#ifndef AGENT_VOTING_H
#define AGENT_VOTING_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT VOTING — FROZEN
 * ============================================================
 * Majority/weighted voting for redundant sensors.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 30 µs max per run.
 * ============================================================ */

#define AGENT_VOTING_MAX_VOTERS       5U
#define AGENT_VOTING_MAJORITY         3U
#define AGENT_VOTING_HISTORY_DEPTH    8U

/* ---------- Result ---------- */
typedef enum {
    AGENT_VOTING_OK          = 0,
    AGENT_VOTING_NO_MAJORITY = 1,
    AGENT_VOTING_ALL_AGREE   = 2,
    AGENT_VOTING_FAIL        = 3
} agent_voting_result_t;

/* ---------- Kind ---------- */
typedef enum {
    AGENT_VOTING_KIND_UNANIMOUS   = 0,
    AGENT_VOTING_KIND_MAJORITY    = 1,
    AGENT_VOTING_KIND_PLURALITY   = 2,
    AGENT_VOTING_KIND_WEIGHTED    = 3
} agent_voting_kind_t;

/* ---------- Ballot ---------- */
typedef struct {
    uint32_t  voter_id;
    int32_t   value;
    float     weight;
    bool      valid;
} agent_voting_ballot_t;

/* ---------- Input ---------- */
typedef struct {
    agent_voting_ballot_t ballots[AGENT_VOTING_MAX_VOTERS];
    uint32_t              voter_count;
    uint32_t              timestamp_us;
    agent_voting_kind_t   kind;
    uint32_t              reserved[2];   /* ABI-safe future extension */
} agent_voting_input_t;

/* ---------- Output ---------- */
typedef struct {
    int32_t               agreed_value;
    uint32_t              agree_count;
    uint32_t              total_count;
    uint32_t              agree_mask;
    uint32_t              dissent_mask;
    agent_voting_result_t result;
    agent_voting_kind_t   kind;
    float                 confidence;
    uint32_t              timestamp_us;
    uint32_t              reserved[2];   /* ABI-safe future extension */
} agent_voting_output_t;

/* ---------- Stats ---------- */
typedef struct {
    uint32_t  total_votes;
    uint32_t  unanimous_votes;
    uint32_t  majority_votes;
    uint32_t  no_majority_votes;
    uint32_t  last_vote_us;
    uint32_t  reserved[2];
} agent_voting_stats_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_voting_init(void);
agent_status_t                  agent_voting_reset(void);

agent_voting_result_t           agent_voting_run(const agent_voting_input_t *in,
                                                 agent_voting_output_t *out);

agent_voting_result_t           agent_voting_run_weighted(const agent_voting_input_t *in,
                                                          agent_voting_output_t *out);

bool                            agent_voting_is_agreed(uint32_t mask, uint32_t voter);

const agent_voting_stats_t     *agent_voting_stats(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_VOTING_H */