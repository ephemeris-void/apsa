#ifndef AGENT_STATS_H
#define AGENT_STATS_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT STATS — FROZEN
 * ============================================================
 * Aggregate statistics across agent lifetime.
 * Success/fail rates, uptime, recovery counts.
 * Board: NUCLEO-H755ZI-Q
 * WCET: 5 µs max per call.
 * ============================================================ */

/* ---------- Stats Snapshot ---------- */
typedef struct {
    uint32_t    uptime_us;
    uint32_t    iterations;
    uint32_t    state_entries[AGENT_STATE_MAX];
    uint32_t    transitions_ok;
    uint32_t    transitions_blocked;
    uint32_t    transitions_forced;
    uint32_t    faults_total;
    uint32_t    recoveries_total;
    uint32_t    deadline_miss_total;
    uint32_t    wcet_violate_total;
    uint32_t    ai_inference_total;
    uint32_t    ai_latency_min_us;
    uint32_t    ai_latency_max_us;
    uint32_t    ai_latency_avg_us;
    uint32_t    imu_samples_total;
    uint32_t    imu_dropouts_total;
    uint32_t    reserved[4];
} agent_stats_t;

/* ---------- Stats API (FROZEN) ---------- */
agent_status_t  agent_stats_init(void);
void            agent_stats_snapshot(agent_stats_t *out);
void            agent_stats_reset(void);

/* ---------- Increments ---------- */
void            agent_stats_inc_iteration(void);
void            agent_stats_inc_transition_ok(void);
void            agent_stats_inc_transition_blocked(void);
void            agent_stats_inc_transition_forced(void);
void            agent_stats_inc_fault(void);
void            agent_stats_inc_recovery(void);
void            agent_stats_inc_deadline_miss(void);
void            agent_stats_inc_wcet_violate(void);
void            agent_stats_inc_ai_inference(uint32_t latency_us);
void            agent_stats_inc_state_entry(agent_state_t state);
void            agent_stats_inc_imu_sample(void);
void            agent_stats_inc_imu_dropout(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_STATS_H */