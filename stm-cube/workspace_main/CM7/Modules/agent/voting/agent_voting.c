#include "agent_voting.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT VOTING — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 30 µs per run
 * ============================================================ */

/* PERF: stats in DTCM (zero wait state) */
static AGENT_DTCM agent_voting_stats_t g_stats;

/* ---------- INIT ---------- */
agent_status_t agent_voting_init(void)
{
    memset(&g_stats, 0, sizeof(g_stats));    /* zero-init incl. reserved[] */
    g_stats.last_vote_us = agent_time_now_us();
    return AGENT_OK;
}

agent_status_t agent_voting_reset(void)
{
    return agent_voting_init();
}

/* ---------- Find best matching group ---------- */
static uint32_t find_matching_group(const agent_voting_ballot_t *b,
                                    uint32_t n,
                                    uint32_t seed,
                                    int32_t tolerance,
                                    uint32_t *mask_out)
{
    int32_t ref = b[seed].value;
    uint32_t mask  = 0U;
    uint32_t count = 0U;

    for (uint32_t i = 0U; i < n; i++) {

        if (!b[i].valid) continue;

        int32_t d = b[i].value - ref;
        if (d < 0) d = -d;

        if (d <= tolerance) {
            mask |= (1UL << i);
            count++;
        }
    }

    *mask_out = mask;
    return count;
}

/* ---------- Majority vote ---------- */
agent_voting_result_t agent_voting_run(const agent_voting_input_t *in,
                                       agent_voting_output_t *out)
{
    if (in == NULL || out == NULL) {
        return AGENT_VOTING_FAIL;
    }

    uint32_t n = in->voter_count;

    if (n == 0U || n > AGENT_VOTING_MAX_VOTERS) {
        return AGENT_VOTING_FAIL;
    }

    int32_t  tolerance  = 100;
    uint32_t best_mask  = 0U;
    uint32_t best_count = 0U;

    for (uint32_t i = 0U; i < n; i++) {
        if (!in->ballots[i].valid) continue;

        uint32_t mask = 0U;
        uint32_t cnt  = find_matching_group(in->ballots, n, i, tolerance, &mask);

        if (cnt > best_count) {
            best_count = cnt;
            best_mask  = mask;
        }
    }

    int64_t  sum       = 0;
    uint32_t sum_count = 0;

    for (uint32_t i = 0U; i < n; i++) {
        if (best_mask & (1UL << i)) {
            sum += in->ballots[i].value;
            sum_count++;
        }
    }

    int32_t agreed = (sum_count > 0U)
                     ? (int32_t)(sum / (int64_t)sum_count)
                     : 0;

    memset(out, 0, sizeof(*out));    /* zero-init incl. reserved[] */

    out->agreed_value = agreed;
    out->agree_count  = best_count;
    out->total_count  = n;
    out->agree_mask   = best_mask;
    out->dissent_mask = ((1UL << n) - 1UL) & (~best_mask);
    out->kind         = in->kind;
    out->timestamp_us = in->timestamp_us;
    out->confidence   = (float)best_count / (float)n;

    g_stats.total_votes++;
    g_stats.last_vote_us = agent_time_now_us();

    if (best_count == n) {
        out->result = AGENT_VOTING_ALL_AGREE;
        g_stats.unanimous_votes++;
        return AGENT_VOTING_ALL_AGREE;
    }

    if (best_count >= AGENT_VOTING_MAJORITY) {
        out->result = AGENT_VOTING_OK;
        g_stats.majority_votes++;
        return AGENT_VOTING_OK;
    }

    out->result = AGENT_VOTING_NO_MAJORITY;
    g_stats.no_majority_votes++;
    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, best_count, n);
    return AGENT_VOTING_NO_MAJORITY;
}

/* ---------- Weighted vote ---------- */
agent_voting_result_t agent_voting_run_weighted(const agent_voting_input_t *in,
                                                agent_voting_output_t *out)
{
    if (in == NULL || out == NULL) {
        return AGENT_VOTING_FAIL;
    }

    uint32_t n = in->voter_count;

    if (n == 0U || n > AGENT_VOTING_MAX_VOTERS) {
        return AGENT_VOTING_FAIL;
    }

    int32_t  tolerance   = 100;
    float    best_weight = 0.0f;
    uint32_t best_mask   = 0U;
    uint32_t best_count  = 0U;
    float    total_weight = 0.0f;

    for (uint32_t i = 0U; i < n; i++) {
        if (!in->ballots[i].valid) continue;
        total_weight += in->ballots[i].weight;
    }

    for (uint32_t i = 0U; i < n; i++) {
        if (!in->ballots[i].valid) continue;

        int32_t ref  = in->ballots[i].value;
        float   wsum = 0.0f;
        uint32_t mask = 0U;
        uint32_t cnt  = 0U;

        for (uint32_t j = 0U; j < n; j++) {
            if (!in->ballots[j].valid) continue;

            int32_t d = in->ballots[j].value - ref;
            if (d < 0) d = -d;

            if (d <= tolerance) {
                mask |= (1UL << j);
                wsum += in->ballots[j].weight;
                cnt++;
            }
        }

        if (wsum > best_weight) {
            best_weight = wsum;
            best_mask   = mask;
            best_count  = cnt;
        }
    }

    int64_t sum    = 0;
    float   wtotal = 0.0f;

    for (uint32_t i = 0U; i < n; i++) {
        if (best_mask & (1UL << i)) {
            sum    += (int64_t)in->ballots[i].value *
                      (int64_t)(in->ballots[i].weight * 1000.0f);
            wtotal += in->ballots[i].weight;
        }
    }

    int32_t agreed = (wtotal > 0.0f)
                     ? (int32_t)(sum / (int64_t)(wtotal * 1000.0f))
                     : 0;

    memset(out, 0, sizeof(*out));    /* zero-init incl. reserved[] */

    out->agreed_value = agreed;
    out->agree_count  = best_count;
    out->total_count  = n;
    out->agree_mask   = best_mask;
    out->dissent_mask = ((1UL << n) - 1UL) & (~best_mask);
    out->kind         = AGENT_VOTING_KIND_WEIGHTED;
    out->confidence   = (total_weight > 0.0f) ? (best_weight / total_weight) : 0.0f;
    out->timestamp_us = in->timestamp_us;

    g_stats.total_votes++;
    g_stats.last_vote_us = agent_time_now_us();

    if (best_count == n) {
        out->result = AGENT_VOTING_ALL_AGREE;
        g_stats.unanimous_votes++;
        return AGENT_VOTING_ALL_AGREE;
    }

    if (best_count >= AGENT_VOTING_MAJORITY) {
        out->result = AGENT_VOTING_OK;
        g_stats.majority_votes++;
        return AGENT_VOTING_OK;
    }

    out->result = AGENT_VOTING_NO_MAJORITY;
    g_stats.no_majority_votes++;
    agent_log_event(AGENT_LOG_SAFETY_ESCALATE, best_count, n);
    return AGENT_VOTING_NO_MAJORITY;
}

/* ---------- IS AGREED ---------- */
bool agent_voting_is_agreed(uint32_t mask, uint32_t voter)
{
    return (mask & (1UL << voter)) != 0U;
}

/* ---------- STATS ---------- */
const agent_voting_stats_t *agent_voting_stats(void)
{
    return &g_stats;
}