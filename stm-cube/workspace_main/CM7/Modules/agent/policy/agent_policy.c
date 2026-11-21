#include "agent_policy.h"
#include "agent_policy_registry.h"
#include "agent_policy_arbiter.h"
#include "agent_policy_safety.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../states/agent_transitions.h"
#include "../trace/agent_trace.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POLICY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 50 µs per arbitrate
 * ============================================================ */

/* ---------- COLLECT ---------- */
agent_status_t agent_policy_collect(agent_policy_candidate_t *out,
                                    uint32_t                  max,
                                    uint32_t                 *count_out)
{
    if (out == NULL || count_out == NULL || max == 0U) {
        return AGENT_ERR;
    }

    uint32_t written = 0U;
    uint32_t total   = agent_policy_registry_count();

    for (uint32_t i = 0U; i < total && written < max; i++) {

        const agent_policy_desc_t *desc = agent_policy_registry_at(i);
        if (desc == NULL || desc->eval == NULL) {
            continue;
        }

        agent_policy_output_t po;
        po.result           = AGENT_POLICY_FAIL;
        po.priority         = AGENT_PRIO_LOW;
        po.suggested_state  = AGENT_STATE_MONITOR;
        po.confidence_milli = 0U;
        po.reason_code      = (uint32_t)AGENT_REASON_NONE;
        po.timestamp_us     = 0U;
        po.wcet_us          = 0U;

        uint32_t t0 = agent_time_now_us();

        agent_status_t st = desc->eval(&po);

        uint32_t t1 = agent_time_now_us();
        uint32_t dt = t1 - t0;

        if (st != AGENT_OK) {
            agent_log_event(AGENT_LOG_AI_FAIL, (uint32_t)desc->id, dt);
            continue;
        }

        if (desc->wcet_budget_us != 0U && dt > desc->wcet_budget_us) {
            agent_log_event(AGENT_LOG_WCET_VIOLATE, (uint32_t)desc->id, dt);
            continue;
        }

        agent_policy_candidate_t *c = &out[written];

        memset(c, 0, sizeof(*c));    /* zero-init including reserved[] */

        c->id                = desc->id;
        c->priority          = po.priority;
        c->target_state      = po.suggested_state;
        c->reason            = (agent_reason_t)po.reason_code;
        c->confidence_milli  = po.confidence_milli;
        c->timestamp_us      = (po.timestamp_us != 0U) ? po.timestamp_us : t1;
        c->wcet_us           = dt;

        written++;
    }

    *count_out = written;
    return AGENT_OK;
}

/* ---------- ARBITRATE ---------- */
agent_status_t agent_policy_arbitrate(const agent_policy_candidate_t *in,
                                      uint32_t                        count,
                                      agent_policy_candidate_t       *winner_out)
{
    if (in == NULL || winner_out == NULL || count == 0U) {
        return AGENT_ERR;
    }

    agent_policy_candidate_t best = in[0];

    for (uint32_t i = 1U; i < count; i++) {

        const agent_policy_candidate_t *c = &in[i];

        /* Primary: priority */
        if (c->priority > best.priority) {
            best = *c;
            continue;
        }

        if (c->priority == best.priority) {

            /* Secondary: confidence */
            if (c->confidence_milli > best.confidence_milli) {
                best = *c;
                continue;
            }

            if (c->confidence_milli == best.confidence_milli) {

                /* Tertiary: reason code */
                if ((uint32_t)c->reason > (uint32_t)best.reason) {
                    best = *c;
                    continue;
                }

                if ((uint32_t)c->reason == (uint32_t)best.reason) {

                    /* Quaternary: newest timestamp */
                    if (c->timestamp_us > best.timestamp_us) {
                        best = *c;
                    }
                }
            }
        }
    }

    *winner_out = best;
    return AGENT_OK;
}

/* ---------- APPLY ---------- */
agent_status_t agent_policy_apply(const agent_policy_candidate_t *winner)
{
    if (winner == NULL) {
        return AGENT_ERR;
    }

    agent_context_full_t *ctx = agent_context_get();

    ctx->current_priority = winner->priority;

    agent_state_t cur = ctx->current_state;

    if (winner->target_state != cur) {
        agent_status_t st = agent_transition_to(winner->target_state,
                                                (uint32_t)winner->reason);
        if (st != AGENT_OK) {
            agent_log_event(AGENT_LOG_TRANSITION_BLOCKED,
                            ((uint32_t)cur << 16) | (uint32_t)winner->target_state,
                            (uint32_t)winner->reason);
            return st;
        }
    }

    return AGENT_OK;
}

/* ---------- SAFETY FILTER ---------- */
agent_status_t agent_policy_safety_filter(agent_policy_candidate_t *candidate)
{
    if (candidate == NULL) {
        return AGENT_ERR;
    }

    agent_health_t sys = agent_context_get()->system_health;

    /* System failure → force EMERGENCY (unless already EMERGENCY/LANDING) */
    if (sys == AGENT_HEALTH_FAIL &&
        candidate->target_state != AGENT_STATE_EMERGENCY &&
        candidate->target_state != AGENT_STATE_LANDING) {

        candidate->priority         = AGENT_PRIO_CRITICAL;
        candidate->target_state     = AGENT_STATE_EMERGENCY;
        candidate->reason           = AGENT_REASON_ENVELOPE;
        candidate->confidence_milli = 1000U;
        return AGENT_OK;
    }

    /* Low confidence + FUSION → VIRTUALIZE */
    if (candidate->confidence_milli < (uint32_t)(AGENT_CONFIDENCE_MIN * 1000.0f) &&
        candidate->target_state == AGENT_STATE_FUSION) {

        candidate->target_state = AGENT_STATE_VIRTUALIZE;
        candidate->reason       = AGENT_REASON_CONFIDENCE;
        return AGENT_OK;
    }

    return AGENT_OK;
}