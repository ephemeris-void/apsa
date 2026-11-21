#include "agent_postevent.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT POST-EVENT — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per record
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_postevent_state_t g_post;
static bool                               g_post_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_postevent_init(void)
{
    memset(&g_post, 0, sizeof(g_post));    /* zero-init incl. reserved[] */

    g_post.head          = 0U;
    g_post.count         = 0U;
    g_post.total_events  = 0U;
    g_post.last_event_us = 0U;

    g_post_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_postevent_reset(void)
{
    g_post_initialized = false;
    return agent_postevent_init();
}

/* ---------- RECORD ---------- */
agent_status_t agent_postevent_record(agent_postevent_type_t type)
{
    if (!g_post_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    agent_context_full_t *ctx = agent_context_get();

    uint32_t idx = g_post.head;
    agent_postevent_record_t *r = &g_post.records[idx];

    memset(r, 0, sizeof(*r));    /* zero-init incl. reserved[] */

    r->type             = type;
    r->timestamp_us     = agent_time_now_us();
    r->iteration        = ctx->iteration_count;
    r->state            = (uint32_t)ctx->current_state;
    r->priority         = (uint32_t)ctx->current_priority;
    r->fault_count      = ctx->fault_count;
    r->recovery_count   = ctx->recovery_count;
    r->confidence_milli = (uint32_t)(ctx->confidence * 1000.0f);

    /* Snapshot — reserved for extended diagnostic capture */
    for (uint32_t i = 0U; i < AGENT_POSTEVENT_SNAPSHOT_SIZE; i++) {
        r->snapshot[i] = 0U;
    }

    g_post.head = (idx + 1U) % AGENT_POSTEVENT_MAX_RECORDS;
    if (g_post.count < AGENT_POSTEVENT_MAX_RECORDS) {
        g_post.count++;
    }
    g_post.total_events++;
    g_post.last_event_us = r->timestamp_us;

    agent_log_event(AGENT_LOG_PERSIST,
                    (uint32_t)type,
                    g_post.total_events);

    return AGENT_OK;
}

/* ---------- LAST ---------- */
const agent_postevent_record_t *agent_postevent_last(void)
{
    if (g_post.count == 0U) {
        return NULL;
    }

    uint32_t idx = (g_post.head == 0U)
                   ? (AGENT_POSTEVENT_MAX_RECORDS - 1U)
                   : (g_post.head - 1U);

    return &g_post.records[idx];
}

/* ---------- GET ---------- */
const agent_postevent_record_t *agent_postevent_get(uint32_t idx)
{
    if (idx >= g_post.count) {
        return NULL;
    }
    return &g_post.records[idx];
}

/* ---------- COUNT ---------- */
uint32_t agent_postevent_count(void)
{
    return g_post.count;
}

/* ---------- CLEAR ---------- */
void agent_postevent_clear(void)
{
    g_post.head  = 0U;
    g_post.count = 0U;
}

/* ---------- STATE ---------- */
const agent_postevent_state_t *agent_postevent_state(void)
{
    return &g_post;
}