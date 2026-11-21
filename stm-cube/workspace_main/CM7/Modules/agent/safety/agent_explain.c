#include "agent_explain.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT EXPLAIN — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per record
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_explain_state_t g_expl;
static bool                             g_expl_initialized = false;

/* ---------- Safe string copy ---------- */
static void copy_text(char *dst, const char *src, uint32_t max_len)
{
    if (dst == NULL) {
        return;
    }

    if (src == NULL) {
        dst[0] = '\0';
        return;
    }

    uint32_t i = 0U;

    while (i < (max_len - 1U) && src[i] != '\0') {
        dst[i] = src[i];
        i++;
    }

    dst[i] = '\0';
}

/* ---------- INIT ---------- */
agent_status_t agent_explain_init(void)
{
    memset(&g_expl, 0, sizeof(g_expl));    /* zero-init incl. reserved[] */

    g_expl.head          = 0U;
    g_expl.count         = 0U;
    g_expl.total_entries = 0U;
    g_expl.last_us       = 0U;

    g_expl_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_explain_reset(void)
{
    g_expl_initialized = false;
    return agent_explain_init();
}

/* ---------- RECORD ---------- */
agent_status_t agent_explain_record(agent_explain_reason_t reason,
                                    uint32_t                from_state,
                                    uint32_t                to_state,
                                    const char             *text)
{
    if (!g_expl_initialized) {
        return AGENT_ERR_NOT_INIT;
    }

    uint32_t idx = g_expl.head;
    agent_explain_entry_t *e = &g_expl.entries[idx];

    memset(e, 0, sizeof(*e));    /* zero-init incl. reserved[] */

    e->timestamp_us = agent_time_now_us();
    e->reason       = reason;
    e->from_state   = from_state;
    e->to_state     = to_state;
    e->priority     = (uint32_t)agent_context_get()->current_priority;
    e->confidence   = agent_confidence_get();

    copy_text(e->text, text, AGENT_EXPLAIN_TEXT_LEN);

    g_expl.head = (idx + 1U) % AGENT_EXPLAIN_MAX_ENTRIES;
    if (g_expl.count < AGENT_EXPLAIN_MAX_ENTRIES) {
        g_expl.count++;
    }
    g_expl.total_entries++;
    g_expl.last_us = e->timestamp_us;

    return AGENT_OK;
}

/* ---------- LAST ---------- */
const agent_explain_entry_t *agent_explain_last(void)
{
    if (g_expl.count == 0U) {
        return NULL;
    }

    uint32_t idx = (g_expl.head == 0U)
                   ? (AGENT_EXPLAIN_MAX_ENTRIES - 1U)
                   : (g_expl.head - 1U);

    return &g_expl.entries[idx];
}

/* ---------- GET ---------- */
const agent_explain_entry_t *agent_explain_get(uint32_t idx)
{
    if (idx >= g_expl.count) {
        return NULL;
    }
    return &g_expl.entries[idx];
}

/* ---------- COUNT ---------- */
uint32_t agent_explain_count(void)
{
    return g_expl.count;
}

/* ---------- CLEAR ---------- */
void agent_explain_clear(void)
{
    g_expl.head  = 0U;
    g_expl.count = 0U;
}

/* ---------- STATE ---------- */
const agent_explain_state_t *agent_explain_state(void)
{
    return &g_expl;
}