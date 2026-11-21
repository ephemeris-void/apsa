#include "agent_model_mgr.h"
#include "agent_model_mgr_version.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT MODEL MANAGER — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 500 µs per verify (boot only)
 * ============================================================ */

/* PERF: context in DTCM (zero wait state) */
static AGENT_DTCM agent_model_mgr_ctx_t g_mgr;
static bool                             g_mgr_initialized = false;

/* ---------- Safe string copy ---------- */
static void str_copy_safe(char *dst, const char *src, uint32_t max_len)
{
    if (dst == NULL || src == NULL || max_len == 0U) {
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
agent_status_t agent_model_mgr_init(void)
{
    memset(&g_mgr, 0, sizeof(g_mgr));    /* zero-init incl. reserved[] */

    for (uint32_t i = 0U; i < AGENT_MODEL_MAX_SLOTS; i++) {
        g_mgr.slots[i].slot         = (agent_model_slot_t)i;
        g_mgr.slots[i].state        = AGENT_MODEL_STATE_EMPTY;
        g_mgr.slots[i].model_id     = 0U;
        g_mgr.slots[i].size_bytes   = 0U;
        g_mgr.slots[i].crc32        = 0U;
        g_mgr.slots[i].load_us      = 0U;
        g_mgr.slots[i].switch_count = 0U;
        g_mgr.slots[i].verified     = false;
        g_mgr.slots[i].name[0]      = '\0';
        g_mgr.slots[i].version[0]   = '\0';
    }

    g_mgr.active_slot    = AGENT_MODEL_SLOT_NONE;
    g_mgr.fallback_slot  = AGENT_MODEL_SLOT_NONE;
    g_mgr.total_switches = 0U;
    g_mgr.total_loads    = 0U;
    g_mgr.total_failures = 0U;
    g_mgr.last_switch_us = 0U;

    agent_model_mgr_version_init();

    g_mgr_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_model_mgr_reset(void)
{
    g_mgr_initialized = false;
    return agent_model_mgr_init();
}

/* ---------- REGISTER ---------- */
agent_status_t agent_model_mgr_register(agent_model_slot_t slot,
                                        const char         *name,
                                        const char         *version,
                                        uint32_t            model_id,
                                        uint32_t            size_bytes,
                                        uint32_t            crc32)
{
    if (!g_mgr_initialized)                return AGENT_ERR_NOT_INIT;
    if (slot >= AGENT_MODEL_MAX_SLOTS)     return AGENT_ERR;

    agent_model_slot_ctx_t *s = &g_mgr.slots[slot];

    str_copy_safe(s->name, name, AGENT_MODEL_NAME_LEN);
    str_copy_safe(s->version, version, AGENT_MODEL_VERSION_LEN);

    s->model_id   = model_id;
    s->size_bytes = size_bytes;
    s->crc32      = crc32;
    s->state      = AGENT_MODEL_STATE_LOADED;
    s->verified   = false;

    g_mgr.total_loads++;

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)slot << 16) | 0xA001U,
                    model_id);

    return AGENT_OK;
}

/* ---------- VERIFY ---------- */
agent_status_t agent_model_mgr_verify(agent_model_slot_t slot)
{
    if (!g_mgr_initialized)                return AGENT_ERR_NOT_INIT;
    if (slot >= AGENT_MODEL_MAX_SLOTS)     return AGENT_ERR;

    agent_model_slot_ctx_t *s = &g_mgr.slots[slot];

    if (s->state == AGENT_MODEL_STATE_EMPTY) {
        return AGENT_ERR_INVALID_STATE;
    }

    uint32_t t0 = agent_time_now_us();

    uint32_t computed_crc = agent_model_mgr_version_crc32(s->model_id,
                                                          s->size_bytes);

    uint32_t t1 = agent_time_now_us();
    s->load_us = t1 - t0;

    if (computed_crc != s->crc32) {
        s->state    = AGENT_MODEL_STATE_INVALID;
        s->verified = false;
        g_mgr.total_failures++;
        agent_log_event(AGENT_LOG_AI_FAIL, (uint32_t)slot, computed_crc);
        return AGENT_ERR;
    }

    s->verified = true;
    s->state    = AGENT_MODEL_STATE_LOADED;

    return AGENT_OK;
}

/* ---------- ACTIVATE ---------- */
agent_status_t agent_model_mgr_activate(agent_model_slot_t slot)
{
    if (!g_mgr_initialized)                return AGENT_ERR_NOT_INIT;
    if (slot >= AGENT_MODEL_MAX_SLOTS)     return AGENT_ERR;

    agent_model_slot_ctx_t *s = &g_mgr.slots[slot];

    if (!s->verified) {
        return AGENT_ERR_INVALID_STATE;
    }

    agent_model_slot_t prev = g_mgr.active_slot;

    if (prev < AGENT_MODEL_MAX_SLOTS && prev != AGENT_MODEL_SLOT_NONE) {
        g_mgr.slots[prev].state = AGENT_MODEL_STATE_LOADED;
    }

    s->state = AGENT_MODEL_STATE_ACTIVE;
    s->switch_count++;

    g_mgr.active_slot    = slot;
    g_mgr.total_switches++;
    g_mgr.last_switch_us = agent_time_now_us();

    agent_log_event(AGENT_LOG_TRANSITION,
                    ((uint32_t)prev << 16) | (uint32_t)slot,
                    0xA002U);

    return AGENT_OK;
}

/* ---------- FALLBACK ---------- */
agent_status_t agent_model_mgr_fallback(void)
{
    if (g_mgr.fallback_slot == AGENT_MODEL_SLOT_NONE) {
        return AGENT_ERR_INVALID_STATE;
    }

    if (g_mgr.active_slot == g_mgr.fallback_slot) {
        return AGENT_ERR_INVALID_STATE;
    }

    agent_log_event(AGENT_LOG_AI_SWITCH_INPUT, 0xA003U, g_mgr.total_failures);

    return agent_model_mgr_activate(g_mgr.fallback_slot);
}

/* ---------- QUERIES ---------- */
agent_model_slot_t agent_model_mgr_active(void)
{
    return g_mgr.active_slot;
}

agent_model_state_t agent_model_mgr_state(agent_model_slot_t slot)
{
    if (slot >= AGENT_MODEL_MAX_SLOTS) {
        return AGENT_MODEL_STATE_INVALID;
    }
    return g_mgr.slots[slot].state;
}

const agent_model_slot_ctx_t *agent_model_mgr_slot(agent_model_slot_t slot)
{
    if (slot >= AGENT_MODEL_MAX_SLOTS) {
        return NULL;
    }
    return &g_mgr.slots[slot];
}

const agent_model_mgr_ctx_t *agent_model_mgr_ctx(void)
{
    return &g_mgr;
}