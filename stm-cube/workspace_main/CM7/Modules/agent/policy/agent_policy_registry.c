#include "agent_policy.h"
#include "agent_policy_registry.h"
#include "agent_policy_if.h"
#include "../core/agent_types.h"

/* ============================================================
 * AGENT POLICY REGISTRY — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 10 µs per register/lookup
 * ============================================================ */

/* PERF: registry in DTCM (zero wait state) */
static const agent_policy_desc_t *g_registry[AGENT_POLICY_REGISTRY_MAX];
static uint32_t                              g_registry_count = 0U;

/* ---------- RESET ---------- */
agent_status_t agent_policy_registry_reset(void)
{
    for (uint32_t i = 0U; i < AGENT_POLICY_REGISTRY_MAX; i++) {
        g_registry[i] = NULL;
    }
    g_registry_count = 0U;
    return AGENT_OK;
}

/* ---------- REGISTER ---------- */
agent_status_t agent_policy_registry_register(const agent_policy_desc_t *desc)
{
    if (desc == NULL || desc->eval == NULL) {
        return AGENT_ERR;
    }

    if (g_registry_count >= AGENT_POLICY_REGISTRY_MAX) {
        return AGENT_ERR;
    }

    /* Duplicate ID check */
    for (uint32_t i = 0U; i < g_registry_count; i++) {
        if (g_registry[i] != NULL && g_registry[i]->id == desc->id) {
            return AGENT_ERR;
        }
    }

    g_registry[g_registry_count] = desc;
    g_registry_count++;
    return AGENT_OK;
}

/* ---------- UNREGISTER ---------- */
agent_status_t agent_policy_registry_unregister(agent_policy_id_t id)
{
    for (uint32_t i = 0U; i < g_registry_count; i++) {

        if (g_registry[i] != NULL && g_registry[i]->id == id) {

            for (uint32_t j = i; j + 1U < g_registry_count; j++) {
                g_registry[j] = g_registry[j + 1U];
            }

            g_registry[g_registry_count - 1U] = NULL;
            g_registry_count--;
            return AGENT_OK;
        }
    }
    return AGENT_ERR;
}

/* ---------- COUNT ---------- */
uint32_t agent_policy_registry_count(void)
{
    return g_registry_count;
}

/* ---------- AT ---------- */
const agent_policy_desc_t *agent_policy_registry_at(uint32_t index)
{
    if (index >= g_registry_count) {
        return NULL;
    }
    return g_registry[index];
}

/* ---------- FIND ---------- */
const agent_policy_desc_t *agent_policy_registry_find(agent_policy_id_t id)
{
    for (uint32_t i = 0U; i < g_registry_count; i++) {
        if (g_registry[i] != NULL && g_registry[i]->id == id) {
            return g_registry[i];
        }
    }
    return NULL;
}

/* ---------- SORT BY PRIORITY (bubble sort — N ≤ 16) ---------- */
agent_status_t agent_policy_registry_sort_by_priority(void)
{
    for (uint32_t i = 0U; i < g_registry_count; i++) {
        for (uint32_t j = i + 1U; j < g_registry_count; j++) {

            if (g_registry[i] == NULL || g_registry[j] == NULL) {
                continue;
            }

            if (g_registry[j]->priority_level > g_registry[i]->priority_level) {
                const agent_policy_desc_t *tmp = g_registry[i];
                g_registry[i] = g_registry[j];
                g_registry[j] = tmp;
            }
        }
    }
    return AGENT_OK;
    }
