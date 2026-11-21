#ifndef AGENT_MODES_H
#define AGENT_MODES_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MODES — FROZEN
 * ============================================================
 * Multi-mode arbitration with dwell-time anti-chatter.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 30 µs max per update.
 * ============================================================ */

/* ---------- Mode IDs ---------- */
typedef enum {
    AGENT_MODE_ID_NONE        = 0x00,
    AGENT_MODE_ID_NOMINAL     = 0x01,
    AGENT_MODE_ID_DEGRADED    = 0x02,
    AGENT_MODE_ID_VIRTUAL     = 0x03,
    AGENT_MODE_ID_FUSION      = 0x04,
    AGENT_MODE_ID_RECOVERY    = 0x05,
    AGENT_MODE_ID_EMERGENCY   = 0x06,
    AGENT_MODE_ID_LANDING     = 0x07,
    AGENT_MODE_ID_COUNT       = 8
} agent_mode_id_t;

/* ---------- Mode Kind ---------- */
typedef enum {
    AGENT_MODE_KIND_NONE      = 0,
    AGENT_MODE_KIND_NORMAL    = 1,
    AGENT_MODE_KIND_SAFETY    = 2,
    AGENT_MODE_KIND_DEGRADED  = 3,
    AGENT_MODE_KIND_CRITICAL  = 4
} agent_mode_kind_t;

/* ---------- Mode Descriptor ---------- */
typedef struct {
    agent_mode_id_t    id;
    agent_mode_kind_t  kind;
    agent_state_t      state;
    agent_priority_t   priority;
    uint32_t           flags;
    uint32_t           min_dwell_us;
} agent_mode_desc_t;

/* ---------- Mode State ---------- */
typedef struct {
    agent_mode_id_t    active;
    agent_mode_id_t    pending;
    agent_mode_id_t    previous;
    uint32_t           entry_us;
    uint32_t           dwell_us;
    uint32_t           switch_count;
    bool               locked;
} agent_mode_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_modes_init(void);
agent_status_t              agent_modes_reset(void);
agent_status_t              agent_modes_update(void);

agent_mode_id_t             agent_modes_active(void);
agent_mode_id_t             agent_modes_pending(void);

agent_status_t              agent_modes_request(agent_mode_id_t id);

const agent_mode_desc_t    *agent_modes_desc(agent_mode_id_t id);
const agent_mode_state_t   *agent_modes_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MODES_H */