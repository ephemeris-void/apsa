#ifndef AGENT_MODEL_MGR_H
#define AGENT_MODEL_MGR_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MODEL MANAGER — FROZEN
 * ============================================================
 * Multi-slot AI model management with CRC + hot-swap.
 * Enables A/B model testing and safe rollback.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 500 µs max per verify (boot only).
 * ============================================================ */

#define AGENT_MODEL_MAX_SLOTS           4U
#define AGENT_MODEL_VERSION_LEN         16U
#define AGENT_MODEL_NAME_LEN            32U

/* ---------- Slot IDs ---------- */
typedef enum {
    AGENT_MODEL_SLOT_NONE       = 0,
    AGENT_MODEL_SLOT_ACTIVE     = 1,
    AGENT_MODEL_SLOT_BACKUP     = 2,
    AGENT_MODEL_SLOT_FALLBACK   = 3,
    AGENT_MODEL_SLOT_TEST       = 4
} agent_model_slot_t;

/* ---------- Slot State ---------- */
typedef enum {
    AGENT_MODEL_STATE_EMPTY     = 0,
    AGENT_MODEL_STATE_LOADED    = 1,
    AGENT_MODEL_STATE_ACTIVE    = 2,
    AGENT_MODEL_STATE_INVALID   = 3,
    AGENT_MODEL_STATE_LOADING   = 4
} agent_model_state_t;

/* ---------- Slot Context ---------- */
typedef struct {
    agent_model_slot_t      slot;
    agent_model_state_t     state;
    char                    name[AGENT_MODEL_NAME_LEN];
    char                    version[AGENT_MODEL_VERSION_LEN];
    uint32_t                model_id;
    uint32_t                size_bytes;
    uint32_t                crc32;
    uint32_t                load_us;
    uint32_t                switch_count;
    bool                    verified;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_model_slot_ctx_t;

/* ---------- Manager Context ---------- */
typedef struct {
    agent_model_slot_ctx_t  slots[AGENT_MODEL_MAX_SLOTS];
    agent_model_slot_t      active_slot;
    agent_model_slot_t      fallback_slot;
    uint32_t                total_switches;
    uint32_t                total_loads;
    uint32_t                total_failures;
    uint32_t                last_switch_us;
    uint32_t                reserved[2];   /* ABI-safe future extension */
} agent_model_mgr_ctx_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_model_mgr_init(void);
agent_status_t                  agent_model_mgr_reset(void);

agent_status_t                  agent_model_mgr_register(agent_model_slot_t slot,
                                                        const char         *name,
                                                        const char         *version,
                                                        uint32_t            model_id,
                                                        uint32_t            size_bytes,
                                                        uint32_t            crc32);

agent_status_t                  agent_model_mgr_verify(agent_model_slot_t slot);
agent_status_t                  agent_model_mgr_activate(agent_model_slot_t slot);
agent_status_t                  agent_model_mgr_fallback(void);

agent_model_slot_t              agent_model_mgr_active(void);
agent_model_state_t             agent_model_mgr_state(agent_model_slot_t slot);

const agent_model_slot_ctx_t   *agent_model_mgr_slot(agent_model_slot_t slot);
const agent_model_mgr_ctx_t    *agent_model_mgr_ctx(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MODEL_MGR_H */