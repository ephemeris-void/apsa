#ifndef AGENT_CONFIDENCE_H
#define AGENT_CONFIDENCE_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT CONFIDENCE — FROZEN
 * ============================================================
 * Weighted multi-source confidence fusion.
 * Drives virtual sensor trust, mode switching, safety escalation.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per update.
 * ============================================================ */

#define AGENT_CONFIDENCE_SOURCES_MAX   8U
#define AGENT_CONFIDENCE_SMOOTH_ALPHA  0.05f
#define AGENT_CONFIDENCE_SCALE         1000U

/* ---------- Confidence Sources ---------- */
typedef enum {
    AGENT_CONFIDENCE_SRC_IMU       = 0,
    AGENT_CONFIDENCE_SRC_AI        = 1,
    AGENT_CONFIDENCE_SRC_EKF       = 2,
    AGENT_CONFIDENCE_SRC_BARO      = 3,
    AGENT_CONFIDENCE_SRC_GPS       = 4,
    AGENT_CONFIDENCE_SRC_MAG       = 5,
    AGENT_CONFIDENCE_SRC_VOTING    = 6,
    AGENT_CONFIDENCE_SRC_CROSSCHK  = 7,
    AGENT_CONFIDENCE_SRC_COUNT     = 8
} agent_confidence_src_t;

/* ---------- Input ---------- */
typedef struct {
    float    weight[AGENT_CONFIDENCE_SRC_COUNT];
    float    value[AGENT_CONFIDENCE_SRC_COUNT];
    bool     valid[AGENT_CONFIDENCE_SRC_COUNT];
    uint32_t timestamp_us[AGENT_CONFIDENCE_SRC_COUNT];
    uint32_t reserved[2];
} agent_confidence_input_t;

/* ---------- Output ---------- */
typedef struct {
    float    combined;
    float    weighted;
    float    min_value;
    float    max_value;
    uint32_t valid_count;
    uint32_t timestamp_us;
    uint32_t reserved[2];
} agent_confidence_output_t;

/* ---------- State ---------- */
typedef struct {
    float    current;
    float    min_ever;
    float    max_ever;
    uint32_t update_count;
    uint32_t last_update_us;
    uint32_t reserved[2];
} agent_confidence_state_t;

/* ============================================================
 * PUBLIC API (FROZEN)
 * ============================================================ */

/* ---------- Init / Reset ---------- */
agent_status_t   agent_confidence_init(void);
agent_status_t   agent_confidence_reset(void);

/* ---------- SINGLE-VALUE SET/GET (used by agent_states) ---------- */
void             agent_confidence_set(float c);
float            agent_confidence_get(void);

/* ---------- Per-Source Management ---------- */
void             agent_confidence_set_source(agent_confidence_src_t src,
                                             float                  value,
                                             float                  weight);
void             agent_confidence_invalidate(agent_confidence_src_t src);

/* ---------- Multi-Source Fusion Update ---------- */
agent_status_t   agent_confidence_update(const agent_confidence_input_t  *in,
                                         agent_confidence_output_t       *out);

/* ---------- Queries ---------- */
float            agent_confidence_get_min(void);
float            agent_confidence_get_max(void);

const agent_confidence_state_t *agent_confidence_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_CONFIDENCE_H */