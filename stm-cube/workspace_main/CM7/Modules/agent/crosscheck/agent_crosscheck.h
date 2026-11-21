#ifndef AGENT_CROSSCHECK_H
#define AGENT_CROSSCHECK_H

#include "../core/agent_types.h"
#include "../core/agent_config.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT CROSSCHECK — FROZEN
 * ============================================================
 * Multi-channel cross-validation via median + voting.
 * Detects outlier channels (spike/drift/stuck).
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 30 µs max per run.
 * ============================================================ */

#define AGENT_CROSSCHECK_MAX_CHANNELS   4U
#define AGENT_CROSSCHECK_DIVERGE_LSB    500
#define AGENT_CROSSCHECK_VOTE_MIN       2U

/* ---------- Result ---------- */
typedef enum {
    AGENT_CROSSCHECK_OK        = 0,
    AGENT_CROSSCHECK_DIVERGED  = 1,
    AGENT_CROSSCHECK_FAIL      = 2
} agent_crosscheck_result_t;

/* ---------- Input ---------- */
typedef struct {
    int16_t  ax[AGENT_CROSSCHECK_MAX_CHANNELS];
    int16_t  ay[AGENT_CROSSCHECK_MAX_CHANNELS];
    int16_t  az[AGENT_CROSSCHECK_MAX_CHANNELS];
    int16_t  gx[AGENT_CROSSCHECK_MAX_CHANNELS];
    int16_t  gy[AGENT_CROSSCHECK_MAX_CHANNELS];
    int16_t  gz[AGENT_CROSSCHECK_MAX_CHANNELS];
    uint32_t count;
    uint32_t timestamp_us;
    uint32_t reserved[2];   /* ABI-safe future extension */
} agent_crosscheck_input_t;

/* ---------- Output ---------- */
typedef struct {
    int16_t  ax;
    int16_t  ay;
    int16_t  az;
    int16_t  gx;
    int16_t  gy;
    int16_t  gz;
    uint32_t outlier_mask;
    uint32_t valid_mask;
    uint32_t timestamp_us;
    uint32_t reserved[2];   /* ABI-safe future extension */
} agent_crosscheck_output_t;

/* ---------- Stats ---------- */
typedef struct {
    uint32_t  divergence_count;
    uint32_t  outlier_count;
    uint32_t  last_check_us;
    int16_t   last_max_delta;
    uint32_t  reserved[2];
} agent_crosscheck_stats_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_crosscheck_init(void);
agent_status_t                  agent_crosscheck_reset(void);

agent_crosscheck_result_t       agent_crosscheck_run(const agent_crosscheck_input_t *in,
                                                     agent_crosscheck_output_t *out);

const agent_crosscheck_stats_t *agent_crosscheck_stats(void);

bool                            agent_crosscheck_is_outlier(uint32_t mask, uint32_t channel);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_CROSSCHECK_H */