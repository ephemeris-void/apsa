#ifndef AGENT_PERSIST_H
#define AGENT_PERSIST_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT PERSIST — FROZEN
 * ============================================================
 * Flash-backed persistent state across resets.
 * CRC32-verified, magic-guarded, boot-count tracked.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 500 µs max per save/load (boot only).
 * ============================================================ */

#define AGENT_PERSIST_MAGIC         0xA5A55A5AU
#define AGENT_PERSIST_VERSION       1U
#define AGENT_PERSIST_SLOT_ADDR     0x080E0000U   /* flash sector 7 */
#define AGENT_PERSIST_SLOT_SIZE     0x20000U      /* 128 KB */
#define AGENT_PERSIST_MAX_ENTRIES   8U

/* ---------- Result ---------- */
typedef enum {
    AGENT_PERSIST_RESULT_OK      = 0,
    AGENT_PERSIST_RESULT_EMPTY   = 1,
    AGENT_PERSIST_RESULT_CORRUPT = 2,
    AGENT_PERSIST_RESULT_FAIL    = 3,
    AGENT_PERSIST_RESULT_WRITE   = 4
} agent_persist_result_t;

/* ---------- Block ---------- */
typedef struct {
    uint32_t    magic;
    uint32_t    version;
    uint32_t    crc;
    uint32_t    timestamp_us;
    uint32_t    fault_count;
    uint32_t    recovery_count;
    uint32_t    last_state;
    uint32_t    last_priority;
    uint32_t    boot_count;
    uint32_t    reserved[7];
} agent_persist_block_t;

/* ---------- State ---------- */
typedef struct {
    agent_persist_block_t  block;
    agent_persist_result_t last_result;
    uint32_t               write_count;
    uint32_t               read_count;
    uint32_t               crc_errors;
    uint32_t               last_write_us;
    bool                   initialized;
    uint32_t               reserved[2];   /* ABI-safe future extension */
} agent_persist_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t              agent_persist_init(void);
agent_status_t              agent_persist_reset(void);

agent_persist_result_t      agent_persist_load(void);
agent_persist_result_t      agent_persist_save(void);
agent_persist_result_t      agent_persist_erase(void);

uint32_t                    agent_persist_crc32(const uint8_t *data, uint32_t len);

const agent_persist_state_t *agent_persist_state(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_PERSIST_H */