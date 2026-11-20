/* ============================================================
 * HAL IPC — INTER-PROCESSOR COMMUNICATION (CM7 ↔ CM4)
 * ============================================================
 * - Shared memory ring buffer with CRC32
 * - Zephyr k_mutex + HSEM hardware lock
 * - Coherency-safe (uses hal_cache)
 * - Zero-copy where possible
 * - Timeout-bounded, no deadlock
 * - Production-grade: validates every transfer
 * ============================================================ */

#ifndef HAL_IPC_H
#define HAL_IPC_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "project_config.h"
#include "validation.h"
#include "system_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ---------- IPC Channel IDs ---------- */
typedef enum {
    HAL_IPC_CH_CM7_TO_CM4 = 0,
    HAL_IPC_CH_CM4_TO_CM7 = 1,
    HAL_IPC_CH_COUNT      = 2
} hal_ipc_channel_t;

/* ---------- Init ---------- */
proj_status_t hal_ipc_init(void);

/* ---------- Send (blocking with timeout) ---------- */
proj_status_t hal_ipc_send(hal_ipc_channel_t ch,
                           const void        *data,
                           size_t             size,
                           uint32_t           timeout_us);

/* ---------- Receive (blocking with timeout) ---------- */
proj_status_t hal_ipc_recv(hal_ipc_channel_t ch,
                           void              *buf,
                           size_t             buf_size,
                           size_t            *received_size,
                           uint32_t           timeout_us);

/* ---------- Status Query ---------- */
uint32_t hal_ipc_pending(hal_ipc_channel_t ch);
uint32_t hal_ipc_overflow_count(hal_ipc_channel_t ch);

/* ---------- Peer Liveness ---------- */
bool hal_ipc_peer_alive(void);

/* ---------- Shared Status Block ---------- */
const ipc_status_t *hal_ipc_status(void);

/* ============================================================
 * HSEM LOCK / UNLOCK — Hardware Semaphore
 * ============================================================
 * Bounded wait (1 ms timeout) to prevent deadlock.
 *
 * Returns:
 *   PROJ_OK          — lock acquired
 *   PROJ_ERR_TIMEOUT — peer holds lock too long
 *   PROJ_ERR_INVALID — bad hsem_id
 * ============================================================ */
proj_status_t hal_ipc_lock(uint32_t hsem_id);
proj_status_t hal_ipc_unlock(uint32_t hsem_id);

#ifdef __cplusplus
}
#endif

#endif /* HAL_IPC_H */