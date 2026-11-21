#ifndef AGENT_IPC_H
#define AGENT_IPC_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT IPC — FROZEN
 * ============================================================
 * Lock-free ring buffer for CM7 ↔ CM4 messaging.
 * Hardware-backed via HSEM (dual-core) or stub (single-core).
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 20 µs max per send/receive.
 * ============================================================ */

#define AGENT_IPC_RING_SIZE         64U
#define AGENT_IPC_TIMEOUT_US        1000U
#define AGENT_IPC_HEARTBEAT_US      100U
#define AGENT_IPC_MAX_PAYLOAD       32U

/* ---------- Message IDs ---------- */
typedef enum {
    AGENT_IPC_MSG_NONE          = 0x0000,
    AGENT_IPC_MSG_EMERGENCY     = 0x0001,
    AGENT_IPC_MSG_GROUND_ALERT  = 0x0002,
    AGENT_IPC_MSG_HEARTBEAT     = 0x0003,
    AGENT_IPC_MSG_STATE_SYNC    = 0x0004,
    AGENT_IPC_MSG_SENSOR_DATA   = 0x0005,
    AGENT_IPC_MSG_FAULT_REPORT  = 0x0006,
    AGENT_IPC_MSG_CMD_REQUEST   = 0x0007,
    AGENT_IPC_MSG_CMD_RESPONSE  = 0x0008
} agent_ipc_msg_id_t;

/* ---------- Message ---------- */
typedef struct {
    uint32_t    msg_id;
    uint32_t    arg1;
    uint32_t    arg2;
    uint32_t    timestamp_us;
    uint8_t     payload[AGENT_IPC_MAX_PAYLOAD];
    uint8_t     payload_size;
    uint8_t     reserved[3];
} agent_ipc_msg_t;

/* ---------- Ring ---------- */
typedef struct {
    agent_ipc_msg_t   ring[AGENT_IPC_RING_SIZE];
    volatile uint32_t head;
    volatile uint32_t tail;
    volatile uint32_t count;
    uint32_t          drop_count;
    uint32_t          last_tx_us;
    uint32_t          last_rx_us;
} agent_ipc_ring_t;

/* ---------- State ---------- */
typedef struct {
    agent_ipc_ring_t tx;
    agent_ipc_ring_t rx;
    uint32_t         tx_total;
    uint32_t         rx_total;
    uint32_t         tx_errors;
    uint32_t         rx_errors;
    uint32_t         timeout_count;
    uint32_t         last_heartbeat_us;
    uint32_t         reserved[2];
} agent_ipc_state_t;

/* ---------- API (FROZEN) ---------- */
agent_status_t                  agent_ipc_init(void);
agent_status_t                  agent_ipc_reset(void);
agent_status_t                  agent_ipc_update(void);

agent_status_t                  agent_ipc_send(uint32_t msg_id,
                                               uint32_t arg1,
                                               uint32_t arg2);

agent_status_t                  agent_ipc_send_payload(uint32_t       msg_id,
                                                       const uint8_t *payload,
                                                       uint8_t        payload_size);

agent_status_t                  agent_ipc_receive(agent_ipc_msg_t *out);

bool                            agent_ipc_has_message(void);

agent_status_t                  agent_ipc_heartbeat(void);
bool                            agent_ipc_is_alive(void);

const agent_ipc_state_t        *agent_ipc_state(void);

void                            agent_ipc_on_timeout(void);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_IPC_H */