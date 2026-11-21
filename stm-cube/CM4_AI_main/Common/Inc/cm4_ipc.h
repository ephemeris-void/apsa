#ifndef CM4_IPC_H
#define CM4_IPC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * CM4 <-> CM7 IPC — Shared Memory Protocol
 * ============================================================
 * Region:  SRAM4 @ 0x38000000, 64 KB
 * Pattern: Lock-free SPSC ring + HSEM + IPCC
 * WCET:    < 100 us per transfer
 * ============================================================ */

#define CM4_IPC_MAGIC           0xCAFEBABEUL
#define CM4_IPC_VERSION         1U

#define CM4_IPC_AI_CHANNELS     4U
#define CM4_IPC_AI_SAMPLES      512U
#define CM4_IPC_AI_BYTES        (CM4_IPC_AI_CHANNELS * CM4_IPC_AI_SAMPLES)

#define CM4_IPC_RING_SIZE       8U

/* ---------- Message Types ---------- */
typedef enum {
    CM4_IPC_MSG_NONE         = 0x0000,
    CM4_IPC_MSG_AI_INPUT     = 0x0001,
    CM4_IPC_MSG_AI_OUTPUT    = 0x0002,
    CM4_IPC_MSG_HEARTBEAT    = 0x0003,
    CM4_IPC_MSG_FAULT        = 0x0004
} cm4_ipc_msg_type_t;

/* ---------- AI Message (Input or Output) ---------- */
typedef struct {
    uint32_t    sequence_id;
    uint32_t    timestamp_us;
    uint32_t    crc32;
    uint32_t    flags;
    int8_t      data[CM4_IPC_AI_BYTES];
} cm4_ipc_ai_msg_t;

/* ---------- Ring Buffer Entry ---------- */
typedef struct {
    uint32_t            msg_type;
    uint32_t            sequence_id;
    cm4_ipc_ai_msg_t    payload;
    uint32_t            reserved[4];
} cm4_ipc_entry_t;

/* ---------- Ring ---------- */
typedef struct {
    volatile uint32_t   head;
    volatile uint32_t   tail;
    volatile uint32_t   count;
    uint32_t            overflow_count;
    uint32_t            reserved[4];
    cm4_ipc_entry_t     entries[CM4_IPC_RING_SIZE];
} cm4_ipc_ring_t;

/* ---------- Status ---------- */
typedef struct {
    uint32_t    cm4_alive;
    uint32_t    cm4_last_heartbeat_us;
    uint32_t    cm4_ai_inference_count;
    uint32_t    cm4_ai_latency_us;
    uint32_t    cm4_ai_latency_max_us;
    uint32_t    cm4_fault_count;
    uint32_t    reserved[8];
} cm4_ipc_status_t;

/* ---------- Full Shared Structure ---------- */
typedef struct {
    uint32_t            magic;
    uint32_t            version;
    uint32_t            reserved_header[6];

    cm4_ipc_ring_t      cm7_to_cm4;
    cm4_ipc_ring_t      cm4_to_cm7;

    cm4_ipc_status_t    status;

    uint32_t            reserved_tail[16];
} cm4_ipc_shared_t;

/* ---------- Memory Address ---------- */
#define CM4_IPC_SHARED_ADDR     ((cm4_ipc_shared_t *)0x38000000UL)

/* ---------- HSEM IDs ---------- */
#define CM4_HSEM_RING_CM7_TO_CM4    0U
#define CM4_HSEM_RING_CM4_TO_CM7    1U
#define CM4_HSEM_STATUS             2U

/* ---------- IPCC Channels ---------- */
#define CM4_IPCC_CH_CM7_TO_CM4      1U
#define CM4_IPCC_CH_CM4_TO_CM7      2U

/* ---------- Public API ---------- */
bool                cm4_ipc_init(void);
bool                cm4_ipc_validate(void);

bool                cm4_ipc_send_input(const cm4_ipc_ai_msg_t *msg);
bool                cm4_ipc_recv_input(cm4_ipc_ai_msg_t *msg);

bool                cm4_ipc_send_output(const cm4_ipc_ai_msg_t *msg);
bool                cm4_ipc_recv_output(cm4_ipc_ai_msg_t *msg);

uint32_t            cm4_ipc_crc32(const uint8_t *data, uint32_t len);

bool                cm4_ipc_status_update(void);
bool                cm4_ipc_status_read(cm4_ipc_status_t *out);

#ifdef __cplusplus
}
#endif

#endif /* CM4_IPC_H */