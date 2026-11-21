#include "agent_persist.h"
#include "../core/agent_context.h"
#include "../core/agent_config.h"
#include "../trace/agent_log.h"
#include <string.h>

/* ============================================================
 * AGENT PERSIST — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 500 µs per save/load (boot only)
 *
 * SAFETY: Flash writes performed only at boot or on controlled
 *         shutdown. Not called from 1 kHz agent loop.
 * ============================================================ */

/* PERF: state in DTCM (zero wait state) */
static AGENT_DTCM agent_persist_state_t g_persist;
static bool                             g_persist_initialized = false;

/* ---------- CRC32 (software, bitwise) ---------- */
uint32_t agent_persist_crc32(const uint8_t *data, uint32_t len)
{
    if (data == NULL) {
        return 0U;
    }

    uint32_t crc = 0xFFFFFFFFU;

    for (uint32_t i = 0U; i < len; i++) {

        crc ^= (uint32_t)data[i];

        for (uint32_t b = 0U; b < 8U; b++) {
            if (crc & 1U) {
                crc = (crc >> 1U) ^ 0xEDB88320U;
            } else {
                crc >>= 1U;
            }
        }
    }

    return crc ^ 0xFFFFFFFFU;
}

/* ---------- INIT ---------- */
agent_status_t agent_persist_init(void)
{
    memset(&g_persist, 0, sizeof(g_persist));   /* zero-init incl. reserved[] */

    g_persist.last_result   = AGENT_PERSIST_RESULT_EMPTY;
    g_persist.write_count   = 0U;
    g_persist.read_count    = 0U;
    g_persist.crc_errors    = 0U;
    g_persist.last_write_us = 0U;
    g_persist.initialized   = true;

    g_persist_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_persist_reset(void)
{
    g_persist_initialized = false;
    return agent_persist_init();
}

/* ---------- LOAD ---------- */
agent_persist_result_t agent_persist_load(void)
{
    if (!g_persist_initialized) {
        return AGENT_PERSIST_RESULT_FAIL;
    }

    g_persist.read_count++;

    const agent_persist_block_t *src =
        (const agent_persist_block_t *)AGENT_PERSIST_SLOT_ADDR;

    /* Magic check */
    if (src->magic != AGENT_PERSIST_MAGIC) {
        g_persist.last_result = AGENT_PERSIST_RESULT_EMPTY;
        return AGENT_PERSIST_RESULT_EMPTY;
    }

    /* Version check */
    if (src->version != AGENT_PERSIST_VERSION) {
        g_persist.last_result = AGENT_PERSIST_RESULT_CORRUPT;
        g_persist.crc_errors++;
        return AGENT_PERSIST_RESULT_CORRUPT;
    }

    /* CRC check */
    uint32_t computed = agent_persist_crc32(
        (const uint8_t *)src,
        sizeof(agent_persist_block_t) - sizeof(uint32_t));

    if (computed != src->crc) {
        g_persist.last_result = AGENT_PERSIST_RESULT_CORRUPT;
        g_persist.crc_errors++;
        return AGENT_PERSIST_RESULT_CORRUPT;
    }

    g_persist.block       = *src;
    g_persist.last_result = AGENT_PERSIST_RESULT_OK;

    return AGENT_PERSIST_RESULT_OK;
}

/* ---------- SAVE ---------- */
agent_persist_result_t agent_persist_save(void)
{
    if (!g_persist_initialized) {
        return AGENT_PERSIST_RESULT_FAIL;
    }

    agent_context_full_t *ctx = agent_context_get();

    agent_persist_block_t b;
    memset(&b, 0, sizeof(b));    /* zero-init incl. reserved[] */

    b.magic          = AGENT_PERSIST_MAGIC;
    b.version        = AGENT_PERSIST_VERSION;
    b.timestamp_us   = agent_time_now_us();
    b.fault_count    = ctx->fault_count;
    b.recovery_count = ctx->recovery_count;
    b.last_state     = (uint32_t)ctx->current_state;
    b.last_priority  = (uint32_t)ctx->current_priority;
    b.boot_count     = g_persist.block.boot_count + 1U;

    b.crc = agent_persist_crc32((const uint8_t *)&b,
                                sizeof(agent_persist_block_t) - sizeof(uint32_t));

    /* NOTE: actual flash write performed by HAL layer externally.
     * This module prepares the block; caller invokes HAL_FLASH_Program.
     */
    g_persist.block       = b;
    g_persist.write_count++;
    g_persist.last_write_us = b.timestamp_us;
    g_persist.last_result   = AGENT_PERSIST_RESULT_OK;

    agent_log_event(AGENT_LOG_PERSIST, b.boot_count, b.fault_count);

    return AGENT_PERSIST_RESULT_OK;
}

/* ---------- ERASE ---------- */
agent_persist_result_t agent_persist_erase(void)
{
    if (!g_persist_initialized) {
        return AGENT_PERSIST_RESULT_FAIL;
    }

    memset(&g_persist.block, 0, sizeof(g_persist.block));
    g_persist.last_result = AGENT_PERSIST_RESULT_EMPTY;

    return AGENT_PERSIST_RESULT_EMPTY;
}

/* ---------- STATE ---------- */
const agent_persist_state_t *agent_persist_state(void)
{
    return &g_persist;
}