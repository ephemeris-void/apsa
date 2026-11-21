#include "agent_model_mgr_version.h"
#include "../core/agent_context.h"
#include "../trace/agent_log.h"

/* ============================================================
 * AGENT MODEL MANAGER VERSION — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 5 µs per call
 * ============================================================ */

static bool g_version_initialized = false;

/* ---------- INIT ---------- */
agent_status_t agent_model_mgr_version_init(void)
{
    g_version_initialized = true;
    return AGENT_OK;
}

agent_status_t agent_model_mgr_version_reset(void)
{
    return agent_model_mgr_version_init();
}

/* ---------- CRC32 (bitwise, ~32 cycles per byte) ---------- */
uint32_t agent_model_mgr_version_crc32(uint32_t model_id, uint32_t size_bytes)
{
    uint32_t crc  = 0xFFFFFFFFU;
    uint32_t seed = model_id ^ size_bytes;

    for (uint32_t i = 0U; i < 32U; i++) {
        uint32_t bit = (crc ^ seed) & 0x01U;
        crc >>= 1U;
        if (bit) {
            crc ^= 0xEDB88320U;
        }
        seed = (seed >> 1U) | ((seed & 0x01U) << 31U);
    }

    return crc ^ 0xFFFFFFFFU;
}

/* ---------- Parse a numeric version component ---------- */
static int32_t parse_version_part(const char *v, uint32_t *idx)
{
    int32_t  val = 0;
    uint32_t i   = *idx;

    while (v[i] >= '0' && v[i] <= '9') {
        val = val * 10 + (v[i] - '0');
        i++;
    }

    *idx = i;
    return val;
}

/* ---------- Compare semantic versions (major.minor.patch) ---------- */
int32_t agent_model_mgr_version_compare(const char *v1, const char *v2)
{
    if (v1 == NULL && v2 == NULL) return  0;
    if (v1 == NULL)               return -1;
    if (v2 == NULL)               return  1;

    uint32_t i1 = 0U;
    uint32_t i2 = 0U;

    for (uint32_t part = 0U; part < 4U; part++) {

        int32_t n1 = parse_version_part(v1, &i1);
        int32_t n2 = parse_version_part(v2, &i2);

        if (n1 < n2) return -1;
        if (n1 > n2) return  1;

        if (v1[i1] == '.') i1++;
        if (v2[i2] == '.') i2++;

        if (v1[i1] == '\0' && v2[i2] == '\0') {
            break;
        }
    }

    return 0;
}

/* ---------- Is newer ---------- */
bool agent_model_mgr_version_is_newer(const char *v1, const char *v2)
{
    return (agent_model_mgr_version_compare(v1, v2) > 0);
}