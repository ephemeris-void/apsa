#ifndef AGENT_MODEL_MGR_VERSION_H
#define AGENT_MODEL_MGR_VERSION_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT MODEL MANAGER — VERSION
 * ============================================================
 * CRC32 verification + semantic version comparison.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 5 µs max per call.
 * ============================================================ */

agent_status_t  agent_model_mgr_version_init(void);
agent_status_t  agent_model_mgr_version_reset(void);

uint32_t        agent_model_mgr_version_crc32(uint32_t model_id,
                                              uint32_t size_bytes);

int32_t         agent_model_mgr_version_compare(const char *v1,
                                                const char *v2);

bool            agent_model_mgr_version_is_newer(const char *v1,
                                                 const char *v2);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_MODEL_MGR_VERSION_H */