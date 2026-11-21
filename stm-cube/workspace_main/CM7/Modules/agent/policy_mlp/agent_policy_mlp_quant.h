#ifndef AGENT_POLICY_MLP_QUANT_H
#define AGENT_POLICY_MLP_QUANT_H

#include "../core/agent_types.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ============================================================
 * AGENT POLICY MLP QUANT — FROZEN
 * ============================================================
 * int8 / int32 quantize-dequantize primitives.
 * Saturation-clamped to avoid overflow.
 * Board: NUCLEO-H755ZI-Q @ 480 MHz
 * WCET: 1 µs max per call.
 * ============================================================ */

int8_t          agent_policy_mlp_quant_i8(float value, float scale, int32_t zp);
float           agent_policy_mlp_dequant_i8(int8_t value, float scale, int32_t zp);

int32_t         agent_policy_mlp_quant_i32(float value, float scale);
float           agent_policy_mlp_dequant_i32(int32_t value, float scale);

#ifdef __cplusplus
}
#endif

#endif /* AGENT_POLICY_MLP_QUANT_H */