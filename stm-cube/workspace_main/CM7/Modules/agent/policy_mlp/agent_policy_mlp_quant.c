#include "agent_policy_mlp_quant.h"
#include "../core/agent_context.h"
#include <math.h>

/* ============================================================
 * AGENT POLICY MLP QUANT — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * WCET: 1 µs per call
 * ============================================================ */

/* ---------- float → int8 ---------- */
int8_t agent_policy_mlp_quant_i8(float value, float scale, int32_t zp)
{
    if (scale <= 0.0f) {
        return 0;
    }

    int32_t q = (int32_t)roundf(value / scale) + zp;

    /* Saturate to int8 */
    if (q >  127) q =  127;
    if (q < -128) q = -128;

    return (int8_t)q;
}

/* ---------- int8 → float ---------- */
float agent_policy_mlp_dequant_i8(int8_t value, float scale, int32_t zp)
{
    return ((float)value - (float)zp) * scale;
}

/* ---------- float → int32 ---------- */
int32_t agent_policy_mlp_quant_i32(float value, float scale)
{
    if (scale <= 0.0f) {
        return 0;
    }
    return (int32_t)roundf(value / scale);
}

/* ---------- int32 → float ---------- */
float agent_policy_mlp_dequant_i32(int32_t value, float scale)
{
    return (float)value * scale;
}