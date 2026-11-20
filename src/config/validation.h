/* ============================================================
 * VALIDATION MACROS — RUNTIME SAFETY LAYER
 * ============================================================
 * Every public function entry validates its inputs.
 * Fail-fast with logs. No silent failures.
 *
 * Usage:
 *   VALIDATE_NOT_NULL(ptr);
 *   VALIDATE_RANGE(speed, 0.0f, 100.0f);
 *   VALIDATE_RETURN(cond, ERR_CODE);
 * ============================================================ */

#ifndef VALIDATION_H
#define VALIDATION_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <math.h>

/* ---------- Error Codes (unified across project) ---------- */
typedef enum {
    PROJ_OK              =  0,
    PROJ_ERR             = -1,
    PROJ_ERR_NULL        = -2,
    PROJ_ERR_RANGE       = -3,
    PROJ_ERR_TIMEOUT     = -4,
    PROJ_ERR_NOT_INIT    = -5,
    PROJ_ERR_HW          = -6,
    PROJ_ERR_INVALID     = -7,
    PROJ_ERR_NO_MEM      = -8,
    PROJ_ERR_BUSY        = -9,
    PROJ_ERR_NOT_FOUND   = -10,
    PROJ_ERR_CHECKSUM    = -11,
    PROJ_ERR_OVERFLOW    = -12
} proj_status_t;

/* ---------- Logging Hook (weak — override in production) ---------- */
__attribute__((weak))
void validation_log_fail(const char *file, int line, const char *msg);

/* ---------- Core Macros ---------- */

/* NULL check — return error code if null */
#define VALIDATE_NOT_NULL(ptr)                                          \
    do {                                                                \
        if ((ptr) == NULL) {                                            \
            validation_log_fail(__FILE__, __LINE__,                     \
                "NULL pointer: " #ptr);                                 \
            return PROJ_ERR_NULL;                                       \
        }                                                               \
    } while (0)

/* NULL check — void function (just log) */
#define VALIDATE_NOT_NULL_VOID(ptr)                                     \
    do {                                                                \
        if ((ptr) == NULL) {                                            \
            validation_log_fail(__FILE__, __LINE__,                     \
                "NULL pointer: " #ptr);                                 \
            return;                                                     \
        }                                                               \
    } while (0)

/* Range check — integer */
#define VALIDATE_RANGE_I(value, min_val, max_val)                       \
    do {                                                                \
        if ((value) < (min_val) || (value) > (max_val)) {               \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Out of range: " #value);                               \
            return PROJ_ERR_RANGE;                                      \
        }                                                               \
    } while (0)

/* Range check — float (with NaN/inf detection) */
#define VALIDATE_RANGE_F(value, min_val, max_val)                       \
    do {                                                                \
        if (isnan((float)(value)) || isinf((float)(value)) ||           \
            (value) < (min_val) || (value) > (max_val)) {               \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Float out of range: " #value);                         \
            return PROJ_ERR_RANGE;                                      \
        }                                                               \
    } while (0)

/* Conditional check — return error if condition true */
#define VALIDATE_RETURN(condition, err_code)                            \
    do {                                                                \
        if ((condition)) {                                              \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Condition failed: " #condition);                       \
            return (err_code);                                          \
        }                                                               \
    } while (0)

/* Conditional check — void function */
#define VALIDATE_RETURN_VOID(condition)                                 \
    do {                                                                \
        if ((condition)) {                                              \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Condition failed: " #condition);                       \
            return;                                                     \
        }                                                               \
    } while (0)

/* Initialization check */
#define VALIDATE_INITIALIZED(flag)                                      \
    do {                                                                \
        if (!(flag)) {                                                  \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Not initialized: " #flag);                             \
            return PROJ_ERR_NOT_INIT;                                   \
        }                                                               \
    } while (0)

/* Alignment check */
#define VALIDATE_ALIGNED(ptr, align)                                    \
    do {                                                                \
        if (((uintptr_t)(ptr) % (align)) != 0U) {                       \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Not aligned: " #ptr);                                  \
            return PROJ_ERR_INVALID;                                    \
        }                                                               \
    } while (0)

/* Buffer size check */
#define VALIDATE_BUFFER_SIZE(actual, required)                          \
    do {                                                                \
        if ((actual) < (required)) {                                    \
            validation_log_fail(__FILE__, __LINE__,                     \
                "Buffer too small: " #actual);                          \
            return PROJ_ERR_OVERFLOW;                                   \
        }                                                               \
    } while (0)

/* ---------- Inline Helpers ---------- */

static inline bool is_valid_quaternion(float w, float x, float y, float z)
{
    if (isnan(w) || isnan(x) || isnan(y) || isnan(z)) return false;
    if (isinf(w) || isinf(x) || isinf(y) || isinf(z)) return false;
    float norm_sq = w*w + x*x + y*y + z*z;
    return (norm_sq > 0.99f && norm_sq < 1.01f);
}

static inline bool is_valid_range_f(float v, float min_val, float max_val)
{
    if (isnan(v) || isinf(v)) return false;
    return (v >= min_val && v <= max_val);
}

static inline bool is_valid_range_i(int32_t v, int32_t min_val, int32_t max_val)
{
    return (v >= min_val && v <= max_val);
}

static inline float clamp_f(float v, float min_val, float max_val)
{
    if (isnan(v)) return 0.0f;
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return v;
}

static inline int32_t clamp_i(int32_t v, int32_t min_val, int32_t max_val)
{
    if (v < min_val) return min_val;
    if (v > max_val) return max_val;
    return v;
}

/* ---------- Safe Cast Helpers ---------- */

static inline int16_t safe_f_to_i16(float v, int16_t fallback)
{
    if (isnan(v) || isinf(v)) return fallback;
    if (v > 32767.0f) return 32767;
    if (v < -32768.0f) return -32768;
    return (int16_t)v;
}

static inline int32_t safe_f_to_i32(float v, int32_t fallback)
{
    if (isnan(v) || isinf(v)) return fallback;
    if (v > 2147483647.0f) return 2147483647;
    if (v < -2147483648.0f) return (-2147483647 - 1);
    return (int32_t)v;
}

static inline uint32_t safe_f_to_u32(float v, uint32_t fallback)
{
    if (isnan(v) || isinf(v) || v < 0.0f) return fallback;
    if (v > 4294967295.0f) return 4294967295U;
    return (uint32_t)v;
}

/* ---------- Division Safety ---------- */

static inline float safe_div_f(float num, float den, float fallback)
{
    if (fabsf(den) < 1e-9f) return fallback;
    return num / den;
}

/* ---------- Assertion (debug only) ---------- */

#ifdef DEBUG
    #define PROJ_ASSERT(cond)                                           \
        do {                                                            \
            if (!(cond)) {                                              \
                validation_log_fail(__FILE__, __LINE__,                 \
                    "Assert failed: " #cond);                           \
                while (1) { __asm volatile ("nop"); }                   \
            }                                                           \
        } while (0)
#else
    #define PROJ_ASSERT(cond) ((void)0)
#endif

#endif /* VALIDATION_H */