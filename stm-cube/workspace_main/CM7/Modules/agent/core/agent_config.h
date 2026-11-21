#ifndef AGENT_CONFIG_H
#define AGENT_CONFIG_H

/* ============================================================
 * AGENT CONFIG — TUNABLE
 * ============================================================
 * Board: NUCLEO-H755ZI-Q
 * CPU:   Cortex-M7 @ 480 MHz, VOS0, Flash Latency 4
 * Cache: I-Cache + D-Cache enabled
 * ============================================================ */

/* ---------- Timing (all in microseconds) ---------- */
#define AGENT_MAIN_LOOP_PERIOD_US        1000U      /* 1 kHz */
#define AGENT_DEADLINE_US                1000U      /* hard deadline */
#define AGENT_WCET_MARGIN_US             200U       /* safety margin */
#define AGENT_STATE_MIN_DWELL_US         10000U     /* 10 ms anti-chatter */
#define AGENT_STATE_TIMEOUT_US           50000U     /* 50 ms state watchdog */
#define AGENT_LANDING_MIN_DWELL_US       1000000U   /* 1 s min in LANDING */

/* ---------- State Transition Thresholds ---------- */
#define AGENT_FAIL_CONFIRM_COUNT         3U         /* consecutive fails */
#define AGENT_RECOVER_CONFIRM_COUNT      10U        /* consecutive OKs */
#define AGENT_CONFIDENCE_MIN             0.70f      /* min trust */
#define AGENT_CONFIDENCE_RECOVER         0.90f      /* recover trust */

/* ---------- Sensor Fail Detection ---------- */
#define AGENT_INNOVATION_SIGMA           5.0f       /* chi-square gate */
#define AGENT_RANGE_ACCEL_MIN            (-32000)
#define AGENT_RANGE_ACCEL_MAX            (32000)
#define AGENT_RANGE_GYRO_MIN             (-32000)
#define AGENT_RANGE_GYRO_MAX             (32000)
#define AGENT_DROPOUT_THRESHOLD_US       1500U      /* > 1.5 ms = drop */

/* ---------- AI Inference ---------- */
#define AGENT_AI_MAX_LATENCY_US          11000U     /* 11 ms measured */
#define AGENT_AI_TARGET_LATENCY_US       5000U      /* 5 ms target */
#define AGENT_AI_INPUT_SHAPE             2048U      /* 4 × 512 */
#define AGENT_AI_OUTPUT_SHAPE            2048U
#define AGENT_AI_CONFIDENCE_THRESHOLD    0.80f

/* ---------- WCET Budgets (cycles @ 480 MHz) ---------- */
#define AGENT_WCET_CYCLES_DETECT         240000U    /* 500 µs */
#define AGENT_WCET_CYCLES_DECIDE         96000U     /* 200 µs */
#define AGENT_WCET_CYCLES_ACT            48000U     /* 100 µs */
#define AGENT_WCET_CYCLES_AI             5280000U   /* 11000 µs */
#define AGENT_WCET_CYCLES_LOOP           480000U    /* 1 ms */

/* ---------- Fault Escalation ---------- */
#define AGENT_FAULT_ESCALATE_THRESHOLD   5U
#define AGENT_FAULT_CRITICAL_THRESHOLD   10U
#define AGENT_FAULT_LOG_DEPTH            128U

/* ---------- Watchdog ---------- */
#define AGENT_WDT_TIMEOUT_US             10000U     /* 10 ms */
#define AGENT_WDT_KICK_INTERVAL_US       500U

/* ---------- IPC (CM7 ↔ CM4) ---------- */
#define AGENT_IPC_RING_SIZE              64U
#define AGENT_IPC_TIMEOUT_US             1000U
#define AGENT_IPC_HEARTBEAT_US           100U

/* ---------- Recovery ---------- */
#define AGENT_RECOVERY_MAX_ATTEMPTS      3U
#define AGENT_RECOVERY_WINDOW_US         30000000U  /* 30 s */
#define AGENT_EMERGENCY_TIMEOUT_US       300000000U /* 5 min */

/* ---------- Ground Link ---------- */
#define AGENT_GROUND_ALERT_TIMEOUT_US    50000U     /* 50 ms */
#define AGENT_GROUND_RETRY_COUNT         3U

/* ---------- Persistence ---------- */
#define AGENT_PERSIST_MAGIC              0xA5A55A5AU
#define AGENT_PERSIST_SLOT_ADDR          0x080E0000U /* flash sector 7 */
#define AGENT_PERSIST_SLOT_SIZE          0x20000U    /* 128 KB */

/* ---------- Trace Ring Buffer ---------- */
#define AGENT_TRACE_RING_SIZE            1024U
#define AGENT_TRACE_ENABLE               1
#define AGENT_TRACE_DWT_ENABLE           1

/* ---------- DWT Cycle Counter ---------- */
#define AGENT_CPU_FREQ_HZ                480000000U
#define AGENT_CYCLES_TO_US(c)            ((c) / 480U)
#define AGENT_US_TO_CYCLES(us)           ((us) * 480U)

/* ---------- Envelope (physical limits) ---------- */
#define AGENT_ENVELOPE_ACCEL_G           16.0f
#define AGENT_ENVELOPE_GYRO_DPS          2000.0f
#define AGENT_ENVELOPE_ALT_MAX_M         5000.0f
#define AGENT_ENVELOPE_VEL_MAX_MS        200.0f

/* ---------- Optimization Flags ---------- */
#define AGENT_USE_CACHE                  1
#define AGENT_USE_CMSIS_NN               1
#define AGENT_USE_ITCM_HOTPATH           1
#define AGENT_USE_DTCM_DATA              1
#define AGENT_USE_DMA                    1

/* ---------- Compiler Hints ---------- */
#define AGENT_INLINE_FAST                __attribute__((always_inline)) static inline
#define AGENT_ITCM                       __attribute__((section(".itcm")))
#define AGENT_DTCM                       __attribute__((section(".dtcm")))
#define AGENT_PACKED                     __attribute__((packed))

/* ---------- Fast Math (avoid float in hot path) ---------- */
#define AGENT_RECIP_480                   0.002083333f
#define AGENT_RECIP_2048                  0.000488281f
#define AGENT_RECIP_16_4                  0.060975610f
#define AGENT_RECIP_1000                  0.001f

#endif /* AGENT_CONFIG_H */