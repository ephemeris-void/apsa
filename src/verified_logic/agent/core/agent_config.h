#ifndef AGENT_CONFIG_H
#define AGENT_CONFIG_H

#define AGENT_MAIN_LOOP_PERIOD_US        1000U
#define AGENT_DEADLINE_US                5000U
#define AGENT_WCET_MARGIN_US             200U
#define AGENT_STATE_MIN_DWELL_US         100U
#define AGENT_STATE_TIMEOUT_US           5000U

#define AGENT_FAIL_CONFIRM_COUNT         3U
#define AGENT_RECOVER_CONFIRM_COUNT      10U

#define AGENT_AI_MAX_LATENCY_US          11000U
#define AGENT_AI_CONFIDENCE_THRESHOLD    0.80f

#define AGENT_WCET_CYCLES_DETECT         240000U
#define AGENT_WCET_CYCLES_DECIDE         96000U
#define AGENT_WCET_CYCLES_ACT            48000U
#define AGENT_WCET_CYCLES_AI             5280000U
#define AGENT_WCET_CYCLES_LOOP           2400000U

#define AGENT_FAULT_LOG_DEPTH            128U

#define AGENT_WDT_TIMEOUT_US             10000U
#define AGENT_WDT_KICK_INTERVAL_US       500U

#define AGENT_IPC_RING_SIZE              64U
#define AGENT_IPC_TIMEOUT_US             1000U

#define AGENT_RECOVERY_MAX_ATTEMPTS      3U
#define AGENT_RECOVERY_WINDOW_US         30000000U

#define AGENT_EMERGENCY_LAND_TIMEOUT_US  3000000U
#define AGENT_VIRTUAL_ONLY_TIMEOUT_US    180000000U

#define AGENT_GROUND_ALERT_TIMEOUT_US    50000U
#define AGENT_GROUND_RETRY_COUNT         3U

#define AGENT_PERSIST_MAGIC              0xA5A55A5AU
#define AGENT_PERSIST_SLOT_ADDR          0x080E0000U
#define AGENT_PERSIST_SLOT_SIZE          0x20000U

#define AGENT_TRACE_RING_SIZE            1024U
#define AGENT_TRACE_ENABLE               1
#define AGENT_TRACE_DWT_ENABLE           1

#define AGENT_CPU_FREQ_HZ                480000000U
#define AGENT_CYCLES_TO_US(c)            ((c) / 480U)
#define AGENT_US_TO_CYCLES(us)           ((us) * 480U)

#define AGENT_ENVELOPE_ACCEL_G           16.0f
#define AGENT_ENVELOPE_GYRO_DPS          2000.0f
#define AGENT_ENVELOPE_ALT_MAX_M         5000.0f
#define AGENT_ENVELOPE_VEL_MAX_MS        200.0f

#define AGENT_INLINE_FAST                __attribute__((always_inline)) static inline
#define AGENT_ITCM                       __attribute__((section(".itcm")))
#define AGENT_DTCM                       __attribute__((section(".dtcm")))
#define AGENT_PACKED                     __attribute__((packed))

#define AGENT_RECIP_480                  0.002083333f
#define AGENT_RECIP_2048                 0.000488281f
#define AGENT_RECIP_1000                 0.001f

#endif