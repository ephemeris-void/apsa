#ifndef SAL_CONFIG_H
#define SAL_CONFIG_H

/* ============================================================
 * SENSOR ABSTRACTION LAYER — CONFIG (FINAL)
 * ============================================================
 * Board: NUCLEO-H755ZI-Q
 * ============================================================ */

/* ============================================================
 * ENABLE / DISABLE SENSORS
 * ============================================================ */
#define SAL_ENABLE_IMU      1
#define SAL_ENABLE_GPS      1
#define SAL_ENABLE_MAG      1
#define SAL_ENABLE_BARO     1
#define SAL_ENABLE_LORA     1

/* ============================================================
 * SENSOR RATES (Hz)
 * ============================================================ */
#define SAL_IMU_RATE_HZ     1000
#define SAL_GPS_RATE_HZ     50      /* ← 10 → 50 (NMEA parsing-এর জন্য) */
#define SAL_MAG_RATE_HZ     50
#define SAL_BARO_RATE_HZ    25
#define SAL_LORA_RATE_HZ    1

/* ============================================================
 * SENSOR HEALTH TIMEOUTS (ms)
 * ============================================================ */
#define SAL_IMU_TIMEOUT_MS      10
#define SAL_GPS_TIMEOUT_MS      500
#define SAL_MAG_TIMEOUT_MS      100
#define SAL_BARO_TIMEOUT_MS     100
#define SAL_LORA_TIMEOUT_MS     5000

/* ============================================================
 * GPS — u-blox NEO-M8N (USART1 on D0/D1)
 * ============================================================ */
#define SAL_GPS_UART_NODE       "usart1"
#define SAL_GPS_BAUD            9600U
#define SAL_GPS_RX_BUF_SIZE     256U
#define SAL_GPS_LINE_BUF_SIZE   128U

/* ============================================================
 * COMPASS — QMC5883P @ 0x2C (I2C4 on PF14/PF15)
 * ============================================================ */
#define SAL_MAG_I2C_NODE        "i2c4"
#define SAL_MAG_I2C_ADDR        0x2C        /* QMC5883P 7-bit address */
#define SAL_MAG_DECLINATION_DEG 1.0f

/* ============================================================
 * BAROMETER — DPS310 (I2C1 on PB8/PB9)
 * ============================================================ */
#define SAL_BARO_I2C_NODE       "i2c1"
#define SAL_BARO_I2C_ADDR       0x77
#define SAL_BARO_SEA_LEVEL_PA   101325.0f

/* ============================================================
 * LORA — DFRobot LoRaWAN Node (US915) — UART4 on PC10/PC11
 * ============================================================ */
#define SAL_LORA_UART_NODE      "uart4"
#define SAL_LORA_BAUD           9600U
#define SAL_LORA_TX_BUF_SIZE    256U
#define SAL_LORA_RX_BUF_SIZE    256U
#define SAL_LORA_LINE_BUF_SIZE  192U

/* ============================================================
 * VALIDATION
 * ============================================================ */
#if SAL_ENABLE_IMU + SAL_ENABLE_GPS + SAL_ENABLE_MAG + \
    SAL_ENABLE_BARO + SAL_ENABLE_LORA == 0
#warning "No sensors enabled"
#endif

#endif /* SAL_CONFIG_H */