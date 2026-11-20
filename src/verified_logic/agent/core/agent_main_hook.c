#include "agent_main_hook.h"
#include "agent.h"
#include "agent_init.h"
#include "agent_context.h"
#include "agent_config.h"

/* ============================================================
 * AGENT MAIN HOOK — IMPLEMENTATION
 * ============================================================
 * Board: NUCLEO-H755ZI-Q @ 480 MHz, VOS0, I+D Cache
 * Integration point with main.c / Zephyr thread.
 * This is the ONLY file main.c should call.
 * WCET: init ~2 ms (once), step ≤ 500 µs
 * ============================================================ */

/* ---------- Called once at boot ---------- */
agent_status_t agent_main_init(void)
{
    return agent_system_init();
}

/* ---------- Called every iteration (1 kHz target) ---------- */
agent_status_t agent_main_step(void)
{
    return agent_step();
}

/* ---------- Called on shutdown ---------- */
agent_status_t agent_main_shutdown(void)
{
    return agent_shutdown();
}

/* ============================================================
 * ZEPHYR INTEGRATION (uncomment when Zephyr is added)
 * ============================================================
 *
 * #include <zephyr/kernel.h>
 *
 * K_THREAD_STACK_DEFINE(agent_stack, 8192);
 * static struct k_thread agent_thread_data;
 *
 * static void agent_thread_entry(void *a, void *b, void *c)
 * {
 *     agent_main_init();
 *
 *     while (1) {
 *         agent_main_step();
 *         k_sleep(K_USEC(1000));   // 1 kHz
 *     }
 * }
 *
 * void agent_zephyr_start(void)
 * {
 *     k_thread_create(&agent_thread_data, agent_stack,
 *                     K_THREAD_STACK_SIZEOF(agent_stack),
 *                     agent_thread_entry,
 *                     NULL, NULL, NULL,
 *                     K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
 * }
 *
 * ============================================================ */