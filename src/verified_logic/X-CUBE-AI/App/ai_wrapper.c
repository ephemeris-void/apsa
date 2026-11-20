#include "ai_wrapper.h"
#include "network.h"
#include "network_data.h"
#include "network_data_params.h"

#include <zephyr/kernel.h>
#include <string.h>

static ai_handle g_network = AI_HANDLE_NULL;
static bool      g_initialized = false;

static ai_buffer *g_input_buf  = NULL;
static ai_buffer *g_output_buf = NULL;

AI_ALIGNED(4) static ai_u8 g_activations[AI_NETWORK_DATA_ACTIVATIONS_SIZE];

int ai_wrapper_init(void)
{
    ai_error err;

    const ai_handle act_addr[] = { g_activations };

    err = ai_network_create_and_init(&g_network, act_addr, NULL);
    if (err.type != AI_ERROR_NONE) {
        printk("[AI] create/init failed: type=%d code=%d\n", err.type, err.code);
        return -1;
    }

    g_input_buf  = ai_network_inputs_get(g_network, NULL);
    g_output_buf = ai_network_outputs_get(g_network, NULL);

    if (g_input_buf == NULL || g_output_buf == NULL) {
        printk("[AI] failed to get buffers\n");
        return -1;
    }

    g_initialized = true;
    printk("[AI] init OK (input=%d bytes, output=%d bytes)\n",
           AI_NETWORK_IN_1_SIZE_BYTES, AI_NETWORK_OUT_1_SIZE_BYTES);
    return 0;
}

int ai_wrapper_run(const int8_t *input, int8_t *output)
{
    if (!g_initialized) {
        return -1;
    }

    memcpy(g_input_buf->data, input, AI_NETWORK_IN_1_SIZE_BYTES);

    ai_i32 n_batch = ai_network_run(g_network, g_input_buf, g_output_buf);
    if (n_batch != 1) {
        ai_error err = ai_network_get_error(g_network);
        printk("[AI] run failed: type=%d code=%d\n", err.type, err.code);
        return -1;
    }

    memcpy(output, g_output_buf->data, AI_NETWORK_OUT_1_SIZE_BYTES);
    return 0;
}

void ai_wrapper_deinit(void)
{
    if (g_network != AI_HANDLE_NULL) {
        ai_network_destroy(g_network);
        g_network = AI_HANDLE_NULL;
    }
    g_initialized = false;
}