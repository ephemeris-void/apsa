#ifndef AI_WRAPPER_H
#define AI_WRAPPER_H

#include <stdint.h>
#include <stdbool.h>

int  ai_wrapper_init(void);
int  ai_wrapper_run(const int8_t *input, int8_t *output);
void ai_wrapper_deinit(void);

#endif /* AI_WRAPPER_H */