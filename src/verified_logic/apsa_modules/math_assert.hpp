#pragma once

#ifndef NDEBUG
    #define MATH_ASSERT(e) do { if (!(e)) __builtin_trap(); } while(0)
#else
    #define MATH_ASSERT(e) do { [[assume(e)]]; } while(0)
#endif