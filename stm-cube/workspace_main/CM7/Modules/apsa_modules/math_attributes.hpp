#pragma once

#if defined(__GNUC__) || defined(__clang__)
    #define APSA_CONST    gnu::const
    #define APSA_PURE     gnu::pure
    #define APSA_UNLIKELY [[unlikely]]
    #define APSA_LIKELY   [[likely]]
#else
    #define APSA_CONST
    #define APSA_PURE
    #define APSA_UNLIKELY
    #define APSA_LIKELY
#endif