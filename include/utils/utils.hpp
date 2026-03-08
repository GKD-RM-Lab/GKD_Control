#ifndef __UTILS__
#define __UTILS__

#include <array>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <utility>
// ----------- log -----------
#define ANSI_FG_BLACK   "\33[1;30m"
#define ANSI_FG_RED     "\33[1;31m"
#define ANSI_FG_GREEN   "\33[1;32m"
#define ANSI_FG_YELLOW  "\33[1;33m"
#define ANSI_FG_BLUE    "\33[1;34m"
#define ANSI_FG_MAGENTA "\33[1;35m"
#define ANSI_FG_CYAN    "\33[1;36m"
#define ANSI_FG_WHITE   "\33[1;37m"
#define ANSI_BG_BLACK   "\33[1;40m"
#define ANSI_BG_RED     "\33[1;41m"
#define ANSI_BG_GREEN   "\33[1;42m"
#define ANSI_BG_YELLOW  "\33[1;43m"
#define ANSI_BG_BLUE    "\33[1;44m"
#define ANSI_BG_MAGENTA "\33[1;35m"
#define ANSI_BG_CYAN    "\33[1;46m"
#define ANSI_BG_WHITE   "\33[1;47m"
#define ANSI_NONE       "\33[0m"

#define ANSI_FMT(str, fmt) fmt str ANSI_NONE

#ifdef __DEBUG__
#define LOG_OK(s, ...)                                                 \
    do {                                                               \
        printf(ANSI_FMT(s, ANSI_FG_GREEN) __VA_OPT__(, ) __VA_ARGS__); \
    } while (0)

#define LOG_INFO(s, ...)                                              \
    do {                                                              \
        printf(ANSI_FMT(s, ANSI_FG_CYAN) __VA_OPT__(, ) __VA_ARGS__); \
    } while (0)

#define LOG_ERR(s, ...)                                              \
    do {                                                             \
        printf(ANSI_FMT(s, ANSI_FG_RED) __VA_OPT__(, ) __VA_ARGS__); \
    } while (0)

#else
#define LOG_OK(s, ...) \
    do {               \
    } while (0)

#define LOG_INFO(s, ...) \
    do {                 \
    } while (0)

#define LOG_ERR(s, ...) \
    do {                \
    } while (0)

#endif

namespace Utils
{
    namespace Log
    {
        struct BitDesc
        {
            uint8_t bit;
            const char* name;
        };

        template<size_t N>
        inline void bitmask_to_cstr(
            uint8_t mask,
            const std::array<BitDesc, N>& descs,
            char* out,
            size_t out_size,
            const char* none = "none") {
            if (out == nullptr || out_size == 0U) {
                return;
            }

            if (mask == 0U) {
                std::snprintf(out, out_size, "%s", none);
                return;
            }

            size_t pos = 0U;
            bool first = true;
            uint8_t unknown = mask;

            for (const auto& d : descs) {
                if ((mask & d.bit) == 0U) {
                    continue;
                }
                unknown = static_cast<uint8_t>(unknown & static_cast<uint8_t>(~d.bit));
                int written = std::snprintf(
                    out + pos,
                    out_size - pos,
                    "%s%s",
                    first ? "" : "|",
                    d.name == nullptr ? "?" : d.name);
                if (written < 0) {
                    out[out_size - 1U] = '\0';
                    return;
                }
                size_t add = static_cast<size_t>(written);
                if (add >= out_size - pos) {
                    out[out_size - 1U] = '\0';
                    return;
                }
                pos += add;
                first = false;
            }

            if (unknown != 0U) {
                std::snprintf(
                    out + pos,
                    out_size - pos,
                    "%sunk:0x%02X",
                    first ? "" : "|",
                    unknown);
            }
        }

        template<typename EnumT, size_t N>
        inline const char* enum_to_cstr(
            EnumT value,
            const std::array<std::pair<EnumT, const char*>, N>& descs,
            const char* unknown = "unknown") {
            for (const auto& [k, v] : descs) {
                if (k == value) {
                    return v == nullptr ? "?" : v;
                }
            }
            return unknown;
        }
    }  // namespace Log
}  // namespace Utils

#endif
