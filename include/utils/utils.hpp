#ifndef __UTILS__
#define __UTILS__
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

#endif
