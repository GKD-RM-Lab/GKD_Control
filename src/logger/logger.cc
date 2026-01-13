#include "logger/logger.hpp"

#include <algorithm>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <utility>
#include <cstdlib>

constexpr std::string_view _error_color = "\033[31m";
constexpr std::string_view _info_color = "\033[32m";
constexpr std::string_view _warn_color = "\033[33m";
constexpr std::string_view _debug_color = "\033[34m";
constexpr std::string_view _time_color    = "\033[35m";
constexpr std::string_view _role_color    = "\033[36m";
constexpr std::string_view _reset_color = "\033[0m";

namespace {
std::string current_timestamp() {
    using namespace std::chrono;
    const auto now = system_clock::now();
    const auto time_t = system_clock::to_time_t(now);

    std::tm tm_snapshot;
#if defined(_WIN32)
    localtime_s(&tm_snapshot, &time_t);
#else
    localtime_r(&time_t, &tm_snapshot);
#endif

    auto micros = duration_cast<microseconds>(now.time_since_epoch()) % 1'000'000;

    std::ostringstream oss;
    oss << std::put_time(&tm_snapshot, "%H:%M:%S")
        << '.' << std::setw(6) << std::setfill('0') << micros.count();
    return oss.str();
}
}

void Logger::set_level(log_level level) {
    instance()._level.store(level);
}

log_level Logger::level(){
    return instance()._level.load();
}

void Logger::set_filter(const std::string filter){
    instance().filter_ = filter;
}

static inline std::string_view level_to_color(log_level level){
    switch(level){
        case log_level::Debug:
            return _debug_color;
        case log_level::Info:
            return _info_color;
        case log_level::Warn:
            return _warn_color;
        case log_level::Error:
            return _error_color;
        default:
            ;
    }
}

void Logger::log_impl(log_level level, std::string_view role, std::string_view message) {
#ifndef _DEBUG
    auto &stream = (level == log_level::Error) ? std::cerr : std::cout;
    const auto role_view = role.empty() ? std::string_view{"-"} : role;

    stream << _time_color <<'[' << current_timestamp() << ']' << level_to_color(level)
           << " [" << level_to_string(level) << "] " << _role_color
           << '[' << role_view << "] " << level_to_color(level)
           << message << _reset_color << std::endl;  
#endif
}

std::string_view Logger::level_to_string(log_level level) {
    switch (level) {
    case log_level::Debug:
        return "DEBUG";
    case log_level::Info:
        return "INFO";
    case log_level::Warn:
        return "WARN";
    case log_level::Error:
        return "ERROR";
    default:
        return "UNKNOWN";
    }
}
