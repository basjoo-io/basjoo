/**
 * @file logging.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-05-09
 *
 * @copyright Copyright (c) 2024 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <memory>
#include <source_location>
#include <string_view>
#include <type_traits>
#include <utility>

#include "spdlog/async.h"
#include "spdlog/sinks/basic_file_sink.h"
#include "spdlog/sinks/stdout_color_sinks.h"
#include "spdlog/spdlog.h"

#define BASJOO_LOG_LEVEL_TRACE SPDLOG_LEVEL_TRACE
#define BASJOO_LOG_LEVEL_DEBUG SPDLOG_LEVEL_DEBUG
#define BASJOO_LOG_LEVEL_INFO SPDLOG_LEVEL_INFO
#define BASJOO_LOG_LEVEL_WARN SPDLOG_LEVEL_WARN
#define BASJOO_LOG_LEVEL_ERROR SPDLOG_LEVEL_ERROR
#define BASJOO_LOG_LEVEL_CRITICAL SPDLOG_LEVEL_CRITICAL
#define BASJOO_LOG_LEVEL_OFF SPDLOG_LEVEL_OFF

#if !defined(BASJOO_LOG_ACTIVE_LEVEL)
#define BASJOO_LOG_ACTIVE_LEVEL BASJOO_LOG_LEVEL_INFO
#endif

namespace basjoo::common {

using Logger = spdlog::logger;
using LogLevel = spdlog::level::level_enum;

inline constexpr LogLevel kActiveLogLevel{BASJOO_LOG_ACTIVE_LEVEL};

namespace detail {

// Bundles a compile-time checked format string with the call site's source_location, captured
// via a defaulted argument on this converting constructor so it reflects the caller's location
// rather than this header's.
template <typename... Args>
struct FormatStringWithLoc {
    spdlog::format_string_t<Args...> fmt;
    std::source_location loc;

    template <typename T>
        requires std::constructible_from<spdlog::format_string_t<Args...>, T const&>
    consteval FormatStringWithLoc(
        T const& str, std::source_location location = std::source_location::current()
    )
        : fmt(str), loc(location) {}
};

template <LogLevel Level, typename... Args>
[[using gnu: always_inline]]
inline auto log(
    Logger* logger, FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    if constexpr (Level >= kActiveLogLevel) {
        logger->log(
            spdlog::source_loc{
                fmt.loc.file_name(), static_cast<int>(fmt.loc.line()), fmt.loc.function_name()
            },
            Level, fmt.fmt, std::forward<Args>(args)...
        );
    }
}

} // namespace detail

template <typename... Args>
[[using gnu: always_inline]]
inline auto logTrace(
    detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::trace>(spdlog::default_logger_raw(), fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logTrace(
    Logger* logger, detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::trace>(logger, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logDebug(
    detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::debug>(spdlog::default_logger_raw(), fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logDebug(
    Logger* logger, detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::debug>(logger, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logInfo(
    detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::info>(spdlog::default_logger_raw(), fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logInfo(
    Logger* logger, detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::info>(logger, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logWarn(
    detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::warn>(spdlog::default_logger_raw(), fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logWarn(
    Logger* logger, detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::warn>(logger, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logError(
    detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::err>(spdlog::default_logger_raw(), fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logError(
    Logger* logger, detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::err>(logger, fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logCritical(
    detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::critical>(spdlog::default_logger_raw(), fmt, std::forward<Args>(args)...);
}

template <typename... Args>
[[using gnu: always_inline]]
inline auto logCritical(
    Logger* logger, detail::FormatStringWithLoc<std::type_identity_t<Args>...> fmt, Args&&... args
) noexcept -> void {
    detail::log<LogLevel::critical>(logger, fmt, std::forward<Args>(args)...);
}

[[using gnu: pure]]
inline auto makeLogger(std::string_view name, std::string_view log_file = "")
    -> std::shared_ptr<Logger> {
    spdlog::init_thread_pool(8192, 1);

    auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    console_sink->set_level(spdlog::level::trace);
    console_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] [thread %t] %@ %v");

    if (log_file.empty()) {
        return std::make_shared<spdlog::async_logger>(
            name.data(), console_sink, spdlog::thread_pool()
        );
    }

    auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(log_file.data(), true);
    file_sink->set_level(spdlog::level::trace);
    file_sink->set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%n] [%l] [thread %t] %@ %v");

    return std::make_shared<spdlog::async_logger>(
        name.data(), spdlog::sinks_init_list{console_sink, file_sink}, spdlog::thread_pool()
    );
}

[[using gnu: pure, always_inline]] [[nodiscard]]
inline auto getLogger(std::string_view name) -> std::shared_ptr<Logger> {
    return spdlog::get(name.data());
}

[[using gnu: always_inline]]
inline auto dropLogger(std::string_view name) -> void {
    spdlog::drop(name.data());
}

[[using gnu: always_inline]]
inline auto dropAllLogger() -> void {
    spdlog::drop_all();
}

[[using gnu: always_inline]]
inline auto registerLogger(std::shared_ptr<Logger> logger) -> void {
    spdlog::register_logger(std::move(logger));
}

[[using gnu: always_inline]]
inline auto setDefaultLogger(std::shared_ptr<Logger> logger) -> void {
    spdlog::set_default_logger(std::move(logger));
}

[[using gnu: always_inline]] [[nodiscard]]
inline auto getDefaultLogger() -> std::shared_ptr<Logger> {
    return spdlog::default_logger();
}

} // namespace basjoo::common
