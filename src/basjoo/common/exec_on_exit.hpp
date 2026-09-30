/**
 * @file exec_on_exit.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-23
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <functional>
#include <utility>

namespace basjoo::common {

class [[nodiscard]] ExecOnExit final {
  public:
    using Callback = std::function<auto()->void>;

    ExecOnExit() noexcept = delete;
    ExecOnExit(const ExecOnExit& other) noexcept = delete;
    auto operator=(const ExecOnExit& other) noexcept -> ExecOnExit& = delete;
    ExecOnExit(ExecOnExit&& other) noexcept = delete;
    auto operator=(ExecOnExit&& other) noexcept -> ExecOnExit& = delete;
    ~ExecOnExit() { m_callback(); }

    [[using gnu: always_inline]]
    ExecOnExit(Callback callback) noexcept
        : m_callback{std::move(callback)} {}

  private:
    Callback m_callback;
};

} // namespace basjoo::common
