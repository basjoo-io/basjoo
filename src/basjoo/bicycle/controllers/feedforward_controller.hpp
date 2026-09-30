/**
 * @file feedforward_controller.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-08-04
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>

namespace basjoo::bicycle {

template <std::floating_point T>
class [[nodiscard]] FeedforwardController final {
  public:
    using value_type = T;
    using size_type = std::size_t;

    FeedforwardController() noexcept = default;
    FeedforwardController(const FeedforwardController& other) noexcept = delete;
    auto operator=(const FeedforwardController& other) noexcept -> FeedforwardController& = delete;
    FeedforwardController(FeedforwardController&& other) noexcept = default;
    auto operator=(FeedforwardController&& other) noexcept -> FeedforwardController& = default;
    ~FeedforwardController() noexcept = default;

  private:
};

} // namespace basjoo::bicycle
