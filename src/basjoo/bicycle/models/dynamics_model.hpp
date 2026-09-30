/**
 * @file dynamics_model.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-08-08
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>

#include "basjoo/bicycle/models/state.hpp"

namespace basjoo::bicycle {

template <std::floating_point T>
class DynamicsModel final {
  public:
    using value_type = T;
    using state_type = State<value_type>;

    DynamicsModel() noexcept = default;
    DynamicsModel(const DynamicsModel& other) noexcept = default;
    auto operator=(const DynamicsModel& other) noexcept -> DynamicsModel& = default;
    DynamicsModel(DynamicsModel&& other) noexcept = default;
    auto operator=(DynamicsModel&& other) noexcept -> DynamicsModel& = default;
    ~DynamicsModel() noexcept = default;
};

} // namespace basjoo::bicycle
