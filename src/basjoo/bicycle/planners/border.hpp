/**
 * @file border.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-06
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cstdint>
#include <limits>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/bicycle/planners/dualism.hpp"
#include "basjoo/math/dense/vec2.hpp"

namespace basjoo::bicycle {

template <::basjoo::math::Vec2Arithmetic T>
struct HardBorder final {
    using serialize = zpp::bits::members<3>;

    using value_type = T;
    std::uint64_t id{std::numeric_limits<std::uint64_t>::quiet_NaN()};
    ::basjoo::bicycle::Chirality chirality{::basjoo::bicycle::Chirality::LEFT};
    std::pmr::vector<value_type> bound_points;
};

template <::basjoo::math::Vec2Arithmetic T>
struct SoftBorder final {
    using serialize = zpp::bits::members<5>;

    using value_type = T;
    std::uint64_t id{std::numeric_limits<std::uint64_t>::quiet_NaN()};
    ::basjoo::bicycle::Chirality chirality{::basjoo::bicycle::Chirality::LEFT};
    std::pmr::vector<value_type> bound_points;
    typename value_type::value_type linear_weight{0.0};
    typename value_type::value_type quadratic_weight{0.0};
};

using HardBorder2s = HardBorder<::basjoo::math::Vec2s>;

using HardBorder2d = HardBorder<::basjoo::math::Vec2d>;

using SoftBorder2s = SoftBorder<::basjoo::math::Vec2s>;

using SoftBorder2d = SoftBorder<::basjoo::math::Vec2d>;

} // namespace basjoo::bicycle
