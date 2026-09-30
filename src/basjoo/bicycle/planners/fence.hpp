/**
 * @file fence.hpp
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

#include <concepts>
#include <cstdint>
#include <limits>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/bicycle/planners/dualism.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"
#include "basjoo/math/dense/vec2.hpp"

namespace basjoo::bicycle {

template <typename T>
    requires std::floating_point<T> || ::basjoo::math::VecArithmetic<T>
struct HardFence final {
    using serialize = zpp::bits::members<4>;

    using value_type = T;
    using param_type = ::basjoo::math::detail::DenseDegenerateTraitT<value_type>;
    std::uint64_t id{std::numeric_limits<std::uint64_t>::quiet_NaN()};
    ::basjoo::bicycle::Actio actio{::basjoo::bicycle::Actio::BLOCKING};
    std::pmr::vector<param_type> bound_ts;
    std::pmr::vector<value_type> bound_ss;
};

template <typename T>
    requires std::floating_point<T> || ::basjoo::math::VecArithmetic<T>
struct SoftFence final {
    using serialize = zpp::bits::members<6>;

    using value_type = T;
    using param_type = ::basjoo::math::detail::DenseDegenerateTraitT<value_type>;
    std::uint64_t id{std::numeric_limits<std::uint64_t>::quiet_NaN()};
    ::basjoo::bicycle::Actio actio{::basjoo::bicycle::Actio::BLOCKING};
    std::pmr::vector<param_type> bound_ts;
    std::pmr::vector<value_type> bound_ss;
    param_type linear_weight{0.0};
    param_type quadratic_weight{0.0};
};

using HardFence1s = HardFence<float>;

using HardFence1d = HardFence<double>;

using SoftFence1s = SoftFence<float>;

using SoftFence1d = SoftFence<double>;

using HardFence2s = HardFence<::basjoo::math::Vec2s>;

using HardFence2d = HardFence<::basjoo::math::Vec2d>;

using SoftFence2s = SoftFence<::basjoo::math::Vec2s>;

using SoftFence2d = SoftFence<::basjoo::math::Vec2d>;

} // namespace basjoo::bicycle
