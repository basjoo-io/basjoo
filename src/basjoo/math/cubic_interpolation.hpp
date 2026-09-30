/**
 * @file cubic_interpolation.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <array>
#include <concepts>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/utils.hpp"

namespace basjoo::math {

template <GeneralArithmetic T, std::floating_point U = double>
[[using gnu: const, always_inline, hot]]
inline constexpr auto cuberp(T start, T end, T ddstart, T ddend, U ratio, U scale = 1.0) noexcept
    -> T {
    constexpr std::array<double, 2> kFactors{-(1.0 / 6.0), 1.0 / 6.0};
    const U ratio3{ratio * ratio * ratio};
    const U resi_ratio{1.0 - ratio};
    const U resi_ratio3{resi_ratio * resi_ratio * resi_ratio};
    return lerp(start, end, ratio) +
           ((resi_ratio * kFactors[0] + resi_ratio3 * kFactors[1]) * ddstart +
            (ratio * kFactors[0] + ratio3 * kFactors[1]) * ddend) *
               scale * scale;
}

template <GeneralArithmetic T, std::floating_point U = double>
[[using gnu: const, always_inline, hot]]
inline constexpr auto cuberpd(T start, T end, T ddstart, T ddend, U ratio, U scale = 1.0) noexcept
    -> T {
    constexpr std::array<U, 2> kFactors{-(1.0 / 6.0), 0.5};
    const U ratio2{ratio * ratio};
    const U resi_ratio{1.0 - ratio};
    const U resi_ratio2{resi_ratio * resi_ratio};
    return (end - start) / scale + (-(kFactors[0] + resi_ratio2 * kFactors[1]) * ddstart +
                                    (kFactors[0] + ratio2 * kFactors[1]) * ddend) *
                                       scale;
}

template <std::floating_point T = double>
[[using gnu: const, always_inline]]
inline constexpr auto cuberpCoeffs(T ratio, T scale = 1.0) noexcept -> std::array<T, 4> {
    constexpr std::array<T, 2> kFactors{-(1.0 / 6.0), 1.0 / 6.0};
    const T ratio3{ratio * ratio * ratio};
    const T resi_ratio{1.0 - ratio};
    const T resi_ratio3{resi_ratio * resi_ratio * resi_ratio};
    const T scale2{scale * scale};
    return std::array<T, 4>{
        resi_ratio, ratio, (resi_ratio * kFactors[0] + resi_ratio3 * kFactors[1]) * scale2,
        (ratio * kFactors[0] + ratio3 * kFactors[1]) * scale2
    };
}

template <std::floating_point T = double>
[[using gnu: const, always_inline]]
inline constexpr auto cuberpdCoeffs(T ratio, T scale = 1.0) noexcept -> std::array<T, 4> {
    constexpr std::array<T, 2> kFactors{-(1.0 / 6.0), 0.5};
    const T ratio2{ratio * ratio};
    const T resi_ratio{1.0 - ratio};
    const T resi_ratio2{resi_ratio * resi_ratio};
    const T reci_scale{1.0 / scale};
    return std::array<T, 4>{
        -reci_scale, reci_scale, -(kFactors[0] + resi_ratio2 * kFactors[1]) * scale,
        (kFactors[0] + ratio2 * kFactors[1]) * scale
    };
}

template <VecArithmetic T>
[[using gnu: const, flatten, leaf, hot]]
inline constexpr auto calcArcLength(
    T start, T end, T ddstart, T ddend, typename T::value_type scale
) noexcept -> typename T::value_type {
    using value_type = typename T::value_type;

    /* using boost::math::ccmath::sqrt;
    constexpr std::size_t kNumGaussLegendreKnots{5};
    constexpr std::array<value_type, 5> kGaussLegendreKnots{
        0.5 - sqrt(5.0 + 2.0 * sqrt(10.0 / 7.0)) / 6.0,
        0.5 - sqrt(5.0 - 2.0 * sqrt(10.0 / 7.0)) / 6.0, 0.5,
        0.5 + sqrt(5.0 - 2.0 * sqrt(10.0 / 7.0)) / 6.0,
        0.5 + sqrt(5.0 + 2.0 * sqrt(10.0 / 7.0)) / 6.0};
    constexpr std::array<value_type, 5> kGaussLegendreWeights{
        (322.0 - 13.0 * sqrt(70.0)) / 1800.0, (322.0 + 13.0 * sqrt(70.0)) / 1800.0,
        128.0 / 450.0, (322.0 + 13.0 * sqrt(70.0)) / 1800.0,
        (322.0 - 13.0 * sqrt(70.0)) / 1800.0}; */
    constexpr std::size_t kNumGaussLegendreKnots{5};
    constexpr std::array<value_type, 5> kGaussLegendreKnots{
        0.04691007703066800360, 0.23076534494715845448, 0.5, 0.76923465505284154551,
        0.95308992296933199639
    };
    constexpr std::array<value_type, 5> kGaussLegendreWeights{
        0.11846344252809454375, 0.23931433524968323402, 128.0 / 450.0, 0.23931433524968323402,
        0.11846344252809454375
    };

    value_type result{0.0};
    for (std::size_t i = 0; i < kNumGaussLegendreKnots; ++i) {
        result += cuberpd(start, end, ddstart, ddend, kGaussLegendreKnots[i], scale).euclidean() *
                  kGaussLegendreWeights[i];
    }
    result *= scale;
    return result;
}

} // namespace basjoo::math
