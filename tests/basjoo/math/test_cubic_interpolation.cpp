/**
 * @file test_cubic_interpolation.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/cubic_interpolation.hpp"

#include <array>
#include <ranges>

#include "basjoo/math/utils.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("CubicPolynomial") {
    constexpr auto func = [](double t) noexcept -> double {
        return 28.483 - 2.34 * t + 1.54 * t * t + 0.365 * t * t * t;
    };
    constexpr auto dfunc = [](double t) noexcept -> double {
        return -2.34 + 3.08 * t + 1.095 * t * t;
    };
    constexpr auto ddfunc = [](double t) noexcept -> double { return 3.08 + 2.19 * t; };

    constexpr double lo = -23.576;
    constexpr double up = 97.273;
    constexpr double start = func(lo);
    constexpr double end = func(up);
    constexpr double ddstart = ddfunc(lo);
    constexpr double ddend = ddfunc(up);
    constexpr double scale = up - lo;

    for (double ratio : linspace(0.0, 1.0, 191)) {
        CHECK_EQ(
            cuberp(start, end, ddstart, ddend, ratio, scale),
            doctest::Approx(func(lo + scale * ratio)).epsilon(kEpsilon)
        );
        CHECK_EQ(
            cuberpd(start, end, ddstart, ddend, ratio, scale),
            doctest::Approx(dfunc(lo + scale * ratio)).epsilon(kEpsilon)
        );
    }
}

TEST_CASE("FloatAgreesWithDouble") {
    // A float ratio used to be narrowed from the double 1.0 - ratio inside a braced initializer,
    // which clang rejects outright. Every input is exact in float, so float and double agree to
    // float precision.
    constexpr double start{1.25};
    constexpr double end{-0.75};
    constexpr double ddstart{0.5};
    constexpr double ddend{-1.5};
    constexpr double scale{2.0};
    constexpr double kFloatEpsilon{1E-5};

    for (const double ratio : linspace(0.0, 1.0, 11)) {
        const auto ratio_s{static_cast<float>(ratio)};
        CHECK_EQ(
            cuberp(1.25F, -0.75F, 0.5F, -1.5F, ratio_s, 2.0F),
            doctest::Approx(cuberp(start, end, ddstart, ddend, ratio, scale)).epsilon(kFloatEpsilon)
        );
        CHECK_EQ(
            cuberpd(1.25F, -0.75F, 0.5F, -1.5F, ratio_s, 2.0F),
            doctest::Approx(cuberpd(start, end, ddstart, ddend, ratio, scale))
                .epsilon(kFloatEpsilon)
        );
        const std::array<float, 4> coeffs_s{cuberpCoeffs(ratio_s, 2.0F)};
        const std::array<double, 4> coeffs{cuberpCoeffs(ratio, scale)};
        const std::array<float, 4> dcoeffs_s{cuberpdCoeffs(ratio_s, 2.0F)};
        const std::array<double, 4> dcoeffs{cuberpdCoeffs(ratio, scale)};
        for (const auto [c_s, c, d_s, d] : std::views::zip(coeffs_s, coeffs, dcoeffs_s, dcoeffs)) {
            CHECK_EQ(c_s, doctest::Approx(c).epsilon(kFloatEpsilon));
            CHECK_EQ(d_s, doctest::Approx(d).epsilon(kFloatEpsilon));
        }
    }
}

} // namespace basjoo::math
