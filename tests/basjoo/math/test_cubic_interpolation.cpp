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

} // namespace basjoo::math
