/**
 * @file test_complex_formatter.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit tests for std::formatter of complex scalars.
 * @version 0.1
 * @date 2026-10-13
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/complex_formatter.hpp"

#include <complex>
#include <format>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

TEST_CASE_TEMPLATE(
    "std::formatter prints complex scalars as (re+imi)", T, std::complex<float>,
    std::complex<double>
) {
    CHECK_EQ(std::format("{}", T(1.0, -1.0)), "(1-1i)");
    CHECK_EQ(std::format("{}", T(0.0, 2.0)), "(0+2i)");
    CHECK_EQ(std::format("{:.1f}", T(1.0, 2.0)), "(1.0+2.0i)");
    CHECK_EQ(std::format("{:.2e}", T(-1.0, -0.5)), "(-1.00e+00-5.00e-01i)");
}

TEST_CASE_TEMPLATE(
    "std::formatter always shows the imaginary sign of complex scalars", T, std::complex<float>,
    std::complex<double>
) {
    const T a{1.0, 2.0};
    CHECK_EQ(std::format("{:+}", a), "(+1+2i)");
    CHECK_EQ(std::format("{:-}", a), "(1+2i)");
    CHECK_EQ(std::format("{: }", a), "( 1+2i)");
}

TEST_CASE_TEMPLATE(
    "std::formatter applies fill and align to both complex parts", T, std::complex<float>,
    std::complex<double>
) {
    const T a{1.0, 2.0};
    CHECK_EQ(std::format("{:>4}", a), "(   1  +2i)");
    CHECK_EQ(std::format("{:*<4}", a), "(1***+2**i)");
    CHECK_EQ(std::format("{:*>+4}", a), "(**+1**+2i)");
}

TEST_CASE_TEMPLATE(
    "std::formatter rejects dynamic width for complex scalars", T, std::complex<float>,
    std::complex<double>
) {
    const T a{1.0, 2.0};
    int width{8};
    CHECK_THROWS_AS(
        static_cast<void>(std::vformat("{:{}}", std::make_format_args(a, width))), std::format_error
    );
}

TEST_CASE("std::formatter supports wide characters") {
    CHECK(std::format(L"{:.1f}", std::complex<double>{1.0, -2.0}) == L"(1.0-2.0i)");
}
