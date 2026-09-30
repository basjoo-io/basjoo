/**
 * @file test_vector.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vector_view.hpp"
#include "basjoo/math/dense/vectorx.hpp"

#include <array>
#include <complex>

#include "zpp_bits.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace {

constexpr std::size_t kNRows{16};

constexpr std::array<double, kNRows> kRealValues{-281.2042, 201.0305,  860.8241,  574.7355,
                                                 -478.0013, 922.1276,  297.3931,  82.7102,
                                                 20.8434,   178.7692,  -637.5437, -287.4113,
                                                 417.9586,  -963.2638, 852.9532,  -409.2006};

constexpr std::array<double, kNRows> kImagValues{941.3074,  219.3982,  -898.9647, 512.4636,
                                                 -862.3371, -639.5703, 820.1686,  524.2200,
                                                 575.2414,  219.9055,  -736.2512, -789.8754,
                                                 -511.0101, -449.6304, 651.3532,  -448.5029};

} // namespace

namespace basjoo::math {

using namespace std::literals::complex_literals;

// clang-format off
TEST_CASE_TEMPLATE("VectorTest", T,
    Vector<float, 16>, Vector<double, 16>, VectorX<float>, VectorX<double>,
    Vector<std::complex<float>, 16>, Vector<std::complex<double>, 16>,
    VectorX<std::complex<float>>, VectorX<std::complex<double>>) {
    // clang-format on

    T b(kNRows);
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            b[i] = kRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            b[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
        }
    }

    CHECK(b.identicalTo(b * 1.000000001));
    CHECK_FALSE(b.identicalTo(b * 1.0000001));

    const T b_conj = b.conjugated();
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_same_v<typename T::value_type, float>) {
            CHECK_EQ(b_conj[i], doctest::Approx(kRealValues[i]).epsilon(6E-8));
        }
        if constexpr (std::is_same_v<typename T::value_type, double>) {
            CHECK_EQ(b_conj[i], doctest::Approx(kRealValues[i]).epsilon(1E-16));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
            const typename T::value_type& val{b_conj[i]};
            CHECK_EQ(val.real(), doctest::Approx(kRealValues[i]).epsilon(6E-8));
            CHECK_EQ(val.imag(), doctest::Approx(-kImagValues[i]).epsilon(6E-8));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
            const typename T::value_type& val{b_conj[i]};
            CHECK_EQ(val.real(), doctest::Approx(kRealValues[i]).epsilon(1E-16));
            CHECK_EQ(val.imag(), doctest::Approx(-kImagValues[i]).epsilon(1E-16));
        }
    }

    VectorView<typename T::value_type> vector_view{b.data() + 4, 3, 4};
    for (std::size_t i{0}; i < 3; ++i) {
        if constexpr (std::is_arithmetic_v<typename T::value_type>) {
            CHECK_EQ(b[4 + 4 * i], vector_view[i]);
        } else {
            CHECK_EQ(b[4 + 4 * i].real(), vector_view[i].real());
            CHECK_EQ(b[4 + 4 * i].imag(), vector_view[i].imag());
        }
    }

    const T result = (b * 10.5634) + (b / 5.34) - (b * 0.67);
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_same_v<typename T::value_type, float>) {
            CHECK_EQ(
                result[i], doctest::Approx(b[i] * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-7)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, double>) {
            CHECK_EQ(
                result[i], doctest::Approx(b[i] * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-12)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
            CHECK_EQ(
                result[i].real(),
                doctest::Approx(b[i].real() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-7)
            );
            CHECK_EQ(
                result[i].imag(),
                doctest::Approx(b[i].imag() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-7)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
            CHECK_EQ(
                result[i].real(),
                doctest::Approx(b[i].real() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-12)
            );
            CHECK_EQ(
                result[i].imag(),
                doctest::Approx(b[i].imag() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-12)
            );
        }
    }

    auto [data, out] = zpp::bits::data_out();
    out(b).or_throw();

    T other_b;
    auto in = zpp::bits::in(data);
    in(other_b).or_throw();

    CHECK_EQ(b, other_b);
}

} // namespace basjoo::math
