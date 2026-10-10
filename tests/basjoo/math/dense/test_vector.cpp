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

#include "basjoo/math/dense/matrix.hpp"
#include "basjoo/math/dense/vector.hpp"

#include <array>
#include <complex>
#include <span>
#include <type_traits>
#include <utility>

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
    Vector<float, 16>, Vector<double, 16>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>,
    Vector<std::complex<float>, 16>, Vector<std::complex<double>, 16>,
    Vector<std::complex<float>, std::dynamic_extent>, Vector<std::complex<double>, std::dynamic_extent>) {
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

    // Strided reading through a matrix column block (replaces the old strided
    // VectorView case): a 4x4 column-major matrix's column slice exposes the
    // same stride-4 access pattern — element k sits at storage offset 4 + 4k.
    Matrix<typename T::value_type, std::dextents<std::size_t, 2>, std::layout_left> square{
        4, 4, typename T::value_type{0.0}
    };
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            square.data()[i] = kRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            square.data()[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
        }
    }
    auto col_block{square.submdspan(
        std::pair<std::size_t, std::size_t>{0, 1}, std::pair<std::size_t, std::size_t>{1, 4}
    )};
    CHECK_EQ(col_block.stride(1), 4);
    for (std::size_t i{0}; i < 3; ++i) {
        if constexpr (std::is_arithmetic_v<typename T::value_type>) {
            CHECK_EQ(b[4 + 4 * i], col_block[0, i]);
        } else {
            CHECK_EQ(b[4 + 4 * i].real(), col_block[0, i].real());
            CHECK_EQ(b[4 + 4 * i].imag(), col_block[0, i].imag());
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

// clang-format off
TEST_CASE_TEMPLATE("operator value categories reuse rvalue operand storage", T,
    Vector<float, 16>, Vector<double, 16>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>,
    Vector<std::complex<float>, 16>, Vector<std::complex<double>, 16>,
    Vector<std::complex<float>, std::dynamic_extent>, Vector<std::complex<double>, std::dynamic_extent>) {
    // clang-format on

    T a(kNRows);
    T b(kNRows);
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            a[i] = kRealValues[i];
            b[i] = kImagValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            a[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
            b[i] = typename T::value_type(kImagValues[i], kRealValues[i]);
        }
    }

    SUBCASE("lvalue + lvalue returns an independent object") {
        const T expected = a + b;
        CHECK(expected.identicalTo(a + b));
        CHECK(&expected != &a);
        CHECK(&expected != &b);
    }

    SUBCASE("lvalue + rvalue reuses the rvalue operand") {
        T c{b};
        const T expected = a + c;
        T&& r = a + std::move(c);
        static_assert(std::is_same_v<decltype((a + std::move(c))), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue + lvalue reuses the rvalue self") {
        T d{a};
        const T expected = d + b;
        T&& r = std::move(d) + b;
        static_assert(std::is_same_v<decltype((std::move(d) + b)), T&&>);
        CHECK(&r == &d);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue + rvalue reuses the rvalue self") {
        T d{a};
        const T expected = d + b;
        T&& r = std::move(d) + T(b);
        CHECK(&r == &d);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("lvalue - rvalue reuses the rvalue operand") {
        T c{b};
        const T expected = a - c;
        T&& r = a - std::move(c);
        static_assert(std::is_same_v<decltype((a - std::move(c))), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("unary minus on rvalue reuses the operand") {
        T c{a};
        const T expected = -c;
        T&& r = -std::move(c);
        static_assert(std::is_same_v<decltype((-std::move(c))), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue * scalar reuses the operand") {
        T c{a};
        const T expected = c * 2.0;
        T&& r = std::move(c) * 2.0;
        static_assert(std::is_same_v<decltype((std::declval<T&&>() * 2.0)), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("scalar * rvalue reuses the operand") {
        T c{b};
        const T expected = 3.0 * c;
        T&& r = 3.0 * std::move(c);
        static_assert(std::is_same_v<decltype((3.0 * std::declval<T&&>())), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue / scalar reuses the operand") {
        T c{a};
        const T expected = c / 2.0;
        T&& r = std::move(c) / 2.0;
        static_assert(std::is_same_v<decltype((std::declval<T&&>() / 2.0)), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("conjugated on rvalue reuses the operand") {
        T c{b};
        const T expected = c.conjugated();
        T&& r = std::move(c).conjugated();
        static_assert(std::is_same_v<decltype((std::declval<T&&>().conjugated())), T&&>);
        CHECK(&r == &c);
        CHECK(r.identicalTo(expected));
    }
}

// clang-format off
TEST_CASE_TEMPLATE("span and subspan expose contiguous storage with static extent", T,
    Vector<float, 16>, Vector<double, 16>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>,
    Vector<std::complex<float>, 16>, Vector<std::complex<double>, 16>,
    Vector<std::complex<float>, std::dynamic_extent>, Vector<std::complex<double>, std::dynamic_extent>) {
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

    using span_type = decltype(std::span<const typename T::value_type>{b});
    using const_span_type = decltype(std::span<const typename T::value_type>{std::as_const(b)});
    static_assert(std::is_same_v<typename span_type::element_type, const typename T::value_type>);
    static_assert(
        std::is_same_v<typename const_span_type::element_type, const typename T::value_type>
    );
    if constexpr (span_type::extent == std::dynamic_extent) {
        static_assert(span_type::extent == std::dynamic_extent);
    } else {
        static_assert(span_type::extent == kNRows);
    }
    const std::span<const typename T::value_type> b_view{b};
    CHECK_EQ(b_view.data(), b.data());
    CHECK_EQ(b_view.size(), kNRows);
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_arithmetic_v<typename T::value_type>) {
            CHECK_EQ(b[i], b_view[i]);
        } else {
            CHECK_EQ(b[i].real(), b_view[i].real());
            CHECK_EQ(b[i].imag(), b_view[i].imag());
        }
    }
    std::span<typename T::value_type> b_mut_view{b};
    b_mut_view[0] = b[0];
    CHECK_EQ(b_mut_view[0], b[0]);

    SUBCASE("vector construction from a matrix mdspan requires vector-like shape") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        Matrix<typename T::value_type, std::dextents<std::size_t, 2>, std::layout_left> square{
            4, 4, typename T::value_type{0.0}
        };
        for (std::size_t i{0}; i < kNRows; ++i) {
            if constexpr (std::is_floating_point_v<typename T::value_type>) {
                square.data()[i] = kRealValues[i];
            }
            if constexpr (isComplexArithmeticV<typename T::value_type>) {
                square.data()[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
            }
        }
        // a full row (1x16) and a full column (16x1) both convert to the vector
        Matrix<typename T::value_type, std::dextents<std::size_t, 2>, std::layout_right>
            row_like{1, kNRows, typename T::value_type{0.0}};
        Matrix<typename T::value_type, std::dextents<std::size_t, 2>, std::layout_left>
            col_like{kNRows, 1, typename T::value_type{0.0}};
        for (std::size_t i{0}; i < kNRows; ++i) {
            if constexpr (std::is_floating_point_v<typename T::value_type>) {
                row_like.data()[i] = kRealValues[i];
                col_like.data()[i] = kRealValues[i];
            }
            if constexpr (isComplexArithmeticV<typename T::value_type>) {
                row_like.data()[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
                col_like.data()[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
            }
        }
        T from_row{row_like.submdspan(pair_type{0, 1}, pair_type{0, kNRows})};
        T from_col{col_like.submdspan(pair_type{0, kNRows}, pair_type{0, 1})};
        for (std::size_t i{0}; i < kNRows; ++i) {
            if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                CHECK_EQ(b[i], from_row[i]);
                CHECK_EQ(b[i], from_col[i]);
            } else {
                CHECK_EQ(b[i].real(), from_row[i].real());
                CHECK_EQ(b[i].imag(), from_row[i].imag());
                CHECK_EQ(b[i].real(), from_col[i].real());
                CHECK_EQ(b[i].imag(), from_col[i].imag());
            }
        }
#if BASJOO_CHECK_PARAMS == 1
        CHECK_THROWS_AS(T(square.submdspan(pair_type{0, 2}, pair_type{0, 2})), std::out_of_range);
#endif
    }

    SUBCASE("subspan returns a contiguous view over the half-open range") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        auto sub{b.subspan(pair_type{4, 9})};
        static_assert(std::is_same_v<typename decltype(sub)::element_type, typename T::value_type>);
        CHECK_EQ(sub.size(), 5);
        CHECK_EQ(sub.data(), b.data() + 4);
        for (std::size_t k{0}; k < 5; ++k) {
            if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                CHECK_EQ(b[4 + k], sub[k]);
            } else {
                CHECK_EQ(b[4 + k].real(), sub[k].real());
                CHECK_EQ(b[4 + k].imag(), sub[k].imag());
            }
        }
        auto empty{b.subspan(pair_type{7, 7})};
        CHECK(empty.empty());
        auto whole{b.subspan(pair_type{0, kNRows})};
        CHECK_EQ(whole.data(), b.data());
        CHECK_EQ(whole.size(), kNRows);
    }

#if BASJOO_CHECK_PARAMS == 1
    SUBCASE("subspan validates the range") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        CHECK_THROWS_AS(b.subspan(pair_type{5, 4}), std::out_of_range);
        CHECK_THROWS_AS(b.subspan(pair_type{0, kNRows + 1}), std::out_of_range);
    }
#endif
}

// Implicit conversion to std::span keeps the static extent of the vector.
static_assert(std::is_convertible_v<Vector<double, 16>, std::span<double, 16>>);
static_assert(std::is_convertible_v<const Vector<double, 16>, std::span<const double, 16>>);
static_assert(std::is_convertible_v<Vector<double, std::dynamic_extent>, std::span<double>>);
static_assert(
    std::is_convertible_v<const Vector<double, std::dynamic_extent>, std::span<const double>>
);


// clang-format off
TEST_CASE_TEMPLATE("span-shaped query surface: front/back/size_bytes/iterators", T,
    Vector<float, 16>, Vector<double, 16>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>,
    Vector<std::complex<float>, 16>, Vector<std::complex<double>, 16>,
    Vector<std::complex<float>, std::dynamic_extent>, Vector<std::complex<double>, std::dynamic_extent>) {
    // clang-format on
    static_assert(std::ranges::contiguous_range<T>);
    static_assert(std::is_same_v<typename T::element_type, typename T::value_type>);
    T b(kNRows);
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            b[i] = kRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            b[i] = typename T::value_type(kRealValues[i], kImagValues[i]);
        }
    }
    CHECK_EQ(b.front(), b[0]);
    CHECK_EQ(b.back(), b[kNRows - 1]);
    CHECK_EQ(b.size_bytes(), kNRows * sizeof(typename T::value_type));
    double forward_sum{0.0};
    for (auto it = b.begin(); it != b.end(); ++it) {
        forward_sum += 0.0; // complex handled below
    }
    if constexpr (std::is_floating_point_v<typename T::value_type>) {
        double total{0.0};
        for (auto x : b) { total += x; }
        CHECK_EQ(total, doctest::Approx(b.euclideanSqr()).epsilon(1E-6));
    }
    for (auto rit = b.rbegin(); rit != b.rend(); ++rit) {
        (void)rit;
    }
    CHECK_EQ(*b.rbegin(), b[kNRows - 1]);
    CHECK_EQ(*b.cbegin(), b[0]);
    (void)forward_sum;
}

} // namespace basjoo::math
