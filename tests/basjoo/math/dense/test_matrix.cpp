/**
 * @file test_matrix.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-12-30
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/matrix.hpp"

#include <array>
#include <complex>
#include <mdspan>
#include <span>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace {

constexpr std::size_t kNRows{16};
constexpr std::size_t kNCols{16};
constexpr std::size_t kNumValues{77};

constexpr std::array<double, kNumValues> kRealValues{
    -341.9800, 389.5306,  49.7875,   636.8560,  56.3279,   382.6567,  199.5139,  223.3316,
    -704.0624, 116.5993,  519.7040,  -467.3498, 455.8335,  -245.0498, -529.3769, 315.3701,
    565.4057,  -881.4848, 348.5747,  708.7613,  -204.9656, -88.6024,  480.0442,  -270.1055,
    250.5040,  -501.4125, 957.2561,  623.6217,  -315.1810, -696.8023, 654.1732,  429.1730,
    -778.6769, -943.8449, 731.2714,  461.2086,  -930.5009, -197.0783, 783.7394,  -409.2122,
    -956.0648, 569.6860,  -239.4499, 39.0720,   -349.2786, 72.3607,   -635.8949, 855.9547,
    -423.7498, 3.6599,    -13.8725,  -388.9426, 200.5421,  -906.4672, 376.0567,  -631.1843,
    -362.1610, -687.6382, 596.7392,  585.3998,  971.3544,  -8.6622,   300.9929,  360.6437,
    -114.1879, -115.8336, 919.4183,  -689.1939, 926.1460,  -827.5229, 272.8555,  682.6263,
    262.3042,  -912.8410, 392.7486,  130.4278,  -589.0882
};

constexpr std::array<double, kNumValues> kImagValues{
    -190.0146, 348.1628,  -371.7530, 818.8321,  -496.5735, 935.1387,  609.8435,  -734.0485,
    465.8235,  -98.9205,  -658.6971, -646.8584, -658.6615, -919.6176, -313.8385, 328.3800,
    -274.6760, 131.3840,  788.5373,  -73.3230,  -867.8488, 172.0961,  -634.2364, -566.8694,
    305.7612,  790.1574,  -153.8988, 423.7151,  -499.5920, -227.2687, 452.6295,  235.8998,
    -820.1324, -813.8525, 3.5817,    961.9284,  453.5152,  610.3250,  -292.2984, -208.5202,
    -549.0027, 52.4853,   175.2202,  423.0728,  -868.1409, -313.0624, -821.3611, 770.6318,
    439.8532,  343.7817,  680.7724,  -739.1827, 281.3673,  895.8740,  -792.6267, 806.2494,
    580.9240,  -973.9816, -507.2850, -196.6030, 282.2714,  348.9859,  -630.7426, 13.9469,
    -503.2322, 646.6180,  -929.6460, 211.3390,  570.2121,  -227.1294, 696.4385,  -725.3564,
    -563.1742, -647.3890, 727.4664,  277.0857,  -131.9151
};

constexpr std::array<std::size_t, kNumValues> kRowIndices{
    0,  1,  2, 3,  4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 1,  8,  8, 14,
    11, 11, 9, 10, 15, 13, 8,  13, 9,  12, 12, 11, 0,  15, 8,  2,  9,  3,  0, 0,
    1,  11, 9, 11, 7,  11, 14, 5,  14, 8,  10, 5,  14, 8,  1,  9,  15, 13, 3, 5,
    8,  14, 0, 5,  9,  10, 14, 15, 3,  11, 1,  8,  15, 9,  7,  11, 6
};

constexpr std::array<std::size_t, kNumValues> kColIndices{
    0,  1, 2,  3, 4,  5,  6,  7,  8,  9,  10, 11, 12, 13, 14, 15, 4,  2,  0,  3,
    1,  9, 5,  4, 13, 14, 10, 5,  2,  3,  5,  4,  4,  14, 1,  9,  15, 11, 8,  1,
    12, 5, 10, 8, 8,  10, 4,  10, 2,  5,  6,  9,  12, 4,  11, 11, 12, 3,  10, 0,
    13, 7, 15, 8, 1,  11, 15, 7,  12, 12, 10, 3,  9,  8,  13, 6,  1
};

} // namespace

namespace basjoo::math {

using namespace std::literals::complex_literals;

// clang-format off
TEST_CASE_TEMPLATE("MatrixTest", T,
    Matrix<float, std::extents<std::size_t, 16, 16>, std::layout_left>, Matrix<double, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<float, std::extents<std::size_t, 16, 16>, std::layout_right>, Matrix<double, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<float, std::dextents<std::size_t, 2>, std::layout_left>, Matrix<double, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<float, std::dextents<std::size_t, 2>, std::layout_right>, Matrix<double, std::dextents<std::size_t, 2>, std::layout_right>,
    Matrix<std::complex<float>, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<std::complex<double>, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<std::complex<float>, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<std::complex<double>, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<std::complex<float>, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<std::complex<double>, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<std::complex<float>, std::dextents<std::size_t, 2>, std::layout_right>,
    Matrix<std::complex<double>, std::dextents<std::size_t, 2>, std::layout_right>) {
    // clang-format on

    T A(kNRows, kNCols, typename T::value_type{0.0});
    for (std::size_t i{0}; i < kNumValues; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] = kRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] =
                typename T::value_type(kRealValues[i], kImagValues[i]);
        }
    }

    CHECK(A.identicalTo(A * 1.000000001));
    CHECK_FALSE(A.identicalTo(A * 1.0000001));

    const T A_adj = A.adjoint();
    for (std::size_t i{0}; i < kNumValues; ++i) {
        if constexpr (std::is_same_v<typename T::value_type, float>) {
            CHECK_EQ(
                A_adj[kColIndices[i], kRowIndices[i]], doctest::Approx(kRealValues[i]).epsilon(5E-8)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, double>) {
            CHECK_EQ(
                A_adj[kColIndices[i], kRowIndices[i]],
                doctest::Approx(kRealValues[i]).epsilon(1E-16)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
            const typename T::value_type& val{A_adj[kColIndices[i], kRowIndices[i]]};
            CHECK_EQ(val.real(), doctest::Approx(kRealValues[i]).epsilon(5E-8));
            CHECK_EQ(val.imag(), doctest::Approx(-kImagValues[i]).epsilon(5E-8));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
            const typename T::value_type& val{A_adj[kColIndices[i], kRowIndices[i]]};
            CHECK_EQ(val.real(), doctest::Approx(kRealValues[i]).epsilon(1E-16));
            CHECK_EQ(val.imag(), doctest::Approx(-kImagValues[i]).epsilon(1E-16));
        }
    }

    auto matrix_mdspan{A.submdspan({6, 12}, {1, 7})};
    for (std::size_t i{0}; i < 6; ++i) {
        for (std::size_t j{0}; j < 6; ++j) {
            if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                CHECK_EQ(A[6 + i, 1 + j], matrix_mdspan[i, j]);
            } else {
                CHECK_EQ(A[6 + i, 1 + j].real(), matrix_mdspan[i, j].real());
                CHECK_EQ(A[6 + i, 1 + j].imag(), matrix_mdspan[i, j].imag());
            }
        }
    }

    const T result = (A * 10.5634) + (A / 5.34) - (A * 0.67);
    for (std::size_t i{0}; i < kNRows; ++i) {
        for (std::size_t j{0}; j < kNCols; ++j) {
            if constexpr (std::is_same_v<typename T::value_type, float>) {
                CHECK_EQ(
                    result[i, j],
                    doctest::Approx(A[i, j] * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-7)
                );
            }
            if constexpr (std::is_same_v<typename T::value_type, double>) {
                CHECK_EQ(
                    result[i, j],
                    doctest::Approx(A[i, j] * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-12)
                );
            }
            if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
                CHECK_EQ(
                    result[i, j].real(),
                    doctest::Approx(A[i, j].real() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-7)
                );
                CHECK_EQ(
                    result[i, j].imag(),
                    doctest::Approx(A[i, j].imag() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-7)
                );
            }
            if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
                CHECK_EQ(
                    result[i, j].real(),
                    doctest::Approx(A[i, j].real() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-12)
                );
                CHECK_EQ(
                    result[i, j].imag(),
                    doctest::Approx(A[i, j].imag() * (10.5634 + 1.0 / 5.34 - 0.67)).epsilon(2E-12)
                );
            }
        }
    }

    auto [data, out] = zpp::bits::data_out();
    out(A).or_throw();

    T other_A;
    auto in = zpp::bits::in(data);
    in(other_A).or_throw();

    CHECK_EQ(A, other_A);
}

// clang-format off
TEST_CASE_TEMPLATE("operator value categories reuse rvalue operand storage", T,
    Matrix<float, std::extents<std::size_t, 16, 16>, std::layout_left>, Matrix<double, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<float, std::extents<std::size_t, 16, 16>, std::layout_right>, Matrix<double, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<float, std::dextents<std::size_t, 2>, std::layout_left>, Matrix<double, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<float, std::dextents<std::size_t, 2>, std::layout_right>, Matrix<double, std::dextents<std::size_t, 2>, std::layout_right>,
    Matrix<std::complex<float>, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<std::complex<double>, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<std::complex<float>, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<std::complex<double>, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<std::complex<float>, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<std::complex<double>, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<std::complex<float>, std::dextents<std::size_t, 2>, std::layout_right>,
    Matrix<std::complex<double>, std::dextents<std::size_t, 2>, std::layout_right>) {
    // clang-format on

    T A(kNRows, kNCols, typename T::value_type{0.0});
    T B(kNRows, kNCols, typename T::value_type{0.0});
    for (std::size_t i{0}; i < kNumValues; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] = kRealValues[i];
            B[kRowIndices[i], kColIndices[i]] = kImagValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] =
                typename T::value_type(kRealValues[i], kImagValues[i]);
            B[kRowIndices[i], kColIndices[i]] =
                typename T::value_type(kImagValues[i], kRealValues[i]);
        }
    }

    SUBCASE("lvalue + lvalue returns an independent object") {
        const T expected = A + B;
        CHECK(expected.identicalTo(A + B));
        CHECK(&expected != &A);
        CHECK(&expected != &B);
    }

    SUBCASE("lvalue + rvalue reuses the rvalue operand") {
        T C{B};
        const T expected = A + C;
        T&& r = A + std::move(C);
        static_assert(std::is_same_v<decltype((A + std::move(C))), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue + lvalue reuses the rvalue self") {
        T D{A};
        const T expected = D + B;
        T&& r = std::move(D) + B;
        static_assert(std::is_same_v<decltype((std::move(D) + B)), T&&>);
        CHECK(&r == &D);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("lvalue - rvalue reuses the rvalue operand") {
        T C{B};
        const T expected = A - C;
        T&& r = A - std::move(C);
        static_assert(std::is_same_v<decltype((A - std::move(C))), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("unary minus on rvalue reuses the operand") {
        T C{A};
        const T expected = -C;
        T&& r = -std::move(C);
        static_assert(std::is_same_v<decltype((-std::move(C))), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue * scalar reuses the operand") {
        T C{A};
        const T expected = C * 2.0;
        T&& r = std::move(C) * 2.0;
        static_assert(std::is_same_v<decltype((std::declval<T&&>() * 2.0)), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("scalar * rvalue reuses the operand") {
        T C{B};
        const T expected = 3.0 * C;
        T&& r = 3.0 * std::move(C);
        static_assert(std::is_same_v<decltype((3.0 * std::declval<T&&>())), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("rvalue / scalar reuses the operand") {
        T C{A};
        const T expected = C / 2.0;
        T&& r = std::move(C) / 2.0;
        static_assert(std::is_same_v<decltype((std::declval<T&&>() / 2.0)), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }

    SUBCASE("conjugated on rvalue reuses the operand") {
        T C{B};
        const T expected = C.conjugated();
        T&& r = std::move(C).conjugated();
        static_assert(std::is_same_v<decltype((std::declval<T&&>().conjugated())), T&&>);
        CHECK(&r == &C);
        CHECK(r.identicalTo(expected));
    }
}

// clang-format off
TEST_CASE_TEMPLATE("mdspan and submdspan expose storage with order-matching layout", T,
    Matrix<float, std::extents<std::size_t, 16, 16>, std::layout_left>, Matrix<double, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<float, std::extents<std::size_t, 16, 16>, std::layout_right>, Matrix<double, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<float, std::dextents<std::size_t, 2>, std::layout_left>, Matrix<double, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<float, std::dextents<std::size_t, 2>, std::layout_right>, Matrix<double, std::dextents<std::size_t, 2>, std::layout_right>,
    Matrix<std::complex<float>, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<std::complex<double>, std::extents<std::size_t, 16, 16>, std::layout_left>,
    Matrix<std::complex<float>, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<std::complex<double>, std::extents<std::size_t, 16, 16>, std::layout_right>,
    Matrix<std::complex<float>, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<std::complex<double>, std::dextents<std::size_t, 2>, std::layout_left>,
    Matrix<std::complex<float>, std::dextents<std::size_t, 2>, std::layout_right>,
    Matrix<std::complex<double>, std::dextents<std::size_t, 2>, std::layout_right>) {
    // clang-format on

    T A(kNRows, kNCols, typename T::value_type{0.0});
    for (std::size_t i{0}; i < kNumValues; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] = kRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] =
                typename T::value_type(kRealValues[i], kImagValues[i]);
        }
    }

    using mdspan_type = std::mdspan<
        typename T::value_type, typename T::extents_type, typename T::layout_type,
        std::default_accessor<typename T::value_type>>;
    using const_mdspan_type = std::mdspan<
        const typename T::value_type, typename T::extents_type, typename T::layout_type,
        std::default_accessor<const typename T::value_type>>;
    static_assert(std::is_same_v<typename mdspan_type::element_type, typename T::value_type>);
    static_assert(
        std::is_same_v<typename const_mdspan_type::element_type, const typename T::value_type>
    );
    static_assert(mdspan_type::rank() == 2);
    if constexpr (std::is_same_v<typename T::layout_type, std::layout_left>) {
        static_assert(std::is_same_v<typename mdspan_type::layout_type, std::layout_left>);
    } else {
        static_assert(std::is_same_v<typename mdspan_type::layout_type, std::layout_right>);
    }
    if constexpr (mdspan_type::static_extent(0) != std::dynamic_extent) {
        static_assert(mdspan_type::static_extent(0) == kNRows);
        static_assert(mdspan_type::static_extent(1) == kNCols);
    } else {
        static_assert(mdspan_type::static_extent(0) == std::dynamic_extent);
        static_assert(mdspan_type::static_extent(1) == std::dynamic_extent);
    }
    // Whole-storage views go through the conversion operators, exercised via
    // explicit mdspan construction over A's own data.
    const mdspan_type a_view{A.data(), A.nrows(), A.ncols()};
    const const_mdspan_type a_const_view{A.data(), A.nrows(), A.ncols()};
    CHECK_EQ(a_view.data_handle(), A.data());
    CHECK_EQ(a_view.extent(0), kNRows);
    CHECK_EQ(a_view.extent(1), kNCols);
    for (std::size_t i{0}; i < kNRows; ++i) {
        for (std::size_t j{0}; j < kNCols; ++j) {
            if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                CHECK_EQ(A[i, j], a_view[i, j]);
                CHECK_EQ(A[i, j], a_const_view[i, j]);
            } else {
                CHECK_EQ(A[i, j].real(), a_view[i, j].real());
                CHECK_EQ(A[i, j].imag(), a_view[i, j].imag());
            }
        }
    }
    mdspan_type a_mut_view{A.data(), A.nrows(), A.ncols()};
    a_mut_view[0, 0] = A[0, 0];
    CHECK_EQ(a_mut_view[0, 0], A[0, 0]);

    SUBCASE("submdspan returns a strided block view of the storage") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        auto block{A.submdspan(pair_type{6, 12}, pair_type{1, 7})};
        using block_type = decltype(block);
        static_assert(std::is_same_v<typename block_type::element_type, typename T::value_type>);
        static_assert(std::is_same_v<typename block_type::layout_type, std::layout_stride>);
        static_assert(block_type::rank() == 2);
        CHECK_EQ(block.extent(0), 6);
        CHECK_EQ(block.extent(1), 6);
        for (std::size_t i{0}; i < 6; ++i) {
            for (std::size_t j{0}; j < 6; ++j) {
                if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                    CHECK_EQ(A[6 + i, 1 + j], block[i, j]);
                } else {
                    CHECK_EQ(A[6 + i, 1 + j].real(), block[i, j].real());
                    CHECK_EQ(A[6 + i, 1 + j].imag(), block[i, j].imag());
                }
            }
        }
    }

    // Implicit conversion to the stride mdspan layout (identicalTo parameter type).
    using stride_mdspan_type = std::mdspan<
        const typename T::value_type, std::dextents<std::size_t, 2>, std::layout_stride>;
    static_assert(std::is_convertible_v<const T, stride_mdspan_type>);
    static_assert(
        std::is_convertible_v<
            const T,
            std::mdspan<
                typename T::value_type, std::dextents<std::size_t, 2>, std::layout_stride>> == false
    );

    SUBCASE("matrix construction from an mdspan copies elementwise across orders") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        auto block{A.submdspan(pair_type{3, 9}, pair_type{2, 8})};
        Matrix<typename T::value_type, std::dextents<std::size_t, 2>, std::layout_right> copy{
            block
        };
        CHECK_EQ(copy.nrows(), 6);
        CHECK_EQ(copy.ncols(), 6);
        for (std::size_t i{0}; i < 6; ++i) {
            for (std::size_t j{0}; j < 6; ++j) {
                if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                    CHECK_EQ(A[3 + i, 2 + j], copy[i, j]);
                } else {
                    CHECK_EQ(A[3 + i, 2 + j].real(), copy[i, j].real());
                    CHECK_EQ(A[3 + i, 2 + j].imag(), copy[i, j].imag());
                }
            }
        }
    }

    SUBCASE("submdspan vector-style slice applies along the non-unit dimension") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        Matrix<typename T::value_type, std::dextents<std::size_t, 2>, typename T::layout_type>
            row_like(1, kNCols, typename T::value_type{0.0});
        Matrix<typename T::value_type, std::dextents<std::size_t, 2>, typename T::layout_type>
            col_like(kNRows, 1, typename T::value_type{0.0});
        for (std::size_t j{0}; j < kNCols; ++j) {
            if constexpr (std::is_floating_point_v<typename T::value_type>) {
                row_like[0, j] = kRealValues[j % kNumValues];
            }
            if constexpr (isComplexArithmeticV<typename T::value_type>) {
                row_like[0, j] = typename T::value_type(
                    kRealValues[j % kNumValues], kImagValues[j % kNumValues]
                );
            }
        }
        for (std::size_t i{0}; i < kNRows; ++i) {
            if constexpr (std::is_floating_point_v<typename T::value_type>) {
                col_like[i, 0] = kRealValues[i % kNumValues];
            }
            if constexpr (isComplexArithmeticV<typename T::value_type>) {
                col_like[i, 0] = typename T::value_type(
                    kRealValues[i % kNumValues], kImagValues[i % kNumValues]
                );
            }
        }
        auto row_slice{row_like.submdspan(pair_type{2, 5})};
        auto col_slice{col_like.submdspan(pair_type{2, 5})};
        CHECK_EQ(row_slice.extent(0), 1);
        CHECK_EQ(row_slice.extent(1), 3);
        CHECK_EQ(col_slice.extent(0), 3);
        CHECK_EQ(col_slice.extent(1), 1);
        for (std::size_t k{0}; k < 3; ++k) {
            if constexpr (std::is_arithmetic_v<typename T::value_type>) {
                CHECK_EQ(row_like[0, 2 + k], row_slice[0, k]);
                CHECK_EQ(col_like[2 + k, 0], col_slice[k, 0]);
            } else {
                CHECK_EQ(row_like[0, 2 + k].real(), row_slice[0, k].real());
                CHECK_EQ(col_like[2 + k, 0].real(), col_slice[k, 0].real());
            }
        }
    }

#if BASJOO_CHECK_PARAMS == 1
    SUBCASE("submdspan validates ranges and vector-like shape") {
        using pair_type = std::pair<std::size_t, std::size_t>;
        CHECK_THROWS_AS(A.submdspan(pair_type{7, 5}, pair_type{0, 1}), std::out_of_range);
        CHECK_THROWS_AS(A.submdspan(pair_type{0, 17}, pair_type{0, 1}), std::out_of_range);
        CHECK_THROWS_AS(A.submdspan(pair_type{0, 1}, pair_type{0, 17}), std::out_of_range);
        CHECK_THROWS(A.submdspan(pair_type{2, 5}));
    }
#endif
}

// clang-format off
using MS = Matrix<float, 16, 16, ::basjoo::common::AlignedAllocator<float, 32>>;
using MD = Matrix<double, 16, 16, ::basjoo::common::AlignedAllocator<double, 32>>;
using MC = Matrix<std::complex<float>, 16, 16, ::basjoo::common::AlignedAllocator<std::complex<float>, 32>>;
using MF = Matrix<float>;
TEST_CASE_TEMPLATE("mdspan-shaped query surface: data_handle/is_exhaustive/is_always_*", T,
    MS, MD, MC, MF) {
    // clang-format on
    static_assert(T::is_always_unique());
    static_assert(T::is_always_exhaustive());
    static_assert(T::is_always_strided());
    T A(kNRows, kNCols, typename T::value_type{0.0});
    CHECK_EQ(A.data_handle(), A.data());
    CHECK(A.is_exhaustive());
}

} // namespace basjoo::math
