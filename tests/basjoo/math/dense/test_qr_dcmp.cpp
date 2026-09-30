/**
 * @file test_qr_dcmp.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-01-14
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/dense/qr_dcmp.hpp"

#include <array>
#include <complex>
#include <type_traits>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"
#include "basjoo/math/dense/matrix.hpp"
#include "basjoo/math/dense/matrixx.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vectorx.hpp"
#include "basjoo/math/type_traits.hpp"

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

constexpr std::array<double, kNRows> kBRealValues{-281.2042, 201.0305,  860.8241,  574.7355,
                                                  -478.0013, 922.1276,  297.3931,  82.7102,
                                                  20.8434,   178.7692,  -637.5437, -287.4113,
                                                  417.9586,  -963.2638, 852.9532,  -409.2006};

constexpr std::array<double, kNRows> kBImagValues{941.3074,  219.3982,  -898.9647, 512.4636,
                                                  -862.3371, -639.5703, 820.1686,  524.2200,
                                                  575.2414,  219.9055,  -736.2512, -789.8754,
                                                  -511.0101, -449.6304, 651.3532,  -448.5029};

} // namespace

namespace basjoo::math {

using namespace std::literals::complex_literals;

// clang-format off
TEST_CASE_TEMPLATE("MatrixTest", T,
    Matrix<float, 16, 16, MatrixOrder::COL_MAJOR>, Matrix<double, 16, 16, MatrixOrder::COL_MAJOR>,
    Matrix<float, 16, 16, MatrixOrder::ROW_MAJOR>, Matrix<double, 16, 16, MatrixOrder::ROW_MAJOR>,
    MatrixX<float, MatrixOrder::COL_MAJOR>, MatrixX<double, MatrixOrder::COL_MAJOR>,
    MatrixX<float, MatrixOrder::ROW_MAJOR>, MatrixX<double, MatrixOrder::ROW_MAJOR>,
    Matrix<std::complex<float>, 16, 16, MatrixOrder::COL_MAJOR>,
    Matrix<std::complex<double>, 16, 16, MatrixOrder::COL_MAJOR>,
    Matrix<std::complex<float>, 16, 16, MatrixOrder::ROW_MAJOR>,
    Matrix<std::complex<double>, 16, 16, MatrixOrder::ROW_MAJOR>,
    MatrixX<std::complex<float>, MatrixOrder::COL_MAJOR>,
    MatrixX<std::complex<double>, MatrixOrder::COL_MAJOR>,
    MatrixX<std::complex<float>, MatrixOrder::ROW_MAJOR>,
    MatrixX<std::complex<double>, MatrixOrder::ROW_MAJOR>) {
    // clang-format on

    T A(kNRows, kNCols, 0.0);
    for (std::size_t i{0}; i < kNumValues; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] = kRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            A[kRowIndices[i], kColIndices[i]] =
                typename T::value_type(kRealValues[i], kImagValues[i]);
        }
    }

    CHECK_EQ(A.nrows(), kNRows);
    CHECK_EQ(A.ncols(), kNCols);

    for (std::size_t i{0}; i < kNumValues; ++i) {
        if constexpr (std::is_same_v<typename T::value_type, float>) {
            CHECK_EQ(
                A[kRowIndices[i], kColIndices[i]], doctest::Approx(kRealValues[i]).epsilon(5E-8)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, double>) {
            CHECK_EQ(
                A[kRowIndices[i], kColIndices[i]], doctest::Approx(kRealValues[i]).epsilon(1E-16)
            );
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
            const typename T::value_type& val{A[kRowIndices[i], kColIndices[i]]};
            CHECK_EQ(val.real(), doctest::Approx(kRealValues[i]).epsilon(5E-8));
            CHECK_EQ(val.imag(), doctest::Approx(kImagValues[i]).epsilon(5E-8));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
            const typename T::value_type& val{A[kRowIndices[i], kColIndices[i]]};
            CHECK_EQ(val.real(), doctest::Approx(kRealValues[i]).epsilon(1E-16));
            CHECK_EQ(val.imag(), doctest::Approx(kImagValues[i]).epsilon(1E-16));
        }
    }

    detail::DenseDegenerateTraitT<T> b(kNRows);
    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_floating_point_v<typename T::value_type>) {
            b[i] = kBRealValues[i];
        }
        if constexpr (isComplexArithmeticV<typename T::value_type>) {
            b[i] = typename T::value_type(kBRealValues[i], kBImagValues[i]);
        }
    }

    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_same_v<typename T::value_type, float>) {
            CHECK_EQ(b[i], doctest::Approx(kBRealValues[i]).epsilon(6E-8));
        }
        if constexpr (std::is_same_v<typename T::value_type, double>) {
            CHECK_EQ(b[i], doctest::Approx(kBRealValues[i]).epsilon(1E-16));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
            const typename T::value_type& val{b[i]};
            CHECK_EQ(val.real(), doctest::Approx(kBRealValues[i]).epsilon(6E-8));
            CHECK_EQ(val.imag(), doctest::Approx(kBImagValues[i]).epsilon(6E-8));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
            const typename T::value_type& val{b[i]};
            CHECK_EQ(val.real(), doctest::Approx(kBRealValues[i]).epsilon(1E-16));
            CHECK_EQ(val.imag(), doctest::Approx(kBImagValues[i]).epsilon(1E-16));
        }
    }

    const QrDcmp<T> qr_dcmp(A);

    const T A_inv = qr_dcmp.solve();
    const T I = A.dot(A_inv);

    for (std::size_t i{0}; i < kNRows; ++i) {
        for (std::size_t j{0}; j < kNCols; ++j) {
            if constexpr (std::is_same_v<typename T::value_type, float>) {
                CHECK_EQ(
                    I[i, j],
                    doctest::Approx(static_cast<typename T::value_type>(i == j)).epsilon(2E-4)
                );
            }
            if constexpr (std::is_same_v<typename T::value_type, double>) {
                CHECK_EQ(
                    I[i, j],
                    doctest::Approx(static_cast<typename T::value_type>(i == j)).epsilon(1E-12)
                );
            }
            if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
                const typename T::value_type& val{I[i, j]};
                CHECK_EQ(
                    val.real(),
                    doctest::Approx(static_cast<typename T::value_type::value_type>(i == j))
                        .epsilon(2E-4)
                );
                CHECK_EQ(val.imag(), doctest::Approx(0.0).epsilon(2E-4));
            }
            if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
                const typename T::value_type& val{I[i, j]};
                CHECK_EQ(
                    val.real(),
                    doctest::Approx(static_cast<typename T::value_type::value_type>(i == j))
                        .epsilon(1E-12)
                );
                CHECK_EQ(val.imag(), doctest::Approx(0.0).epsilon(1E-12));
            }
        }
    }

    const auto x = qr_dcmp.solve(b);
    const auto Ax = A.dot(x);

    for (std::size_t i{0}; i < kNRows; ++i) {
        if constexpr (std::is_same_v<typename T::value_type, float>) {
            CHECK_EQ(Ax[i], doctest::Approx(b[i]).epsilon(2E-4));
        } else if constexpr (std::is_same_v<typename T::value_type, double>) {
            CHECK_EQ(Ax[i], doctest::Approx(b[i]).epsilon(1E-12));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<float>>) {
            CHECK_EQ(Ax[i].real(), doctest::Approx(b[i].real()).epsilon(2E-4));
            CHECK_EQ(Ax[i].imag(), doctest::Approx(b[i].imag()).epsilon(8E-5));
        }
        if constexpr (std::is_same_v<typename T::value_type, std::complex<double>>) {
            CHECK_EQ(Ax[i].real(), doctest::Approx(b[i].real()).epsilon(1E-12));
            CHECK_EQ(Ax[i].imag(), doctest::Approx(b[i].imag()).epsilon(2E-13));
        }
    }
}

} // namespace basjoo::math
