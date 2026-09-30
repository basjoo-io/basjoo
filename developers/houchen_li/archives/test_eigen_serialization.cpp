/**
 * @file test_eigen_serialization.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-12-11
 *
 * @copyright Copyright (c) 2024 basjoo development team.
 *            All rights reserved.
 *
 */

#include "eigen_serialization.hpp"

#include <array>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace {

constexpr std::size_t kNumRows{16};
constexpr std::size_t kNumCols{8};
constexpr std::size_t kNumNonZeros{64};
constexpr std::array<int, kNumNonZeros> kRowIndices{
    14, 5,  5,  15, 13, 6,  12, 15, 12, 5, 2,  4,  6,  0,  8,  12, 10, 9,  1,  8,  13, 7,
    4,  15, 14, 14, 9,  11, 0,  9,  0,  1, 1,  11, 10, 15, 6,  4,  15, 14, 6,  10, 10, 7,
    12, 11, 5,  9,  6,  7,  12, 7,  5,  6, 12, 13, 1,  5,  14, 9,  10, 1,  10, 9
};
constexpr std::array<int, kNumNonZeros> kColIndices{4, 2, 6, 1, 6, 6, 5, 0, 6, 1, 5, 3, 0, 7, 6, 2,
                                                    4, 7, 7, 5, 4, 6, 6, 3, 0, 7, 5, 7, 4, 2, 2, 4,
                                                    3, 5, 5, 4, 1, 0, 2, 2, 4, 7, 2, 4, 1, 3, 7, 6,
                                                    7, 2, 7, 5, 3, 2, 3, 0, 0, 4, 1, 4, 3, 5, 6, 3};
constexpr std::array<double, kNumNonZeros> kValues{
    374.1488,  -568.0397, -513.2097, -793.3472, 141.8823,  643.0921,  917.4021,  -276.7426,
    913.9728,  -817.4795, -988.9644, -593.0302, 706.7459,  -401.7407, -511.3804, 438.2672,
    774.9172,  -289.0229, -914.5727, -667.5602, 71.7045,   138.8144,  -366.6240, 773.5203,
    -182.8523, -838.5585, 329.6116,  220.4898,  -51.4952,  -305.8169, -278.9727, -642.2998,
    -928.8941, -178.6184, -70.0642,  645.3163,  817.8438,  -895.8777, 34.3875,   516.7986,
    -981.8743, 281.2453,  -289.6776, 33.1199,   926.8460,  7.8566,    -355.4008, 567.3317,
    584.6418,  919.5240,  497.6420,  -420.1268, -176.5938, 201.2556,  -596.1191, -851.0748,
    -479.3167, -468.8229, 119.2431,  369.5376,  -267.3212, -760.8866, 81.4256,   793.8413
};

} // namespace

namespace basjoo::math {

TEST_CASE_TEMPLATE(
    "DenseMatrixSerializationTest", T, Eigen::Matrix<double, kNumRows, kNumCols>,
    Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>
) {
    T dense_matrix;
    dense_matrix.resize(kNumRows, kNumCols);
    dense_matrix.setZero();
    for (std::size_t i = 0; i < kNumNonZeros; ++i) {
        dense_matrix.coeffRef(kRowIndices[i], kColIndices[i]) = kValues[i];
    }

    auto [data, out] = zpp::bits::data_out();
    out(dense_matrix).or_throw();

    T other_dense_matrix;
    auto in = zpp::bits::in(data);
    in(other_dense_matrix).or_throw();

    REQUIRE_EQ(dense_matrix.rows(), other_dense_matrix.rows());
    REQUIRE_EQ(dense_matrix.cols(), other_dense_matrix.cols());

    for (std::size_t i = 0; i < kNumRows; ++i) {
        for (std::size_t j = 0; j < kNumCols; ++j) {
            CHECK_EQ(dense_matrix.coeff(i, j), other_dense_matrix.coeff(i, j));
        }
    }
}

TEST_CASE_TEMPLATE(
    "SparseCscMatrixSerializationTest", T, Eigen::SparseMatrix<double, Eigen::ColMajor, int>,
    Eigen::SparseMatrix<double, Eigen::RowMajor, int>
) {
    T sparse_matrix(kNumRows, kNumCols);
    for (std::size_t i = 0; i < kNumNonZeros; ++i) {
        sparse_matrix.coeffRef(kRowIndices[i], kColIndices[i]) = kValues[i];
    }
    sparse_matrix.makeCompressed();

    auto [data, out] = zpp::bits::data_out();
    out(sparse_matrix).or_throw();

    T other_sparse_matrix;
    auto in = zpp::bits::in(data);
    in(other_sparse_matrix).or_throw();

    REQUIRE_EQ(sparse_matrix.rows(), other_sparse_matrix.rows());
    REQUIRE_EQ(sparse_matrix.cols(), other_sparse_matrix.cols());
    REQUIRE_EQ(sparse_matrix.nonZeros(), other_sparse_matrix.nonZeros());

    for (std::size_t i = 0; i < kNumCols + 1; ++i) {
        CHECK_EQ(sparse_matrix.outerIndexPtr()[i], other_sparse_matrix.outerIndexPtr()[i]);
    }
    for (std::size_t i = 0; i < kNumNonZeros; ++i) {
        CHECK_EQ(sparse_matrix.innerIndexPtr()[i], other_sparse_matrix.innerIndexPtr()[i]);
    }
    for (std::size_t i = 0; i < kNumNonZeros; ++i) {
        CHECK_EQ(sparse_matrix.valuePtr()[i], other_sparse_matrix.valuePtr()[i]);
    }
}

} // namespace basjoo::math
