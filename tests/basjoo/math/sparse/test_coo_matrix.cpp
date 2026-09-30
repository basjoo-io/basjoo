/**
 * @file test_coo_matrix.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-27
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/sparse/coo_matrix.hpp"

#include "zpp_bits.h"

#include "basjoo//math/sparse/csr_matrix.hpp"
#include "basjoo/math/sparse/csc_matrix.hpp"
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/lil_matrix.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("Basic") {
    CooMatrix<double, int> coo_matrix(8, 8);

    CHECK_EQ(coo_matrix.nrows(), 8);
    CHECK_EQ(coo_matrix.ncols(), 8);

    coo_matrix.updateCoeff(2, 5, 5.0);
    coo_matrix.updateCoeff(7, 3, 2874.0843);
    coo_matrix.updateCoeff(0, 6, -408.876);
    coo_matrix.updateCoeff(4, 6, 0.0);
    coo_matrix.updateCoeff(8, 0, 1.5);

    CHECK_EQ(coo_matrix.coeff(2, 5), 5.0);
    CHECK_EQ(coo_matrix.coeff(7, 3), 2874.0843);
    CHECK_EQ(coo_matrix.coeff(0, 6), -408.876);
    CHECK_EQ(coo_matrix.coeff(4, 6), 0.0);
    CHECK_EQ(coo_matrix.coeff(8, 0), 0.0);

    CHECK_EQ(coo_matrix.coeff(1, 3), 0.0);
    CHECK_EQ(coo_matrix.coeff(2, 4), 0.0);

    CHECK_EQ(coo_matrix[2, 5], 5.0);
    CHECK_EQ(coo_matrix[7, 3], 2874.0843);
    CHECK_EQ(coo_matrix[0, 6], -408.876);
    CHECK_EQ(coo_matrix[4, 6], 0.0);
    CHECK_EQ(coo_matrix[8, 0], 0.0);

    CHECK_EQ(coo_matrix[1, 3], 0.0);
    CHECK_EQ(coo_matrix[2, 4], 0.0);

    const auto& values{coo_matrix.values()};
    const auto& row_indices{coo_matrix.rowIndices()};
    const auto& col_indices{coo_matrix.colIndices()};

    CHECK_EQ(coo_matrix.nnzs(), 4);

    for (std::size_t i{0}; i < coo_matrix.nnzs(); ++i) {
        CHECK_EQ(values[i], coo_matrix.coeff(row_indices[i], col_indices[i]));
    }

    coo_matrix.updateCoeff(7, 3, 28234.0843);
    coo_matrix.updateCoeff(2, 5, 7.0);
    coo_matrix.updateCoeff(0, 6, 408.876);
    coo_matrix.updateCoeff(4, 6, 1.0);

    CHECK_EQ(coo_matrix.nnzs(), 4);

    CHECK_EQ(coo_matrix.coeff(2, 5), 7.0);
    CHECK_EQ(coo_matrix.coeff(7, 3), 28234.0843);
    CHECK_EQ(coo_matrix.coeff(0, 6), 408.876);
    CHECK_EQ(coo_matrix.coeff(4, 6), 1.0);

    CHECK_EQ(coo_matrix[2, 5], 7.0);
    CHECK_EQ(coo_matrix[7, 3], 28234.0843);
    CHECK_EQ(coo_matrix[0, 6], 408.876);
    CHECK_EQ(coo_matrix[4, 6], 1.0);

    coo_matrix.clear();

    CHECK_EQ(coo_matrix.nnzs(), 0);
}

TEST_CASE("Resize") {
    CooMatrix<double, int> coo_matrix(8, 8);

    CHECK_EQ(coo_matrix.nrows(), 8);
    CHECK_EQ(coo_matrix.ncols(), 8);

    coo_matrix.updateCoeff(2, 5, 5.0);
    coo_matrix.updateCoeff(7, 3, 2874.0843);
    coo_matrix.updateCoeff(0, 6, -408.876);
    coo_matrix.updateCoeff(4, 6, 0.0);
    coo_matrix.updateCoeff(8, 0, 1.5);

    CHECK_EQ(coo_matrix.nnzs(), 4);

    coo_matrix.resize(6, 6);

    CHECK_EQ(coo_matrix.nrows(), 6);
    CHECK_EQ(coo_matrix.ncols(), 6);

    coo_matrix.updateCoeff(2, 5, 5.0);
    coo_matrix.updateCoeff(7, 3, 0.0);
    coo_matrix.updateCoeff(0, 6, 0.0);
    coo_matrix.updateCoeff(4, 6, 0.0);
    coo_matrix.updateCoeff(8, 0, 0.0);

    CHECK_EQ(coo_matrix.nnzs(), 1);
}

TEST_SUITE("Conversion") {
    TEST_CASE_TEMPLATE(
        "FromCooMatrix", T, DokMatrix<double, int>, CscMatrix<double, int>, CsrMatrix<double, int>,
        LilMatrix<double, int>
    ) {
        CooMatrix<double, int> coo_matrix(4, 8);

        coo_matrix.updateCoeff(0, 3, 1.5);
        coo_matrix.updateCoeff(0, 1, 345.2);
        coo_matrix.updateCoeff(0, 0, 567.4);
        coo_matrix.updateCoeff(0, 5, 0.0);
        coo_matrix.updateCoeff(3, 2, 1.57);
        coo_matrix.updateCoeff(3, 4, -35.2);
        coo_matrix.updateCoeff(3, 1, 0.0);
        coo_matrix.updateCoeff(3, 3, 3775.0);

        T sparse_matrix{coo_matrix};

        CHECK_EQ(sparse_matrix.nrows(), 4);
        CHECK_EQ(sparse_matrix.ncols(), 8);
        CHECK_EQ(sparse_matrix.nnzs(), 8);

        CHECK_EQ(sparse_matrix.coeff(0, 3), 1.5);
        CHECK_EQ(sparse_matrix.coeff(0, 1), 345.2);
        CHECK_EQ(sparse_matrix.coeff(0, 0), 567.4);
        CHECK_EQ(sparse_matrix.coeff(0, 5), 0.0);
        CHECK_EQ(sparse_matrix.coeff(0, 2), 0.0);
        CHECK_EQ(sparse_matrix.coeff(3, 2), 1.57);
        CHECK_EQ(sparse_matrix.coeff(3, 4), -35.2);
        CHECK_EQ(sparse_matrix.coeff(3, 1), 0.0);
        CHECK_EQ(sparse_matrix.coeff(3, 3), 3775.0);

        CHECK_EQ(sparse_matrix[0, 3], 1.5);
        CHECK_EQ(sparse_matrix[0, 1], 345.2);
        CHECK_EQ(sparse_matrix[0, 0], 567.4);
        CHECK_EQ(sparse_matrix[0, 5], 0.0);
        CHECK_EQ(sparse_matrix[0, 2], 0.0);
        CHECK_EQ(sparse_matrix[3, 2], 1.57);
        CHECK_EQ(sparse_matrix[3, 4], -35.2);
        CHECK_EQ(sparse_matrix[3, 1], 0.0);
        CHECK_EQ(sparse_matrix[3, 3], 3775.0);
    }

    TEST_CASE_TEMPLATE(
        "ToCooMatrix", T, DokMatrix<double, int>, CscMatrix<double, int>, CsrMatrix<double, int>,
        LilMatrix<double, int>
    ) {
        T sparse_matrix(4, 8);

        sparse_matrix.updateCoeff(0, 3, 1.5);
        sparse_matrix.updateCoeff(0, 1, 345.2);
        sparse_matrix.updateCoeff(0, 0, 567.4);
        sparse_matrix.updateCoeff(0, 5, 0.0);
        sparse_matrix.updateCoeff(3, 2, 1.57);
        sparse_matrix.updateCoeff(3, 4, -35.2);
        sparse_matrix.updateCoeff(3, 1, 0.0);
        sparse_matrix.updateCoeff(3, 3, 3775.0);

        CooMatrix<double, int> coo_matrix{sparse_matrix};

        CHECK_EQ(coo_matrix.nrows(), 4);
        CHECK_EQ(coo_matrix.ncols(), 8);
        CHECK_EQ(coo_matrix.nnzs(), 8);

        CHECK_EQ(coo_matrix.coeff(0, 3), 1.5);
        CHECK_EQ(coo_matrix.coeff(0, 1), 345.2);
        CHECK_EQ(coo_matrix.coeff(0, 0), 567.4);
        CHECK_EQ(coo_matrix.coeff(0, 5), 0.0);
        CHECK_EQ(coo_matrix.coeff(0, 2), 0.0);
        CHECK_EQ(coo_matrix.coeff(3, 2), 1.57);
        CHECK_EQ(coo_matrix.coeff(3, 4), -35.2);
        CHECK_EQ(coo_matrix.coeff(3, 1), 0.0);
        CHECK_EQ(coo_matrix.coeff(3, 3), 3775.0);

        CHECK_EQ(coo_matrix[0, 3], 1.5);
        CHECK_EQ(coo_matrix[0, 1], 345.2);
        CHECK_EQ(coo_matrix[0, 0], 567.4);
        CHECK_EQ(coo_matrix[0, 5], 0.0);
        CHECK_EQ(coo_matrix[0, 2], 0.0);
        CHECK_EQ(coo_matrix[3, 2], 1.57);
        CHECK_EQ(coo_matrix[3, 4], -35.2);
        CHECK_EQ(coo_matrix[3, 1], 0.0);
        CHECK_EQ(coo_matrix[3, 3], 3775.0);
    }
}

TEST_CASE("Serialization") {
    CooMatrix<double, int> coo_matrix(8, 8);

    coo_matrix.updateCoeff(2, 5, 5.0);
    coo_matrix.updateCoeff(7, 3, 2874.0843);
    coo_matrix.updateCoeff(0, 6, -408.876);
    coo_matrix.updateCoeff(4, 6, 0.0);
    coo_matrix.updateCoeff(8, 0, 1.5);

    auto [data, out] = zpp::bits::data_out();
    out(coo_matrix).or_throw();

    CooMatrix<double, int> other_coo_matrix;
    auto in = zpp::bits::in(data);
    in(other_coo_matrix).or_throw();

    CHECK_EQ(other_coo_matrix.nrows(), 8);
    CHECK_EQ(other_coo_matrix.ncols(), 8);
    CHECK_EQ(other_coo_matrix.nnzs(), 4);

    CHECK_EQ(other_coo_matrix.coeff(2, 5), 5.0);
    CHECK_EQ(other_coo_matrix.coeff(7, 3), 2874.0843);
    CHECK_EQ(other_coo_matrix.coeff(0, 6), -408.876);
    CHECK_EQ(other_coo_matrix.coeff(4, 6), 0.0);
    CHECK_EQ(other_coo_matrix.coeff(8, 0), 0.0);

    CHECK_EQ(other_coo_matrix[2, 5], 5.0);
    CHECK_EQ(other_coo_matrix[7, 3], 2874.0843);
    CHECK_EQ(other_coo_matrix[0, 6], -408.876);
    CHECK_EQ(other_coo_matrix[4, 6], 0.0);
    CHECK_EQ(other_coo_matrix[8, 0], 0.0);
}

} // namespace basjoo::math
