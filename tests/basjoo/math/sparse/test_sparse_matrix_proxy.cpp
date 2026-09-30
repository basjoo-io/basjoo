/**
 * @file test_sparse_matrix_proxy.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-27
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/sparse/sparse_matrix_proxy.hpp"

#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/sparse/coo_matrix.hpp"
#include "basjoo/math/sparse/csc_matrix.hpp"
#include "basjoo/math/sparse/csr_matrix.hpp"
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/lil_matrix.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("Polymorphism") {
    DokMatrix<double, int> dok_matrix(4, 8);
    dok_matrix.updateCoeff(0, 3, 1.5);
    dok_matrix.updateCoeff(0, 1, 345.2);
    dok_matrix.updateCoeff(0, 0, 567.4);
    dok_matrix.updateCoeff(0, 5, 0.0);
    dok_matrix.updateCoeff(3, 2, 1.57);
    dok_matrix.updateCoeff(3, 4, -35.2);
    dok_matrix.updateCoeff(3, 1, 0.0);
    dok_matrix.updateCoeff(3, 3, 3775.0);

    LilMatrix<double, int> lil_matrix(4, 8);
    lil_matrix.updateCoeff(0, 3, 1.5);
    lil_matrix.updateCoeff(0, 1, 345.2);
    lil_matrix.updateCoeff(0, 0, 567.4);
    lil_matrix.updateCoeff(0, 5, 0.0);
    lil_matrix.updateCoeff(3, 2, 1.57);
    lil_matrix.updateCoeff(3, 4, -35.2);
    lil_matrix.updateCoeff(3, 1, 0.0);
    lil_matrix.updateCoeff(3, 3, 3775.0);

    CooMatrix<double, int> coo_matrix(4, 8);
    coo_matrix.updateCoeff(0, 3, 1.5);
    coo_matrix.updateCoeff(0, 1, 345.2);
    coo_matrix.updateCoeff(0, 0, 567.4);
    coo_matrix.updateCoeff(0, 5, 0.0);
    coo_matrix.updateCoeff(3, 2, 1.57);
    coo_matrix.updateCoeff(3, 4, -35.2);
    coo_matrix.updateCoeff(3, 1, 0.0);
    coo_matrix.updateCoeff(3, 3, 3775.0);

    CscMatrix<double, int> csc_matrix(4, 8);
    csc_matrix.updateCoeff(0, 3, 1.5);
    csc_matrix.updateCoeff(0, 1, 345.2);
    csc_matrix.updateCoeff(0, 0, 567.4);
    csc_matrix.updateCoeff(0, 5, 0.0);
    csc_matrix.updateCoeff(3, 2, 1.57);
    csc_matrix.updateCoeff(3, 4, -35.2);
    csc_matrix.updateCoeff(3, 1, 0.0);
    csc_matrix.updateCoeff(3, 3, 3775.0);

    CsrMatrix<double, int> csr_matrix(4, 8);
    csr_matrix.updateCoeff(0, 3, 1.5);
    csr_matrix.updateCoeff(0, 1, 345.2);
    csr_matrix.updateCoeff(0, 0, 567.4);
    csr_matrix.updateCoeff(0, 5, 0.0);
    csr_matrix.updateCoeff(3, 2, 1.57);
    csr_matrix.updateCoeff(3, 4, -35.2);
    csr_matrix.updateCoeff(3, 1, 0.0);
    csr_matrix.updateCoeff(3, 3, 3775.0);

    std::vector<SparseMatrixProxy<double, int>> matrices{};
    matrices.emplace_back(makeSparseMatrixProxy(dok_matrix));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(lil_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(coo_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(csc_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(csr_matrix)));

    for (const SparseMatrixProxy<double, int>& sparse_matrix : matrices) {
        CHECK_EQ(sparse_matrix->nrows(), 4);
        CHECK_EQ(sparse_matrix->ncols(), 8);
        CHECK_EQ(sparse_matrix->nnzs(), 8);

        CHECK_EQ(sparse_matrix->coeff(0, 3), 1.5);
        CHECK_EQ(sparse_matrix->coeff(0, 1), 345.2);
        CHECK_EQ(sparse_matrix->coeff(0, 0), 567.4);
        CHECK_EQ(sparse_matrix->coeff(0, 5), 0.0);
        CHECK_EQ(sparse_matrix->coeff(0, 2), 0.0);
        CHECK_EQ(sparse_matrix->coeff(3, 2), 1.57);
        CHECK_EQ(sparse_matrix->coeff(3, 4), -35.2);
        CHECK_EQ(sparse_matrix->coeff(3, 1), 0.0);
        CHECK_EQ(sparse_matrix->coeff(3, 3), 3775.0);

        CHECK_EQ(sparse_matrix->operator[](0, 3), 1.5);
        CHECK_EQ(sparse_matrix->operator[](0, 1), 345.2);
        CHECK_EQ(sparse_matrix->operator[](0, 0), 567.4);
        CHECK_EQ(sparse_matrix->operator[](0, 5), 0.0);
        CHECK_EQ(sparse_matrix->operator[](0, 2), 0.0);
        CHECK_EQ(sparse_matrix->operator[](3, 2), 1.57);
        CHECK_EQ(sparse_matrix->operator[](3, 4), -35.2);
        CHECK_EQ(sparse_matrix->operator[](3, 1), 0.0);
        CHECK_EQ(sparse_matrix->operator[](3, 3), 3775.0);
    }
}

TEST_CASE("SerializationTest") {
    DokMatrix<double, int> dok_matrix(4, 8);
    dok_matrix.updateCoeff(0, 3, 1.5);
    dok_matrix.updateCoeff(3, 3, 3775.0);

    LilMatrix<double, int> lil_matrix(4, 8);
    lil_matrix.updateCoeff(0, 3, 1.5);
    lil_matrix.updateCoeff(3, 3, 3775.0);

    CooMatrix<double, int> coo_matrix(4, 8);
    coo_matrix.updateCoeff(0, 3, 1.5);
    coo_matrix.updateCoeff(3, 3, 3775.0);

    CscMatrix<double, int> csc_matrix(4, 8);
    csc_matrix.updateCoeff(0, 3, 1.5);
    csc_matrix.updateCoeff(3, 3, 3775.0);

    CsrMatrix<double, int> csr_matrix(4, 8);
    csr_matrix.updateCoeff(0, 3, 1.5);
    csr_matrix.updateCoeff(3, 3, 3775.0);

    std::vector<SparseMatrixProxy<double, int>> matrices{};
    matrices.emplace_back(makeSparseMatrixProxy(std::move(dok_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(lil_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(coo_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(csc_matrix)));
    matrices.emplace_back(makeSparseMatrixProxy(std::move(csr_matrix)));

    for (const auto& sparse_matrix : matrices) {
        auto [data, out] = zpp::bits::data_out();
        out(sparse_matrix).or_throw();

        SparseMatrixProxy<double, int> other_sparse_matrix{sparse_matrix};
        auto in = zpp::bits::in(data);
        in(other_sparse_matrix).or_throw();

        CHECK(proxy_typeid(*other_sparse_matrix) == proxy_typeid(*sparse_matrix));

        CHECK_EQ(other_sparse_matrix->nrows(), sparse_matrix->nrows());
        CHECK_EQ(other_sparse_matrix->ncols(), sparse_matrix->ncols());
        CHECK_EQ(other_sparse_matrix->nnzs(), sparse_matrix->nnzs());
        CHECK_EQ(other_sparse_matrix->coeff(0, 3), sparse_matrix->coeff(0, 3));
        CHECK_EQ(other_sparse_matrix->coeff(3, 3), sparse_matrix->coeff(3, 3));
        CHECK_EQ(other_sparse_matrix->coeff(0, 0), sparse_matrix->coeff(0, 0));
    }
}

} // namespace basjoo::math
