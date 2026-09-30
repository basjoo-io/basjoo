/**
 * @file test_csr_matrix.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-04
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/sparse/csr_matrix.hpp"

#include "zpp_bits.h"
#include <array>

#include "basjoo/math/sparse/coo_matrix.hpp"
#include "basjoo/math/sparse/csc_matrix.hpp"
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/lil_matrix.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("Basic") {
    CsrMatrix<double, int> csr_matrix(5, 5);

    CHECK_EQ(csr_matrix.nrows(), 5);
    CHECK_EQ(csr_matrix.ncols(), 5);
    CHECK_EQ(csr_matrix.nnzs(), 0);
    CHECK_EQ(csr_matrix.outerIndices().size(), 6);

    csr_matrix.updateCoeff(4, 4, 8.0);
    csr_matrix.updateCoeff(1, 4, 17.0);
    csr_matrix.updateCoeff(2, 3, 1.0);
    csr_matrix.updateCoeff(4, 2, 14.0);
    csr_matrix.updateCoeff(2, 1, 5.0);
    csr_matrix.updateCoeff(0, 1, 3.0);
    csr_matrix.updateCoeff(2, 0, 7.0);
    csr_matrix.updateCoeff(1, 0, 22.0);
    csr_matrix.updateCoeff(5, 5, 1.0);

    CHECK_EQ(csr_matrix.coeff(1, 0), 22.0);
    CHECK_EQ(csr_matrix.coeff(2, 0), 7.0);
    CHECK_EQ(csr_matrix.coeff(0, 1), 3.0);
    CHECK_EQ(csr_matrix.coeff(2, 1), 5.0);
    CHECK_EQ(csr_matrix.coeff(4, 2), 14.0);
    CHECK_EQ(csr_matrix.coeff(2, 3), 1.0);
    CHECK_EQ(csr_matrix.coeff(1, 4), 17.0);
    CHECK_EQ(csr_matrix.coeff(4, 4), 8.0);
    CHECK_EQ(csr_matrix.coeff(5, 5), 0.0);
    CHECK_EQ(csr_matrix.coeff(3, 3), 0.0);

    CHECK_EQ(csr_matrix[1, 0], 22.0);
    CHECK_EQ(csr_matrix[2, 0], 7.0);
    CHECK_EQ(csr_matrix[0, 1], 3.0);
    CHECK_EQ(csr_matrix[2, 1], 5.0);
    CHECK_EQ(csr_matrix[4, 2], 14.0);
    CHECK_EQ(csr_matrix[2, 3], 1.0);
    CHECK_EQ(csr_matrix[1, 4], 17.0);
    CHECK_EQ(csr_matrix[4, 4], 8.0);
    CHECK_EQ(csr_matrix[5, 5], 0.0);
    CHECK_EQ(csr_matrix[3, 3], 0.0);

    const std::size_t values_size{csr_matrix.values().size()};
    const std::size_t inner_indices_size{csr_matrix.innerIndices().size()};
    const std::size_t outer_indices_size{csr_matrix.outerIndices().size()};
    const auto& values{csr_matrix.values()};
    const auto& inner_indices{csr_matrix.innerIndices()};
    const auto& outer_indices{csr_matrix.outerIndices()};

    constexpr std::array<double, 8> exact_values{3.0, 22.0, 17.0, 7.0, 5.0, 1.0, 14.0, 8.0};
    constexpr std::array<int, 8> exact_inner_indices{1, 0, 4, 0, 1, 3, 2, 4};
    constexpr std::array<int, 6> exact_outer_indices{0, 1, 3, 6, 6, 8};

    CHECK_EQ(values_size, exact_values.size());
    CHECK_EQ(inner_indices_size, exact_inner_indices.size());
    CHECK_EQ(outer_indices_size, exact_outer_indices.size());

    for (std::size_t i{0}; i < values_size; ++i) {
        CHECK_EQ(values[i], exact_values[i]);
    }
    for (std::size_t i{0}; i < inner_indices_size; ++i) {
        CHECK_EQ(inner_indices[i], exact_inner_indices[i]);
    }
    for (std::size_t i{0}; i < outer_indices_size; ++i) {
        CHECK_EQ(outer_indices[i], exact_outer_indices[i]);
    }
}

TEST_CASE("Resize") {
    CsrMatrix<double, int> csr_matrix(5, 5);

    csr_matrix.updateCoeff(4, 4, 8.0);
    csr_matrix.updateCoeff(1, 4, 17.0);
    csr_matrix.updateCoeff(2, 3, 1.0);
    csr_matrix.updateCoeff(4, 2, 14.0);
    csr_matrix.updateCoeff(2, 1, 5.0);
    csr_matrix.updateCoeff(0, 1, 3.0);
    csr_matrix.updateCoeff(2, 0, 7.0);
    csr_matrix.updateCoeff(1, 0, 22.0);
    csr_matrix.updateCoeff(5, 5, 1.0);

    SUBCASE("EnlargeSize") {
        csr_matrix.resize(10, 10);

        const std::size_t values_size{csr_matrix.values().size()};
        const std::size_t inner_indices_size{csr_matrix.innerIndices().size()};
        const std::size_t outer_indices_size{csr_matrix.outerIndices().size()};
        const auto& values{csr_matrix.values()};
        const auto& inner_indices{csr_matrix.innerIndices()};
        const auto& outer_indices{csr_matrix.outerIndices()};

        constexpr std::array<double, 8> exact_values{3.0, 22.0, 17.0, 7.0, 5.0, 1.0, 14.0, 8.0};
        constexpr std::array<int, 8> exact_inner_indices{1, 0, 4, 0, 1, 3, 2, 4};
        constexpr std::array<int, 11> exact_outer_indices{0, 1, 3, 6, 6, 8, 8, 8, 8, 8, 8};

        CHECK_EQ(values_size, exact_values.size());
        CHECK_EQ(inner_indices_size, exact_inner_indices.size());
        CHECK_EQ(outer_indices_size, exact_outer_indices.size());

        for (std::size_t i{0}; i < values_size; ++i) {
            CHECK_EQ(values[i], exact_values[i]);
        }
        for (std::size_t i{0}; i < inner_indices_size; ++i) {
            CHECK_EQ(inner_indices[i], exact_inner_indices[i]);
        }
        for (std::size_t i{0}; i < outer_indices_size; ++i) {
            CHECK_EQ(outer_indices[i], exact_outer_indices[i]);
        }
    }

    SUBCASE("DescendSize") {
        csr_matrix.resize(3, 3);

        const std::size_t values_size{csr_matrix.values().size()};
        const std::size_t inner_indices_size{csr_matrix.innerIndices().size()};
        const std::size_t outer_indices_size{csr_matrix.outerIndices().size()};
        const auto& values{csr_matrix.values()};
        const auto& inner_indices{csr_matrix.innerIndices()};
        const auto& outer_indices{csr_matrix.outerIndices()};

        constexpr std::array<double, 4> exact_values{3.0, 22.0, 7.0, 5.0};
        constexpr std::array<int, 4> exact_inner_indices{1, 0, 0, 1};
        constexpr std::array<int, 4> exact_outer_indices{0, 1, 2, 4};

        CHECK_EQ(values_size, exact_values.size());
        CHECK_EQ(inner_indices_size, exact_inner_indices.size());
        CHECK_EQ(outer_indices_size, exact_outer_indices.size());

        for (std::size_t i{0}; i < values_size; ++i) {
            CHECK_EQ(values[i], exact_values[i]);
        }
        for (std::size_t i{0}; i < inner_indices_size; ++i) {
            CHECK_EQ(inner_indices[i], exact_inner_indices[i]);
        }
        for (std::size_t i{0}; i < outer_indices_size; ++i) {
            CHECK_EQ(outer_indices[i], exact_outer_indices[i]);
        }
    }
}

TEST_SUITE("Conversion") {
    TEST_CASE_TEMPLATE(
        "FromCsrMatrix", T, DokMatrix<double, int>, CooMatrix<double, int>, CscMatrix<double, int>,
        LilMatrix<double, int>
    ) {
        CsrMatrix<double, int> csr_matrix(4, 8);

        csr_matrix.updateCoeff(0, 3, 1.5);
        csr_matrix.updateCoeff(0, 1, 345.2);
        csr_matrix.updateCoeff(0, 0, 567.4);
        csr_matrix.updateCoeff(0, 5, 0.0);
        csr_matrix.updateCoeff(3, 2, 1.57);
        csr_matrix.updateCoeff(3, 4, -35.2);
        csr_matrix.updateCoeff(3, 1, 0.0);
        csr_matrix.updateCoeff(3, 3, 3775.0);

        T sparse_matrix{csr_matrix};

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
        "ToCsrMatrix", T, DokMatrix<double, int>, CooMatrix<double, int>, CscMatrix<double, int>,
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

        CsrMatrix<double, int> csr_matrix{sparse_matrix};

        CHECK_EQ(csr_matrix.nrows(), 4);
        CHECK_EQ(csr_matrix.ncols(), 8);
        CHECK_EQ(csr_matrix.nnzs(), 8);

        CHECK_EQ(csr_matrix.coeff(0, 3), 1.5);
        CHECK_EQ(csr_matrix.coeff(0, 1), 345.2);
        CHECK_EQ(csr_matrix.coeff(0, 0), 567.4);
        CHECK_EQ(csr_matrix.coeff(0, 5), 0.0);
        CHECK_EQ(csr_matrix.coeff(0, 2), 0.0);
        CHECK_EQ(csr_matrix.coeff(3, 2), 1.57);
        CHECK_EQ(csr_matrix.coeff(3, 4), -35.2);
        CHECK_EQ(csr_matrix.coeff(3, 1), 0.0);
        CHECK_EQ(csr_matrix.coeff(3, 3), 3775.0);

        CHECK_EQ(csr_matrix[0, 3], 1.5);
        CHECK_EQ(csr_matrix[0, 1], 345.2);
        CHECK_EQ(csr_matrix[0, 0], 567.4);
        CHECK_EQ(csr_matrix[0, 5], 0.0);
        CHECK_EQ(csr_matrix[0, 2], 0.0);
        CHECK_EQ(csr_matrix[3, 2], 1.57);
        CHECK_EQ(csr_matrix[3, 4], -35.2);
        CHECK_EQ(csr_matrix[3, 1], 0.0);
        CHECK_EQ(csr_matrix[3, 3], 3775.0);
    }
}

TEST_CASE("Serialization") {
    CsrMatrix<double, int> csr_matrix(5, 5);

    csr_matrix.updateCoeff(4, 4, 8.0);
    csr_matrix.updateCoeff(1, 4, 17.0);
    csr_matrix.updateCoeff(2, 3, 1.0);
    csr_matrix.updateCoeff(4, 2, 14.0);
    csr_matrix.updateCoeff(2, 1, 5.0);
    csr_matrix.updateCoeff(0, 1, 3.0);
    csr_matrix.updateCoeff(2, 0, 7.0);
    csr_matrix.updateCoeff(1, 0, 22.0);
    csr_matrix.updateCoeff(5, 5, 1.0);

    auto [data, out] = zpp::bits::data_out();
    out(csr_matrix).or_throw();

    CsrMatrix<double, int> other_csr_matrix;
    auto in = zpp::bits::in(data);
    in(other_csr_matrix).or_throw();

    const std::size_t values_size{other_csr_matrix.values().size()};
    const std::size_t inner_indices_size{other_csr_matrix.innerIndices().size()};
    const std::size_t outer_indices_size{other_csr_matrix.outerIndices().size()};
    const auto& values{other_csr_matrix.values()};
    const auto& inner_indices{other_csr_matrix.innerIndices()};
    const auto& outer_indices{other_csr_matrix.outerIndices()};

    constexpr std::array<double, 8> exact_values{3.0, 22.0, 17.0, 7.0, 5.0, 1.0, 14.0, 8.0};
    constexpr std::array<int, 8> exact_inner_indices{1, 0, 4, 0, 1, 3, 2, 4};
    constexpr std::array<int, 6> exact_outer_indices{0, 1, 3, 6, 6, 8};

    CHECK_EQ(values_size, exact_values.size());
    CHECK_EQ(inner_indices_size, exact_inner_indices.size());
    CHECK_EQ(outer_indices_size, exact_outer_indices.size());

    for (std::size_t i{0}; i < values_size; ++i) {
        CHECK_EQ(values[i], exact_values[i]);
    }
    for (std::size_t i{0}; i < inner_indices_size; ++i) {
        CHECK_EQ(inner_indices[i], exact_inner_indices[i]);
    }
    for (std::size_t i{0}; i < outer_indices_size; ++i) {
        CHECK_EQ(outer_indices[i], exact_outer_indices[i]);
    }
}

} // namespace basjoo::math
