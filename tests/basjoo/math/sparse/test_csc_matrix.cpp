/**
 * @file test_csc_matrix.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-04
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/sparse/csc_matrix.hpp"

#include "zpp_bits.h"
#include <array>

#include "basjoo/math/sparse/coo_matrix.hpp"
#include "basjoo/math/sparse/csr_matrix.hpp"
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/lil_matrix.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("Basic") {
    CscMatrix<double, int> csc_matrix(5, 5);

    CHECK_EQ(csc_matrix.nrows(), 5);
    CHECK_EQ(csc_matrix.ncols(), 5);
    CHECK_EQ(csc_matrix.nnzs(), 0);
    CHECK_EQ(csc_matrix.outerIndices().size(), 6);

    csc_matrix.updateCoeff(4, 4, 8.0);
    csc_matrix.updateCoeff(1, 4, 17.0);
    csc_matrix.updateCoeff(2, 3, 1.0);
    csc_matrix.updateCoeff(4, 2, 14.0);
    csc_matrix.updateCoeff(2, 1, 5.0);
    csc_matrix.updateCoeff(0, 1, 3.0);
    csc_matrix.updateCoeff(2, 0, 7.0);
    csc_matrix.updateCoeff(1, 0, 22.0);
    csc_matrix.updateCoeff(5, 5, 1.0);

    CHECK_EQ(csc_matrix.coeff(1, 0), 22.0);
    CHECK_EQ(csc_matrix.coeff(2, 0), 7.0);
    CHECK_EQ(csc_matrix.coeff(0, 1), 3.0);
    CHECK_EQ(csc_matrix.coeff(2, 1), 5.0);
    CHECK_EQ(csc_matrix.coeff(4, 2), 14.0);
    CHECK_EQ(csc_matrix.coeff(2, 3), 1.0);
    CHECK_EQ(csc_matrix.coeff(1, 4), 17.0);
    CHECK_EQ(csc_matrix.coeff(4, 4), 8.0);
    CHECK_EQ(csc_matrix.coeff(5, 5), 0.0);
    CHECK_EQ(csc_matrix.coeff(3, 3), 0.0);

    CHECK_EQ(csc_matrix[1, 0], 22.0);
    CHECK_EQ(csc_matrix[2, 0], 7.0);
    CHECK_EQ(csc_matrix[0, 1], 3.0);
    CHECK_EQ(csc_matrix[2, 1], 5.0);
    CHECK_EQ(csc_matrix[4, 2], 14.0);
    CHECK_EQ(csc_matrix[2, 3], 1.0);
    CHECK_EQ(csc_matrix[1, 4], 17.0);
    CHECK_EQ(csc_matrix[4, 4], 8.0);
    CHECK_EQ(csc_matrix[5, 5], 0.0);
    CHECK_EQ(csc_matrix[3, 3], 0.0);

    const std::size_t values_size{csc_matrix.values().size()};
    const std::size_t inner_indices_size{csc_matrix.innerIndices().size()};
    const std::size_t outer_indices_size{csc_matrix.outerIndices().size()};
    const auto& values{csc_matrix.values()};
    const auto& inner_indices{csc_matrix.innerIndices()};
    const auto& outer_indices{csc_matrix.outerIndices()};

    constexpr std::array<double, 8> exact_values{22.0, 7.0, 3.0, 5.0, 14.0, 1.0, 17.0, 8.0};
    constexpr std::array<int, 8> exact_inner_indices{1, 2, 0, 2, 4, 2, 1, 4};
    constexpr std::array<int, 6> exact_outer_indices{0, 2, 4, 5, 6, 8};

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
    CscMatrix<double, int> csc_matrix(5, 5);

    csc_matrix.updateCoeff(4, 4, 8.0);
    csc_matrix.updateCoeff(1, 4, 17.0);
    csc_matrix.updateCoeff(2, 3, 1.0);
    csc_matrix.updateCoeff(4, 2, 14.0);
    csc_matrix.updateCoeff(2, 1, 5.0);
    csc_matrix.updateCoeff(0, 1, 3.0);
    csc_matrix.updateCoeff(2, 0, 7.0);
    csc_matrix.updateCoeff(1, 0, 22.0);
    csc_matrix.updateCoeff(5, 5, 1.0);

    SUBCASE("EnlargeSize") {
        csc_matrix.resize(10, 10);

        const std::size_t values_size{csc_matrix.values().size()};
        const std::size_t inner_indices_size{csc_matrix.innerIndices().size()};
        const std::size_t outer_indices_size{csc_matrix.outerIndices().size()};
        const auto& values{csc_matrix.values()};
        const auto& inner_indices{csc_matrix.innerIndices()};
        const auto& outer_indices{csc_matrix.outerIndices()};

        constexpr std::array<double, 8> exact_values{22.0, 7.0, 3.0, 5.0, 14.0, 1.0, 17.0, 8.0};
        constexpr std::array<int, 8> exact_inner_indices{1, 2, 0, 2, 4, 2, 1, 4};
        constexpr std::array<int, 11> exact_outer_indices{0, 2, 4, 5, 6, 8, 8, 8, 8, 8, 8};

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
        csc_matrix.resize(3, 3);

        const std::size_t values_size{csc_matrix.values().size()};
        const std::size_t inner_indices_size{csc_matrix.innerIndices().size()};
        const std::size_t outer_indices_size{csc_matrix.outerIndices().size()};
        const auto& values{csc_matrix.values()};
        const auto& inner_indices{csc_matrix.innerIndices()};
        const auto& outer_indices{csc_matrix.outerIndices()};

        constexpr std::array<double, 4> exact_values{22.0, 7.0, 3.0, 5.0};
        constexpr std::array<int, 4> exact_inner_indices{1, 2, 0, 2};
        constexpr std::array<int, 4> exact_outer_indices{0, 2, 4, 4};

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
        "FromCscMatrix", T, DokMatrix<double, int>, CooMatrix<double, int>, CsrMatrix<double, int>,
        LilMatrix<double, int>
    ) {
        CscMatrix<double, int> csc_matrix(4, 8);

        csc_matrix.updateCoeff(0, 3, 1.5);
        csc_matrix.updateCoeff(0, 1, 345.2);
        csc_matrix.updateCoeff(0, 0, 567.4);
        csc_matrix.updateCoeff(0, 5, 0.0);
        csc_matrix.updateCoeff(3, 2, 1.57);
        csc_matrix.updateCoeff(3, 4, -35.2);
        csc_matrix.updateCoeff(3, 1, 0.0);
        csc_matrix.updateCoeff(3, 3, 3775.0);

        T sparse_matrix{csc_matrix};

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
        "ToCscMatrix", T, DokMatrix<double, int>, CooMatrix<double, int>, CsrMatrix<double, int>,
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

        CscMatrix<double, int> csc_matrix{sparse_matrix};

        CHECK_EQ(csc_matrix.nrows(), 4);
        CHECK_EQ(csc_matrix.ncols(), 8);
        CHECK_EQ(csc_matrix.nnzs(), 8);

        CHECK_EQ(csc_matrix.coeff(0, 3), 1.5);
        CHECK_EQ(csc_matrix.coeff(0, 1), 345.2);
        CHECK_EQ(csc_matrix.coeff(0, 0), 567.4);
        CHECK_EQ(csc_matrix.coeff(0, 5), 0.0);
        CHECK_EQ(csc_matrix.coeff(0, 2), 0.0);
        CHECK_EQ(csc_matrix.coeff(3, 2), 1.57);
        CHECK_EQ(csc_matrix.coeff(3, 4), -35.2);
        CHECK_EQ(csc_matrix.coeff(3, 1), 0.0);
        CHECK_EQ(csc_matrix.coeff(3, 3), 3775.0);

        CHECK_EQ(csc_matrix[0, 3], 1.5);
        CHECK_EQ(csc_matrix[0, 1], 345.2);
        CHECK_EQ(csc_matrix[0, 0], 567.4);
        CHECK_EQ(csc_matrix[0, 5], 0.0);
        CHECK_EQ(csc_matrix[0, 2], 0.0);
        CHECK_EQ(csc_matrix[3, 2], 1.57);
        CHECK_EQ(csc_matrix[3, 4], -35.2);
        CHECK_EQ(csc_matrix[3, 1], 0.0);
        CHECK_EQ(csc_matrix[3, 3], 3775.0);
    }
}

TEST_CASE("Serialization") {
    CscMatrix<double, int> csc_matrix(5, 5);

    csc_matrix.updateCoeff(4, 4, 8.0);
    csc_matrix.updateCoeff(1, 4, 17.0);
    csc_matrix.updateCoeff(2, 3, 1.0);
    csc_matrix.updateCoeff(4, 2, 14.0);
    csc_matrix.updateCoeff(2, 1, 5.0);
    csc_matrix.updateCoeff(0, 1, 3.0);
    csc_matrix.updateCoeff(2, 0, 7.0);
    csc_matrix.updateCoeff(1, 0, 22.0);
    csc_matrix.updateCoeff(5, 5, 1.0);

    auto [data, out] = zpp::bits::data_out();
    out(csc_matrix).or_throw();

    CscMatrix<double, int> other_csc_matrix;
    auto in = zpp::bits::in(data);
    in(other_csc_matrix).or_throw();

    const std::size_t values_size{other_csc_matrix.values().size()};
    const std::size_t inner_indices_size{other_csc_matrix.innerIndices().size()};
    const std::size_t outer_indices_size{other_csc_matrix.outerIndices().size()};
    const auto& values{other_csc_matrix.values()};
    const auto& inner_indices{other_csc_matrix.innerIndices()};
    const auto& outer_indices{other_csc_matrix.outerIndices()};

    constexpr std::array<double, 8> exact_values{22.0, 7.0, 3.0, 5.0, 14.0, 1.0, 17.0, 8.0};
    constexpr std::array<int, 8> exact_inner_indices{1, 2, 0, 2, 4, 2, 1, 4};
    constexpr std::array<int, 6> exact_outer_indices{0, 2, 4, 5, 6, 8};

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
