/**
 * @file test_qdldl_dcmp.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-06-06
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/math/sparse/qdldl_dcmp.hpp"

#include <array>
#include <stdexcept>

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/sparse/csc_matrix.hpp"

namespace basjoo::math {

// Test matrix (upper triangular part only):
//
//   A = [ 4  1  0 ]
//       [ 1  3  1 ]
//       [ 0  1  2 ]
//
//   det(A) = 18
//
//   A^{-1} = (1/18) * [  5  -2   1 ]
//                     [ -2   8  -4 ]
//                     [  1  -4  11 ]

TEST_CASE("FactorAndSolve") {
    CscMatrix<double, int> mat(3, 3);
    mat.updateCoeff(0, 0, 4.0);
    mat.updateCoeff(0, 1, 1.0);
    mat.updateCoeff(1, 1, 3.0);
    mat.updateCoeff(1, 2, 1.0);
    mat.updateCoeff(2, 2, 2.0);

    const QdldlDcmp<CscMatrix<double, int>> dcmp{mat};

    SUBCASE("SolveB0") {
        Vector<double, std::dynamic_extent> b(3, 0.0);
        b[0] = 1.0;
        const auto x = dcmp.solve(b);
        CHECK(x[0] == doctest::Approx(5.0 / 18.0).epsilon(1e-14));
        CHECK(x[1] == doctest::Approx(-2.0 / 18.0).epsilon(1e-14));
        CHECK(x[2] == doctest::Approx(1.0 / 18.0).epsilon(1e-14));
    }

    SUBCASE("SolveB1") {
        Vector<double, std::dynamic_extent> b(3, 0.0);
        b[1] = 1.0;
        const auto x = dcmp.solve(b);
        CHECK(x[0] == doctest::Approx(-2.0 / 18.0).epsilon(1e-14));
        CHECK(x[1] == doctest::Approx(8.0 / 18.0).epsilon(1e-14));
        CHECK(x[2] == doctest::Approx(-4.0 / 18.0).epsilon(1e-14));
    }

    SUBCASE("SolveB2") {
        Vector<double, std::dynamic_extent> b(3, 0.0);
        b[2] = 1.0;
        const auto x = dcmp.solve(b);
        CHECK(x[0] == doctest::Approx(1.0 / 18.0).epsilon(1e-14));
        CHECK(x[1] == doctest::Approx(-4.0 / 18.0).epsilon(1e-14));
        CHECK(x[2] == doctest::Approx(11.0 / 18.0).epsilon(1e-14));
    }
}

TEST_CASE("ErrorNotUpperTriangular") {
    CscMatrix<double, int> mat(2, 2);
    mat.updateCoeff(0, 0, 4.0);
    mat.updateCoeff(1, 0, 1.0); // below diagonal — violates upper-triangular requirement
    mat.updateCoeff(1, 1, 3.0);
    CHECK_THROWS_AS((QdldlDcmp<CscMatrix<double, int>>{mat}), std::invalid_argument);
}

TEST_CASE("ErrorZeroDiagonal") {
    CscMatrix<double, int> mat(2, 2);
    mat.updateCoeff(0, 0, 0.0); // zero diagonal → D[0] = 0 → factorization fails
    mat.updateCoeff(0, 1, 1.0);
    mat.updateCoeff(1, 1, 1.0);
    CHECK_THROWS_AS((QdldlDcmp<CscMatrix<double, int>>{mat}), std::runtime_error);
}

// Data taken verbatim from qdldl/examples/example.c.
// A is 10x10, quasi-definite, upper-triangular part stored in CSC.
TEST_CASE("ExampleFromQdldlSource") {
    constexpr int kN{10};
    constexpr std::array<int, 11> kAp{0, 1, 2, 4, 5, 6, 8, 10, 12, 14, 17};
    constexpr std::array<int, 17> kAi{0, 1, 1, 2, 3, 4, 1, 5, 0, 6, 3, 7, 6, 8, 1, 2, 9};
    constexpr std::array<double, 17> kAx{1.0,        0.460641,   -0.121189, 0.417928,  0.177828,
                                         0.1,        -0.0290058, -1.0,      0.350321,  -0.441092,
                                         -0.0845395, -0.316228,  0.178663,  -0.299077, 0.182452,
                                         -1.56506,   -0.1};
    constexpr std::array<double, kN> kB{1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0};

    CscMatrix<double, int> mat(kN, kN);
    for (int j{0}; j < kN; ++j) {
        for (int k{kAp[j]}; k < kAp[j + 1]; ++k) {
            mat.updateCoeff(kAi[k], j, kAx[k]);
        }
    }

    const QdldlDcmp<CscMatrix<double, int>> dcmp{mat};

    Vector<double, std::dynamic_extent> rhs(kN, 0.0);
    for (int i{0}; i < kN; ++i) {
        rhs[i] = kB[i];
    }
    const auto x = dcmp.solve(rhs);

    // Verify residual r = A*x ≈ b using the symmetric upper-triangular structure.
    std::array<double, kN> r{};
    for (int j{0}; j < kN; ++j) {
        for (int k{kAp[j]}; k < kAp[j + 1]; ++k) {
            const int row{kAi[k]};
            r[row] += kAx[k] * x[j];
            if (row != j) {
                r[j] += kAx[k] * x[row];
            }
        }
    }
    for (int i{0}; i < kN; ++i) {
        CHECK(r[i] == doctest::Approx(kB[i]).epsilon(1e-10));
    }
}

} // namespace basjoo::math
