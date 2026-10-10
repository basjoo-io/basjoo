/**
 * @file test_quadratic_mdfunction.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-09-07
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/math/mdfunctions/quadratic_mdfunction.hpp"

#include "zpp_bits.h"

#include "basjoo/math/dense/detail/dense_generate_trait.hpp"
#include "basjoo/math/dense/matrix.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vector.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE_TEMPLATE(
    "Basic", T, Vector<float, 5>, Vector<double, 5>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>
) {
    T linear_coeffs(5);
    linear_coeffs[0] = 674.0;
    linear_coeffs[1] = -194.45;
    linear_coeffs[2] = 22.55;
    linear_coeffs[3] = 1.056;
    linear_coeffs[4] = -0.47656;

    ::basjoo::math::detail::DenseGenerateTraitT<T> quadratic_coeffs(5, 5);
    quadratic_coeffs[0, 0] = 498.1031;
    quadratic_coeffs[0, 1] = -463.8805;
    quadratic_coeffs[0, 2] = 576.4003;
    quadratic_coeffs[0, 3] = -433.1227;
    quadratic_coeffs[0, 4] = -241.3050;
    quadratic_coeffs[1, 0] = -463.8805;
    quadratic_coeffs[1, 1] = -390.2700;
    quadratic_coeffs[1, 2] = -192.6189;
    quadratic_coeffs[1, 3] = 356.3896;
    quadratic_coeffs[1, 4] = -951.7053;
    quadratic_coeffs[2, 0] = 576.4003;
    quadratic_coeffs[2, 1] = -192.6189;
    quadratic_coeffs[2, 2] = -95.0645;
    quadratic_coeffs[2, 3] = -305.2551;
    quadratic_coeffs[2, 4] = -186.8198;
    quadratic_coeffs[3, 0] = -433.1227;
    quadratic_coeffs[3, 1] = 356.3896;
    quadratic_coeffs[3, 2] = -305.2551;
    quadratic_coeffs[3, 3] = -948.3788;
    quadratic_coeffs[3, 4] = 69.7871;
    quadratic_coeffs[4, 0] = -241.3050;
    quadratic_coeffs[4, 1] = -951.7053;
    quadratic_coeffs[4, 2] = -186.8198;
    quadratic_coeffs[4, 3] = 69.7871;
    quadratic_coeffs[4, 4] = -919.1390;

    QuadraticMdFunction<T> quadratic_mdfunction(1.264, linear_coeffs, quadratic_coeffs);

    CHECK_EQ(quadratic_mdfunction.num_dimensions(), 5);

    CHECK_EQ(quadratic_mdfunction.bias(), doctest::Approx(1.264).epsilon(1E-6));
    for (std::size_t i{0}; i < 5; ++i) {
        CHECK_EQ(
            quadratic_mdfunction.linearCoeff(i), doctest::Approx(linear_coeffs[i]).epsilon(1E-6)
        );
    }
    for (std::size_t i{0}; i < 5; ++i) {
        for (std::size_t j{0}; j < 5; ++j) {
            CHECK_EQ(
                quadratic_mdfunction.quadraticCoeff(i, j),
                doctest::Approx(quadratic_coeffs[i, j] * (i != j ? 1.0 : 0.5)).epsilon(1E-6)
            );
        }
    }

    quadratic_mdfunction.updateBias(103.984);
    quadratic_mdfunction.updateLinearCoeff(3, 2.475);
    quadratic_mdfunction.updateQuadraticCoeff(2, 4, 11.5081);

    CHECK_NE(quadratic_mdfunction.bias(), doctest::Approx(1.264).epsilon(1E-6));
    CHECK_NE(quadratic_mdfunction.linearCoeff(3), doctest::Approx(linear_coeffs[3]).epsilon(1E-6));
    CHECK_NE(
        quadratic_mdfunction.quadraticCoeff(2, 4),
        doctest::Approx(quadratic_coeffs[2, 4]).epsilon(1E-6)
    );

    CHECK_EQ(quadratic_mdfunction.bias(), doctest::Approx(103.984).epsilon(1E-6));
    CHECK_EQ(quadratic_mdfunction.linearCoeff(3), doctest::Approx(2.475).epsilon(1E-6));
    CHECK_EQ(quadratic_mdfunction.quadraticCoeff(2, 4), doctest::Approx(11.5081).epsilon(1E-6));

    quadratic_mdfunction.updateBias(1.264);
    quadratic_mdfunction.updateLinearCoeff(3, 1.056);
    quadratic_mdfunction.updateQuadraticCoeff(2, 4, -186.8198);

    T x(5);
    x[0] = 365.0;
    x[1] = -586.44;
    x[2] = 0.0;
    x[3] = 35.46;
    x[4] = -75.468;

    double linear_evaluation{0.0};
    for (std::size_t i{0}; i < 5; ++i) {
        linear_evaluation += linear_coeffs[i] * x[i];
    }

    double quadratic_evaluation{0.0};
    for (std::size_t i{0}; i < 5; ++i) {
        for (std::size_t j{0}; j < 5; ++j) {
            quadratic_evaluation += quadratic_coeffs[i, j] * x[i] * x[j] * 0.5;
        }
    }

    CHECK_EQ(
        quadratic_mdfunction(x),
        doctest::Approx(1.264 + linear_evaluation + quadratic_evaluation).epsilon(1E-6)
    );

    const T gradient{quadratic_mdfunction.gradient(x)};
    for (std::size_t i{0}; i < 5; ++i) {
        typename T::value_type quadratic_component{0.0};
        for (std::size_t j{0}; j < 5; ++j) {
            quadratic_component += quadratic_coeffs[i, j] * x[j];
        }
        CHECK_EQ(
            gradient[i], doctest::Approx(linear_coeffs[i] + quadratic_component).epsilon(1E-6)
        );
    }

    for (std::size_t i{0}; i < 5; ++i) {
        typename T::value_type quadratic_component{0.0};
        for (std::size_t j{0}; j < 5; ++j) {
            quadratic_component += quadratic_coeffs[i, j] * x[j];
        }
        CHECK_EQ(
            quadratic_mdfunction.gradient(x, i),
            doctest::Approx(linear_coeffs[i] + quadratic_component).epsilon(1E-6)
        );
    }
}

TEST_CASE_TEMPLATE(
    "Serialization", T, Vector<float, 5>, Vector<double, 5>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>
) {
    T linear_coeffs(5);
    linear_coeffs[0] = 674.0;
    linear_coeffs[1] = -194.45;
    linear_coeffs[2] = 22.55;
    linear_coeffs[3] = 1.056;
    linear_coeffs[4] = -0.47656;

    ::basjoo::math::detail::DenseGenerateTraitT<T> quadratic_coeffs(5, 5);
    quadratic_coeffs[0, 0] = 498.1031;
    quadratic_coeffs[0, 1] = -463.8805;
    quadratic_coeffs[0, 2] = 576.4003;
    quadratic_coeffs[0, 3] = -433.1227;
    quadratic_coeffs[0, 4] = -241.3050;
    quadratic_coeffs[1, 0] = -463.8805;
    quadratic_coeffs[1, 1] = -390.2700;
    quadratic_coeffs[1, 2] = -192.6189;
    quadratic_coeffs[1, 3] = 356.3896;
    quadratic_coeffs[1, 4] = -951.7053;
    quadratic_coeffs[2, 0] = 576.4003;
    quadratic_coeffs[2, 1] = -192.6189;
    quadratic_coeffs[2, 2] = -95.0645;
    quadratic_coeffs[2, 3] = -305.2551;
    quadratic_coeffs[2, 4] = -186.8198;
    quadratic_coeffs[3, 0] = -433.1227;
    quadratic_coeffs[3, 1] = 356.3896;
    quadratic_coeffs[3, 2] = -305.2551;
    quadratic_coeffs[3, 3] = -948.3788;
    quadratic_coeffs[3, 4] = 69.7871;
    quadratic_coeffs[4, 0] = -241.3050;
    quadratic_coeffs[4, 1] = -951.7053;
    quadratic_coeffs[4, 2] = -186.8198;
    quadratic_coeffs[4, 3] = 69.7871;
    quadratic_coeffs[4, 4] = -919.1390;

    QuadraticMdFunction<T> quadratic_mdfunction(1.264, linear_coeffs, quadratic_coeffs);

    auto [data, out] = zpp::bits::data_out();
    out(quadratic_mdfunction).or_throw();

    QuadraticMdFunction<T> other_quadratic_mdfunction;
    auto in = zpp::bits::in(data);
    in(other_quadratic_mdfunction).or_throw();

    CHECK_EQ(quadratic_mdfunction.bias(), other_quadratic_mdfunction.bias());
    for (std::size_t i{0}; i < 5; ++i) {
        CHECK_EQ(quadratic_mdfunction.linearCoeff(i), other_quadratic_mdfunction.linearCoeff(i));
    }
}

} // namespace basjoo::math
