/**
 * @file test_dense_problem.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-15
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/cvxopm/problems/dense_problem.hpp"

#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/mdfunctions/linear_mdfunction.hpp"
#include "basjoo/math/mdfunctions/quadratic_mdfunction.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"
#include "basjoo/math/utils.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::cvxopm {

TEST_CASE_TEMPLATE("QuadraticTest", T, float, double, long double) {
    auto exact_quadratic_mdfunction = [](const ::basjoo::math::pmr::Vector<T, std::dynamic_extent>& x) noexcept -> T {
        return x[0] * x[0] + x[0] * x[1] + x[1] * x[1] - x[0] * static_cast<T>(2.0) -
               x[1] * static_cast<T>(4.0) + static_cast<T>(1.0);
    };
    ::basjoo::math::QuadraticMdFunction<::basjoo::math::pmr::Vector<T, std::dynamic_extent>> quadratic_mdfunction(2);

    quadratic_mdfunction.updateBias(1.0);
    quadratic_mdfunction.updateLinearCoeff(0, -2.0);
    quadratic_mdfunction.updateLinearCoeff(1, -4.0);
    quadratic_mdfunction.updateQuadraticCoeff(0, 0, 1.0);
    quadratic_mdfunction.updateQuadraticCoeff(0, 1, 1.0);
    quadratic_mdfunction.updateQuadraticCoeff(1, 1, 1.0);

    const DenseProblem<T> dense_problem(
        ::basjoo::math::makeMdFunctionProxy(std::move(quadratic_mdfunction))
    );

    const std::vector<double> x0s = ::basjoo::math::linspace(-2.0, 4.0, 400);
    const std::vector<double> x1s = ::basjoo::math::linspace(-2.0, 4.0, 400);

    CHECK_EQ(dense_problem.num_variables(), 2);

    for (T x0 : x0s) {
        for (T x1 : x1s) {
            const ::basjoo::math::pmr::Vector<T, std::dynamic_extent> x{{x0, x1}};
            CHECK_EQ(
                dense_problem.cost(x), doctest::Approx(exact_quadratic_mdfunction(x)).epsilon(1E-5)
            );
        }
    }
}

TEST_CASE_TEMPLATE("RosenBrockTest", T, float, double, long double) {
    auto exact_rosenbrock_function = [](const ::basjoo::math::pmr::Vector<T, std::dynamic_extent>& x) noexcept -> T {
        return (1.0 - x[0]) * (1.0 - x[0]) + 100.0 * (x[1] - x[0] * x[0]) * (x[1] - x[0] * x[0]);
    };
    const DenseProblem<T> dense_problem(
        ::basjoo::math::makeMdFunctionProxy(
            ::basjoo::math::RosenbrockFunction<::basjoo::math::pmr::Vector<T, std::dynamic_extent>>(1.0, 100.0)
        )
    );

    const std::vector<double> x0s = ::basjoo::math::linspace(-2.0, 2.0, 400);
    const std::vector<double> x1s = ::basjoo::math::linspace(-1.0, 3.0, 400);

    CHECK_EQ(dense_problem.num_variables(), 2);

    for (T x0 : x0s) {
        for (T x1 : x1s) {
            const ::basjoo::math::pmr::Vector<T, std::dynamic_extent> x{{x0, x1}};
            CHECK_EQ(
                dense_problem.cost(x), doctest::Approx(exact_rosenbrock_function(x)).epsilon(1E-5)
            );
        }
    }
}

TEST_CASE("SerializationTest") {
    using param_type = ::basjoo::math::pmr::Vector<double, std::dynamic_extent>;

    param_type linear_coeffs{{1.5, -2.5, 4.0}};
    const DenseProblem<double> dense_problem(
        ::basjoo::math::makeMdFunctionProxy(
            ::basjoo::math::LinearMdFunction<param_type>{0.75, linear_coeffs}
        )
    );

    auto [data, out] = zpp::bits::data_out();
    out(dense_problem).or_throw();

    DenseProblem<double> other_dense_problem{};
    auto in = zpp::bits::in(data);
    in(other_dense_problem).or_throw();

    CHECK_EQ(other_dense_problem.num_variables(), dense_problem.num_variables());

    const param_type x{{2.0, 3.0, 1.0}};
    CHECK_EQ(other_dense_problem.cost(x), dense_problem.cost(x));
}

} // namespace basjoo::cvxopm
