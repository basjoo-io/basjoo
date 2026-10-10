/**
 * @file test_lbfgs_solver.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-16
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/cvxopm/solvers/lbfgs_solver.hpp"

#include <utility>

#include "basjoo/cvxopm/problems/dense_problem.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/mdfunctions/quadratic_mdfunction.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::cvxopm {

TEST_CASE_TEMPLATE("QuadraticTest", T, float, double, long double) {
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

    const LbfgsSolver<T> lbfgs_solver{Settings<T>{.eps_abs{1E-6}}};

    ::basjoo::math::pmr::Vector<T, std::dynamic_extent> x0({0.0, -1.0});

    const auto [result, info] = lbfgs_solver.solve(dense_problem, std::move(x0));

    CHECK_LE(info.iter, 6);
    CHECK_EQ(result.prim_vars[0], doctest::Approx(0.0).epsilon(2E-4));
    CHECK_EQ(result.prim_vars[1], doctest::Approx(2.0).epsilon(6E-5));
}

TEST_CASE_TEMPLATE("RosenBrockTest", T, float, double, long double) {
    const DenseProblem<T> dense_problem(
        ::basjoo::math::makeMdFunctionProxy(
            ::basjoo::math::RosenbrockFunction<::basjoo::math::pmr::Vector<T, std::dynamic_extent>>(1.0, 100.0)
        )
    );

    const LbfgsSolver<T> lbfgs_solver{Settings<T>{.eps_abs{1E-6}}};

    ::basjoo::math::pmr::Vector<T, std::dynamic_extent> x0({-1.0, 1.0});

    auto [result, info] = lbfgs_solver.solve(dense_problem, std::move(x0));

    CHECK_LE(info.iter, 40);
    CHECK_EQ(result.prim_vars[0], doctest::Approx(1.0).epsilon(1E-6));
    CHECK_EQ(result.prim_vars[1], doctest::Approx(1.0).epsilon(1E-6));
}

} // namespace basjoo::cvxopm
