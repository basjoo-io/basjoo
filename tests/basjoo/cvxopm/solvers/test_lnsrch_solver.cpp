/**
 * @file test_lnsrch_solver.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-15
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/cvxopm/solvers/lnsrch_solver.hpp"

#include <utility>

#include "basjoo/cvxopm/problems/dense_problem.hpp"
#include "basjoo/math/dense/matrixx.hpp"
#include "basjoo/math/dense/vectorx.hpp"
#include "basjoo/math/mdfunctions/quadratic_mdfunction.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::cvxopm {

TEST_CASE_TEMPLATE("QuadraticTest", T, float, double, long double) {
    ::basjoo::math::QuadraticMdFunction<::basjoo::math::pmr::VectorX<T>> quadratic_mdfunction(2);

    quadratic_mdfunction.updateBias(1.0);
    quadratic_mdfunction.updateLinearCoeff(0, -2.0);
    quadratic_mdfunction.updateLinearCoeff(1, -4.0);
    quadratic_mdfunction.updateQuadraticCoeff(0, 0, 1.0);
    quadratic_mdfunction.updateQuadraticCoeff(0, 1, 1.0);
    quadratic_mdfunction.updateQuadraticCoeff(1, 1, 1.0);

    const DenseProblem<T> dense_problem(
        ::basjoo::math::makeMdFunctionProxy(std::move(quadratic_mdfunction))
    );

    const LnsrchSolver<T> lnsrch_solver{Settings<T>{.eps_abs{1E-6}}};

    ::basjoo::math::pmr::VectorX<T> x0({0.0, -1.0});

    const auto [result, info] = lnsrch_solver.solve(dense_problem, std::move(x0));

    CHECK_EQ(info.iter, 7);
    CHECK_EQ(result.prim_vars[0], doctest::Approx(0.0).epsilon(1E-6));
    CHECK_EQ(result.prim_vars[1], doctest::Approx(2.0).epsilon(1E-6));
}

TEST_CASE_TEMPLATE("RosenBrockTest", T, float, double, long double) {
    const DenseProblem<T> dense_problem(
        ::basjoo::math::makeMdFunctionProxy(
            ::basjoo::math::RosenbrockFunction<::basjoo::math::pmr::VectorX<T>>(1.0, 100.0)
        )
    );

    const LnsrchSolver<T> lnsrch_solver{Settings<T>{.eps_abs{1E-6}}};

    ::basjoo::math::pmr::VectorX<T> x0({-1.0, 1.0});

    const auto [result, info] = lnsrch_solver.solve(dense_problem, std::move(x0));

    CHECK_LE(info.iter, 4000);
    CHECK_EQ(result.prim_vars[0], doctest::Approx(1.0).epsilon(7E-3));
    CHECK_EQ(result.prim_vars[1], doctest::Approx(1.0).epsilon(2E-2));
}

} // namespace basjoo::cvxopm
