/**
 * @file test_osqp_solver.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-21
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/cvxopm/solvers/osqp_solver.hpp"

#include "basjoo/cvxopm/info.hpp"
#include "basjoo/cvxopm/problems/osqp_problem.hpp"
#include "basjoo/cvxopm/result.hpp"
#include "basjoo/cvxopm/settings.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::cvxopm {

/**
 * @brief Here we construct a simple Quadratic programming model to test.
 *        cost:
 *            f(x0, x1) = 2 * x0^2 + x1^2 + x0*x1 + x0 + x1
 *        constraints:
 *            1 <= x0 + x1 <= 1
 *            0 <= x0 <= 0.7
 *            0 <= x1 <= 0.7
 *
 */
class OsqpSolverTestFixture {
  public:
    OsqpSolverTestFixture(const OsqpSolverTestFixture& other) noexcept = delete;
    auto operator=(const OsqpSolverTestFixture& other) noexcept -> OsqpSolverTestFixture& = delete;
    OsqpSolverTestFixture(OsqpSolverTestFixture&& other) noexcept = delete;
    auto operator=(OsqpSolverTestFixture&& other) noexcept -> OsqpSolverTestFixture& = delete;
    ~OsqpSolverTestFixture() noexcept = default;

    OsqpSolverTestFixture() noexcept : osqp_problem(2, 3) {
        osqp_problem.updateQuadCostTerm(0, 0, 2.0);
        osqp_problem.updateQuadCostTerm(1, 1, 1.0);
        osqp_problem.updateQuadCostTerm(0, 1, 1.0);
        osqp_problem.updateLinCostTerm(0, 1.0);
        osqp_problem.updateLinCostTerm(1, 1.0);
        osqp_problem.updateConstraintTerm(0, {{0, 1.0}, {1, 1.0}}, 1.0, 1.0);
        osqp_problem.updateConstraintTerm(1, {{0, 1.0}}, 0.0, 0.7);
        osqp_problem.updateConstraintTerm(2, {{1, 1.0}}, 0.0, 0.7);
    }

  protected:
    OsqpProblem<double, int> osqp_problem;
};

TEST_CASE_FIXTURE(OsqpSolverTestFixture, "ColdStart") {
    const OsqpSolver<double, int> solver{Settings<double, int>{.polishing = true}};
    const auto [result, info] = solver.solve(osqp_problem);

    CHECK_EQ(solver.settings.polishing, true);
    CHECK_EQ(result.prim_vars[0], doctest::Approx(0.3).epsilon(1e-7));
    CHECK_EQ(result.prim_vars[1], doctest::Approx(0.7).epsilon(1e-7));
    CHECK_EQ(result.dual_vars[0], doctest::Approx(-2.9).epsilon(1e-7));
    CHECK_EQ(result.dual_vars[1], 0.0);
    CHECK_EQ(result.dual_vars[2], doctest::Approx(0.2).epsilon(1e-6));
    CHECK_EQ(info.iter, 25);
}

TEST_CASE_FIXTURE(OsqpSolverTestFixture, "WarmStart") {
    const std::vector<double> prim_vars_0{0.3, 0.7};
    const std::vector<double> dual_vars_0{-2.9, 0.0, 0.2};

    const OsqpSolver<double, int> solver{Settings<double, int>{.polishing = true}};
    const auto [result, info] = solver.solve(osqp_problem, prim_vars_0, dual_vars_0);

    CHECK_EQ(result.prim_vars[0], doctest::Approx(0.3).epsilon(1e-7));
    CHECK_EQ(result.prim_vars[1], doctest::Approx(0.7).epsilon(1e-7));
    CHECK_EQ(result.dual_vars[0], doctest::Approx(-2.9).epsilon(1e-7));
    CHECK_EQ(result.dual_vars[1], 0.0);
    CHECK_EQ(result.dual_vars[2], doctest::Approx(0.2).epsilon(1e-6));
    CHECK_EQ(info.iter, 25);
}

} // namespace basjoo::cvxopm
