/**
 * @file test_osqp_problem.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit tests for OsqpProblem sparse QP formulation.
 * @version 0.1
 * @date 2023-07-20
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/cvxopm/problems/osqp_problem.hpp"

#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/utils.hpp"

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
class OsqpProblemTestFixture {
  public:
    OsqpProblemTestFixture(const OsqpProblemTestFixture& other) noexcept = delete;
    auto operator=(const OsqpProblemTestFixture& other) noexcept
        -> OsqpProblemTestFixture& = delete;
    OsqpProblemTestFixture(OsqpProblemTestFixture&& other) noexcept = delete;
    auto operator=(OsqpProblemTestFixture&& other) noexcept -> OsqpProblemTestFixture& = delete;
    ~OsqpProblemTestFixture() noexcept = default;

    OsqpProblemTestFixture() noexcept : osqp_problem(2, 3) {
        osqp_problem.addQuadCostTerm(0, 0, 2.0);
        osqp_problem.addQuadCostTerm(1, 1, 1.0);
        osqp_problem.addQuadCostTerm(0, 1, 1.0);
        osqp_problem.addLinCostTerm(0, 1.0);
        osqp_problem.addLinCostTerm(1, 1.0);
        osqp_problem.updateConstraintTerm(0, {{0, 1.0}, {1, 1.0}}, 1.0, 1.0);
        osqp_problem.updateConstraintTerm(1, {{0, 1.0}}, 0.0, 0.7);
        osqp_problem.updateConstraintTerm(2, {{1, 1.0}}, 0.0, 0.7);
    }

  protected:
    OsqpProblem<double, int> osqp_problem;
};

TEST_CASE_FIXTURE(OsqpProblemTestFixture, "Basic") {
    std::vector<double> state_vec;
    double exact_cost{0.0};
    SUBCASE("state_0") {
        state_vec = std::vector<double>{0.0, 0.0};
        exact_cost = 0.0;
        CHECK_FALSE(osqp_problem.validate(state_vec));
    }

    SUBCASE("state_1") {
        state_vec = std::vector<double>{0.462, 0.538};
        exact_cost = state_vec[0] * state_vec[0] * 2.0 + state_vec[1] * state_vec[1] +
                     state_vec[0] * state_vec[1] + state_vec[0] + state_vec[1];
        CHECK(osqp_problem.validate(state_vec));
    }

    SUBCASE("state_2") {
        state_vec = std::vector<double>{0.462, 0.538 + ::basjoo::math::kEpsilon};
        exact_cost = state_vec[0] * state_vec[0] * 2.0 + state_vec[1] * state_vec[1] +
                     state_vec[0] * state_vec[1] + state_vec[0] + state_vec[1];
        CHECK_FALSE(osqp_problem.validate(state_vec));
    }

    SUBCASE("state_3") {
        state_vec = std::vector<double>{0.462 - ::basjoo::math::kEpsilon, 0.538};
        exact_cost = state_vec[0] * state_vec[0] * 2.0 + state_vec[1] * state_vec[1] +
                     state_vec[0] * state_vec[1] + state_vec[0] + state_vec[1];
        CHECK_FALSE(osqp_problem.validate(state_vec));
    }

    CHECK_EQ(
        osqp_problem.cost(state_vec), doctest::Approx(exact_cost).epsilon(::basjoo::math::kEpsilon)
    );
}

TEST_CASE_FIXTURE(OsqpProblemTestFixture, "Serialization") {
    auto [data, out] = zpp::bits::data_out();
    out(osqp_problem).or_throw();

    OsqpProblem<double, int> other_problem;
    auto in = zpp::bits::in(data);
    in(other_problem).or_throw();

    const std::vector<double> state_vec{0.462, 0.538};
    const double exact_cost = state_vec[0] * state_vec[0] * 2.0 + state_vec[1] * state_vec[1] +
                              state_vec[0] * state_vec[1] + state_vec[0] + state_vec[1];
    CHECK(other_problem.validate(state_vec));
    CHECK_EQ(
        other_problem.cost(state_vec), doctest::Approx(exact_cost).epsilon(::basjoo::math::kEpsilon)
    );
}

} // namespace basjoo::cvxopm
