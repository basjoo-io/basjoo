/**
 * @file osqp_solver.h
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-10-29
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <span>

#include "basjoo/cvxopm/info.hpp"
#include "basjoo/cvxopm/problems/osqp_problem.hpp"
#include "basjoo/cvxopm/result.hpp"
#include "basjoo/cvxopm/settings.hpp"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index = int>
struct OsqpSolver final {
    OsqpSolver() noexcept = delete;
    OsqpSolver(const OsqpSolver& other) noexcept = delete;
    auto operator=(const OsqpSolver& other) noexcept -> OsqpSolver& = delete;
    OsqpSolver(OsqpSolver&& other) noexcept = delete;
    auto operator=(OsqpSolver&& other) noexcept -> OsqpSolver& = delete;
    ~OsqpSolver() noexcept = default;

    [[using gnu: always_inline]]
    explicit OsqpSolver(const ::basjoo::cvxopm::Settings<Scalar, Index>& c_settings) noexcept
        : settings{c_settings} {}

    [[using gnu: pure, visibility("default")]] [[nodiscard]]
    auto solve(
        const OsqpProblem<Scalar, Index>& osqp_problem, std::span<const Scalar> prim_vars_0 = {},
        std::span<const Scalar> dual_vars_0 = {}
    ) const -> std::pair<::basjoo::cvxopm::Result<Scalar>, ::basjoo::cvxopm::Info<Scalar, Index>>;

    ::basjoo::cvxopm::Settings<Scalar, Index> settings;
};

} // namespace basjoo::cvxopm
