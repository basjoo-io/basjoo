/**
 * @file bfgs_solver.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-16
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <span>
#include <utility>

#include "basjoo/cvxopm/info.hpp"
#include "basjoo/cvxopm/problems/dense_problem.hpp"
#include "basjoo/cvxopm/result.hpp"
#include "basjoo/cvxopm/settings.hpp"
#include "basjoo/cvxopm/solvers/detail/bfgs.hpp"
#include "basjoo/cvxopm/solvers/detail/lnsrch.hpp"
#include "basjoo/math/dense/matrixx.hpp"
#include "basjoo/math/dense/vectorx.hpp"
#include "basjoo/math/mdfunctions/mdfunction_proxy.hpp"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index = int>
struct BfgsSolver final {
    using value_type = Scalar;
    using index_type = Index;
    using param_type = ::basjoo::math::pmr::VectorX<value_type>;
    using size_type = std::size_t;

    BfgsSolver() noexcept = delete;
    BfgsSolver(const BfgsSolver& other) noexcept = delete;
    auto operator=(const BfgsSolver& other) noexcept -> BfgsSolver& = delete;
    BfgsSolver(BfgsSolver&& other) noexcept = delete;
    auto operator=(BfgsSolver&& other) noexcept -> BfgsSolver& = delete;
    ~BfgsSolver() noexcept = default;

    [[using gnu: always_inline]]
    explicit BfgsSolver(const Settings<value_type, index_type>& c_settings) noexcept
        : settings{c_settings} {}

    [[using gnu: pure]] [[nodiscard]]
    auto solve(
        const DenseProblem<value_type, index_type>& problem,
        std::span<const value_type> prim_vars_0 = {},
        [[maybe_unused]] std::span<const value_type> dual_vars_0 = {}
    ) const -> std::pair<Result<value_type>, Info<value_type, index_type>> {
        const ::basjoo::math::MdFunctionProxy<param_type>& objective_function{
            problem.objective_function()
        };
        const size_type num_vars{objective_function->num_dimensions()};
        const param_type zeros(num_vars, 0.0, settings.memory_resource);

        param_type x{zeros};
        if (!prim_vars_0.empty()) {
            std::ranges::copy_n(prim_vars_0.data(), num_vars, x.data());
        }
        param_type g0 = objective_function->gradient(x);
        param_type dx = detail::lnsrch<param_type>(
            objective_function, x, -g0, settings.max_step, settings.wolfe_rate, settings.eps_abs
        );
        x += dx;
        detail::BfgsHessinv<param_type> hessinv(num_vars, settings.memory_resource);

        index_type i{0};
        for (; i < settings.max_iter && !dx.identicalTo(zeros, settings.eps_abs); ++i, x += dx) {
            param_type g = objective_function->gradient(x);
            hessinv.update(dx, g - g0);
            dx = -hessinv.dot(g);
            dx = detail::lnsrch<param_type>(
                objective_function, x, std::move(dx), settings.max_step, settings.wolfe_rate,
                settings.eps_abs
            );
            g0 = g;
        }
        Result<value_type> result{
            .prim_vars{std::move(x)}, .dual_vars{}, .prim_inf_cert{}, .dual_inf_cert{}
        };
        Info<value_type, index_type> info;
        info.iter = i;

        return std::make_pair(std::move(result), info);
    }

    Settings<value_type, index_type> settings;
};

} // namespace basjoo::cvxopm
