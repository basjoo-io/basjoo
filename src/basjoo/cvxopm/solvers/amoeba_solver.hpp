/**
 * @file amoeba_solver.hpp
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
#include <list>
#include <span>
#include <utility>

#include "basjoo/cvxopm/info.hpp"
#include "basjoo/cvxopm/problems/dense_problem.hpp"
#include "basjoo/cvxopm/result.hpp"
#include "basjoo/cvxopm/settings.hpp"
#include "basjoo/cvxopm/solvers/detail/amoeba.hpp"
#include "basjoo/math/dense/vectorx.hpp"
#include "basjoo/math/mdfunctions/mdfunction_proxy.hpp"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index = int>
struct AmoebaSolver final {
    using value_type = Scalar;
    using index_type = Index;
    using param_type = ::basjoo::math::pmr::VectorX<value_type>;
    using vertex_node_type = typename detail::Amoeba<param_type>::VertexNode;
    using size_type = std::size_t;

    AmoebaSolver() noexcept = delete;
    AmoebaSolver(const AmoebaSolver& other) noexcept = delete;
    auto operator=(const AmoebaSolver& other) noexcept -> AmoebaSolver& = delete;
    AmoebaSolver(AmoebaSolver&& other) noexcept = delete;
    auto operator=(AmoebaSolver&& other) noexcept -> AmoebaSolver& = delete;
    ~AmoebaSolver() noexcept = default;

    [[using gnu: always_inline]]
    explicit AmoebaSolver(const Settings<value_type, index_type>& c_settings) noexcept
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

        typename detail::Amoeba<param_type> amoeba(
            objective_function, x, settings.max_step, settings.memory_resource
        );

        index_type i{0};
        for (; i < settings.max_iter && !amoeba.converged(settings.eps_abs); ++i) {
            const auto [f, x] = amoeba.lerpTopVertex(-1.0);
            if (f <= amoeba.vertex_nodes().front().f) {
                const auto [f, x] = amoeba.lerpTopVertex(2.0);
            } else if (f >= (std::prev(amoeba.vertex_nodes().cend(), 2)->f)) {
                value_type fsave{amoeba.vertex_nodes().back().f};
                const auto [f, x] = amoeba.lerpTopVertex(0.5);
                if (f >= fsave) {
                    amoeba.contract(0.5);
                }
            }
        }

        Result<value_type> result{
            .prim_vars{amoeba.vertex_nodes().front().x},
            .dual_vars{},
            .prim_inf_cert{},
            .dual_inf_cert{}
        };
        Info<value_type, index_type> info;
        info.iter = i;

        return std::make_pair(std::move(result), info);
    }

    Settings<value_type, index_type> settings;
};

} // namespace basjoo::cvxopm
