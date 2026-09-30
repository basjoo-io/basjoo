/**
 * @file dense_problem.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-03
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>

#include "zpp_bits.h"

#include "basjoo/math/dense/vectorx.hpp"
#include "basjoo/math/mdfunctions/mdfunction_proxy.hpp"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index = int>
class DenseProblem final {
  public:
    using value_type = Scalar;
    using index_type = Index;
    using param_type = ::basjoo::math::pmr::VectorX<value_type>;
    using size_type = typename param_type::size_type;

    DenseProblem() noexcept = default;
    DenseProblem(const DenseProblem& other) noexcept = default;
    auto operator=(const DenseProblem& other) noexcept -> DenseProblem& = default;
    DenseProblem(DenseProblem&& other) noexcept = default;
    auto operator=(DenseProblem&& other) noexcept -> DenseProblem& = default;
    ~DenseProblem() noexcept = default;

    explicit DenseProblem(::basjoo::math::MdFunctionProxy<param_type> objective_function) noexcept
        : m_objective_function{std::move(objective_function)} {}

    [[using gnu: pure, always_inline]]
    auto cost(const param_type& x) const noexcept -> value_type {
        return m_objective_function->eval(x);
    }

    [[using gnu: pure, always_inline]]
    auto num_variables() const noexcept -> size_type {
        return m_objective_function->num_dimensions();
    }

    [[using gnu: pure, always_inline]]
    auto objective_function() const noexcept -> const ::basjoo::math::MdFunctionProxy<param_type>& {
        return m_objective_function;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    ::basjoo::math::MdFunctionProxy<param_type> m_objective_function;
};

} // namespace basjoo::cvxopm
