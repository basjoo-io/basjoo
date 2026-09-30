/**
 * @file osqp_problem.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Sparse QP problem formulation consumed by OsqpSolver.
 * @version 0.1
 * @date 2023-10-28
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <initializer_list>
#include <limits>
#include <memory_resource>
#include <span>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/sparse/dok_matrix.hpp"
#include "basjoo/math/sparse/lil_matrix.hpp"

namespace basjoo::cvxopm {

template <std::floating_point Scalar, std::integral Index>
class OsqpProblem final {
    template <std::floating_point, std::integral>
    friend class OsqpSolver;

  public:
    using value_type = Scalar;
    using index_type = Index;
    using size_type = std::size_t;
    using allocator_type = std::pmr::polymorphic_allocator<value_type>;

    OsqpProblem() noexcept = default;
    OsqpProblem(const OsqpProblem& other) noexcept = default;
    auto operator=(const OsqpProblem& other) noexcept -> OsqpProblem& = default;
    OsqpProblem(OsqpProblem&& other) noexcept = default;
    auto operator=(OsqpProblem&& other) noexcept -> OsqpProblem& = default;
    ~OsqpProblem() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_objective_matrix.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit OsqpProblem(const allocator_type& alloc) noexcept
        : m_objective_matrix(alloc), m_objective_vector(alloc), m_constraint_matrix(alloc),
          m_lower_bounds(alloc), m_upper_bounds(alloc) {}

    [[using gnu: always_inline]]
    OsqpProblem(size_type num_vars, size_type num_cons, const allocator_type& alloc = {}) noexcept
        : m_objective_matrix(num_vars, num_vars, alloc), m_objective_vector(num_vars, 0.0, alloc),
          m_constraint_matrix(num_cons, num_vars, alloc),
          m_lower_bounds(num_cons, std::numeric_limits<value_type>::lowest(), alloc),
          m_upper_bounds(num_cons, std::numeric_limits<value_type>::max(), alloc) {}

    [[using gnu: always_inline]]
    auto resize(size_type num_vars, size_type num_cons) noexcept -> void {
        m_objective_matrix.resize(num_vars, num_vars);
        m_objective_vector.resize(num_vars, 0.0);
        m_constraint_matrix.resize(num_cons, num_vars);
        m_lower_bounds.resize(num_cons, std::numeric_limits<value_type>::lowest());
        m_upper_bounds.resize(num_cons, std::numeric_limits<value_type>::max());
        return;
    }

    [[using gnu: always_inline]]
    auto clear() noexcept -> void {
        m_objective_matrix.clear();
        std::ranges::fill(m_objective_vector, 0.0);
        m_constraint_matrix.clear();
        std::ranges::fill(m_lower_bounds, std::numeric_limits<value_type>::lowest());
        std::ranges::fill(m_upper_bounds, std::numeric_limits<value_type>::max());
        return;
    }

    [[using gnu: pure, always_inline]]
    auto num_variables() const noexcept -> size_type {
        return m_objective_matrix.nrows();
    }

    [[using gnu: pure, always_inline]]
    auto num_constraints() const noexcept -> size_type {
        return m_constraint_matrix.nrows();
    }

    [[using gnu: pure]]
    auto cost(std::span<const value_type> x) const noexcept -> value_type {
        if (x.size() != num_variables()) [[unlikely]] {
            return 0.0;
        }
        value_type cost{0.0};
        for (const auto& [index_pair, value] : m_objective_matrix.dictionary()) {
            const auto& [row, col] = index_pair;
            if (row != col) {
                cost += value * x[row] * x[col];
            } else {
                cost += value * x[row] * x[col] * 0.5;
            }
        }
        const size_type num_vars{num_variables()};
        for (size_type i{0}; i < num_vars; ++i) {
            cost += m_objective_vector[i] * x[i];
        }
        return cost;
    }

    [[using gnu: pure]]
    auto validate(std::span<const value_type> x) const noexcept -> bool {
        if (x.size() != num_variables()) [[unlikely]] {
            return false;
        }
        for (const auto& row_dictionary : m_constraint_matrix.row_dictionaries()) {
            value_type inner_prod{0.0};
            for (const auto& [col, value] : row_dictionary.second) {
                inner_prod += value * x[col];
            }
            if (inner_prod < m_lower_bounds[row_dictionary.first] ||
                inner_prod > m_upper_bounds[row_dictionary.first]) {
                return false;
            }
        }
        return true;
    }

    [[using gnu: always_inline, hot]]
    auto addQuadCostTerm(size_type row, size_type col, value_type coeff) noexcept -> void {
        if (row >= num_variables() || col >= num_variables() ||
            coeff == static_cast<value_type>(0.0)) [[unlikely]] {
            return;
        }
        if (row < col) {
            coeff += m_objective_matrix.coeff(row, col);
            m_objective_matrix.updateCoeff(row, col, coeff);
        } else if (row == col) {
            coeff *= 2.0;
            coeff += m_objective_matrix.coeff(row, col);
            m_objective_matrix.updateCoeff(row, col, coeff);
        } else {
            coeff += m_objective_matrix.coeff(col, row);
            m_objective_matrix.updateCoeff(col, row, coeff);
        }
        return;
    }

    [[using gnu: always_inline, hot]]
    auto updateQuadCostTerm(size_type row, size_type col, value_type coeff) noexcept -> void {
        if (row >= num_variables() || col >= num_variables()) [[unlikely]] {
            return;
        }
        if (row < col) {
            m_objective_matrix.updateCoeff(row, col, coeff);
        } else if (row == col) {
            m_objective_matrix.updateCoeff(row, col, coeff * 2.0);
        } else {
            m_objective_matrix.updateCoeff(col, row, coeff);
        }
        return;
    }

    [[using gnu: always_inline, hot]]
    auto addLinCostTerm(size_type row, value_type coeff) noexcept -> void {
        if (row >= num_variables()) [[unlikely]] {
            return;
        }
        m_objective_vector[row] += coeff;
        return;
    }

    [[using gnu: always_inline, hot]]
    auto updateLinCostTerm(size_type row, value_type coeff) noexcept -> void {
        if (row >= num_variables()) [[unlikely]] {
            return;
        }
        m_objective_vector[row] = coeff;
        return;
    }

    template <
        std::ranges::input_range R = std::initializer_list<std::pair<const index_type, value_type>>>
    [[using gnu: always_inline, hot]]
    auto addRampCostTerm(
        R&& constrain_vec, value_type offset, value_type linear_coeff,
        value_type quadratic_coeff = 0.0
    ) noexcept -> void
        requires std::same_as<
            std::ranges::range_value_t<R>, std::pair<const index_type, value_type>>
    {
        const size_type num_vars{num_variables()}, num_cons{num_constraints()};
        m_objective_matrix.resize(num_vars + 1, num_vars + 1);
        m_objective_matrix.updateCoeff(num_vars, num_vars, quadratic_coeff * 2.0);
        m_objective_vector.push_back(linear_coeff);
        m_constraint_matrix.resize(num_cons + 2, num_vars + 1);
        m_constraint_matrix.updateRow(num_cons, {{num_vars, 1.0}});
        m_lower_bounds.push_back(0.0);
        m_upper_bounds.push_back(std::numeric_limits<value_type>::max());
        m_constraint_matrix.updateRow(num_cons + 1, constrain_vec);
        m_constraint_matrix.updateCoeff(num_cons + 1, num_vars, -1.0);
        m_lower_bounds.push_back(std::numeric_limits<value_type>::lowest());
        m_upper_bounds.push_back(offset);
        return;
    }

    template <
        std::ranges::input_range R = std::initializer_list<std::pair<const index_type, value_type>>>
    [[using gnu: always_inline, hot]]
    auto addConstraintTerm(
        R&& constrain_vec, value_type lower_bound, value_type upper_bound
    ) noexcept -> void
        requires std::same_as<
            std::ranges::range_value_t<R>, std::pair<const index_type, value_type>>
    {
        const size_type num_cons{num_constraints()};
        m_constraint_matrix.resize(num_cons + 1, m_constraint_matrix.ncols());
        m_constraint_matrix.updateRow(num_cons, constrain_vec);
        m_lower_bounds.push_back(lower_bound);
        m_upper_bounds.push_back(upper_bound);
        return;
    }

    template <
        std::ranges::input_range R = std::initializer_list<std::pair<const index_type, value_type>>>
    [[using gnu: always_inline, hot]]
    auto updateConstraintTerm(
        index_type row, R&& constrain_vec, value_type lower_bound, value_type upper_bound
    ) noexcept -> void
        requires std::same_as<
            std::ranges::range_value_t<R>, std::pair<const index_type, value_type>>
    {
        if (static_cast<size_type>(row) >= num_constraints()) [[unlikely]] {
            return;
        }
        m_constraint_matrix.updateRow(row, constrain_vec);
        m_lower_bounds[row] = lower_bound;
        m_upper_bounds[row] = upper_bound;
        return;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<5>;

    ::basjoo::math::DokMatrix<value_type, index_type, allocator_type> m_objective_matrix;
    std::vector<value_type, allocator_type> m_objective_vector;
    ::basjoo::math::LilMatrix<value_type, index_type, allocator_type> m_constraint_matrix;
    std::vector<value_type, allocator_type> m_lower_bounds;
    std::vector<value_type, allocator_type> m_upper_bounds;
};

} // namespace basjoo::cvxopm
