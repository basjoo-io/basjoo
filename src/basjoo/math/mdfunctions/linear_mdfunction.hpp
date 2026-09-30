/**
 * @file linear_mdfunction.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-09-07
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

template <VecArithmetic T>
class LinearMdFunction final {
  public:
    using param_type = T;
    using value_type = typename param_type::value_type;
    using reference = typename param_type::reference;
    using const_reference = typename param_type::const_reference;
    using pointer = typename param_type::pointer;
    using const_pointer = typename param_type::const_pointer;
    using size_type = typename param_type::size_type;
    using allocator_type = typename param_type::allocator_type;

    LinearMdFunction() noexcept = default;
    LinearMdFunction(const LinearMdFunction& other) noexcept = default;
    auto operator=(const LinearMdFunction& other) noexcept -> LinearMdFunction& = default;
    LinearMdFunction(LinearMdFunction&& other) noexcept = default;
    auto operator=(LinearMdFunction&& other) noexcept -> LinearMdFunction& = default;
    ~LinearMdFunction() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_linear_coeffs.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit LinearMdFunction(const allocator_type& alloc) noexcept
        : m_bias{0.0}, m_linear_coeffs(alloc) {}

    [[using gnu: always_inline]]
    explicit LinearMdFunction(size_type num_dims, const allocator_type& alloc = {}) noexcept
        : m_bias{0.0}, m_linear_coeffs(num_dims, 0.0, alloc) {}

    [[using gnu: always_inline]]
    explicit LinearMdFunction(value_type bias, param_type linear_coeffs) noexcept
        : m_bias{bias}, m_linear_coeffs{std::move(linear_coeffs)} {}

    [[using gnu: pure, always_inline]]
    auto num_dimensions() const noexcept -> size_type {
        return m_linear_coeffs.size();
    }

    [[using gnu: always_inline]]
    auto resize(size_type num_dims) noexcept -> void {
        m_linear_coeffs.resize(num_dims);
        return;
    }

    [[using gnu: always_inline]]
    auto resize(size_type num_dims, const_reference value) noexcept -> void {
        m_linear_coeffs.resize(num_dims, value);
        return;
    }

    [[using gnu: pure, always_inline]]
    auto operator()(const param_type& x) const noexcept -> value_type {
        return eval(x);
    }

    [[using gnu: pure, always_inline]]
    auto eval(const param_type& x) const noexcept -> value_type {
        return x.dot(m_linear_coeffs) + m_bias;
    }

    [[using gnu: pure, always_inline]]
    auto gradient([[maybe_unused]] const param_type& x) const noexcept -> param_type {
        return m_linear_coeffs;
    }

    [[using gnu: pure, always_inline]]
    auto gradient([[maybe_unused]] const param_type& x, size_type idx) const noexcept
        -> value_type {
        return m_linear_coeffs[idx];
    }

    [[using gnu: pure, always_inline]]
    auto has_extrema(const param_type& x, value_type tol = 1E-8) const noexcept -> bool {
        const value_type rf{
            static_cast<value_type>(1.0) / std::max(std::abs(eval(x)), static_cast<value_type>(1.0))
        };
        const param_type g{gradient(x)};
        const value_type test = std::transform_reduce(
            g.data(), g.data() + g.size(), x.data(), 0.0,
            [](const value_type& a, const value_type& b) noexcept -> value_type {
                return std::max(a, b);
            },
            [rf](const value_type& a, const value_type& b) noexcept -> value_type {
                return std::abs(a) * std::max(std::abs(b), static_cast<value_type>(1.0)) * rf;
            }
        );
        return test < tol;
    }

    [[using gnu: pure, always_inline]]
    auto bias() const noexcept -> value_type {
        return m_bias;
    }

    [[using gnu: always_inline]]
    auto updateBias(value_type bias) noexcept -> void {
        m_bias = bias;
        return;
    }

    [[using gnu: pure, always_inline]]
    auto linearCoeff(size_type row) const noexcept -> value_type {
        return m_linear_coeffs[row];
    }

    [[using gnu: always_inline]]
    auto updateLinearCoeff(size_type row, value_type value) noexcept -> void {
        m_linear_coeffs[row] = value;
        return;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<2>;

    value_type m_bias;
    param_type m_linear_coeffs;
};

} // namespace basjoo::math
