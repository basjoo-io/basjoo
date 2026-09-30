/**
 * @file rosenbrock_function.hpp
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

#include <algorithm>
#include <cmath>
#include <numeric>

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"

namespace basjoo::math {

template <VecArithmetic T>
struct RosenbrockFunction final {
    using serialize = zpp::bits::members<2>;

    using param_type = T;
    using value_type = typename param_type::value_type;
    using reference = typename param_type::reference;
    using const_reference = typename param_type::const_reference;
    using pointer = typename param_type::pointer;
    using const_pointer = typename param_type::const_pointer;
    using size_type = typename param_type::size_type;
    using allocator_type = typename param_type::allocator_type;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: pure, always_inline]]
    auto num_dimensions() const noexcept -> size_type {
        return 2;
    }

    [[using gnu: pure, always_inline]]
    auto operator()(const param_type& x) const noexcept -> value_type {
        return eval(x);
    }

    [[using gnu: pure, always_inline]]
    auto eval(const param_type& x) const noexcept -> value_type {
        return (a - x[0]) * (a - x[0]) + b * (x[0] * x[0] - x[1]) * (x[0] * x[0] - x[1]);
    }

    [[using gnu: pure, always_inline]]
    auto gradient([[maybe_unused]] const param_type& x) const noexcept -> param_type {
        param_type g{x};
        g[0] = 2.0 * (x[0] - a) + 4.0 * b * x[0] * (x[0] * x[0] - x[1]);
        g[1] = 2.0 * b * (x[1] - x[0] * x[0]);
        return g;
    }

    [[using gnu: pure, always_inline]]
    auto gradient([[maybe_unused]] const param_type& x, size_type idx) const noexcept
        -> value_type {
        if (idx == 0) {
            return 2.0 * (x[0] - a) + 4.0 * b * x[0] * (x[0] * x[0] - x[1]);
        } else if (idx == 1) {
            return 2.0 * b * (x[1] - x[0] * x[0]);
        } else {
            return 0.0;
        }
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

    value_type a{1.0}, b{100.0};
};

} // namespace basjoo::math
