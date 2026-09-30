/**
 * @file lnsrch.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-10-24
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <cmath>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/mdfunctions/mdfunction_proxy.hpp"

namespace basjoo::cvxopm::detail {

template <basjoo::math::VecArithmetic T>
[[using gnu: pure, flatten, leaf, hot]]
auto lnsrch(
    const ::basjoo::math::MdFunctionProxy<T>& func, const T& x0, T p,
    typename T::value_type max_step = 1.0, typename T::value_type wolfe_rate = 1E-4,
    typename T::value_type tol_abs = 1E-8
) -> T {
    using value_type = typename T::value_type;
    using size_type = typename T::size_type;

    const size_type num_dims{func->num_dimensions()};
    value_type f0{func->eval(x0)};
    T g0{func->gradient(x0)};

    const value_type slope{p.dot(g0)};
    if (slope >= std::numeric_limits<value_type>::epsilon()) [[unlikely]] {
        throw std::runtime_error(
            "Runtime error detected! basjoo::cvxopm::detail::lnsrch() hits roundoff problem."
        );
    }

    const value_type p_euclidean = p.euclidean();
    if (p_euclidean > max_step) {
        p *= max_step / p_euclidean;
    }

    value_type test{0.0};
    for (std::size_t i{0}; i < num_dims; ++i) {
        test = std::max(
            test, std::abs(p[i]) / std::max(std::abs(x0[i]), static_cast<value_type>(1.0))
        );
    }

    const value_type alamin{tol_abs / test};
    value_type alam{1.0}, alam2{0.0}, f{0.0}, f2{0.0};
    while (true) {
        if (alam < alamin) {
            return T(num_dims, 0.0);
        }
        const T tmpp{p * alam};
        f = func->eval(x0 + tmpp);
        if (f <= f0 + wolfe_rate * alam * slope) {
            return tmpp;
        }
        value_type tmplam;
        if (alam == static_cast<value_type>(1.0)) {
            tmplam = -slope / ((f - f0 - slope) * static_cast<value_type>(2.0));
        } else {
            const value_type rhs1{f - f0 - alam * slope};
            const value_type rhs2{f2 - f0 - alam2 * slope};
            const value_type a{(rhs1 / (alam * alam) - rhs2 / (alam2 * alam2)) / (alam - alam2)};
            const value_type b{
                (-alam2 * rhs1 / (alam * alam) + alam * rhs2 / (alam2 * alam2)) / (alam - alam2)
            };
            if (a == 0.0) {
                tmplam = -slope / (b * static_cast<value_type>(2.0));
            } else {
                const value_type disc{b * b - a * slope * static_cast<value_type>(3.0)};
                if (disc < static_cast<value_type>(0.0)) {
                    tmplam = alam * static_cast<value_type>(0.5);
                } else if (b <= static_cast<value_type>(0.0)) {
                    tmplam = (-b + std::sqrt(disc)) / (a * static_cast<value_type>(3.0));
                } else {
                    tmplam = -slope / (b + std::sqrt(disc));
                }
            }
            tmplam = std::min(tmplam, alam * static_cast<value_type>(0.5));
        }
        alam2 = alam;
        f2 = f;
        alam = std::max(tmplam, alam * static_cast<value_type>(0.1));
    }
}

} // namespace basjoo::cvxopm::detail
