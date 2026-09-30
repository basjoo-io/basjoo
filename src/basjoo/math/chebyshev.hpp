/**
 * @file chebyshev.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-06
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <array>
#include <cmath>
#include <concepts>
#include <functional>
#include <limits>

#include "zpp_bits.h"

namespace basjoo::math {

template <std::floating_point Scalar = double, std::size_t Size = 10>
class Chebyshev final {
  public:
    using value_type = Scalar;

    Chebyshev() noexcept = default;
    Chebyshev(const Chebyshev&) noexcept = default;
    auto operator=(const Chebyshev&) noexcept -> Chebyshev& = default;
    Chebyshev(Chebyshev&&) noexcept = default;
    auto operator=(Chebyshev&&) noexcept -> Chebyshev& = default;
    ~Chebyshev() noexcept = default;

    [[using gnu: ]]
    explicit Chebyshev(
        std::function<auto(value_type)->value_type> func, value_type lower_bound,
        value_type upper_bound, value_type abs_tol = 1E-8
    ) noexcept
        : m_lower_bound{lower_bound}, m_upper_bound{upper_bound} {
        constexpr value_type kFactor{2.0 / Size};
        const value_type middle{(m_upper_bound + m_lower_bound) * 0.5};
        const value_type radius{(m_upper_bound - m_lower_bound) * 0.5};

        std::array<value_type, Size> func_knots;
        for (std::size_t i{0}; i < Size; ++i) {
            func_knots[i] = func(middle + radius * std::cos(M_PI * (i + 0.5) / Size));
        }

        for (std::size_t i{0}; i < Size; ++i) {
            m_coeffs[i] = 0.0;
            for (std::size_t j{0}; j < Size; ++j) {
                m_coeffs[i] += func_knots[j] * std::cos(M_PI * i * (j + 0.5) / Size);
            }
            m_coeffs[i] *= kFactor;
        }

        truncate(abs_tol);
    }

    template <std::floating_point OtherScalar, std::size_t OtherSize>
    [[using gnu: always_inline, leaf]]
    explicit Chebyshev(const Chebyshev<OtherScalar, OtherSize>& other) noexcept
        requires(Size >= OtherSize)
        : m_lower_bound(other.m_lower_bound), m_upper_bound(other.m_upper_bound),
          m_num_trunc(other.m_num_trunc) {
        for (std::size_t i{0}; i < OtherSize; ++i) {
            m_coeffs[i] = other.m_coeffs[i];
        }
    }

    [[using gnu: always_inline]]
    auto truncate(value_type abs_tol) noexcept -> void {
        m_num_trunc = Size;
        while (m_num_trunc > 1 && std::abs(m_coeffs[m_num_trunc - 1]) < abs_tol) {
            --m_num_trunc;
        }
        return;
    }

    [[using gnu: pure, always_inline, leaf]]
    constexpr auto size() const noexcept -> std::size_t {
        return Size;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto num_trunc() const noexcept -> std::size_t {
        return m_num_trunc;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto coeff(std::size_t i) const noexcept -> value_type {
        return m_coeffs[i];
    }

    [[using gnu: pure, always_inline, leaf]]
    auto coeffs() const noexcept -> const std::array<value_type, Size>& {
        return m_coeffs;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto lower_bound() const noexcept -> value_type {
        return m_lower_bound;
    }

    [[using gnu: pure, always_inline, leaf]]
    auto upper_bound() const noexcept -> value_type {
        return m_upper_bound;
    }

    [[using gnu: pure, always_inline]]
    auto eval(value_type t) const noexcept -> value_type {
        return eval(t, m_num_trunc);
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto eval(value_type t, std::size_t num_trunc) const noexcept -> value_type {
        if ((t - m_lower_bound) * (t - m_upper_bound) > 0.0) {
            return std::numeric_limits<value_type>::quiet_NaN();
        }

        const value_type y{
            (t * 2.0 - m_lower_bound - m_upper_bound) / (m_upper_bound - m_lower_bound)
        };
        const value_type y2{y * 2.0};
        value_type d{0.0}, dd{0.0};
        for (int i = num_trunc - 1; i > 0; --i) {
            const value_type sv{d};
            d = y2 * d - dd + m_coeffs[i];
            dd = sv;
        }

        return y * d - dd + 0.5 * m_coeffs[0];
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator()(value_type t) const noexcept -> value_type {
        return eval(t);
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator()(value_type t, std::size_t num_trunc) const noexcept -> value_type {
        return eval(t, num_trunc);
    }

    [[using gnu: pure]]
    auto derivative() const noexcept -> Chebyshev {
        const value_type factor{2.0 / (m_upper_bound - m_lower_bound)};
        Chebyshev derivative;
        derivative.m_lower_bound = m_lower_bound;
        derivative.m_upper_bound = m_upper_bound;
        derivative.m_num_trunc = m_num_trunc;

        derivative.m_coeffs[m_num_trunc - 1] = 0.0;
        derivative.m_coeffs[m_num_trunc - 2] = m_coeffs[m_num_trunc - 1] * (m_num_trunc - 1) * 2;
        for (int i = m_num_trunc - 2; i > 0; --i) {
            derivative.m_coeffs[i - 1] = derivative.m_coeffs[i + 1] + m_coeffs[i] * i * 2;
        }
        for (value_type& coeff : derivative.m_coeffs) {
            coeff *= factor;
        }

        return derivative;
    }

    [[using gnu: pure]]
    auto integral() const noexcept -> Chebyshev {
        const value_type factor{(m_upper_bound - m_lower_bound) * 0.25};
        Chebyshev integral;
        integral.m_lower_bound = m_lower_bound;
        integral.m_upper_bound = m_upper_bound;
        integral.m_num_trunc = m_num_trunc;

        value_type sum{0.0}, sign{1.0};
        for (std::size_t i{1}; i < m_num_trunc - 1; ++i) {
            integral.m_coeffs[i] = factor * (m_coeffs[i - 1] - m_coeffs[i + 1]) / i;
            sum += sign * integral.m_coeffs[i];
            sign = -sign;
        }
        integral.m_coeffs[m_num_trunc - 1] = factor * m_coeffs[m_num_trunc - 2] / (m_num_trunc - 1);
        sum += sign * integral.m_coeffs[m_num_trunc - 1];
        integral.m_coeffs[0] = sum * 2.0;

        return integral;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<4>;

    std::array<value_type, Size> m_coeffs{};
    value_type m_lower_bound{0.0}, m_upper_bound{1.0};
    std::size_t m_num_trunc{0};
};

} // namespace basjoo::math
