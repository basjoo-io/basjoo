/**
 * @file piecewise_linear_function.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-10
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <concepts>
#include <format>
#include <memory>
#include <memory_resource>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vectorx.hpp"
#include "basjoo/math/utils.hpp"

namespace basjoo::math {

template <GeneralArithmetic T, Allocatory Alloc = ::basjoo::common::AlignedAllocator<T, 32>>
class PiecewiseLinearFunction final {
  public:
    using value_type = T;
    using param_type = detail::DenseDegenerateTraitT<value_type>;
    using size_type = std::size_t;
    using allocator_type = Alloc;

    static constexpr param_type kDuplicateCriterion{1E-8};

    PiecewiseLinearFunction() noexcept = default;
    PiecewiseLinearFunction(const PiecewiseLinearFunction& other) = default;
    auto operator=(const PiecewiseLinearFunction& other) -> PiecewiseLinearFunction& = default;
    PiecewiseLinearFunction(PiecewiseLinearFunction&& other) noexcept = default;
    auto operator=(PiecewiseLinearFunction&& other) noexcept -> PiecewiseLinearFunction& = default;
    ~PiecewiseLinearFunction() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_ys.get_allocator();
    }

    [[using gnu: pure, always_inline]]
    auto periodic() const noexcept -> bool {
        return m_periodic;
    }

    template <
        std::ranges::input_range R0 = std::initializer_list<param_type>,
        std::ranges::input_range R1 = std::initializer_list<value_type>>
    [[using gnu: ]]
    explicit PiecewiseLinearFunction(R0&& ts, R1&& ys, const allocator_type& alloc = {})
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                     std::same_as<std::ranges::range_value_t<R1>, value_type>
        : m_ts(ts.cbegin(), ts.cend(), alloc), m_ys(ys.cbegin(), ys.cend(), alloc) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_ts.size() < 2 || m_ys.size() < 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! sizes of ts, ys must be greater than 2: ts.size() "
                    "= {0:d} while ys.size() = {1:d}.",
                    m_ts.size(), m_ys.size()
                )
            );
        }
        if (m_ts.size() != m_ys.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! ts, ys must share the same size: ts.size() = "
                    "{0:d} while ys.size() = {1:d}.",
                    m_ts.size(), m_ys.size()
                )
            );
        }
        if (!std::ranges::is_sorted(m_ts)) [[unlikely]] {
            throw std::invalid_argument(
                std::format("Invalid arguments detected! ts has to be a sorted array!")
            );
        }
        if (hasDuplicates(m_ts, kDuplicateCriterion)) [[unlikely]] {
            throw std::invalid_argument(
                std::format("Invalid arguments detected! ts can not have duplicated elements!")
            );
        }
#endif
    }

    template <
        std::ranges::input_range R0 = std::initializer_list<param_type>,
        std::ranges::input_range R1 = std::initializer_list<value_type>>
    [[using gnu: ]]
    explicit PiecewiseLinearFunction(
        [[maybe_unused]] periodic_tag tag, R0&& ts, R1&& ys, const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                 std::same_as<std::ranges::range_value_t<R1>, value_type>
        : PiecewiseLinearFunction(std::forward<R0>(ts), std::forward<R1>(ys), alloc) {
#if BASJOO_CHECK_PARAMS == 1
        if (std::abs(m_ys.front() - m_ys.back()) > kDuplicateCriterion) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! When choosing periodic boundary condition, it "
                    "requires ys.front() == ys.back() while ys.front() = {0}, ys.back() = "
                    "{1} here.",
                    m_ys.front(), m_ys.back()
                )
            );
        }
#endif
        m_periodic = true;
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto eval(param_type t) const noexcept -> value_type {
        if (m_periodic && (t < m_ts.front() || t > m_ts.back())) [[unlikely]] {
            t = wrapPeriodic(t, m_ts.front(), m_ts.back());
        }
        const size_type pos = nearestUpperElement(m_ts, t) - m_ts.cbegin();
        if (pos == 0) {
            return lerp(m_ys[0], m_ys[1], (t - m_ts[0]) / (m_ts[1] - m_ts[0]));
        }
        if (pos == m_ts.size()) {
            return lerp(
                m_ys[pos - 1], m_ys[pos - 2], (t - m_ts[pos - 1]) / (m_ts[pos - 2] - m_ts[pos - 1])
            );
        }
        return lerp(m_ys[pos - 1], m_ys[pos], (t - m_ts[pos - 1]) / (m_ts[pos] - m_ts[pos - 1]));
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto derivative(param_type t) const noexcept -> value_type {
        if (m_periodic && (t < m_ts.front() || t > m_ts.back())) [[unlikely]] {
            t = wrapPeriodic(t, m_ts.front(), m_ts.back());
        }
        const size_type pos = nearestUpperElement(m_ts, t) - m_ts.cbegin();
        if (pos == 0) {
            return (m_ys[1] - m_ys[0]) / (m_ts[1] - m_ts[0]);
        }
        if (pos == m_ts.size()) {
            return (m_ys[pos - 1] - m_ys[pos - 2]) / (m_ts[pos - 1] - m_ts[pos - 2]);
        }
        return (m_ys[pos] - m_ys[pos - 1]) / (m_ts[pos] - m_ts[pos - 1]);
    }

    [[using gnu: pure, always_inline]]
    auto derivative(param_type t, [[maybe_unused]] unsigned int order) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (order != 1) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid argument error! The PiecewiseLinearFunction only has first order "
                    "derivative: order = {0:d}.",
                    order
                )
            );
        }
#endif
        return derivative(t);
    }

    [[using gnu: pure]]
    auto integral(param_type lower_bound, param_type upper_bound) const noexcept -> value_type {
        if (m_periodic) [[unlikely]] {
            return periodicAccum(upper_bound) - periodicAccum(lower_bound);
        }
        return integralImpl(lower_bound, upper_bound);
    }

    [[using gnu: pure, always_inline]]
    auto minT() const noexcept -> param_type {
        return m_ts.front();
    }

    [[using gnu: pure, always_inline]]
    auto maxT() const noexcept -> param_type {
        return m_ts.back();
    }

    [[using gnu: pure, always_inline]]
    auto minY() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        return std::ranges::min(m_ys);
    }

    [[using gnu: pure, always_inline]]
    auto maxY() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        return std::ranges::max(m_ys);
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator()(param_type t) const noexcept -> value_type {
        return eval(t);
    }

    [[using gnu: pure, always_inline]]
    auto ts() const noexcept -> std::span<const param_type> {
        return m_ts;
    }

    [[using gnu: pure, always_inline]]
    auto ys() const noexcept -> std::span<const value_type> {
        return m_ys;
    }

  private:
    using param_allocator_type =
        typename std::allocator_traits<allocator_type>::template rebind_alloc<param_type>;

    [[using gnu: pure, always_inline]]
    auto periodicAccum(param_type t) const noexcept -> value_type {
        const param_type period{maxT() - minT()};
        const param_type cycles{std::floor((t - minT()) / period)};
        return integralImpl(minT(), maxT()) * cycles +
               integralImpl(minT(), wrapPeriodic(t, minT(), maxT()));
    }

    [[using gnu: pure]]
    auto integralImpl(param_type lower_bound, param_type upper_bound) const noexcept -> value_type {
        int sign{1};
        if (lower_bound > upper_bound) {
            std::swap(lower_bound, upper_bound);
            sign = -1;
        }
        const size_type size{m_ts.size()};
        const size_type istart = std::ranges::lower_bound(m_ts, lower_bound) - m_ts.cbegin();
        const size_type iend = std::ranges::lower_bound(m_ts, upper_bound) - m_ts.cbegin();
        if (istart == size || iend == 0 || istart == iend) {
            return (eval(lower_bound) + eval(upper_bound)) * (upper_bound - lower_bound) * 0.5 *
                   sign;
        }
        value_type result{0.0};
        result += (eval(lower_bound) + m_ys[istart]) * (m_ts[istart] - lower_bound) * 0.5;
        for (size_type i{istart}; i < iend - 1; ++i) {
            result += (m_ys[i] + m_ys[i + 1]) * (m_ts[i + 1] - m_ts[i]) * 0.5;
        }
        result += (m_ys[iend - 1] + eval(upper_bound)) * (upper_bound - m_ts[iend - 1]) * 0.5;
        result *= sign;
        return result;
    }

    friend zpp::bits::access;
    using serialize = zpp::bits::members<3>;

    std::vector<param_type, param_allocator_type> m_ts;
    std::vector<value_type, allocator_type> m_ys;
    bool m_periodic{false};
};

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<float, 32>>
using PiecewiseLinearFunction1s = PiecewiseLinearFunction<float, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<double, 32>>
using PiecewiseLinearFunction1d = PiecewiseLinearFunction<double, Alloc>;

namespace pmr {

template <GeneralArithmetic T>
using PiecewiseLinearFunction =
    ::basjoo::math::PiecewiseLinearFunction<T, std::pmr::polymorphic_allocator<T>>;

using PiecewiseLinearFunction1s =
    ::basjoo::math::PiecewiseLinearFunction<float, std::pmr::polymorphic_allocator<float>>;

using PiecewiseLinearFunction1d =
    ::basjoo::math::PiecewiseLinearFunction<double, std::pmr::polymorphic_allocator<double>>;

} // namespace pmr

} // namespace basjoo::math
