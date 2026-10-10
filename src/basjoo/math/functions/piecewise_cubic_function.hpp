/**
 * @file piecewise_cubic_function.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-15
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstdint>
#include <format>
#include <memory_resource>
#include <ranges>
#include <span>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/cubic_interpolation.hpp"
#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"
#include "basjoo/math/utils.hpp"

namespace basjoo::math {

template <GeneralArithmetic T, Allocatory Alloc = ::basjoo::common::AlignedAllocator<T, 32>>
class PiecewiseCubicFunction final {
  public:
    using value_type = T;
    using param_type = detail::DenseDegenerateTraitT<value_type>;
    using size_type = std::size_t;
    using allocator_type = Alloc;

    static constexpr param_type kDuplicateCriterion{1E-8};

    PiecewiseCubicFunction() noexcept = default;
    PiecewiseCubicFunction(const PiecewiseCubicFunction& other) = default;
    auto operator=(const PiecewiseCubicFunction& other) -> PiecewiseCubicFunction& = default;
    PiecewiseCubicFunction(PiecewiseCubicFunction&& other) noexcept = default;
    auto operator=(PiecewiseCubicFunction&& other) noexcept -> PiecewiseCubicFunction& = default;
    ~PiecewiseCubicFunction() noexcept = default;

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
    [[using gnu: always_inline]]
    explicit PiecewiseCubicFunction(R0&& ts, R1&& ys, const allocator_type& alloc = {})
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                 std::same_as<std::ranges::range_value_t<R1>, value_type>
        : PiecewiseCubicFunction(
              ts, ys, BoundaryMode<value_type>{2, 0.0}, BoundaryMode<value_type>{2, 0.0}, alloc
          ) {}

    template <
        std::ranges::input_range R0 = std::initializer_list<param_type>,
        std::ranges::input_range R1 = std::initializer_list<value_type>>
    [[using gnu: ]]
    explicit PiecewiseCubicFunction(
        R0&& ts, R1&& ys, BoundaryMode<value_type> b0, BoundaryMode<value_type> bf,
        const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                     std::same_as<std::ranges::range_value_t<R1>, value_type>
        : m_ts(ts.cbegin(), ts.cend(), alloc), m_ys(ys.cbegin(), ys.cend(), alloc) {
#if BASJOO_CHECK_PARAMS == 1
        if (m_ts.size() < 2 || m_ys.size() < 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! sizes of ts, ys must be greater than 2: ts.size() "
                    "= {0:d} while ys.size() = {1:d}",
                    m_ts.size(), m_ys.size()
                )
            );
        }
        if (m_ts.size() != m_ys.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! ts, ys must share the same size: ts.size() = "
                    "{0:d} while ys.size() = {1:d}",
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

        if (b0.order == 0 || b0.order > 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid argument detected! The derivative order of b0 can only be 1, 2, 3: "
                    "b0.order = {0:d}.",
                    b0.order
                )
            );
        }
        if (bf.order == 0 || bf.order > 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid argument detected! The derivative order of b0 can only be 1, 2, 3: "
                    "bf.order = {0:d}.",
                    bf.order
                )
            );
        }
#endif
        const size_type size{m_ts.size()};
        std::vector<param_type, param_allocator_type> hs(size - 1, get_allocator());
        std::vector<value_type, allocator_type> ds(size - 1, get_allocator());
        std::vector<param_type, param_allocator_type> a_diag(size, get_allocator());
        std::vector<value_type, allocator_type> b(size, get_allocator());

        hs[0] = m_ts[1] - m_ts[0];
        ds[0] = (m_ys[1] - m_ys[0]) / hs[0];
        for (size_type i{1}; i < size - 1; i++) {
            hs[i] = m_ts[i + 1] - m_ts[i];
            ds[i] = (m_ys[i + 1] - m_ys[i]) / hs[i];
            a_diag[i] = (hs[i] + hs[i - 1]) * 2.0;
            b[i] = (ds[i] - ds[i - 1]) * 6.0;
        }

        std::vector<param_type, param_allocator_type> a_low{hs};
        std::vector<param_type, param_allocator_type> a_up{hs};

        if (b0.order == 2) {
            a_diag[0] = 1.0;
            a_up[0] = 0.0;
            b[0] = b0.derivative;
        } else {
            a_diag[0] = 1.0;
            a_up[0] = 0.5;
            b[0] = (ds[0] - b0.derivative) * 3.0 / hs[0];
        }
        if (bf.order == 2) {
            a_diag[size - 1] = 1.0;
            a_low[size - 2] = 0.0;
            b[size - 1] = bf.derivative;
        } else {
            a_diag[size - 1] = 1.0;
            a_low[size - 2] = 0.5;
            b[size - 1] = (bf.derivative - ds[size - 2]) * 3.0 / hs[size - 2];
        }

        const TridiagonalMatrix A{std::move(a_low), std::move(a_diag), std::move(a_up)};

        m_ddys = A.luDcmp(b);
    }

    template <
        std::ranges::input_range R0 = std::initializer_list<param_type>,
        std::ranges::input_range R1 = std::initializer_list<value_type>>
    [[using gnu: ]] explicit PiecewiseCubicFunction(
        [[maybe_unused]] periodic_tag tag, R0&& ts, R1&& ys, const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                     std::same_as<std::ranges::range_value_t<R1>, value_type>
        : m_ts(ts.cbegin(), ts.cend(), alloc), m_ys(ys.cbegin(), ys.cend(), alloc), m_ddys(alloc),
          m_periodic{true} {
#if BASJOO_CHECK_PARAMS == 1
        if (m_ts.size() < 2 || m_ys.size() < 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! sizes of ts, ys must be greater than 2: ts.size() "
                    "= {0:d} while ys.size() = {1:d}",
                    m_ts.size(), m_ys.size()
                )
            );
        }
        if (m_ts.size() != m_ys.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! ts, ys must share the same size: ts.size() = "
                    "{0:d} while ys.size() = {1:d}",
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
        if (std::abs(m_ys.front() - m_ys.back()) > kDuplicateCriterion) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! When choosing periodic boundary condition, it "
                    "requires ys.front() == m_ys.back() while m_ys.front() = {0}, m_ys.back() = "
                    "{1} here.",
                    m_ys.front(), m_ys.back()
                )
            );
        }
#endif
        const std::size_t size{m_ts.size() - 1};
        std::vector<param_type, param_allocator_type> hs(size, get_allocator());
        std::vector<value_type, allocator_type> ds(size, get_allocator());
        std::vector<param_type, param_allocator_type> a_diag(size, get_allocator());
        std::vector<value_type, allocator_type> b(size, get_allocator());

        hs[0] = m_ts[1] - m_ts[0];
        ds[0] = (m_ys[1] - m_ys[0]) / hs[0];
        for (std::size_t i{1}; i < size; i++) {
            hs[i] = m_ts[i + 1] - m_ts[i];
            ds[i] = (m_ys[i + 1] - m_ys[i]) / hs[i];
            a_diag[i] = (hs[i] + hs[i - 1]) * 2.0;
            b[i] = (ds[i] - ds[i - 1]) * 6.0;
        }
        a_diag[0] = (hs[0] + hs[size - 1]) * 2.0;
        b[0] = (ds[0] - ds[size - 1]) * 6.0;

        std::vector<param_type, param_allocator_type> a_low(
            hs.cbegin(), hs.cend() - 1, get_allocator()
        );
        std::vector<param_type, param_allocator_type> a_up{
            hs.cbegin(), hs.cend() - 1, get_allocator()
        };
        const param_type a_bottom{hs[size - 1]};
        const param_type a_top{hs[size - 1]};

        const PeriodicTridiagonalMatrix A{
            a_bottom, std::move(a_low), std::move(a_diag), std::move(a_up), a_top
        };

        m_ddys = A.luDcmp(b);
        m_ddys.push_back(m_ddys[0]);
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto eval(param_type t) const noexcept -> value_type {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (m_periodic && (t < m_ts.front() || t > m_ts.back())) [[unlikely]] {
            t = wrapPeriodic(t, m_ts.front(), m_ts.back());
        }
        const size_type pos = nearestUpperElement(m_ts, t) - m_ts.cbegin();
        if (pos == 0) {
            const param_type h{m_ts[1] - m_ts[0]};
            const param_type ratio{(t - m_ts[0]) / h};
            return lerp(m_ys[0], m_ys[1], ratio) +
                   (m_ddys[0] * kFactors[0] + m_ddys[1] * kFactors[1]) * (t - m_ts[0]) * h;
        }
        if (pos == m_ts.size()) {
            const param_type h{m_ts[pos - 2] - m_ts[pos - 1]};
            const param_type ratio{(t - m_ts[pos - 1]) / h};
            return lerp(m_ys[pos - 1], m_ys[pos - 2], ratio) +
                   (m_ddys[pos - 1] * kFactors[0] + m_ddys[pos - 2] * kFactors[1]) *
                       (t - m_ts[pos - 1]) * h;
        }
        const param_type h{m_ts[pos] - m_ts[pos - 1]};
        const param_type ratio{(t - m_ts[pos - 1]) / (m_ts[pos] - m_ts[pos - 1])};
        return cuberp(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], ratio, h);
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto derivative(param_type t) const noexcept -> value_type {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (m_periodic && (t < m_ts.front() || t > m_ts.back())) [[unlikely]] {
            t = wrapPeriodic(t, m_ts.front(), m_ts.back());
        }
        const size_type pos = nearestUpperElement(m_ts, t) - m_ts.cbegin();
        if (pos == 0) {
            const param_type h{m_ts[1] - m_ts[0]};
            return (m_ys[1] - m_ys[0]) / h +
                   (m_ddys[0] * kFactors[0] + m_ddys[1] * kFactors[1]) * h;
        }
        if (pos == m_ts.size()) {
            const param_type h{m_ts[pos - 2] - m_ts[pos - 1]};
            return (m_ys[pos - 2] - m_ys[pos - 1]) / h +
                   (m_ddys[pos - 1] * kFactors[0] + m_ddys[pos - 2] * kFactors[1]) * h;
        }
        const param_type h{m_ts[pos] - m_ts[pos - 1]};
        const param_type ratio{(t - m_ts[pos - 1]) / h};
        return cuberpd(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], ratio, h);
    }

    [[using gnu: pure, always_inline]]
    auto derivative(param_type t, unsigned int order) const noexcept(!BASJOO_CHECK_PARAMS)
        -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (order < 1 || order > 3) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid argument error! The PiecewiseLinearFunction only has 1, 2, 3 order "
                    "derivatives: order = {0:d}.",
                    order
                )
            );
        }
#endif
        if (m_periodic && (t < m_ts.front() || t > m_ts.back())) [[unlikely]] {
            t = wrapPeriodic(t, m_ts.front(), m_ts.back());
        }
        value_type result;
        switch (order) {
        case 1:
            result = derivative(t);
            break;
        case 2:
            result = derivative2(t);
            break;
        case 3:
            result = derivative3(t);
            break;
        [[unlikely]] default:
            std::unreachable();
        }
        return result;
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

    [[using gnu: pure]]
    auto minY() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        size_type pos = std::ranges::min_element(m_ys) - m_ys.cbegin();
        param_type h, ratio;
        value_type derivative, derivative2;
        if (pos == 0) {
            h = m_ts[1] - m_ts[0];
            derivative = cuberpd(m_ys[0], m_ys[1], m_ddys[0], m_ddys[1], 0.0, h);
            if (derivative < 0.0) {
                pos += 1;
            } else if (m_periodic) {
                pos = m_ts.size() - 1;
            } else {
                return m_ys[0];
            }
        } else if (pos == m_ts.size() - 1) {
            h = m_ts[pos] - m_ts[pos - 1];
            derivative = cuberpd(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], 1.0, h);
            if (derivative < 0.0) {
                if (m_periodic) {
                    pos = 1;
                } else {
                    return m_ys[pos];
                }
            }
        } else {
            h = m_ts[pos + 1] - m_ts[pos];
            derivative = cuberpd(m_ys[pos], m_ys[pos + 1], m_ddys[pos], m_ddys[pos + 1], 0.0, h);
            if (derivative < 0.0) {
                pos += 1;
            }
        }
        h = m_ts[pos] - m_ts[pos - 1];
        ratio = 0.5;
        for (size_type num_iter{3}; num_iter != 0U; --num_iter) {
            derivative = cuberpd(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], ratio, h);
            derivative2 = lerp(m_ddys[pos - 1], m_ddys[pos], ratio);
            ratio -= derivative / (derivative2 * h);
        }
        return cuberp(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], ratio, h);
    }

    [[using gnu: pure]]
    auto maxY() const noexcept -> value_type
        requires std::floating_point<value_type>
    {
        size_type pos = std::ranges::max_element(m_ys) - m_ys.cbegin();
        param_type h, ratio;
        value_type derivative, derivative2;
        if (pos == 0) {
            h = m_ts[1] - m_ts[0];
            derivative = cuberpd(m_ys[0], m_ys[1], m_ddys[0], m_ddys[1], 0.0, h);
            if (derivative > 0.0) {
                pos += 1;
            } else if (m_periodic) {
                pos = m_ts.size() - 1;
            } else {
                return m_ys[0];
            }
        } else if (pos == m_ts.size() - 1) {
            h = m_ts[pos] - m_ts[pos - 1];
            derivative = cuberpd(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], 1.0, h);
            if (derivative > 0.0) {
                if (m_periodic) {
                    pos = 1;
                } else {
                    return m_ys[pos];
                }
            }
        } else {
            h = m_ts[pos + 1] - m_ts[pos];
            derivative = cuberpd(m_ys[pos], m_ys[pos + 1], m_ddys[pos], m_ddys[pos + 1], 0.0, h);
            if (derivative > 0.0) {
                pos += 1;
            }
        }
        h = m_ts[pos] - m_ts[pos - 1];
        ratio = 0.5;
        for (size_type num_iter{3}; num_iter != 0U; --num_iter) {
            derivative = cuberpd(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], ratio, h);
            derivative2 = lerp(m_ddys[pos - 1], m_ddys[pos], ratio);
            ratio -= derivative / (derivative2 * h);
        }
        return cuberp(m_ys[pos - 1], m_ys[pos], m_ddys[pos - 1], m_ddys[pos], ratio, h);
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

    [[using gnu: pure, always_inline]]
    auto ddys() const noexcept -> std::span<const value_type> {
        return m_ddys;
    }

  private:
    using param_allocator_type =
        typename std::allocator_traits<allocator_type>::template rebind_alloc<param_type>;

    struct TridiagonalMatrix final {
        [[using gnu: pure, flatten, leaf, hot]] [[nodiscard]]
        auto luDcmp(const std::vector<value_type, allocator_type>& b) const noexcept
            -> std::vector<value_type, allocator_type> {
            const size_type mat_size{a_diag.size()};
            std::vector<value_type, allocator_type> x(mat_size, b.get_allocator());
            std::vector<param_type, param_allocator_type> u0(mat_size, b.get_allocator());
            std::vector<param_type, param_allocator_type> l1(mat_size - 1, b.get_allocator());
            const std::vector<param_type, param_allocator_type>& u1{a_up};

            u0[0] = a_diag[0];
            l1[0] = a_low[0] / u0[0];
            for (size_type i{1}; i < mat_size - 1; ++i) {
                u0[i] = a_diag[i] - l1[i - 1] * u1[i - 1];
                l1[i] = a_low[i] / u0[i];
            }
            u0[mat_size - 1] = a_diag[mat_size - 1] - l1[mat_size - 2] * u1[mat_size - 2];

            x[0] = b[0];
            for (size_type i{1}; i < mat_size; ++i) {
                x[i] = b[i] - l1[i - 1] * x[i - 1];
            }

            x[mat_size - 1] = x[mat_size - 1] / u0[mat_size - 1];
            for (int i = mat_size - 2; i > -1; --i) {
                x[i] = (x[i] - u1[i] * x[i + 1]) / u0[i];
            }
            return x;
        }

        std::vector<param_type, param_allocator_type> a_low;
        std::vector<param_type, param_allocator_type> a_diag;
        std::vector<param_type, param_allocator_type> a_up;
    };

    struct PeriodicTridiagonalMatrix final {
        [[using gnu: pure, flatten, leaf, hot]] [[nodiscard]]
        auto luDcmp(const std::vector<value_type, allocator_type>& b) const noexcept
            -> std::vector<value_type, allocator_type> {
            const size_type mat_size{a_diag.size()};
            std::vector<value_type, allocator_type> x(mat_size, b.get_allocator());
            std::vector<param_type, param_allocator_type> u0(mat_size, b.get_allocator());
            std::vector<param_type, param_allocator_type> l1(mat_size - 1, b.get_allocator());
            const std::vector<param_type, param_allocator_type>& u1{a_up};
            std::vector<param_type, param_allocator_type> l_bottom(mat_size - 2, b.get_allocator());
            std::vector<param_type, param_allocator_type> u_top(mat_size - 2, b.get_allocator());

            u0[0] = a_diag[0];
            l1[0] = a_low[0] / u0[0];
            u_top[0] = a_top;
            l_bottom[0] = a_bottom / u0[0];
            for (size_type i{1}; i < mat_size - 2; ++i) {
                u0[i] = a_diag[i] - l1[i - 1] * u1[i - 1];
                l1[i] = a_low[i] / u0[i];
                u_top[i] = -l1[i - 1] * u_top[i - 1];
                l_bottom[i] = -l_bottom[i - 1] * u1[i - 1] / u0[i];
            }
            u0[mat_size - 2] = a_diag[mat_size - 2] - l1[mat_size - 3] * u1[mat_size - 3];
            l1[mat_size - 2] = a_low[mat_size - 2] / u0[mat_size - 2];
            u0[mat_size - 1] = a_diag[mat_size - 1] - l1[mat_size - 2] * u1[mat_size - 2];
            for (size_type i{0}; i < mat_size - 2; ++i) {
                u0[mat_size - 1] -= l_bottom[i] * u_top[i];
            }

            x[0] = b[0];
            for (size_type i{1}; i < mat_size; ++i) {
                x[i] = b[i] - l1[i - 1] * x[i - 1];
            }
            for (size_type i{0}; i < mat_size - 2; ++i) {
                x[mat_size - 1] -= l_bottom[i] * x[i];
            }

            x[mat_size - 1] = x[mat_size - 1] / u0[mat_size - 1];
            x[mat_size - 2] =
                (x[mat_size - 2] - u1[mat_size - 2] * x[mat_size - 1]) / u0[mat_size - 2];
            for (int i = mat_size - 3; i > -1; --i) {
                x[i] = (x[i] - u1[i] * x[i + 1] - u_top[i] * x[mat_size - 1]) / u0[i];
            }
            return x;
        }

        param_type a_bottom;
        std::vector<param_type, param_allocator_type> a_low;
        std::vector<param_type, param_allocator_type> a_diag;
        std::vector<param_type, param_allocator_type> a_up;
        param_type a_top;
    };

    [[using gnu: pure, flatten, leaf, hot]]
    auto derivative2(param_type t) const noexcept -> value_type {
        const size_type pos = nearestUpperElement(m_ts, t) - m_ts.cbegin();
        if (pos == 0) {
            return static_cast<value_type>(0.0);
        }
        if (pos == m_ts.size()) {
            return static_cast<value_type>(0.0);
        }
        const param_type ratio = (t - m_ts[pos - 1]) / (m_ts[pos] - m_ts[pos - 1]);
        return lerp(m_ddys[pos - 1], m_ddys[pos], ratio);
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto derivative3(param_type t) const noexcept -> value_type {
        const size_type pos = nearestUpperElement(m_ts, t) - m_ts.cbegin();
        if (pos == 0) {
            return static_cast<value_type>(0.0);
        }
        if (pos == m_ts.size()) {
            return static_cast<value_type>(0.0);
        }
        return (m_ddys[pos] - m_ddys[pos - 1]) / (m_ts[pos] - m_ts[pos - 1]);
    }

    [[using gnu: pure, always_inline]]
    auto periodicAccum(param_type t) const noexcept -> value_type {
        const param_type period{maxT() - minT()};
        const param_type cycles{std::floor((t - minT()) / period)};
        return integralImpl(minT(), maxT()) * cycles +
               integralImpl(minT(), wrapPeriodic(t, minT(), maxT()));
    }

    [[using gnu: pure]]
    auto integralImpl(param_type lower_bound, param_type upper_bound) const noexcept -> value_type {
        constexpr param_type kFactor{-(1.0 / 24.0)};
        int sign = 1;
        if (lower_bound > upper_bound) {
            std::swap(lower_bound, upper_bound);
            sign = -1;
        }
        const size_type size{m_ts.size()};
        const size_type istart = std::ranges::lower_bound(m_ts, lower_bound) - m_ts.cbegin();
        const size_type iend = std::ranges::lower_bound(m_ts, upper_bound) - m_ts.cbegin();
        param_type h;
        if (istart == size || iend == 0 || istart == iend) {
            h = upper_bound - lower_bound;
            return (eval(lower_bound) + eval(upper_bound)) * h * 0.5 * sign;
        }
        value_type result{0.0};
        h = m_ts[istart] - lower_bound;
        result += (eval(lower_bound) + m_ys[istart]) * h * 0.5 +
                  (derivative2(lower_bound) + m_ddys[istart]) * h * h * h * kFactor;
        for (size_type i{istart}; i < iend - 1; ++i) {
            h = m_ts[i + 1] - m_ts[i];
            result += (m_ys[i] + m_ys[i + 1]) * h * 0.5 +
                      (m_ddys[i] + m_ddys[i + 1]) * h * h * h * kFactor;
        }
        h = upper_bound - m_ts[iend - 1];
        result += (m_ys[iend - 1] + eval(upper_bound)) * h * 0.5 +
                  (m_ddys[iend - 1] + derivative2(upper_bound)) * h * h * h * kFactor;
        result *= sign;
        return result;
    }

    friend zpp::bits::access;
    using serialize = zpp::bits::members<4>;

    std::vector<param_type, param_allocator_type> m_ts;
    std::vector<value_type, allocator_type> m_ys;
    std::vector<value_type, allocator_type> m_ddys;
    bool m_periodic{false};
};

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<float, 32>>
using PiecewiseCubicFunction1s = PiecewiseCubicFunction<float, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<double, 32>>
using PiecewiseCubicFunction1d = PiecewiseCubicFunction<double, Alloc>;

namespace pmr {

template <GeneralArithmetic T>
using PiecewiseCubicFunction =
    ::basjoo::math::PiecewiseCubicFunction<T, std::pmr::polymorphic_allocator<T>>;

using PiecewiseCubicFunction1s =
    ::basjoo::math::PiecewiseCubicFunction<float, std::pmr::polymorphic_allocator<float>>;

using PiecewiseCubicFunction1d =
    ::basjoo::math::PiecewiseCubicFunction<double, std::pmr::polymorphic_allocator<double>>;

} // namespace pmr

} // namespace basjoo::math
