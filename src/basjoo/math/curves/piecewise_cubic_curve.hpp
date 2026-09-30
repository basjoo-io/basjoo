/**
 * @file piecewise_cubic_curve.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-01
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
#include <ranges>
#include <span>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/cubic_interpolation.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"
#include "basjoo/math/functions/piecewise_cubic_function.hpp"
#include "basjoo/math/utils.hpp"

namespace basjoo::math {

template <VecArithmetic T, Allocatory Alloc = ::basjoo::common::AlignedAllocator<T, alignof(T)>>
    requires std::floating_point<typename T::value_type>
class PiecewiseCubicCurve final {
  public:
    using value_type = T;
    using param_type = typename T::value_type;
    using size_type = std::size_t;
    using allocator_type = Alloc;

    static constexpr param_type kDuplicateCriterion{1E-8};

    PiecewiseCubicCurve() noexcept = default;
    PiecewiseCubicCurve(const PiecewiseCubicCurve& other) = default;
    auto operator=(const PiecewiseCubicCurve& other) -> PiecewiseCubicCurve& = default;
    PiecewiseCubicCurve(PiecewiseCubicCurve&& other) noexcept = default;
    auto operator=(PiecewiseCubicCurve&& other) noexcept -> PiecewiseCubicCurve& = default;
    ~PiecewiseCubicCurve() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_vec_of_s.get_allocator();
    }

    [[using gnu: pure, always_inline]]
    auto periodic() const noexcept -> bool {
        return m_vec_of_s.periodic();
    }

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    explicit PiecewiseCubicCurve(
        R&& anchor_points, param_type s0 = 0.0, const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R>, value_type>
        : PiecewiseCubicCurve(
              anchor_points, BoundaryMode<value_type>{2, value_type(0.0)},
              BoundaryMode<value_type>{2, value_type(0.0)}, s0, alloc
          ) {}

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: ]]
    explicit PiecewiseCubicCurve(
        R&& anchor_points, BoundaryMode<value_type> b0, BoundaryMode<value_type> bf,
        param_type s0 = 0.0, const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R>, value_type>
    {
#if BASJOO_CHECK_PARAMS == 1
        if (anchor_points.size() < 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! sizes of anchor_points must be greater than 2: "
                    "anchor_points.size() = {0:d}",
                    anchor_points.size()
                )
            );
        }
#endif
        const size_type size{anchor_points.size()};
        std::vector<param_type, param_allocator_type> temp_arc_lengths(size, alloc);
        temp_arc_lengths[0] = s0;
        for (size_type i{1}; i < size; ++i) {
            temp_arc_lengths[i] =
                temp_arc_lengths[i - 1] + anchor_points[i].euclideanTo(anchor_points[i - 1]);
        }
        const PiecewiseCubicFunction<value_type, allocator_type> temp_vec_of_s(
            temp_arc_lengths, anchor_points, b0, bf, alloc
        );

        auto ddys{temp_vec_of_s.ddys()};
        std::vector<param_type, param_allocator_type> arc_lengths(size, alloc);
        arc_lengths[0] = s0;
        for (size_type i{1}; i < size; ++i) {
            arc_lengths[i] =
                arc_lengths[i - 1] + ::basjoo::math::calcArcLength(
                                         anchor_points[i - 1], anchor_points[i], ddys[i - 1],
                                         ddys[i], temp_arc_lengths[i] - temp_arc_lengths[i - 1]
                                     );
        }
        m_vec_of_s = PiecewiseCubicFunction<value_type, allocator_type>(
            arc_lengths, anchor_points, b0, bf, alloc
        );
    }

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: ]] explicit PiecewiseCubicCurve(
        [[maybe_unused]] periodic_tag tag, R&& anchor_points, param_type s0 = 0.0,
        const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R>, value_type>
    {
#if BASJOO_CHECK_PARAMS == 1
        if (anchor_points.size() < 2) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! sizes of anchor_points must be greater than 2: "
                    "anchor_points.size() = {0:d}",
                    anchor_points.size()
                )
            );
        }
#endif
        const size_type size{anchor_points.size()};
        std::vector<param_type, param_allocator_type> temp_arc_lengths(size, alloc);
        temp_arc_lengths[0] = s0;
        for (size_type i{1}; i < size; ++i) {
            temp_arc_lengths[i] =
                temp_arc_lengths[i - 1] + anchor_points[i].euclideanTo(anchor_points[i - 1]);
        }
        const PiecewiseCubicFunction<value_type, allocator_type> temp_vec_of_s{
            tag, temp_arc_lengths, anchor_points, alloc
        };

        auto ddys{temp_vec_of_s.ddys()};
        std::vector<param_type, param_allocator_type> arc_lengths(size, alloc);
        arc_lengths[0] = s0;
        for (size_type i{1}; i < size; ++i) {
            arc_lengths[i] =
                arc_lengths[i - 1] + ::basjoo::math::calcArcLength(
                                         anchor_points[i - 1], anchor_points[i], ddys[i - 1],
                                         ddys[i], temp_arc_lengths[i] - temp_arc_lengths[i - 1]
                                     );
        }
        m_vec_of_s = PiecewiseCubicFunction<value_type, allocator_type>(
            tag, arc_lengths, anchor_points, alloc
        );
    }

    [[using gnu: pure, always_inline]]
    auto eval(param_type s) const noexcept -> value_type {
        return m_vec_of_s.eval(s);
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto eval(param_type s, param_type l) const noexcept -> value_type
        requires Vec2Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const param_type ratio{(s - arc_lengths[0]) / h};
            const value_type val{
                lerp(anchor_points[0], anchor_points[1], ratio) +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * (s - arc_lengths[0]) * h
            };
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            const value_type normal{
                derivative.rotateHalfPi().normalized() *
                (derivative.crossProj(derivative2) > 0 ? 1 : -1)
            };
            return val + normal * l;
        }
        if (pos == size) {
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const param_type ratio{
                (s - arc_lengths[size - 1]) / (arc_lengths[size - 2] - arc_lengths[size - 1])
            };
            const value_type val{
                lerp(anchor_points[size - 1], anchor_points[size - 2], ratio) +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) *
                    (s - arc_lengths[pos - 1]) * h
            };
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[size - 1]};
            const value_type normal{
                derivative.rotateHalfPi().normalized() *
                (derivative.crossProj(derivative2) > 0 ? 1 : -1)
            };
            return val + normal * l;
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type val{
            cuberp(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        const value_type normal{
            derivative.rotateHalfPi().normalized() *
            (derivative.crossProj(derivative2) > 0 ? 1 : -1)
        };
        return val + normal * l;
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto eval(param_type s, param_type l, param_type v) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const param_type ratio{(s - arc_lengths[0]) / h};
            const value_type val{
                lerp(anchor_points[0], anchor_points[1], ratio) +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * (s - arc_lengths[0]) * h
            };
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
            const value_type binormal{derivative.cross(derivative2).normalized()};
            return val + normal * l + binormal * v;
        }
        if (pos == size) {
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const param_type ratio{
                (s - arc_lengths[size - 1]) / (arc_lengths[size - 2] - arc_lengths[size - 1])
            };
            const value_type val{
                lerp(anchor_points[size - 1], anchor_points[size - 2], ratio) +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) *
                    (s - arc_lengths[size - 1]) * h
            };
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[size - 1]};
            const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
            const value_type binormal{derivative.cross(derivative2).normalized()};
            return val + normal * l + binormal * v;
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type val{
            cuberp(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
        const value_type binormal{derivative.cross(derivative2).normalized()};
        return val + normal * l + binormal * v;
    }

    [[using gnu: pure, always_inline]]
    auto tangent(param_type s) const noexcept -> value_type {
        return m_vec_of_s.derivative(s).normalized();
    }

    [[using gnu: pure, always_inline]]
    auto normal(param_type s) const noexcept -> value_type
        requires Vec2Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            return derivative.rotateHalfPi().normalized() *
                   (derivative.crossProj(derivative2) > 0 ? 1 : -1);
        }
        if (pos == size) {
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2 = ddys[size - 1];
            return derivative.rotateHalfPi().normalized() *
                   (derivative.crossProj(derivative2) > 0 ? 1 : -1);
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        return derivative.rotateHalfPi().normalized() *
               (derivative.crossProj(derivative2) > 0 ? 1 : -1);
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto normal(param_type s) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            return derivative.cross(derivative2.cross(derivative)).normalized();
        }
        if (pos == size) {
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[size - 1]};
            return derivative.cross(derivative2.cross(derivative)).normalized();
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        return derivative.cross(derivative2.cross(derivative)).normalized();
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto binormal(param_type s) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            return derivative.cross(derivative2).normalized();
        }
        if (pos == size) {
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[size - 1]};
            return derivative.cross(derivative2).normalized();
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        return derivative.cross(derivative2).normalized();
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto curvature(param_type s) const noexcept -> param_type {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            return 0.0;
        }
        if (pos == size) {
            return 0.0;
        }
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        const param_type derivative_norm{derivative.euclidean()};
        return std::abs(
            derivative.crossProj(derivative2) /
            (derivative_norm * derivative_norm * derivative_norm)
        );
    }

    [[using gnu: pure, always_inline]]
    auto torsion(param_type s) const noexcept -> param_type
        requires Vec3Arithmetic<value_type>
    {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0) {
            return 0.0;
        }
        if (pos == size) {
            return 0.0;
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        const param_type ratio{(s - arc_lengths[pos - 1]) / h};
        const value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        const value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        const value_type derivative3{(ddys[pos] - ddys[pos - 1]) / h};
        return derivative.dot(derivative2.cross(derivative3)) /
               (derivative.cross(derivative2).euclideanSqr());
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point) const noexcept -> std::pair<param_type, param_type>
        requires Vec2Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(anchor_points, point) - anchor_points.begin();
        if (pos == 0) {
            const value_type r{point - anchor_points[0]};
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{
                (derivative.rotateHalfPi()).normalized() *
                (derivative.crossProj(derivative2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[0] + r.dot(tangent), r.dot(normal));
        }
        if (pos == size) {
            const value_type r{point - anchor_points[size - 1]};
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[size - 1]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{
                (derivative.rotateHalfPi()).normalized() *
                (derivative.crossProj(derivative2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[size - 1] + r.dot(tangent), r.dot(normal));
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        param_type ratio{0.5};
        value_type val{
            cuberp(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        value_type r{point - val};
        for (size_type num_iter{3}; num_iter != 0U; --num_iter) {
            ratio -= r.dot(derivative) / ((r.dot(derivative2) - derivative.euclideanSqr()) * h);
            val = cuberp(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative = cuberpd(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative2 = lerp(ddys[pos - 1], ddys[pos], ratio);
            r = point - val;
        }
        const value_type normal{
            derivative.rotateHalfPi().normalized() *
            (derivative.crossProj(derivative2) > 0 ? 1 : -1)
        };
        return std::make_pair(lerp(arc_lengths[pos - 1], arc_lengths[pos], ratio), r.dot(normal));
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point, param_type start_s, param_type end_s) const noexcept
        -> std::pair<param_type, param_type>
        requires Vec2Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (start_s > end_s) {
            std::swap(start_s, end_s);
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type istart =
            nearestUpperElement(arc_lengths, start_s) - arc_lengths.begin() - 1;
        const size_type iend = nearestUpperElement(arc_lengths, end_s) - arc_lengths.begin();
        const size_type pos =
            nearestUpperElement(
                anchor_points.begin() + istart, anchor_points.begin() + iend, point
            ) -
            anchor_points.begin();
        if (pos == istart) {
            const value_type r{point - anchor_points[istart]};
            const param_type h{arc_lengths[istart + 1] - arc_lengths[istart]};
            const value_type derivative{
                (anchor_points[istart + 1] - anchor_points[istart]) / h +
                (ddys[istart] * kFactors[0] + ddys[istart + 1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[istart]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{
                derivative.rotateHalfPi().normalized() *
                (derivative.crossProj(derivative2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[istart] + r.dot(tangent), r.dot(normal));
        }
        if (pos == iend) {
            const value_type r{point - anchor_points[iend - 1]};
            const param_type h{arc_lengths[iend - 2] - arc_lengths[iend - 1]};
            const value_type derivative{
                (anchor_points[iend - 2] - anchor_points[iend - 1]) / h +
                (ddys[iend - 1] * kFactors[0] + ddys[iend - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[iend - 1]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{
                derivative.rotateHalfPi().normalized() *
                (derivative.crossProj(derivative2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[iend - 1] + r.dot(tangent), r.dot(normal));
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        param_type ratio{0.5};
        value_type val{
            cuberp(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        value_type r{point - val};
        for (size_type num_iter{3}; num_iter != 0U; --num_iter) {
            ratio -= r.dot(derivative) / ((r.dot(derivative2) - derivative.euclideanSqr()) * h);
            val = cuberp(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative = cuberpd(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative2 = lerp(ddys[pos - 1], ddys[pos], ratio);
            r = point - val;
        }
        const value_type normal{
            derivative.rotateHalfPi().normalized() *
            (derivative.crossProj(derivative2) > 0 ? 1 : -1)
        };
        return std::make_pair(lerp(arc_lengths[pos - 1], arc_lengths[pos], ratio), r.dot(normal));
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point) const noexcept -> std::tuple<param_type, param_type, param_type>
        requires Vec3Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(anchor_points, point) - anchor_points.begin();
        if (pos == 0) {
            const value_type r{point - anchor_points[0]};
            const param_type h{arc_lengths[1] - arc_lengths[0]};
            const value_type derivative{
                (anchor_points[1] - anchor_points[0]) / h +
                (ddys[0] * kFactors[0] + ddys[1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[0]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
            const value_type binormal{derivative.cross(derivative2).normalized()};
            return std::make_tuple(arc_lengths[0] + r.dot(tangent), r.dot(normal), r.dot(binormal));
        }
        if (pos == size) {
            const value_type r{point - anchor_points[size - 1]};
            const param_type h{arc_lengths[size - 2] - arc_lengths[size - 1]};
            const value_type derivative{
                (anchor_points[size - 2] - anchor_points[size - 1]) / h +
                (ddys[size - 1] * kFactors[0] + ddys[size - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[size - 1]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
            const value_type binormal{derivative.cross(derivative2).normalized()};
            return std::make_tuple(arc_lengths[0] + r.dot(tangent), r.dot(normal), r.dot(binormal));
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        param_type ratio{0.5};
        value_type val{
            cuberp(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        value_type r{point - val};
        for (size_type num_iter{3}; num_iter != 0U; --num_iter) {
            ratio -= r.dot(derivative) / ((r.dot(derivative2) - derivative.euclideanSqr()) * h);
            val = cuberp(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative = cuberpd(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative2 = lerp(ddys[pos - 1], ddys[pos], ratio);
            r = point - val;
        }
        const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
        const value_type binormal{derivative.cross(derivative2).normalized()};
        return std::make_tuple(
            lerp(arc_lengths[pos - 1], arc_lengths[pos], ratio), r.dot(normal), r.dot(binormal)
        );
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point, param_type start_s, param_type end_s) const noexcept
        -> std::tuple<param_type, param_type, param_type>
        requires Vec3Arithmetic<value_type>
    {
        constexpr std::array<param_type, 2> kFactors{-(1.0 / 3.0), -(1.0 / 6.0)};
        if (start_s > end_s) {
            std::swap(start_s, end_s);
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        auto ddys{m_vec_of_s.ddys()};
        const size_type istart =
            nearestUpperElement(arc_lengths, start_s) - arc_lengths.begin() - 1;
        const size_type iend = nearestUpperElement(arc_lengths, end_s) - arc_lengths.begin();
        const size_type pos =
            nearestUpperElement(
                anchor_points.begin() + istart, anchor_points.begin() + iend, point
            ) -
            anchor_points.begin();
        if (pos == istart) {
            const value_type r{point - anchor_points[istart]};
            const param_type h{arc_lengths[istart + 1] - arc_lengths[istart]};
            const value_type derivative{
                (anchor_points[istart + 1] - anchor_points[istart]) / h +
                (ddys[istart] * kFactors[0] + ddys[istart + 1] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[istart]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
            const value_type binormal{derivative.cross(derivative2).normalized()};
            return std::make_tuple(
                arc_lengths[istart] + r.dot(tangent), r.dot(normal), r.dot(binormal)
            );
        }
        if (pos == iend) {
            const value_type r{point - anchor_points[iend - 1]};
            const param_type h{arc_lengths[iend - 2] - arc_lengths[iend - 1]};
            const value_type derivative{
                (anchor_points[iend - 2] - anchor_points[iend - 1]) / h +
                (ddys[iend - 1] * kFactors[0] + ddys[iend - 2] * kFactors[1]) * h
            };
            const value_type derivative2{ddys[iend - 1]};
            const value_type tangent{derivative.normalized()};
            const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
            const value_type binormal{derivative.cross(derivative2).normalized()};
            return std::make_tuple(
                arc_lengths[iend - 1] + r.dot(tangent), r.dot(normal), r.dot(binormal)
            );
        }
        const param_type h{arc_lengths[pos] - arc_lengths[pos - 1]};
        param_type ratio{0.5};
        value_type val{
            cuberp(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative{
            cuberpd(anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h)
        };
        value_type derivative2{lerp(ddys[pos - 1], ddys[pos], ratio)};
        value_type r{point - val};
        for (size_type num_iter{3}; num_iter != 0U; --num_iter) {
            ratio -= r.dot(derivative) / ((r.dot(derivative2) - derivative.euclideanSqr()) * h);
            val = cuberp(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative = cuberpd(
                anchor_points[pos - 1], anchor_points[pos], ddys[pos - 1], ddys[pos], ratio, h
            );
            derivative2 = lerp(ddys[pos - 1], ddys[pos], ratio);
            r = point - val;
        }
        const value_type normal{derivative.cross(derivative2.cross(derivative)).normalized()};
        const value_type binormal{derivative.cross(derivative2).normalized()};
        return std::make_tuple(
            lerp(arc_lengths[pos - 1], arc_lengths[pos], ratio), r.dot(normal), r.dot(binormal)
        );
    }

    [[using gnu: pure, always_inline]]
    auto operator()(param_type s) const noexcept -> value_type {
        return eval(s);
    }

    [[using gnu: pure, always_inline]]
    auto operator()(param_type s, param_type l) const noexcept -> value_type
        requires Vec2Arithmetic<value_type>
    {
        return eval(s, l);
    }

    [[using gnu: pure, always_inline]]
    auto operator()(param_type s, param_type l, param_type v) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        return eval(s, l, v);
    }

    [[using gnu: pure, always_inline]]
    auto minS() const noexcept -> param_type {
        return m_vec_of_s.minT();
    }

    [[using gnu: pure, always_inline]]
    auto maxS() const noexcept -> param_type {
        return m_vec_of_s.maxT();
    }

    [[using gnu: pure, always_inline]]
    auto front() const noexcept -> value_type {
        return m_vec_of_s.ys().front();
    }

    [[using gnu: pure, always_inline]]
    auto back() const noexcept -> value_type {
        return m_vec_of_s.ys().back();
    }

    [[using gnu: pure, always_inline]]
    auto arcLengths() const noexcept -> std::span<const param_type> {
        return m_vec_of_s.ts();
    }

    [[using gnu: pure, always_inline]]
    auto anchorPoints() const noexcept -> std::span<const value_type> {
        return m_vec_of_s.ys();
    }

  private:
    using param_allocator_type =
        typename std::allocator_traits<allocator_type>::template rebind_alloc<param_type>;

    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    PiecewiseCubicFunction<value_type, allocator_type> m_vec_of_s;
};

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec2s, alignof(Vec2s)>>
using PiecewiseCubicCurve2s = PiecewiseCubicCurve<Vec2s, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec2d, alignof(Vec2d)>>
using PiecewiseCubicCurve2d = PiecewiseCubicCurve<Vec2d, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec3s, alignof(Vec3s)>>
using PiecewiseCubicCurve3s = PiecewiseCubicCurve<Vec3s, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec3d, alignof(Vec3d)>>
using PiecewiseCubicCurve3d = PiecewiseCubicCurve<Vec3d, Alloc>;

namespace pmr {

template <VecArithmetic T>
using PiecewiseCubicCurve =
    basjoo::math::PiecewiseCubicCurve<T, std::pmr::polymorphic_allocator<T>>;

using PiecewiseCubicCurve2s =
    ::basjoo::math::PiecewiseCubicCurve<Vec2s, std::pmr::polymorphic_allocator<Vec2s>>;

using PiecewiseCubicCurve2d =
    ::basjoo::math::PiecewiseCubicCurve<Vec2d, std::pmr::polymorphic_allocator<Vec2d>>;

using PiecewiseCubicCurve3s =
    ::basjoo::math::PiecewiseCubicCurve<Vec3s, std::pmr::polymorphic_allocator<Vec3s>>;

using PiecewiseCubicCurve3d =
    ::basjoo::math::PiecewiseCubicCurve<Vec3d, std::pmr::polymorphic_allocator<Vec3d>>;

} // namespace pmr

} // namespace basjoo::math
