/**
 * @file piecewise_linear_curve.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-28
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
#include <limits>
#include <ranges>
#include <span>
#include <stdexcept>
#include <tuple>
#include <utility>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/common/aligned_allocator.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/functions/piecewise_linear_function.hpp"
#include "basjoo/math/utils.hpp"

namespace basjoo::math {

template <VecArithmetic T, Allocatory Alloc = ::basjoo::common::AlignedAllocator<T, alignof(T)>>
    requires std::floating_point<typename T::value_type>
class PiecewiseLinearCurve final {
  public:
    using value_type = T;
    using param_type = typename T::value_type;
    using size_type = std::size_t;
    using allocator_type = Alloc;

    static constexpr param_type kDuplicateCriterion{1E-8};

    PiecewiseLinearCurve() noexcept = default;
    PiecewiseLinearCurve(const PiecewiseLinearCurve& other) = default;
    auto operator=(const PiecewiseLinearCurve& other) -> PiecewiseLinearCurve& = default;
    PiecewiseLinearCurve(PiecewiseLinearCurve&& other) noexcept = default;
    auto operator=(PiecewiseLinearCurve&& other) noexcept -> PiecewiseLinearCurve& = default;
    ~PiecewiseLinearCurve() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_vec_of_s.get_allocator();
    }

    [[using gnu: pure, always_inline]]
    auto periodic() const noexcept -> bool {
        return m_vec_of_s.periodic();
    }

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: ]]
    explicit PiecewiseLinearCurve(
        R&& anchor_points, param_type s0 = 0.0, const allocator_type& alloc = {}
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
        std::vector<param_type, param_allocator_type> arc_lengths(size, alloc);
        arc_lengths[0] = s0;
        for (size_type i{1}; i < size; ++i) {
            arc_lengths[i] =
                arc_lengths[i - 1] + anchor_points[i].euclideanTo(anchor_points[i - 1]);
        }
        m_vec_of_s =
            PiecewiseLinearFunction<value_type, allocator_type>(arc_lengths, anchor_points, alloc);
    }

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: ]] explicit PiecewiseLinearCurve(
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
        std::vector<param_type, param_allocator_type> arc_lengths(size, alloc);
        arc_lengths[0] = s0;
        for (size_type i{1}; i < size; ++i) {
            arc_lengths[i] =
                arc_lengths[i - 1] + anchor_points[i].euclideanTo(anchor_points[i - 1]);
        }
        m_vec_of_s = PiecewiseLinearFunction<value_type, allocator_type>(
            tag, arc_lengths, anchor_points, alloc
        );
    }

    [[using gnu: pure, always_inline]]
    auto eval(param_type s) const noexcept -> value_type {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        return m_vec_of_s.eval(s);
    }

    [[using gnu: pure, always_inline]]
    auto eval(param_type s, param_type l) const noexcept -> value_type
        requires Vec2Arithmetic<value_type>
    {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos < 2) {
            const param_type ratio{(s - arc_lengths[0]) / (arc_lengths[1] - arc_lengths[0])};
            const value_type val{lerp(anchor_points[0], anchor_points[1], ratio)};
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            const value_type normal{
                diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
            };
            return val + normal * l;
        }
        if (pos == size) {
            const param_type ratio{
                (s - arc_lengths[size - 1]) / (arc_lengths[size - 2] - arc_lengths[size - 1])
            };
            const value_type val{lerp(anchor_points[size - 1], anchor_points[size - 2], ratio)};
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            const value_type normal{
                diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
            };
            return val + normal * l;
        }
        const param_type ratio{
            (s - arc_lengths[pos - 1]) / (arc_lengths[pos] - arc_lengths[pos - 1])
        };
        const value_type val{lerp(anchor_points[pos - 1], anchor_points[pos], ratio)};
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        const value_type normal{
            diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
        };
        return val + normal * l;
    }

    [[using gnu: pure, always_inline]]
    auto eval(param_type s, param_type l, param_type v) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos < 2) {
            const param_type ratio{(s - arc_lengths[0]) / (arc_lengths[1] - arc_lengths[0])};
            const value_type val{lerp(anchor_points[0], anchor_points[1], ratio)};
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
            const value_type binormal{diff.cross(diff2).normalized()};
            return val + normal * l + binormal * v;
        }
        if (pos == size) {
            const param_type ratio{
                (s - arc_lengths[size - 1]) / (arc_lengths[size - 2] - arc_lengths[size - 1])
            };
            const value_type val{lerp(anchor_points[size - 1], anchor_points[size - 2], ratio)};
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
            const value_type binormal{diff.cross(diff2).normalized()};
            return val + normal * l + binormal * v;
        }
        const param_type ratio{
            (s - arc_lengths[pos - 1]) / (arc_lengths[pos] - arc_lengths[pos - 1])
        };
        const value_type val{lerp(anchor_points[pos - 1], anchor_points[pos], ratio)};
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
        const value_type binormal{diff.cross(diff2).normalized()};
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
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos < 2) {
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            return diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1);
        }
        if (pos == size) {
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            return diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1);
        }
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        return diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1);
    }

    [[using gnu: pure, always_inline]]
    auto normal(param_type s) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos < 2) {
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            return diff.cross(diff2.cross(diff)).normalized();
        }
        if (pos == size) {
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            return diff.cross(diff2.cross(diff)).normalized();
        }
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        return diff.cross(diff2.cross(diff)).normalized();
    }

    [[using gnu: pure, always_inline]]
    auto binormal(param_type s) const noexcept -> value_type
        requires Vec3Arithmetic<value_type>
    {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{arc_lengths.size()};
        const size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos < 2) {
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            return diff.cross(diff2).normalized();
        }
        if (pos == size) {
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            return diff.cross(diff2).normalized();
        }
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        return diff.cross(diff2).normalized();
    }

    [[using gnu: pure, always_inline]]
    auto curvature(param_type s) const noexcept -> param_type {
        if (periodic() && (s < minS() || s > maxS())) [[unlikely]] {
            s = wrapPeriodic(s, minS(), maxS());
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{arc_lengths.size()};
        size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0 || pos == size) {
            return static_cast<param_type>(0.0);
        }
        if (periodic() && pos == 1) {
            const value_type a{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type b{anchor_points[1] - anchor_points[0]};
            const value_type c{anchor_points[1] - anchor_points[size - 2]};
            return a.crossProj(b) / (a.euclidean() * b.euclidean() * c.euclidean()) *
                   static_cast<param_type>(2.0);
        }
        pos = std::clamp(pos, static_cast<size_type>(2), static_cast<size_type>(size - 1));
        const value_type a{anchor_points[pos - 1] - anchor_points[pos - 2]};
        const value_type b{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type c{anchor_points[pos] - anchor_points[pos - 2]};
        return a.crossProj(b) / (a.euclidean() * b.euclidean() * c.euclidean()) *
               static_cast<param_type>(2.0);
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
        const size_type size{arc_lengths.size()};
        if (size < 4) {
            return static_cast<param_type>(0.0);
        }
        size_type pos = nearestUpperElement(arc_lengths, s) - arc_lengths.begin();
        if (pos == 0 || pos == size) {
            return static_cast<param_type>(0.0);
        }
        value_type diff, left_diff, right_diff;
        if (periodic() && pos == 1) {
            diff = anchor_points[1] - anchor_points[0];
            left_diff = anchor_points[size - 1] - anchor_points[size - 2];
            right_diff = anchor_points[2] - anchor_points[1];
        } else if (periodic() && pos == size - 1) {
            diff = anchor_points[size - 1] - anchor_points[size - 2];
            left_diff = anchor_points[size - 2] - anchor_points[size - 3];
            right_diff = anchor_points[1] - anchor_points[0];
        } else {
            pos = std::clamp(pos, static_cast<size_type>(2), static_cast<size_type>(size - 2));
            diff = anchor_points[pos] - anchor_points[pos - 1];
            left_diff = anchor_points[pos - 1] - anchor_points[pos - 2];
            right_diff = anchor_points[pos + 1] - anchor_points[pos];
        }
        const value_type left_normal{left_diff.cross(diff)};
        const value_type right_normal{diff.cross(right_diff)};
        const param_type diff_euclidean{diff.euclidean()};
        const param_type left_normal_euclidean{left_normal.euclidean()};
        const param_type right_normal_euclidean{right_normal.euclidean()};
        return std::copysign(
            std::acos(
                left_normal.dot(right_normal) / (left_normal_euclidean * right_normal_euclidean)
            ) / diff_euclidean,
            diff.dot((left_normal.cross(right_normal)))
        );
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point) const noexcept -> std::pair<param_type, param_type>
        requires Vec2Arithmetic<value_type>
    {
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{anchorPoints().size()};
        const size_type pos = nearestUpperElement(anchor_points, point) - anchor_points.begin();
        if (pos < 2) {
            const value_type r{point - anchor_points[0]};
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{
                diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[0] + r.dot(tangent), r.dot(normal));
        };
        if (pos == size) {
            const value_type r{point - anchor_points[size - 1]};
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{
                diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[size - 1] + r.dot(tangent), r.dot(normal));
        }
        const value_type r{point - anchor_points[pos - 1]};
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        const value_type tangent{diff.normalized()};
        const value_type normal{
            diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
        };
        return std::make_pair(arc_lengths[pos - 1] + r.dot(tangent), r.dot(normal));
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point, param_type start_s, param_type end_s) const noexcept
        -> std::pair<param_type, param_type>
        requires Vec2Arithmetic<value_type>
    {
        if (start_s > end_s) {
            std::swap(start_s, end_s);
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{anchorPoints().size()};
        const size_type istart =
            nearestUpperElement(arc_lengths, start_s) - arc_lengths.begin() - 1;
        const size_type iend = nearestUpperElement(arc_lengths, end_s) - arc_lengths.begin();
        const size_type pos =
            nearestUpperElement(
                anchor_points.begin() + istart, anchor_points.begin() + iend, point
            ) -
            anchor_points.begin();
        if (pos < istart + 2 && size > istart + 2) {
            const value_type r{point - anchor_points[istart]};
            const value_type diff{anchor_points[istart + 1] - anchor_points[istart]};
            const value_type diff2{
                (anchor_points[istart + 2] - anchor_points[istart + 1]) /
                    (arc_lengths[istart + 2] - arc_lengths[istart + 1]) -
                (anchor_points[istart + 1] - anchor_points[istart]) /
                    (arc_lengths[istart + 1] - arc_lengths[istart])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{
                diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[istart] + r.dot(tangent), r.dot(normal));
        };
        if (pos == iend) {
            const value_type r{point - anchor_points[iend - 1]};
            const value_type diff{anchor_points[iend - 1] - anchor_points[iend - 2]};
            const value_type diff2{
                (anchor_points[iend - 1] - anchor_points[iend - 2]) /
                    (arc_lengths[iend - 1] - arc_lengths[iend - 2]) -
                (anchor_points[iend - 2] - anchor_points[iend - 3]) /
                    (arc_lengths[iend - 2] - arc_lengths[iend - 3])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{
                diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
            };
            return std::make_pair(arc_lengths[iend - 1] + r.dot(tangent), r.dot(normal));
        }
        const value_type r{point - anchor_points[pos - 1]};
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        const value_type tangent{diff.normalized()};
        const value_type normal{
            diff.rotateHalfPi().normalized() * (diff.crossProj(diff2) > 0 ? 1 : -1)
        };
        return std::make_pair(arc_lengths[pos - 1] + r.dot(tangent), r.dot(normal));
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse([[maybe_unused]] value_type point) const noexcept
        -> std::tuple<param_type, param_type, param_type>
        requires Vec3Arithmetic<value_type>
    {
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{anchorPoints().size()};
        const size_type pos = nearestUpperElement(anchor_points, point) - anchor_points.begin();
        if (pos < 2) {
            const value_type r{point - anchor_points[0]};
            const value_type diff{anchor_points[1] - anchor_points[0]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[2] - anchor_points[1]) / (arc_lengths[2] - arc_lengths[1]) -
                          (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
            const value_type binormal{diff.cross(diff2).normalized()};
            return std::make_tuple(arc_lengths[0] + r.dot(tangent), r.dot(normal), r.dot(binormal));
        }
        if (pos == size) {
            const value_type r{point - anchor_points[size - 1]};
            const value_type diff{anchor_points[size - 1] - anchor_points[size - 2]};
            const value_type diff2{
                periodic()
                    ? (anchor_points[1] - anchor_points[0]) / (arc_lengths[1] - arc_lengths[0]) -
                          (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2])
                    : (anchor_points[size - 1] - anchor_points[size - 2]) /
                              (arc_lengths[size - 1] - arc_lengths[size - 2]) -
                          (anchor_points[size - 2] - anchor_points[size - 3]) /
                              (arc_lengths[size - 2] - arc_lengths[size - 3])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
            const value_type binormal{diff.cross(diff2).normalized()};
            return std::make_tuple(
                arc_lengths[size - 1] + r.dot(tangent), r.dot(normal), r.dot(binormal)
            );
        }
        const value_type r{point - anchor_points[pos - 1]};
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        const value_type tangent{diff.normalized()};
        const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
        const value_type binormal{diff.cross(diff2).normalized()};
        return std::make_tuple(
            arc_lengths[pos - 1] + r.dot(tangent), r.dot(normal), r.dot(binormal)
        );
    }

    [[using gnu: pure, flatten, leaf, hot]]
    auto inverse(value_type point, param_type start_s, param_type end_s) const noexcept
        -> std::tuple<param_type, param_type, param_type>
        requires Vec3Arithmetic<value_type>
    {
        if (start_s > end_s) {
            std::swap(start_s, end_s);
        }
        auto arc_lengths{arcLengths()};
        auto anchor_points{anchorPoints()};
        const size_type size{anchorPoints().size()};
        const size_type istart =
            nearestUpperElement(arc_lengths, start_s) - arc_lengths.begin() - 1;
        const size_type iend = nearestUpperElement(arc_lengths, end_s) - arc_lengths.begin();
        const size_type pos =
            nearestUpperElement(
                anchor_points.begin() + istart, anchor_points.begin() + iend, point
            ) -
            anchor_points.begin();
        if (pos < istart + 2 && size > istart + 2) {
            const value_type r{point - anchor_points[istart]};
            const value_type diff{anchor_points[istart + 1] - anchor_points[istart]};
            const value_type diff2{
                (anchor_points[istart + 2] - anchor_points[istart + 1]) /
                    (arc_lengths[istart + 2] - arc_lengths[istart + 1]) -
                (anchor_points[istart + 1] - anchor_points[istart]) /
                    (arc_lengths[istart + 1] - arc_lengths[istart])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
            const value_type binormal{diff.cross(diff2).normalized()};
            return std::make_tuple(
                arc_lengths[istart] + r.dot(tangent), r.dot(normal), r.dot(binormal)
            );
        }
        if (pos == iend) {
            const value_type r{point - anchor_points[iend - 1]};
            const value_type diff{anchor_points[iend - 1] - anchor_points[iend - 2]};
            const value_type diff2{
                (anchor_points[iend - 1] - anchor_points[iend - 2]) /
                    (arc_lengths[iend - 1] - arc_lengths[iend - 2]) -
                (anchor_points[iend - 2] - anchor_points[iend - 3]) /
                    (arc_lengths[iend - 2] - arc_lengths[iend - 3])
            };
            const value_type tangent{diff.normalized()};
            const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
            const value_type binormal{diff.cross(diff2).normalized()};
            return std::make_tuple(
                arc_lengths[iend - 1] + r.dot(tangent), r.dot(normal), r.dot(binormal)
            );
        }
        const value_type r{point - anchor_points[pos - 1]};
        const value_type diff{anchor_points[pos] - anchor_points[pos - 1]};
        const value_type diff2{
            (anchor_points[pos] - anchor_points[pos - 1]) /
                (arc_lengths[pos] - arc_lengths[pos - 1]) -
            (anchor_points[pos - 1] - anchor_points[pos - 2]) /
                (arc_lengths[pos - 1] - arc_lengths[pos - 2])
        };
        const value_type tangent{diff.normalized()};
        const value_type normal{diff.cross(diff2.cross(diff)).normalized()};
        const value_type binormal{diff.cross(diff2).normalized()};
        return std::make_tuple(
            arc_lengths[pos - 1] + r.dot(tangent), r.dot(normal), r.dot(binormal)
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

    PiecewiseLinearFunction<value_type, allocator_type> m_vec_of_s;
};

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec2s, alignof(Vec2s)>>
using PiecewiseLinearCurve2s = PiecewiseLinearCurve<Vec2s, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec2d, alignof(Vec2d)>>
using PiecewiseLinearCurve2d = PiecewiseLinearCurve<Vec2d, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec3s, alignof(Vec3s)>>
using PiecewiseLinearCurve3s = PiecewiseLinearCurve<Vec3s, Alloc>;

template <Allocatory Alloc = ::basjoo::common::AlignedAllocator<Vec3d, alignof(Vec3d)>>
using PiecewiseLinearCurve3d = PiecewiseLinearCurve<Vec3d, Alloc>;

namespace pmr {

template <VecArithmetic T>
using PiecewiseLinearCurve =
    basjoo::math::PiecewiseLinearCurve<T, std::pmr::polymorphic_allocator<T>>;

using PiecewiseLinearCurve2s =
    ::basjoo::math::PiecewiseLinearCurve<Vec2s, std::pmr::polymorphic_allocator<Vec2s>>;

using PiecewiseLinearCurve2d =
    ::basjoo::math::PiecewiseLinearCurve<Vec2d, std::pmr::polymorphic_allocator<Vec2d>>;

using PiecewiseLinearCurve3s =
    ::basjoo::math::PiecewiseLinearCurve<Vec3s, std::pmr::polymorphic_allocator<Vec3s>>;

using PiecewiseLinearCurve3d =
    ::basjoo::math::PiecewiseLinearCurve<Vec3d, std::pmr::polymorphic_allocator<Vec3d>>;

} // namespace pmr

} // namespace basjoo::math
