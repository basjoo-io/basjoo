/**
 * @file path.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-06
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <array>
#include <memory_resource>
#include <span>

#include "zpp_bits.h"

#include "basjoo/math/curves/piecewise_quintic_curve.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"

namespace basjoo::bicycle {

template <::basjoo::math::Vec2Arithmetic T>
class Path final {
  public:
    using value_type = T;
    using param_type = typename T::value_type;
    using size_type = std::size_t;
    using allocator_type = std::pmr::polymorphic_allocator<value_type>;

    Path() noexcept = default;
    Path(const Path& other) noexcept = default;
    auto operator=(const Path& other) noexcept -> Path& = default;
    Path(Path&& other) noexcept = default;
    auto operator=(Path&& other) noexcept -> Path& = default;
    ~Path() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_curve.get_allocator();
    }

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    explicit Path(R&& anchor_points, param_type s0 = 0.0, const allocator_type& alloc = {})
        requires std::same_as<std::ranges::range_value_t<R>, value_type>
        : Path(
              anchor_points,
              std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                  ::basjoo::math::BoundaryMode<value_type>{2, value_type{0.0, 0.0}},
                  ::basjoo::math::BoundaryMode<value_type>{4, value_type{0.0, 0.0}}
              },
              std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                  ::basjoo::math::BoundaryMode<value_type>{2, value_type{0.0, 0.0}},
                  ::basjoo::math::BoundaryMode<value_type>{4, value_type{0.0, 0.0}}
              },
              s0, alloc
          ) {}

    template <std::ranges::input_range R = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    explicit Path(
        R&& anchor_points, std::array<::basjoo::math::BoundaryMode<value_type>, 2> b0,
        std::array<::basjoo::math::BoundaryMode<value_type>, 2> bf, param_type s0 = 0.0,
        const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R>, value_type>
        : m_curve{anchor_points, b0, bf, s0, alloc} {}

    [[using gnu: pure, always_inline]]
    auto operator()(param_type s) const noexcept -> value_type {
        return m_curve(s);
    }

    [[using gnu: pure, always_inline]]
    auto operator()(param_type s, param_type l) const noexcept -> value_type {
        return m_curve(s, l);
    }

    [[using gnu: pure, always_inline]]
    auto inverse(value_type point) const noexcept -> std::pair<param_type, param_type> {
        return m_curve.inverse(point);
    }

    [[using gnu: pure, always_inline]]
    auto inverse(value_type point, param_type start_s, param_type end_s) const noexcept
        -> std::pair<param_type, param_type> {
        return m_curve.inverse(point, start_s, end_s);
    }

    [[using gnu: pure, always_inline]]
    auto tangent(param_type s) const noexcept -> value_type {
        return m_curve.tangent(s);
    }

    [[using gnu: pure, always_inline]]
    auto normal(param_type s) const noexcept -> value_type {
        return m_curve.normal(s);
    }

    [[using gnu: pure, always_inline]]
    auto curvature(param_type s) const noexcept -> param_type {
        return m_curve.curvature(s);
    }

    [[using gnu: pure, always_inline]]
    auto minS() const noexcept -> param_type {
        return m_curve.minS();
    }

    [[using gnu: pure, always_inline]]
    auto maxS() const noexcept -> param_type {
        return m_curve.maxS();
    }

    [[using gnu: pure, always_inline]]
    auto arcLengths() const noexcept -> std::span<const param_type> {
        return m_curve.arcLengths();
    }

    [[using gnu: pure, always_inline]]
    auto anchorPoints() const noexcept -> std::span<const value_type> {
        return m_curve.anchorPoints();
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    ::basjoo::math::PiecewiseQuinticCurve<value_type, allocator_type> m_curve;
};

using Path2s = Path<::basjoo::math::Vec2s>;

using Path2d = Path<::basjoo::math::Vec2d>;

} // namespace basjoo::bicycle
