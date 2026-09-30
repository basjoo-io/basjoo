/**
 * @file motion.hpp
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

#include "basjoo/bicycle/planners/path.hpp"
#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"
#include "basjoo/math/functions/piecewise_quintic_function.hpp"

namespace basjoo::bicycle {

template <typename T>
    requires std::floating_point<T> || ::basjoo::math::VecArithmetic<T>
class Motion final {
  public:
    using value_type = T;
    using param_type = ::basjoo::math::detail::DenseDegenerateTraitT<value_type>;
    using size_type = std::size_t;
    using allocator_type = std::pmr::polymorphic_allocator<value_type>;

    Motion() noexcept = default;
    Motion(const Motion& other) noexcept = default;
    auto operator=(const Motion& other) noexcept -> Motion& = default;
    Motion(Motion&& other) noexcept = default;
    auto operator=(Motion&& other) noexcept -> Motion& = default;
    ~Motion() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_y_of_t.get_allocator();
    }

    template <
        std::ranges::input_range R0 = std::initializer_list<param_type>,
        std::ranges::input_range R1 = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    explicit Motion(R0&& ts, R1&& ss, const allocator_type& alloc = {})
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                 std::same_as<std::ranges::range_value_t<R1>, value_type>
        : Motion(
              ts, ss,
              std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                  ::basjoo::math::BoundaryMode<value_type>{2, value_type{0.0}},
                  ::basjoo::math::BoundaryMode<value_type>{4, value_type{0.0}}
              },
              std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                  ::basjoo::math::BoundaryMode<value_type>{2, value_type{0.0}},
                  ::basjoo::math::BoundaryMode<value_type>{4, value_type{0.0}}
              },
              alloc
          ) {}

    template <
        std::ranges::input_range R0 = std::initializer_list<param_type>,
        std::ranges::input_range R1 = std::initializer_list<value_type>>
    [[using gnu: always_inline]]
    explicit Motion(
        R0&& ts, R1&& ss, std::array<::basjoo::math::BoundaryMode<value_type>, 2> b0,
        std::array<::basjoo::math::BoundaryMode<value_type>, 2> bf, const allocator_type& alloc = {}
    )
        requires std::same_as<std::ranges::range_value_t<R0>, param_type> &&
                 std::same_as<std::ranges::range_value_t<R1>, value_type>
        : m_y_of_t(ts, ss, b0, bf, alloc) {}

    [[using gnu: pure, always_inline]]
    auto eval(param_type t) const noexcept -> value_type {
        return m_y_of_t.eval(t);
    }

    [[using gnu: pure, always_inline]]
    auto velocity(param_type t) const noexcept -> value_type {
        return m_y_of_t.derivative(t);
    }

    [[using gnu: pure, always_inline]]
    auto accel(param_type t) const noexcept -> value_type {
        return m_y_of_t.derivative(t, 2);
    }

    [[using gnu: pure, always_inline]]
    auto jerk(param_type t) const noexcept -> value_type {
        return m_y_of_t.derivative(t, 3);
    }

    [[using gnu: pure, always_inline]]
    auto snap(param_type t) const noexcept -> value_type {
        return m_y_of_t.derivative(t, 4);
    }

    [[using gnu: pure, always_inline]]
    auto minT() const noexcept -> param_type {
        return m_y_of_t.minT();
    }

    [[using gnu: pure, always_inline]]
    auto maxT() const noexcept -> param_type {
        return m_y_of_t.maxT();
    }

    [[using gnu: pure, always_inline]]
    auto ts() const noexcept -> std::span<const param_type> {
        return m_y_of_t.ts();
    }

    [[using gnu: pure, always_inline]]
    auto ys() const noexcept -> std::span<const value_type> {
        return m_y_of_t.ys();
    }

    template <typename U>
    explicit operator Motion<U>() const
        requires std::same_as<U, param_type> && ::basjoo::math::VecArithmetic<value_type>
    {
        std::span<const param_type> ts{m_y_of_t.ts()};
        std::span<const value_type> ys{m_y_of_t.ys()};
        std::span<const value_type> ddys{m_y_of_t.ddys()};
        std::span<const value_type> d4ys{m_y_of_t.d4ys()};
        const value_type dy0{m_y_of_t.derivative(ts.front())};
        const value_type dyf{m_y_of_t.derivative(ts.back())};
        const value_type ddy0{ddys.front()};
        const value_type ddyf{ddys.back()};
        const param_type dy0_euclidean{dy0.euclidean()};
        const param_type dyf_euclidean{dyf.euclidean()};
        const size_type size{ts.size()};

        std::pmr::vector<param_type> arc_lengths(size, m_y_of_t.get_allocator());
        arc_lengths[0] = 0;
        for (size_type i{1}; i < size; ++i) {
            arc_lengths[i] = arc_lengths[i - 1] + ::basjoo::math::calcArcLength(
                                                      ys[i - 1], ys[i], ddys[i - 1], ddys[i],
                                                      d4ys[i - 1], d4ys[i], ts[i] - ts[i - 1]
                                                  );
        }

        return Motion<param_type>{
            ts, arc_lengths,
            std::array<::basjoo::math::BoundaryMode<param_type>, 2>{
                ::basjoo::math::BoundaryMode<param_type>{1, dy0_euclidean},
                ::basjoo::math::BoundaryMode<param_type>{2, dy0.dot(ddy0) / dy0_euclidean}
            },
            std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                ::basjoo::math::BoundaryMode<param_type>{1, dyf_euclidean},
                ::basjoo::math::BoundaryMode<param_type>{2, dyf.dot(ddyf) / dyf_euclidean}
            },
            get_allocator()
        };
    }

    template <typename U>
    explicit operator Path<U>() const
        requires std::same_as<U, value_type> && ::basjoo::math::VecArithmetic<value_type>
    {
        const value_type dy0{m_y_of_t.derivative(m_y_of_t.minT())};
        const value_type dyf{m_y_of_t.derivative(m_y_of_t.maxT())};
        const value_type ddy0{m_y_of_t.ddys().front()};
        const value_type ddyf{m_y_of_t.ddys().back()};
        const param_type dy0_euclidean{dy0.euclidean()};
        const param_type dyf_euclidean{dyf.euclidean()};
        const param_type ddy0_euclidean{ddy0.euclidean()};
        const param_type ddyf_euclidean{ddyf.euclidean()};

        return Path<value_type>{
            m_y_of_t.ys(),
            std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                ::basjoo::math::BoundaryMode<param_type>{1, dy0.normalize()},
                ::basjoo::math::BoundaryMode<param_type>{
                    2, ddy0.normalize() *
                           std::abs(
                               dy0.crossProj(ddy0) / (dy0_euclidean * dy0_euclidean * dy0_euclidean)
                           )
                }
            },
            std::array<::basjoo::math::BoundaryMode<value_type>, 2>{
                ::basjoo::math::BoundaryMode<param_type>{1, dyf.normalize()},
                ::basjoo::math::BoundaryMode<param_type>{
                    2, ddyf.normalize() *
                           std::abs(
                               dyf.crossProj(ddyf) / (dyf_euclidean * dyf_euclidean * dyf_euclidean)
                           )
                }
            },
            0.0, get_allocator()
        };
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    ::basjoo::math::PiecewiseQuinticFunction<value_type, allocator_type> m_y_of_t;
};

using Motion1s = Motion<float>;

using Motion1d = Motion<double>;

using Motion2s = Motion<::basjoo::math::Vec2s>;

using Motion2d = Motion<::basjoo::math::Vec2d>;

} // namespace basjoo::bicycle
