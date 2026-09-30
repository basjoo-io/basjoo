/**
 * @file state.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-08-08
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <cmath>
#include <concepts>
#include <cstddef>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

namespace basjoo::bicycle {

/**
 * @brief Nine degrees of freedom of a bicycle: pose, steering angle, body frame velocity and the
 * two controls driving them.
 *
 * x, y and yaw give the pose in the world frame, delta is the front wheel steering angle, and
 * u, v and r are the longitudinal velocity, the lateral velocity and the yaw rate, all three
 * expressed in the body frame rather than in the world frame. a is the longitudinal acceleration
 * du/dt and omega is the steering rate d(delta)/dt.
 */
template <std::floating_point T>
struct State final {
    using value_type = T;
    using reference = T&;
    using const_reference = const T&;
    using pointer = T*;
    using const_pointer = const T*;
    using size_type = std::size_t;

    static constexpr size_type kSize = 9;

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return kSize;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride() noexcept -> size_type {
        return 1;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return &x;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return &x;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_t i) noexcept -> reference {
        return (&x)[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_t i) const noexcept -> const_reference {
        return (&x)[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= kSize) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return (&x)[i];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(size_type i, const_reference value) noexcept(!BASJOO_CHECK_PARAMS)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= kSize) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        (&x)[i] = value;
        return;
    }

    /**
     * @brief Signed magnitude of the body frame velocity.
     *
     * The sign comes from the longitudinal component alone, so reversing keeps the speed negative
     * while the heading stays where it is.
     */
    [[using gnu: pure, always_inline]]
    auto speed() const noexcept -> value_type {
        return std::copysign(std::hypot(u, v), u);
    }

    /**
     * @brief Side slip angle, i.e. the angle between the velocity and the heading.
     */
    [[using gnu: pure, always_inline]]
    auto slipAngle() const noexcept -> value_type {
        return std::atan2(v, u);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator==(const State& obj) const noexcept -> bool = default;

    [[using gnu: always_inline, leaf]]
    constexpr auto operator+=(const State& obj) noexcept -> State& {
        x += obj.x;
        y += obj.y;
        yaw += obj.yaw;
        delta += obj.delta;
        u += obj.u;
        v += obj.v;
        r += obj.r;
        a += obj.a;
        omega += obj.omega;
        return *this;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator+(const State& obj) const& noexcept -> State {
        State result{*this};
        result += obj;
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto operator+(State&& obj) const& noexcept -> State&& {
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline]]
    constexpr auto operator+(const State& obj) && noexcept -> State&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline]]
    constexpr auto operator+(State&& obj) && noexcept -> State&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, leaf]]
    constexpr auto operator-=(const State& obj) noexcept -> State& {
        x -= obj.x;
        y -= obj.y;
        yaw -= obj.yaw;
        delta -= obj.delta;
        u -= obj.u;
        v -= obj.v;
        r -= obj.r;
        a -= obj.a;
        omega -= obj.omega;
        return *this;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator-(const State& obj) const& noexcept -> State {
        State result{*this};
        result -= obj;
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto operator-(State&& obj) const& noexcept -> State&& {
        obj *= -1.0;
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline]]
    constexpr auto operator-(const State& obj) && noexcept -> State&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline]]
    constexpr auto operator-(State&& obj) && noexcept -> State&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, leaf]]
    constexpr auto operator*=(const std::floating_point auto& fac) noexcept -> State& {
        const auto alpha{static_cast<value_type>(fac)};
        x *= alpha;
        y *= alpha;
        yaw *= alpha;
        delta *= alpha;
        u *= alpha;
        v *= alpha;
        r *= alpha;
        a *= alpha;
        omega *= alpha;
        return *this;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator*(const std::floating_point auto& fac) const& noexcept -> State {
        State result{*this};
        result *= fac;
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto operator*(const std::floating_point auto& fac) && noexcept -> State&& {
        operator*=(fac);
        return std::move(*this);
    }

    [[using gnu: always_inline, leaf]]
    constexpr auto operator/=(const std::floating_point auto& den) noexcept -> State& {
        const auto alpha{static_cast<value_type>(1.0) / static_cast<value_type>(den)};
        operator*=(alpha);
        return *this;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator/(const std::floating_point auto& den) const& noexcept -> State {
        State result{*this};
        result /= den;
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto operator/(const std::floating_point auto& den) && noexcept -> State&& {
        operator/=(den);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator-() const& noexcept -> State {
        State result{*this};
        result *= -1.0;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-() && noexcept -> State&& {
        operator*=(-1.0);
        return std::move(*this);
    }

    using serialize = zpp::bits::members<kSize>;

    value_type x{0.0};
    value_type y{0.0};
    value_type yaw{0.0};
    value_type delta{0.0};
    value_type u{0.0};
    value_type v{0.0};
    value_type r{0.0};
    value_type a{0.0};
    value_type omega{0.0};
};

template <std::floating_point T>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator*(const std::floating_point auto& fac, const State<T>& obj) noexcept
    -> State<T> {
    return obj * fac;
}

template <std::floating_point T>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const std::floating_point auto& fac, State<T>&& obj) noexcept
    -> State<T>&& {
    obj *= fac;
    return std::move(obj);
}

} // namespace basjoo::bicycle
