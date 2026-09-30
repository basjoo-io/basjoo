/**
 * @file kinematics_model.hpp
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

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <limits>
#include <mdspan>
#include <numbers>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/integrate_mode.hpp"

namespace basjoo::bicycle {

/**
 * @brief Pose, longitudinal velocity and steering angle of the vehicle.
 *
 * x, y and yaw give the pose in the world frame, u is the longitudinal velocity in the body frame
 * and delta is the front wheel steering angle.
 */
template <std::floating_point T>
class KinematicsModelState final : public ::basjoo::math::DenseArithmeticBase {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using extents_type = std::extents<index_type, 5>;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsModelState() noexcept = default;

    // Every field set to v. Generic code builds the zero of a value type as value_type{0.0}.
    constexpr explicit KinematicsModelState(value_type v) noexcept : m_data{v, v, v, v, v} {}

    constexpr KinematicsModelState(
        value_type x, value_type y, value_type yaw, value_type u, value_type delta
    ) noexcept
        : m_data{x, y, yaw, u, delta} {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank() noexcept -> rank_type {
        return 1;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> index_type {
        return size();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 5;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator[](index_type i) noexcept -> value_type& {
        return data()[i];
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator[](index_type i) const noexcept -> const value_type& {
        return data()[i];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[0];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[1];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto yaw(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[2];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto u(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[3];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto delta(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[4];
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // x, y, yaw, u and delta, in this order.
    std::array<value_type, 5> m_data{0.0, 0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Longitudinal acceleration and steering rate driving the vehicle.
 *
 * a is the longitudinal acceleration du/dt and omega the steering rate d(delta)/dt. The two fields
 * are stored flat like those of KinematicsModelState, so that control sequences combine and
 * interpolate field by field. Default construction gives both zero.
 */
template <std::floating_point T>
class KinematicsModelControl final : public ::basjoo::math::DenseArithmeticBase {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using extents_type = std::extents<index_type, 2>;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsModelControl() noexcept = default;

    // Every field set to v. Generic code builds the zero of a value type as value_type{0.0}.
    constexpr explicit KinematicsModelControl(value_type v) noexcept : m_data{v, v} {}

    constexpr KinematicsModelControl(value_type a, value_type omega) noexcept : m_data{a, omega} {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank() noexcept -> rank_type {
        return 1;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> index_type {
        return size();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 2;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator[](index_type i) noexcept -> value_type& {
        return data()[i];
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator[](index_type i) const noexcept -> const value_type& {
        return data()[i];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto a(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[0];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[1];
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // a and omega, in this order.
    std::array<value_type, 2> m_data{0.0, 0.0};
};

/**
 * @brief Kinematic bicycle model driven by a longitudinal acceleration and a steering rate.
 *
 * The steering angle is part of the state rather than a control input, so the controls are the
 * longitudinal acceleration a = du/dt and the steering rate omega = d(delta)/dt. They are handed to
 * every integrator alongside the state and held constant over a step.
 *
 * The kinematic model leaves the lateral velocity and the yaw rate no freedom of their own: they
 * follow from u and delta through r = u * tan(delta) / L and v = l_r * r, so the state carries
 * neither of them and tangent() derives both where the flow needs them.
 *
 * The two controls are hard bounded, each on its own. tangent(), and with it every integrator,
 * clamps them into [control_lower_bound(), control_upper_bound()] before it acts on them, so the
 * model never applies a control outside its bounds. The bounds default to infinite, which is no
 * constraint at all.
 */
template <std::floating_point T>
class KinematicsModel final {
  public:
    using value_type = T;

    using state_type = KinematicsModelState<value_type>;
    using control_type = KinematicsModelControl<value_type>;

    KinematicsModel() noexcept = default;
    KinematicsModel(const KinematicsModel& other) noexcept = default;
    auto operator=(const KinematicsModel& other) noexcept -> KinematicsModel& = default;
    KinematicsModel(KinematicsModel&& other) noexcept = default;
    auto operator=(KinematicsModel&& other) noexcept -> KinematicsModel& = default;
    ~KinematicsModel() noexcept = default;

    explicit KinematicsModel(
        value_type front_wheelbase, value_type rear_wheelbase,
        control_type control_lower_bound =
            control_type{-std::numeric_limits<value_type>::infinity()},
        control_type control_upper_bound = control_type{std::numeric_limits<value_type>::infinity()}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_lf{front_wheelbase}, m_lr{rear_wheelbase}, m_control_lo{control_lower_bound},
          m_control_up{control_upper_bound} {
#if BASJOO_CHECK_PARAMS == 1
        if (front_wheelbase <= 0.0 || rear_wheelbase <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel requires both "
                    "wheelbases to be positive: front_wheelbase = {0:f} while rear_wheelbase = "
                    "{1:f}.",
                    front_wheelbase, rear_wheelbase
                )
            );
        }
        if (!(m_control_lo.a() <= 0.0 && 0.0 <= m_control_up.a())) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel requires the "
                    "acceleration bounds to bracket zero: a lower bound = {0:f} while a upper "
                    "bound = {1:f}.",
                    m_control_lo.a(), m_control_up.a()
                )
            );
        }
        if (!(m_control_lo.omega() <= 0.0 && 0.0 <= m_control_up.omega())) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel requires the "
                    "steering rate bounds to bracket zero: omega lower bound = {0:f} while omega "
                    "upper bound = {1:f}.",
                    m_control_lo.omega(), m_control_up.omega()
                )
            );
        }
#endif
    }

    [[using gnu: pure, always_inline]]
    constexpr auto front_wheelbase() const noexcept -> value_type {
        return m_lf;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto rear_wheelbase() const noexcept -> value_type {
        return m_lr;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto wheelbase() const noexcept -> value_type {
        return m_lf + m_lr;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto control_lower_bound() const noexcept -> const control_type& {
        return m_control_lo;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto control_upper_bound() const noexcept -> const control_type& {
        return m_control_up;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, const control_type& control, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta()) < std::numbers::pi_v<value_type> * 0.5))
            [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::forwardEuler() "
                    "requires "
                    "the initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta()
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::forwardEuler() "
                    "requires a "
                    "positive time step: h = {0:f}.",
                    h
                )
            );
        }
#endif
        initial_state += tangent(initial_state, control) * h;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, const control_type& control, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta()) < std::numbers::pi_v<value_type> * 0.5))
            [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::rungeKutta() "
                    "requires "
                    "the initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta()
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::rungeKutta() "
                    "requires a "
                    "positive time step: h = {0:f}.",
                    h
                )
            );
        }
#endif
        constexpr value_type kHalf{static_cast<value_type>(0.5)};
        constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
        constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
        const state_type k1{tangent(initial_state, control) * h};
        const state_type k2{tangent(initial_state + k1 * kHalf, control) * h};
        const state_type k3{tangent(initial_state + k2 * kHalf, control) * h};
        const state_type k4{tangent(initial_state + k3, control) * h};
        initial_state += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
        return initial_state;
    }

    /**
     * @brief Marches the state through a sequence of steps with the chosen integrator.
     *
     * Step i holds controls[i] constant over hs[i]. An empty trace records nothing. Otherwise it
     * has to hold one more state than there are steps: trace[0] receives the initial state and
     * trace[i + 1] the state after step i, so the last element equals the return value.
     */
    template <::basjoo::math::IntegrateMode Mode>
    auto evolve(
        state_type initial_state, std::span<const control_type> controls,
        std::span<const value_type> hs, std::span<state_type> trace = {}
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (controls.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::evolve() "
                    "requires the two input spans to have identical sizes: controls.size() = "
                    "{0:d}, hs.size() = {1:d}.",
                    controls.size(), hs.size()
                )
            );
        }
        if (!trace.empty() && trace.size() != controls.size() + 1) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::evolve() "
                    "requires the trace to be empty or one longer than the inputs: trace.size() = "
                    "{0:d} while controls.size() = {1:d}.",
                    trace.size(), controls.size()
                )
            );
        }
#endif
        const std::size_t size{controls.size()};
        const bool tracing{!trace.empty()};
        if (tracing) {
            trace[0] = initial_state;
        }
        for (std::size_t i{0}; i < size; ++i) {
            if constexpr (Mode == ::basjoo::math::IntegrateMode::FORWARD_EULER) {
                initial_state = forwardEuler(std::move(initial_state), controls[i], hs[i]);
            } else if constexpr (Mode == ::basjoo::math::IntegrateMode::RUNGE_KUTTA) {
                initial_state = rungeKutta(std::move(initial_state), controls[i], hs[i]);
            } else {
                static_assert(false, "Unknown basjoo::math::IntegrateMode.");
            }
            if (tracing) {
                trace[i + 1] = initial_state;
            }
        }
        return initial_state;
    }

    /**
     * @brief Tangent vector of the flow at the given state under the given controls.
     *
     * State doubles as its own tangent space here, so the returned object is not a state: its x
     * and y fields hold the world frame velocity, its yaw field holds the yaw rate, its u field
     * holds the longitudinal acceleration and its delta field holds the steering rate. It is only
     * meant to be scaled and added onto a state.
     *
     * The lateral velocity and the yaw rate driving the pose are derived from the u and delta of
     * the given state through v = l_r * u * tan(delta) / L and r = u * tan(delta) / L, so every
     * stage of an integrator evaluates the flow on the kinematic constraint. The two controls are
     * clamped into their bounds here, which is where every integrator picks up the clamp.
     */
    [[using gnu: pure, always_inline]]
    auto tangent(const state_type& state, const control_type& control) const noexcept
        -> state_type {
        const value_type sin_yaw{std::sin(state.yaw())};
        const value_type cos_yaw{std::cos(state.yaw())};
        const value_type r{state.u() * std::tan(state.delta()) / wheelbase()};
        const value_type v{m_lr * r};
        return state_type{
            state.u() * cos_yaw - v * sin_yaw, state.u() * sin_yaw + v * cos_yaw, r,
            std::clamp(control.a(), m_control_lo.a(), m_control_up.a()),
            std::clamp(control.omega(), m_control_lo.omega(), m_control_up.omega())
        };
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<4>;

    value_type m_lf{1.0};
    value_type m_lr{1.0};
    control_type m_control_lo{-std::numeric_limits<value_type>::infinity()};
    control_type m_control_up{std::numeric_limits<value_type>::infinity()};
};

} // namespace basjoo::bicycle
