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
#include <cmath>
#include <concepts>
#include <cstddef>
#include <format>
#include <limits>
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
 * @brief Pose, steering angle and longitudinal velocity of the vehicle.
 *
 * x, y and yaw give the pose in the world frame, delta is the front wheel steering angle and u
 * is the longitudinal velocity in the body frame.
 */
template <std::floating_point T>
struct KinematicsModelState final : public ::basjoo::math::DenseArithmeticBase {
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsModelState() noexcept = default;

    // Every field set to v. Generic code builds the zero of a value type as value_type{0.0}.
    constexpr explicit KinematicsModelState(value_type v) noexcept
        : x{v}, y{v}, yaw{v}, delta{v}, u{v} {}

    constexpr KinematicsModelState(
        value_type x, value_type y, value_type yaw, value_type delta, value_type u
    ) noexcept
        : x{x}, y{y}, yaw{yaw}, delta{delta}, u{u} {}

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
        // The fields are exposed as a contiguous array of size() elements.
        static_assert(std::is_standard_layout_v<KinematicsModelState>);
        static_assert(sizeof(KinematicsModelState) == size() * sizeof(value_type));
        return &x;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return &x;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator[](index_type i) noexcept -> value_type& {
        return data()[i];
    }

    [[using gnu: pure, always_inline]]
    constexpr auto operator[](index_type i) const noexcept -> const value_type& {
        return data()[i];
    }

    using serialize = zpp::bits::members<5>;

    value_type x{0.0};
    value_type y{0.0};
    value_type yaw{0.0};
    value_type delta{0.0};
    value_type u{0.0};
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
 * The two controls are hard bounded. tangent(), and with it every integrator, clamps a and omega
 * into [a_lower_bound(), a_upper_bound()] and [omega_lower_bound(), omega_upper_bound()] before it
 * acts on them, so the model never applies a control outside its bounds. The bounds default to
 * infinite, which is no constraint at all.
 */
template <std::floating_point T>
class KinematicsModel final {
  public:
    using value_type = T;

    using state_type = KinematicsModelState<value_type>;

    KinematicsModel() noexcept = default;
    KinematicsModel(const KinematicsModel& other) noexcept = default;
    auto operator=(const KinematicsModel& other) noexcept -> KinematicsModel& = default;
    KinematicsModel(KinematicsModel&& other) noexcept = default;
    auto operator=(KinematicsModel&& other) noexcept -> KinematicsModel& = default;
    ~KinematicsModel() noexcept = default;

    explicit KinematicsModel(
        value_type front_wheelbase, value_type rear_wheelbase,
        value_type a_lower_bound = -std::numeric_limits<value_type>::infinity(),
        value_type a_upper_bound = std::numeric_limits<value_type>::infinity(),
        value_type omega_lower_bound = -std::numeric_limits<value_type>::infinity(),
        value_type omega_upper_bound = std::numeric_limits<value_type>::infinity()
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_lf{front_wheelbase}, m_lr{rear_wheelbase}, m_a_lo{a_lower_bound}, m_a_up{a_upper_bound},
          m_omega_lo{omega_lower_bound}, m_omega_up{omega_upper_bound} {
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
        if (!(a_lower_bound <= 0.0 && 0.0 <= a_upper_bound)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel requires the "
                    "acceleration bounds to bracket zero: a_lower_bound = {0:f} while "
                    "a_upper_bound = {1:f}.",
                    a_lower_bound, a_upper_bound
                )
            );
        }
        if (!(omega_lower_bound <= 0.0 && 0.0 <= omega_upper_bound)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel requires the "
                    "steering rate bounds to bracket zero: omega_lower_bound = {0:f} while "
                    "omega_upper_bound = {1:f}.",
                    omega_lower_bound, omega_upper_bound
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
    constexpr auto a_lower_bound() const noexcept -> value_type {
        return m_a_lo;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto a_upper_bound() const noexcept -> value_type {
        return m_a_up;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto omega_lower_bound() const noexcept -> value_type {
        return m_omega_lo;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto omega_upper_bound() const noexcept -> value_type {
        return m_omega_up;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, value_type accel, value_type omega, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::forwardEuler() "
                    "requires "
                    "the initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
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
        initial_state += tangent(initial_state, accel, omega) * h;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, value_type accel, value_type omega, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsModel::symplecticEuler() requires the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsModel::symplecticEuler() requires a positive time "
                    "step: h = {0:f}.",
                    h
                )
            );
        }
#endif
        // The two integrated first order quantities go first and the yaw rate and lateral velocity
        // follow from them, then the heading picks up the fresh yaw rate, and the position is
        // finally carried by the fresh body velocity through the fresh heading.
        const state_type k{tangent(initial_state, accel, omega) * h};
        const value_type delta{initial_state.delta + k.delta};
        const value_type u{initial_state.u + k.u};
        const value_type r{u * std::tan(delta) / wheelbase()};
        const value_type v{m_lr * r};
        const value_type yaw{initial_state.yaw + r * h};
        return state_type{
            initial_state.x + (u * std::cos(yaw) - v * std::sin(yaw)) * h,
            initial_state.y + (u * std::sin(yaw) + v * std::cos(yaw)) * h, yaw, delta, u
        };
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, value_type accel, value_type omega, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::rungeKutta() "
                    "requires "
                    "the initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
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
        const state_type k1{tangent(initial_state, accel, omega) * h};
        const state_type k2{tangent(initial_state + k1 * kHalf, accel, omega) * h};
        const state_type k3{tangent(initial_state + k2 * kHalf, accel, omega) * h};
        const state_type k4{tangent(initial_state + k3, accel, omega) * h};
        initial_state += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
        return initial_state;
    }

    /**
     * @brief Marches the state through a sequence of steps with the chosen integrator.
     *
     * Step i holds accels[i] and omegas[i] constant over hs[i]. An empty trace records nothing.
     * Otherwise it has to hold one more state than there are steps: trace[0] receives the initial
     * state and trace[i + 1] the state after step i, so the last element equals the return value.
     */
    template <::basjoo::math::IntegrateMode Mode>
    auto evolve(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs,
        std::span<state_type> trace = {}
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::evolve() "
                    "requires the three input spans to have identical sizes: accels.size() = "
                    "{0:d}, omegas.size() = {1:d}, hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
        if (!trace.empty() && trace.size() != accels.size() + 1) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::evolve() "
                    "requires the trace to be empty or one longer than the inputs: trace.size() = "
                    "{0:d} while accels.size() = {1:d}.",
                    trace.size(), accels.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        const bool tracing{!trace.empty()};
        if (tracing) {
            trace[0] = initial_state;
        }
        for (std::size_t i{0}; i < size; ++i) {
            if constexpr (Mode == ::basjoo::math::IntegrateMode::FORWARD_EULER) {
                initial_state = forwardEuler(std::move(initial_state), accels[i], omegas[i], hs[i]);
            } else if constexpr (Mode == ::basjoo::math::IntegrateMode::SYMPLECTIC_EULER) {
                initial_state =
                    symplecticEuler(std::move(initial_state), accels[i], omegas[i], hs[i]);
            } else if constexpr (Mode == ::basjoo::math::IntegrateMode::RUNGE_KUTTA) {
                initial_state = rungeKutta(std::move(initial_state), accels[i], omegas[i], hs[i]);
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
     * and y fields hold the world frame velocity, its yaw field holds the yaw rate, its delta
     * field holds the steering rate and its u field holds the longitudinal acceleration. It is only
     * meant to be scaled and added onto a state.
     *
     * The lateral velocity and the yaw rate driving the pose are derived from the u and delta of
     * the given state through v = l_r * u * tan(delta) / L and r = u * tan(delta) / L, so every
     * stage of an integrator evaluates the flow on the kinematic constraint. The two controls are
     * clamped into their bounds here, which is where every integrator picks up the clamp.
     */
    [[using gnu: pure, always_inline]]
    auto tangent(const state_type& state, value_type accel, value_type omega) const noexcept
        -> state_type {
        const value_type sin_yaw{std::sin(state.yaw)};
        const value_type cos_yaw{std::cos(state.yaw)};
        const value_type r{state.u * std::tan(state.delta) / wheelbase()};
        const value_type v{m_lr * r};
        return state_type{
            state.u * cos_yaw - v * sin_yaw, state.u * sin_yaw + v * cos_yaw, r,
            std::clamp(omega, m_omega_lo, m_omega_up), std::clamp(accel, m_a_lo, m_a_up)
        };
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<6>;

    value_type m_lf{1.0};
    value_type m_lr{1.0};
    value_type m_a_lo{-std::numeric_limits<value_type>::infinity()};
    value_type m_a_up{std::numeric_limits<value_type>::infinity()};
    value_type m_omega_lo{-std::numeric_limits<value_type>::infinity()};
    value_type m_omega_up{std::numeric_limits<value_type>::infinity()};
};

} // namespace basjoo::bicycle
