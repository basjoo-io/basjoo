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
#include <iterator>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/bicycle/models/state.hpp"

namespace basjoo::bicycle {

/**
 * @brief Kinematic bicycle model driven by a longitudinal acceleration and a steering rate.
 *
 * The steering angle is part of the state rather than a control input, so the controls are the
 * longitudinal acceleration accel = du/dt and the steering rate omega = d(delta)/dt.
 *
 * Only x, y, yaw, delta and u are integrated. The kinematic model leaves v and r no freedom of
 * their own, so every integrator recomputes them from u and delta at the end of each step instead:
 * the states it returns satisfy v = l_r * r and r = u * tan(delta) / L exactly, and the v and r of
 * the state it is handed are never read.
 *
 * The two controls are hard bounded. createState(), the single step integrators and tangent() all
 * clamp a and omega into [a_lower_bound(), a_upper_bound()] and [omega_lower_bound(),
 * omega_upper_bound()] before they act on them or hand them back, so a state that leaves the model
 * never carries a control the model would not apply. The bounds default to infinite, which is no
 * constraint at all.
 */
template <std::floating_point T>
class KinematicsModel final {
  public:
    using value_type = T;
    using state_type = State<value_type>;

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

    /**
     * @brief Builds a state whose lateral velocity and yaw rate agree with the steering angle.
     *
     * The kinematic model leaves v and r fully determined by u and delta, so only those two are
     * asked for and the remaining pair follows from the geometry. The two controls default to zero,
     * i.e. to coasting with the steering wheel held still.
     */
    auto createState(
        value_type x, value_type y, value_type yaw, value_type delta, value_type u,
        value_type a = 0.0, value_type omega = 0.0
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::createState() "
                    "requires the delta to lie in (-pi/2, pi/2): delta = {0:f}.",
                    delta
                )
            );
        }
#endif
        const value_type yaw_rate{u * std::tan(delta) / wheelbase()};
        return state_type{
            .x{x},
            .y{y},
            .yaw{yaw},
            .delta{delta},
            .u{u},
            .v{m_lr * yaw_rate},
            .r{yaw_rate},
            .a{std::clamp(a, m_a_lo, m_a_up)},
            .omega{std::clamp(omega, m_omega_lo, m_omega_up)}
        };
    }

    [[using gnu: always_inline]]
    auto forwardEuler(state_type initial_state, value_type h) const noexcept(!BASJOO_CHECK_PARAMS)
        -> state_type {
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
        initial_state.a = std::clamp(initial_state.a, m_a_lo, m_a_up);
        initial_state.omega = std::clamp(initial_state.omega, m_omega_lo, m_omega_up);
        initial_state += tangent(initial_state) * h;
        initial_state.r = initial_state.u * std::tan(initial_state.delta) / wheelbase();
        initial_state.v = m_lr * initial_state.r;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(state_type initial_state, std::span<const value_type> hs) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = forwardEuler(std::move(initial_state), hs[i]);
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = forwardEuler(std::move(initial_state), hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, value_type accel, value_type omega, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
        initial_state.a = accel;
        initial_state.omega = omega;
        return forwardEuler(std::move(initial_state), h);
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::forwardEuler() "
                    "requires "
                    "the three input spans to have identical sizes: accels.size() = {0:d}, "
                    "omegas.size() = {1:d}, hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state.a = accels[i];
            initial_state.omega = omegas[i];
            initial_state = forwardEuler(std::move(initial_state), hs[i]);
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::forwardEuler() "
                    "requires "
                    "the three input spans to have identical sizes: accels.size() = {0:d}, "
                    "omegas.size() = {1:d}, hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state.a = accels[i];
            initial_state.omega = omegas[i];
            initial_state = forwardEuler(std::move(initial_state), hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(state_type initial_state, value_type h) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
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
        initial_state.a = std::clamp(initial_state.a, m_a_lo, m_a_up);
        initial_state.omega = std::clamp(initial_state.omega, m_omega_lo, m_omega_up);
        // The two integrated first order quantities go first and v and r follow from them, then
        // the heading picks up the fresh yaw rate, and the position is finally carried by the
        // fresh body velocity through the fresh heading. The controls are held over the step, so
        // they are carried over untouched.
        const state_type k{tangent(initial_state) * h};
        const value_type delta{initial_state.delta + k.delta};
        const value_type u{initial_state.u + k.u};
        const value_type r{u * std::tan(delta) / wheelbase()};
        const value_type v{m_lr * r};
        const value_type yaw{initial_state.yaw + r * h};
        return state_type{
            .x{initial_state.x + (u * std::cos(yaw) - v * std::sin(yaw)) * h},
            .y{initial_state.y + (u * std::sin(yaw) + v * std::cos(yaw)) * h},
            .yaw{yaw},
            .delta{delta},
            .u{u},
            .v{v},
            .r{r},
            .a{initial_state.a},
            .omega{initial_state.omega}
        };
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(state_type initial_state, std::span<const value_type> hs) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = symplecticEuler(std::move(initial_state), hs[i]);
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = symplecticEuler(std::move(initial_state), hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, value_type accel, value_type omega, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
        initial_state.a = accel;
        initial_state.omega = omega;
        return symplecticEuler(std::move(initial_state), h);
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsModel::symplecticEuler() requires the three input "
                    "spans to have identical sizes: accels.size() = {0:d}, omegas.size() = {1:d}, "
                    "hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state.a = accels[i];
            initial_state.omega = omegas[i];
            initial_state = symplecticEuler(std::move(initial_state), hs[i]);
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsModel::symplecticEuler() requires the three input "
                    "spans to have identical sizes: accels.size() = {0:d}, omegas.size() = {1:d}, "
                    "hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state.a = accels[i];
            initial_state.omega = omegas[i];
            initial_state = symplecticEuler(std::move(initial_state), hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(state_type initial_state, value_type h) const noexcept(!BASJOO_CHECK_PARAMS)
        -> state_type {
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
        initial_state.a = std::clamp(initial_state.a, m_a_lo, m_a_up);
        initial_state.omega = std::clamp(initial_state.omega, m_omega_lo, m_omega_up);
        constexpr value_type kHalf{static_cast<value_type>(0.5)};
        constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
        constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
        const state_type k1{tangent(initial_state) * h};
        const state_type k2{tangent(initial_state + k1 * kHalf) * h};
        const state_type k3{tangent(initial_state + k2 * kHalf) * h};
        const state_type k4{tangent(initial_state + k3) * h};
        initial_state += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
        initial_state.r = initial_state.u * std::tan(initial_state.delta) / wheelbase();
        initial_state.v = m_lr * initial_state.r;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(state_type initial_state, std::span<const value_type> hs) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = rungeKutta(std::move(initial_state), hs[i]);
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = rungeKutta(std::move(initial_state), hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, value_type accel, value_type omega, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
        initial_state.a = accel;
        initial_state.omega = omega;
        return rungeKutta(std::move(initial_state), h);
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::rungeKutta() "
                    "requires "
                    "the three input spans to have identical sizes: accels.size() = {0:d}, "
                    "omegas.size() = {1:d}, hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state.a = accels[i];
            initial_state.omega = omegas[i];
            initial_state = rungeKutta(std::move(initial_state), hs[i]);
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, std::span<const value_type> accels,
        std::span<const value_type> omegas, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (accels.size() != omegas.size() || accels.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsModel::rungeKutta() "
                    "requires "
                    "the three input spans to have identical sizes: accels.size() = {0:d}, "
                    "omegas.size() = {1:d}, hs.size() = {2:d}.",
                    accels.size(), omegas.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{accels.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state.a = accels[i];
            initial_state.omega = omegas[i];
            initial_state = rungeKutta(std::move(initial_state), hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    /**
     * @brief Tangent vector of the flow at the given state.
     *
     * State doubles as its own tangent space here, so the returned object is not a state: its x
     * and y fields hold the world frame velocity, its yaw field holds the yaw rate, its delta
     * field holds the steering rate and its u field holds the longitudinal acceleration. Its v and
     * r fields are zero, since those two are not integrated but recomputed from u and delta after
     * every step, and its a and omega fields are zero, since both controls are held constant over
     * a step. It is only meant to be scaled and added onto a state.
     *
     * The kinematic model ties v and r to u and delta through v = l_r * u * tan(delta) / L and
     * r = u * tan(delta) / L, so the lateral velocity and the yaw rate driving the pose are derived
     * here from the u and delta of the given state, and its own v and r are never read. Every stage
     * of an integrator therefore evaluates the flow on the constraint, whatever the v and r fields
     * of its intermediate states happen to hold. The two controls are clamped into their bounds
     * here as well, so an intermediate state that has wandered out of them still drives the flow
     * with a control the model would actually apply.
     */
    [[using gnu: pure, always_inline]]
    auto tangent(const state_type& state) const noexcept -> state_type {
        const value_type sin_yaw{std::sin(state.yaw)};
        const value_type cos_yaw{std::cos(state.yaw)};
        const value_type r{state.u * std::tan(state.delta) / wheelbase()};
        const value_type v{m_lr * r};
        return state_type{
            .x{state.u * cos_yaw - v * sin_yaw},
            .y{state.u * sin_yaw + v * cos_yaw},
            .yaw{r},
            .delta{std::clamp(state.omega, m_omega_lo, m_omega_up)},
            .u{std::clamp(state.a, m_a_lo, m_a_up)},
            .v{static_cast<value_type>(0.0)},
            .r{static_cast<value_type>(0.0)},
            .a{static_cast<value_type>(0.0)},
            .omega{static_cast<value_type>(0.0)}
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
