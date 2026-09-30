/**
 * @file kinematics_flow_controller.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Flow controller of KinematicsModel with a fixed or an adaptive preview horizon.
 * @version 0.1
 * @date 2026-09-16
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
#include <numbers>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/bicycle/models/kinematics_model.hpp"
#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/integrate_mode.hpp"

namespace basjoo::bicycle {

/**
 * @brief Plant state of KinematicsModel together with the two controls driving it and, when
 * Adaptive is true, the horizon they are steered over.
 *
 * x, y, yaw, delta and u are the plant state, a is the longitudinal acceleration du/dt and omega is
 * the steering rate d(delta)/dt. tau, present only when Adaptive is true, is the horizon of the
 * prediction the controls are steered by (see KinematicsFlowController::tangent()). The controls,
 * and the horizon where there is one, evolve over time alongside the plant, which is what makes
 * this a controller rather than a model.
 */
template <std::floating_point T, bool Adaptive = false>
struct KinematicsFlowControllerState;

/**
 * @brief State of a KinematicsFlowController whose horizon is fixed: the plant state and the two
 * controls.
 */
template <std::floating_point T>
struct KinematicsFlowControllerState<T, false> final : public ::basjoo::math::DenseArithmeticBase {
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsFlowControllerState() noexcept = default;

    // Every field set to v. Generic code builds the zero of a value type as value_type{0.0}.
    constexpr explicit KinematicsFlowControllerState(value_type v) noexcept
        : x{v}, y{v}, yaw{v}, delta{v}, u{v}, a{v}, omega{v} {}

    constexpr KinematicsFlowControllerState(
        value_type x, value_type y, value_type yaw, value_type delta, value_type u, value_type a,
        value_type omega
    ) noexcept
        : x{x}, y{y}, yaw{yaw}, delta{delta}, u{u}, a{a}, omega{omega} {}

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
        return 7;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        // The fields are exposed as a contiguous array of size() elements.
        static_assert(std::is_standard_layout_v<KinematicsFlowControllerState>);
        static_assert(sizeof(KinematicsFlowControllerState) == size() * sizeof(value_type));
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

    using serialize = zpp::bits::members<7>;

    value_type x{0.0};
    value_type y{0.0};
    value_type yaw{0.0};
    value_type delta{0.0};
    value_type u{0.0};
    value_type a{0.0};
    value_type omega{0.0};
};

/**
 * @brief State of a KinematicsFlowController whose horizon adapts: the plant state, the two
 * controls and the horizon.
 */
template <std::floating_point T>
struct KinematicsFlowControllerState<T, true> final : public ::basjoo::math::DenseArithmeticBase {
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsFlowControllerState() noexcept = default;

    // Every field set to v, the horizon included. Generic code builds the zero of a value type as
    // value_type{0.0}, which the default state is not, as its horizon defaults to one.
    constexpr explicit KinematicsFlowControllerState(value_type v) noexcept
        : x{v}, y{v}, yaw{v}, delta{v}, u{v}, a{v}, omega{v}, tau{v} {}

    constexpr KinematicsFlowControllerState(
        value_type x, value_type y, value_type yaw, value_type delta, value_type u, value_type a,
        value_type omega, value_type tau
    ) noexcept
        : x{x}, y{y}, yaw{yaw}, delta{delta}, u{u}, a{a}, omega{omega}, tau{tau} {}

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
        return 8;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        // The fields are exposed as a contiguous array of size() elements.
        static_assert(std::is_standard_layout_v<KinematicsFlowControllerState>);
        static_assert(sizeof(KinematicsFlowControllerState) == size() * sizeof(value_type));
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

    using serialize = zpp::bits::members<8>;

    value_type x{0.0};
    value_type y{0.0};
    value_type yaw{0.0};
    value_type delta{0.0};
    value_type u{0.0};
    value_type a{0.0};
    value_type omega{0.0};
    value_type tau{1.0};
};

/**
 * @brief Tracking controller of KinematicsModel whose two controls flow down a tracking cost
 * alongside the plant.
 *
 * Every integrator steps the plant under the controls the state carries and steps the controls
 * along the preconditioned negative gradient of a cost that compares a one step prediction of the
 * plant over a horizon tau against a target (see tangent()). With Adaptive false the horizon is a
 * constant of the controller. With Adaptive true it is a field of the state and descends the same
 * cost alongside the controls, and at the horizon a state carries the controls are steered exactly
 * as by the controller with that fixed horizon.
 */
template <std::floating_point T, bool Adaptive = false>
class KinematicsFlowController final {
  public:
    using value_type = T;
    using model_type = ::basjoo::bicycle::KinematicsModel<value_type>;

    using state_type = KinematicsFlowControllerState<value_type, Adaptive>;

    KinematicsFlowController() noexcept = default;
    KinematicsFlowController(const KinematicsFlowController& other) noexcept = delete;
    auto operator=(const KinematicsFlowController& other) noexcept
        -> KinematicsFlowController& = delete;
    KinematicsFlowController(KinematicsFlowController&& other) noexcept = default;
    auto operator=(KinematicsFlowController&& other) noexcept
        -> KinematicsFlowController& = default;
    ~KinematicsFlowController() noexcept = default;

    /**
     * @brief Builds a controller from the plant model, the four cost weights and the horizon.
     *
     * The model carries the geometry and the hard bounds on the two controls, and model() hands it
     * back. The weights weigh in turn the position, yaw, delta and u residuals of the prediction
     * against the target, so the a and omega of any target are ignored. Every term is made
     * dimensionless before it is weighed, so the four weights are pure numbers and directly
     * comparable with each other. One weight covers x and y together, which leaves the cost
     * independent of how the world frame is oriented. See tangent() for the cost they enter.
     */
    explicit KinematicsFlowController(
        model_type model, std::span<const value_type, 4> weights, value_type tau
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires(!Adaptive)
        : m_model{std::move(model)}, m_tau{tau} {
#if BASJOO_CHECK_PARAMS == 1
        if (!(tau > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::bicycle::KinematicsFlowController "
                    "requires a positive horizon: tau = {0:f}.",
                    tau
                )
            );
        }
#endif
        std::ranges::copy_n(weights.begin(), 4, m_weights.begin());
    }

    /**
     * @brief Builds a controller whose horizon adapts from the plant model and the four cost
     * weights.
     *
     * The horizon comes with the state instead, and the a, omega and tau of any target are
     * ignored. The weights are those of the controller with a fixed horizon.
     */
    explicit KinematicsFlowController(
        model_type model, std::span<const value_type, 4> weights
    ) noexcept
        requires Adaptive
        : m_model{std::move(model)} {
        std::ranges::copy_n(weights.begin(), 4, m_weights.begin());
    }

    [[using gnu: pure, always_inline]]
    constexpr auto model() const noexcept -> const model_type& {
        return m_model;
    }

    [[using gnu: pure, always_inline]]
    auto weights() const noexcept -> std::span<const value_type> {
        return {m_weights.cbegin(), m_weights.cend()};
    }

    /**
     * @brief Horizon of the prediction the controls are steered by. Only a controller whose
     * horizon is fixed has one of its own.
     */
    [[using gnu: pure, always_inline]]
    constexpr auto tau() const noexcept -> value_type
        requires(!Adaptive)
    {
        return m_tau;
    }

    /**
     * @brief Advances the closed loop by one step of length h with forward Euler.
     *
     * The plant, the controls and, when Adaptive is true, the horizon all take one Euler step
     * along tangent() evaluated at the start of the step, which leaves the plant where
     * KinematicsModel::forwardEuler() puts it under the controls the step starts with. The controls
     * are then clamped into the bounds the model carries. The horizon the step starts with has to
     * span at least the step.
     */
    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, const state_type& target_state, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires a positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(horizonOf(initial_state) >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires the "
                    "horizon to span at least the time step: tau = {0:f} while h = {1:f}.",
                    horizonOf(initial_state), h
                )
            );
        }
#endif
        initial_state += tangent(initial_state, target_state, horizonOf(initial_state)) * h;
        initial_state.a =
            std::clamp(initial_state.a, m_model.a_lower_bound(), m_model.a_upper_bound());
        initial_state.omega = std::clamp(
            initial_state.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound()
        );
        return initial_state;
    }

    /**
     * @brief Advances the closed loop by one step of length h with symplectic Euler.
     *
     * The plant takes the step KinematicsModel::symplecticEuler() takes under the controls it
     * starts with, while the controls and, when Adaptive is true, the horizon move along tangent()
     * evaluated at the start of the step, after which the controls are clamped into the bounds the
     * model carries. As with forwardEuler(), the horizon the step starts with has to span at least
     * the step.
     */
    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, const state_type& target_state, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires a "
                    "positive time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(horizonOf(initial_state) >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires the "
                    "horizon to span at least the time step: tau = {0:f} while h = {1:f}.",
                    horizonOf(initial_state), h
                )
            );
        }
#endif
        // The same staging as KinematicsModel::symplecticEuler(): delta and u first, the yaw rate
        // and lateral velocity from them, then the heading, then the position.
        const state_type k{tangent(initial_state, target_state, horizonOf(initial_state)) * h};
        const value_type delta{initial_state.delta + k.delta};
        const value_type u{initial_state.u + k.u};
        const value_type r{u * std::tan(delta) / m_model.wheelbase()};
        const value_type v{m_model.rear_wheelbase() * r};
        const value_type yaw{initial_state.yaw + r * h};
        const value_type x{initial_state.x + (u * std::cos(yaw) - v * std::sin(yaw)) * h};
        const value_type y{initial_state.y + (u * std::sin(yaw) + v * std::cos(yaw)) * h};
        const value_type a{
            std::clamp(initial_state.a + k.a, m_model.a_lower_bound(), m_model.a_upper_bound())
        };
        const value_type omega{std::clamp(
            initial_state.omega + k.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound()
        )};
        if constexpr (Adaptive) {
            return state_type{x, y, yaw, delta, u, a, omega, initial_state.tau + k.tau};
        } else {
            return state_type{x, y, yaw, delta, u, a, omega};
        }
    }

    /**
     * @brief Advances the closed loop by one step of length h with the classic fourth order
     * Runge-Kutta scheme.
     *
     * Unlike the two Euler steps, the controls and, when Adaptive is true, the horizon keep moving
     * inside the step: every stage evaluates tangent() afresh on the state that stage has reached,
     * so the plant at each stage runs under the controls of that stage, clamped into the bounds the
     * model carries, as is the result of the step. The target is the point of the reference the
     * horizon looks ahead to from the start of the step, and it is held over the step while the
     * vehicle moves on towards it, so every stage looks ahead over its horizon less its stage time,
     * which is what is left to reach that point from where the stage sits. The stage time enters
     * the stages alone: an adaptive horizon itself moves by its rate only. The horizon the step
     * starts with has to span more than the step, which keeps the target ahead of the last stage
     * as well.
     *
     * Only one target comes in per step, so against a reference that moves on over the step the
     * closed loop is followed to first order in h only, as it is by the two Euler steps: fourth
     * order would take the reference points the later stages look ahead to as well.
     */
    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, const state_type& target_state, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(initial_state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires a positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(horizonOf(initial_state) > h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires the horizon "
                    "to span more than the time step: tau = {0:f} while h = {1:f}.",
                    horizonOf(initial_state), h
                )
            );
        }
#endif
        const state_type k1{tangent(initial_state, target_state, horizonOf(initial_state)) * h};
        const state_type s2{initial_state + k1 * kHalf};
        const state_type k2{tangent(s2, target_state, horizonOf(s2) - h * kHalf) * h};
        const state_type s3{initial_state + k2 * kHalf};
        const state_type k3{tangent(s3, target_state, horizonOf(s3) - h * kHalf) * h};
        const state_type s4{initial_state + k3};
        const state_type k4{tangent(s4, target_state, horizonOf(s4) - h) * h};
        initial_state += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
        initial_state.a =
            std::clamp(initial_state.a, m_model.a_lower_bound(), m_model.a_upper_bound());
        initial_state.omega = std::clamp(
            initial_state.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound()
        );
        return initial_state;
    }

    /**
     * @brief Marches the state through a sequence of steps with the chosen integrator.
     *
     * Step i tracks target_states[i] over hs[i]. An empty trace records nothing. Otherwise it has
     * to hold one more state than there are steps: trace[0] receives the initial state and
     * trace[i + 1] the state after step i, so the last element equals the return value.
     */
    template <::basjoo::math::IntegrateMode Mode>
    auto evolve(
        state_type initial_state, std::span<const state_type> target_states,
        std::span<const value_type> hs, std::span<state_type> trace = {}
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (target_states.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::evolve() requires the two input "
                    "spans to have identical sizes: target_states.size() = {0:d} while hs.size() = "
                    "{1:d}.",
                    target_states.size(), hs.size()
                )
            );
        }
        if (!trace.empty() && trace.size() != target_states.size() + 1) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::evolve() requires the trace to be "
                    "empty or one longer than the inputs: trace.size() = {0:d} while "
                    "target_states.size() = {1:d}.",
                    trace.size(), target_states.size()
                )
            );
        }
#endif
        const std::size_t size{target_states.size()};
        const bool tracing{!trace.empty()};
        if (tracing) {
            trace[0] = initial_state;
        }
        for (std::size_t i{0}; i < size; ++i) {
            if constexpr (Mode == ::basjoo::math::IntegrateMode::FORWARD_EULER) {
                initial_state = forwardEuler(std::move(initial_state), target_states[i], hs[i]);
            } else if constexpr (Mode == ::basjoo::math::IntegrateMode::SYMPLECTIC_EULER) {
                initial_state = symplecticEuler(std::move(initial_state), target_states[i], hs[i]);
            } else if constexpr (Mode == ::basjoo::math::IntegrateMode::RUNGE_KUTTA) {
                initial_state = rungeKutta(std::move(initial_state), target_states[i], hs[i]);
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
     * @brief Tangent vector of the closed loop flow at the given state.
     *
     * State doubles as its own tangent space here, so the returned object is not a state. Its
     * first five fields are the plant flow, the same as KinematicsModel::tangent() returns under
     * the two controls of the given state. Its a and omega fields, and its tau field when Adaptive
     * is true, carry the controller down the tracking cost
     *
     *     J = (w0 (Rx^2 + Ry^2) / L^2 + w1 Ryaw^2 + w2 Rdelta^2 + w3 (tau Ru / L)^2) / 2,
     *
     * which compares a one step prediction of the plant over the horizon tau against the target
     * state, ignoring the controls and the horizon of the target. Every term is divided by the
     * wheelbase L and by tau until it is dimensionless. The prediction is the Taylor expansion of
     * the flow of KinematicsModel under the two controls held over the horizon, exact through tau^2
     * and cut down at tau^3 to the terms at most linear in the controls. That drops only the terms
     * in which the steering rate multiplies the acceleration or itself, and it leaves every
     * residual affine in a and omega:
     *
     *     r = u tan(delta) / L,   v = l_r r,
     *     rdot = (a tan(delta) + u omega sec^2(delta)) / L,   vdot = l_r rdot,
     *     Vx = u cos(yaw) - v sin(yaw),   Vy = u sin(yaw) + v cos(yaw),
     *     Ax = a cos(yaw) - vdot sin(yaw),   Ay = a sin(yaw) + vdot cos(yaw),
     *     Rx = x - x_ref + tau Vx + tau^2/2 (Ax - r Vy) + tau^3/6 (-r^2 Vx - 2 r Ay - rdot Vy),
     *     Ry = y - y_ref + tau Vy + tau^2/2 (Ay + r Vx) + tau^3/6 (-r^2 Vy + 2 r Ax + rdot Vx),
     *     Ryaw = yaw - yaw_ref + tau r + tau^2/2 rdot,
     *     Rdelta = delta - delta_ref + tau omega,   Ru = u - u_ref + tau a.
     *
     * With the steering angle held, the prediction is therefore exact to third order in tau, and
     * along a straight line under a constant acceleration it is exact outright.
     *
     * The controls follow the preconditioned negative gradient
     *
     *     da/dt = -(L^2 / tau^5) dJ/da,   domega/dt = -(1 / tau^3) dJ/domega:
     *
     * for the delta and u terms, an Euler step of length h moves the control by exactly w h / tau
     * times the Newton step of that term alone, so a unit weight is a full Newton step once the
     * step spans the whole horizon. The horizon sets the loop gain and the step only how finely the
     * resulting flow is followed, and scaling all four weights together scales the gain with them.
     * Nothing holds the controls back but the residuals they feed into, so an Euler step needs the
     * horizon to outlast it: along a straight line with unit weights on position and u alone, the
     * longitudinal error decays under forwardEuler() only for tau > 1.408 h and under
     * symplecticEuler() only for tau > 18 h / 11.
     *
     * When Adaptive is true the horizon descends the same cost,
     *
     *     dtau/dt = -tau dJ/dtau,
     *
     * preconditioned by itself, which keeps it positive along the flow,
     * tau(t) = tau(0) exp(-int dJ/dtau dt), though a step of an integrator still crosses zero once
     * h dJ/dtau reaches one. dJ/dtau also takes in the horizon the u term is scaled by, which
     * pulls the horizon down for as long as the speed residual stays off zero. Nothing in the cost
     * ties the horizon to the time the target actually lies ahead: driving straight at the target
     * with its speed u and both controls at zero, a distance d short of it, tau = d / u zeroes
     * every residual and all three rates whatever d. The loop therefore does not close an along
     * track offset, and the horizon takes it up instead. A caller that takes the target the
     * horizon of the state ahead along the reference, rather than a fixed time ahead, ties the two
     * together, and the loop then does close the offset. That leaves nothing to pin the horizon,
     * though: on the reference every horizon all but zeroes the residuals, so the horizon drifts
     * with whatever along track lag or lead the integrator leaves, growing while the vehicle lags
     * and shrinking while it leads. Waypoints sparser than the step are to be interpolated at the
     * time the horizon looks ahead to: the nearest waypoint lies up to half their spacing off that
     * time, which shifts the target along track, and an adaptive horizon takes that offset up like
     * any other.
     *
     * Both the plant and the cost act on the two controls of the given state clamped into the
     * bounds the model carries: whatever the caller hands in, and however far an intermediate stage
     * of rungeKutta() has wandered, the flow comes out as it would at the state within the bounds.
     */
    [[using gnu: always_inline]]
    auto tangent(const state_type& state, const state_type& target_state) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
        return tangent(state, target_state, horizonOf(state));
    }

    /**
     * @brief Tracking cost J that tangent() carries the controller down, at the given state against
     * the target.
     *
     * J compares the one step prediction over the horizon, the controller's own or, when Adaptive
     * is true, the one the state carries, against the target state, whose controls and horizon it
     * ignores. tangent() lists its terms and the residuals they weigh. As there, the two controls
     * of the given state enter clamped into the bounds the model carries.
     */
    [[using gnu: flatten, leaf]]
    auto cost(const state_type& state, const state_type& target_state) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::cost() requires the state.delta "
                    "to lie in (-pi/2, pi/2): state.delta = {0:f}.",
                    state.delta
                )
            );
        }
        if constexpr (Adaptive) {
            if (!(state.tau > 0.0)) [[unlikely]] {
                throw std::invalid_argument(
                    std::format(
                        "Invalid arguments detected! "
                        "basjoo::bicycle::KinematicsFlowController::cost() requires a positive "
                        "horizon: state.tau = {0:f}.",
                        state.tau
                    )
                );
            }
        }
#endif
        const value_type tau{horizonOf(state)};
        const Expansion expansion{expand(state, target_state, tau)};
        return kHalf * weigh(expansion.value, expansion.value, tau);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<3>;

    static constexpr value_type kHalf{static_cast<value_type>(0.5)};
    static constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
    static constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
    static constexpr value_type kTwo{static_cast<value_type>(2.0)};
    static constexpr value_type kThree{static_cast<value_type>(3.0)};

    /**
     * @brief One value per plant field: a residual of the prediction, a partial derivative of the
     * residuals, or the plant flow.
     */
    struct Residuals final {
        value_type x{0.0};
        value_type y{0.0};
        value_type yaw{0.0};
        value_type delta{0.0};
        value_type u{0.0};
    };

    /**
     * @brief What expand() derives at a state: the plant flow, the residuals at the horizon and
     * their partial derivatives by a, omega and tau, the last left at zero unless Adaptive is true.
     */
    struct Expansion final {
        Residuals flow{};
        Residuals value{};
        Residuals by_a{};
        Residuals by_omega{};
        Residuals by_tau{};
    };

    /**
     * @brief Horizon the prediction runs over at the given state: the controller's own, or the one
     * the state carries when Adaptive is true.
     */
    [[using gnu: pure, always_inline]]
    constexpr auto horizonOf([[maybe_unused]] const state_type& state) const noexcept
        -> value_type {
        if constexpr (Adaptive) {
            return state.tau;
        } else {
            return m_tau;
        }
    }

    /**
     * @brief Weighted inner product of two sets of residuals at the horizon tau.
     *
     * weigh(R, R, tau) is twice the cost J. Paired with a partial derivative of the residuals it
     * is the matching partial derivative of J, save that dJ/dtau also takes in the tau the u term
     * is scaled by.
     */
    [[using gnu: pure, always_inline]]
    auto weigh(const Residuals& lhs, const Residuals& rhs, value_type tau) const noexcept
        -> value_type {
        const value_type L{m_model.wheelbase()};
        return (m_weights[0] * (lhs.x * rhs.x + lhs.y * rhs.y) +
                m_weights[3] * tau * tau * lhs.u * rhs.u) /
                   (L * L) +
               m_weights[1] * lhs.yaw * rhs.yaw + m_weights[2] * lhs.delta * rhs.delta;
    }

    /**
     * @brief Residuals of the prediction over tau against the target, their partial derivatives
     * and the plant flow, all under the two controls of the state clamped into the bounds.
     *
     * Every residual is a polynomial of degree three at most in tau whose coefficients are affine
     * in the two controls, so its partial derivatives by a and omega follow from those of the
     * coefficients. The coefficients of tau make up the plant flow itself, which tangent() hands on
     * as it is.
     */
    [[using gnu: pure, always_inline]]
    auto expand(
        const state_type& state, const state_type& target_state, value_type tau
    ) const noexcept -> Expansion {
        const value_type a{std::clamp(state.a, m_model.a_lower_bound(), m_model.a_upper_bound())};
        const value_type omega{
            std::clamp(state.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound())
        };
        const value_type L{m_model.wheelbase()};
        const value_type lr{m_model.rear_wheelbase()};
        const value_type sin_yaw{std::sin(state.yaw)};
        const value_type cos_yaw{std::cos(state.yaw)};
        const value_type tan_delta{std::tan(state.delta)};
        const value_type cos_delta{std::cos(state.delta)};
        const value_type sec2_delta{static_cast<value_type>(1.0) / (cos_delta * cos_delta)};
        const value_type r{state.u * tan_delta / L};
        const value_type v{lr * r};
        const value_type vx{state.u * cos_yaw - v * sin_yaw};
        const value_type vy{state.u * sin_yaw + v * cos_yaw};
        const value_type rdot_by_a{tan_delta / L};
        const value_type rdot_by_omega{state.u * sec2_delta / L};
        const value_type rdot{a * rdot_by_a + omega * rdot_by_omega};
        const value_type vdot{lr * rdot};
        const value_type ax{a * cos_yaw - vdot * sin_yaw};
        const value_type ay{a * sin_yaw + vdot * cos_yaw};
        const value_type x2{kHalf * (ax - r * vy)};
        const value_type x3{kSixth * (-r * r * vx - kTwo * r * ay - rdot * vy)};
        const value_type y2{kHalf * (ay + r * vx)};
        const value_type y3{kSixth * (-r * r * vy + kTwo * r * ax + rdot * vx)};
        const value_type yaw2{kHalf * rdot};
        const value_type ax_by_a{cos_yaw - lr * rdot_by_a * sin_yaw};
        const value_type ay_by_a{sin_yaw + lr * rdot_by_a * cos_yaw};
        const value_type ax_by_omega{-lr * rdot_by_omega * sin_yaw};
        const value_type ay_by_omega{lr * rdot_by_omega * cos_yaw};
        const value_type tau2{tau * tau};
        Expansion expansion{
            .flow = {vx, vy, r, omega, a},
            .value =
                {state.x - target_state.x + tau * (vx + tau * (x2 + tau * x3)),
                 state.y - target_state.y + tau * (vy + tau * (y2 + tau * y3)),
                 state.yaw - target_state.yaw + tau * (r + tau * yaw2),
                 state.delta - target_state.delta + tau * omega,
                 state.u - target_state.u + tau * a},
            .by_a =
                {tau2 * (kHalf * ax_by_a - tau * kSixth * (kTwo * r * ay_by_a + rdot_by_a * vy)),
                 tau2 * (kHalf * ay_by_a + tau * kSixth * (kTwo * r * ax_by_a + rdot_by_a * vx)),
                 tau2 * kHalf * rdot_by_a, static_cast<value_type>(0.0), tau},
            .by_omega =
                {tau2 * (kHalf * ax_by_omega -
                         tau * kSixth * (kTwo * r * ay_by_omega + rdot_by_omega * vy)),
                 tau2 * (kHalf * ay_by_omega +
                         tau * kSixth * (kTwo * r * ax_by_omega + rdot_by_omega * vx)),
                 tau2 * kHalf * rdot_by_omega, tau, static_cast<value_type>(0.0)},
            .by_tau = {}
        };
        if constexpr (Adaptive) {
            expansion.by_tau = {
                vx + tau * (kTwo * x2 + kThree * tau * x3),
                vy + tau * (kTwo * y2 + kThree * tau * y3), r + tau * rdot, omega, a
            };
        }
        return expansion;
    }

    /**
     * @brief tangent() looking ahead over tau instead of the horizon at the given state, so that
     * the later stages of rungeKutta() look ahead over what is left of it.
     */
    [[using gnu: flatten, leaf, hot]]
    auto tangent(
        const state_type& state, const state_type& target_state, value_type tau
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::tangent() requires the state.delta "
                    "to lie in (-pi/2, pi/2): state.delta = {0:f}.",
                    state.delta
                )
            );
        }
        if constexpr (Adaptive) {
            if (!(tau > 0.0)) [[unlikely]] {
                throw std::invalid_argument(
                    std::format(
                        "Invalid arguments detected! "
                        "basjoo::bicycle::KinematicsFlowController::tangent() requires a positive "
                        "horizon: tau = {0:f}.",
                        tau
                    )
                );
            }
        }
#endif
        const Expansion expansion{expand(state, target_state, tau)};
        const value_type L{m_model.wheelbase()};
        const value_type tau3{tau * tau * tau};
        const value_type a_rate{
            -L * L * weigh(expansion.value, expansion.by_a, tau) / (tau3 * tau * tau)
        };
        const value_type omega_rate{-weigh(expansion.value, expansion.by_omega, tau) / tau3};
        const Residuals& flow{expansion.flow};
        if constexpr (Adaptive) {
            const value_type tau_rate{
                -tau * (weigh(expansion.value, expansion.by_tau, tau) +
                        m_weights[3] * tau * expansion.value.u * expansion.value.u / (L * L))
            };
            return state_type{flow.x, flow.y, flow.yaw,   flow.delta,
                              flow.u, a_rate, omega_rate, tau_rate};
        } else {
            return state_type{flow.x, flow.y, flow.yaw, flow.delta, flow.u, a_rate, omega_rate};
        }
    }

    model_type m_model{};
    std::array<value_type, 4> m_weights{
        static_cast<value_type>(1.0), static_cast<value_type>(1.0), static_cast<value_type>(1.0),
        static_cast<value_type>(1.0)
    };
    // NaN and unused when Adaptive is true, where the state carries the horizon.
    value_type m_tau{
        Adaptive ? std::numeric_limits<value_type>::quiet_NaN() : static_cast<value_type>(1.0)
    };
};

} // namespace basjoo::bicycle
