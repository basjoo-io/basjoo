/**
 * @file basjoo/bicycle/controllers/kinematics_adaptive_flow_controller.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Flow controller of KinematicsModel whose preview horizon evolves with the integrator.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026 basjoo development team
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
#include <numbers>
#include <span>
#include <stdexcept>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/bicycle/models/kinematics_model.hpp"
#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/integrate_mode.hpp"

namespace basjoo::bicycle {

namespace detail {

/**
 * @brief Plant state of KinematicsModel together with the two controls driving it and the horizon
 * they are steered over.
 *
 * x, y, yaw, delta and u are the plant state, a is the longitudinal acceleration du/dt and
 * omega is the steering rate d(delta)/dt. tau is the horizon of the prediction the controls are
 * steered by (see KinematicsAdaptiveFlowController::tangent()). The controls and the horizon
 * evolve over time alongside the plant, which is what makes this a controller rather than a model.
 */
template <std::floating_point T>
struct KinematicsAdaptiveFlowControllerState final : public ::basjoo::math::DenseArithmeticBase {
    using value_type = T;
    using size_type = std::size_t;
    using rank_type = std::size_t;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsAdaptiveFlowControllerState() noexcept = default;

    constexpr KinematicsAdaptiveFlowControllerState(
        value_type x, value_type y, value_type yaw, value_type delta, value_type u, value_type a,
        value_type omega, value_type tau
    ) noexcept
        : x{x}, y{y}, yaw{yaw}, delta{delta}, u{u}, a{a}, omega{omega}, tau{tau} {}

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto rank() noexcept -> rank_type {
        return 1;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> size_type {
        return size();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 8;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return &x;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return &x;
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

} // namespace detail

/**
 * @brief KinematicsFlowController with its horizon moved into the state, where it descends the
 * tracking cost alongside the two controls.
 *
 * At the horizon a state carries, the cost and the two control rates are those of
 * KinematicsFlowController, so whatever holds there for a given horizon holds here as well. See
 * tangent() for how the horizon moves.
 */
template <std::floating_point T>
class KinematicsAdaptiveFlowController final {
  public:
    using value_type = T;
    using model_type = ::basjoo::bicycle::KinematicsModel<value_type>;

    using state_type = detail::KinematicsAdaptiveFlowControllerState<value_type>;

    KinematicsAdaptiveFlowController() noexcept = default;
    KinematicsAdaptiveFlowController(const KinematicsAdaptiveFlowController& other) noexcept =
        delete;
    auto operator=(const KinematicsAdaptiveFlowController& other) noexcept
        -> KinematicsAdaptiveFlowController& = delete;
    KinematicsAdaptiveFlowController(KinematicsAdaptiveFlowController&& other) noexcept = default;
    auto operator=(KinematicsAdaptiveFlowController&& other) noexcept
        -> KinematicsAdaptiveFlowController& = default;
    ~KinematicsAdaptiveFlowController() noexcept = default;

    /**
     * @brief Builds a controller from the plant model and the five cost weights.
     *
     * The model carries the geometry and the hard bounds on the two controls, and model() hands it
     * back. The weights follow the state layout and weigh in turn the x, y, yaw, delta and u
     * residuals of the one step prediction against the target, so the a, omega and tau of any
     * target are ignored. Every term is made dimensionless before it is weighed, so the five
     * weights are pure numbers and directly comparable with each other. The horizon comes with the
     * state instead. See tangent() for the cost the weights enter.
     */
    explicit KinematicsAdaptiveFlowController(
        model_type model, std::span<const value_type, 5> weights
    ) noexcept
        : m_model{std::move(model)} {
        std::ranges::copy_n(weights.begin(), 5, m_weights.begin());
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
     * @brief Advances the closed loop by one step of length h with forward Euler.
     *
     * The plant, the controls and the horizon all take one Euler step along tangent() evaluated at
     * the start of the step, which leaves the plant where KinematicsModel::forwardEuler() puts it
     * under the controls the step starts with. The controls are then clamped into the bounds the
     * model carries. The horizon the step starts with has to span at least the step.
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
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::forwardEuler() requires "
                    "the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::forwardEuler() requires a "
                    "positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(initial_state.tau >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::forwardEuler() requires "
                    "the "
                    "horizon to span at least the time step: initial_state.tau = {0:f} while h = "
                    "{1:f}.",
                    initial_state.tau, h
                )
            );
        }
#endif
        initial_state += tangent(initial_state, target_state) * h;
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
     * starts with, and the controls and the horizon move along tangent() evaluated at the start of
     * the step, after which the controls are clamped into the bounds the model carries. As with
     * forwardEuler(), the horizon the step starts with has to span at least the step.
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
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::symplecticEuler() requires "
                    "the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::symplecticEuler() requires "
                    "a positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(initial_state.tau >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::symplecticEuler() requires "
                    "the "
                    "horizon to span at least the time step: initial_state.tau = {0:f} while h = "
                    "{1:f}.",
                    initial_state.tau, h
                )
            );
        }
#endif
        // The same staging as KinematicsModel::symplecticEuler(): delta and u first, the yaw rate
        // and lateral velocity from them, then the heading, then the position.
        const state_type k{tangent(initial_state, target_state) * h};
        const value_type delta{initial_state.delta + k.delta};
        const value_type u{initial_state.u + k.u};
        const value_type r{u * std::tan(delta) / m_model.wheelbase()};
        const value_type v{m_model.rear_wheelbase() * r};
        const value_type yaw{initial_state.yaw + r * h};
        return state_type{
            initial_state.x + (u * std::cos(yaw) - v * std::sin(yaw)) * h,
            initial_state.y + (u * std::sin(yaw) + v * std::cos(yaw)) * h,
            yaw,
            delta,
            u,
            std::clamp(initial_state.a + k.a, m_model.a_lower_bound(), m_model.a_upper_bound()),
            std::clamp(
                initial_state.omega + k.omega, m_model.omega_lower_bound(),
                m_model.omega_upper_bound()
            ),
            initial_state.tau + k.tau
        };
    }

    /**
     * @brief Advances the closed loop by one step of length h with the classic fourth order
     * Runge-Kutta scheme.
     *
     * Unlike the two Euler overloads, the controls and the horizon keep moving inside the step:
     * every stage evaluates tangent() afresh on the state that stage has reached, so the plant at
     * each stage runs under the controls of that stage, clamped into the bounds the model carries,
     * as is the result of the step. The target is the point of the reference the horizon looks
     * ahead to from the start of the step, and it is held over the step while the vehicle moves on
     * towards it, so every stage looks ahead over the horizon it has reached less the stage time,
     * which is what is left to reach that point from where the stage sits. The stage time enters
     * the stages alone: the horizon itself moves by its rates, so held still it leaves the step
     * KinematicsFlowController::rungeKutta() takes. The horizon the step starts with has to span
     * more than the step, which keeps the target ahead of the last stage as well.
     *
     * Only one target comes in per step, so against a reference that moves on over the step the
     * closed loop is followed to first order in h only, as it is by the two Euler overloads:
     * fourth order would take the reference points the later stages look ahead to as well.
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
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::rungeKutta() requires the "
                    "initial_state.delta to lie in (-pi/2, pi/2): initial_state.delta = {0:f}.",
                    initial_state.delta
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::rungeKutta() requires a "
                    "positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(initial_state.tau > h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::rungeKutta() requires the "
                    "horizon to span more than the time step: initial_state.tau = {0:f} while h = "
                    "{1:f}.",
                    initial_state.tau, h
                )
            );
        }
#endif
        constexpr value_type kHalf{static_cast<value_type>(0.5)};
        constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
        constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
        const state_type k1{tangent(initial_state, target_state) * h};
        state_type stage{initial_state + k1 * kHalf};
        stage.tau -= h * kHalf;
        const state_type k2{tangent(stage, target_state) * h};
        stage = initial_state + k2 * kHalf;
        stage.tau -= h * kHalf;
        const state_type k3{tangent(stage, target_state) * h};
        stage = initial_state + k3;
        stage.tau -= h;
        const state_type k4{tangent(stage, target_state) * h};
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
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::evolve() requires the two "
                    "input "
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
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::evolve() requires the "
                    "trace to be "
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
     * the two controls of the given state. Its a, omega and tau fields are what makes this a
     * controller rather than a model: instead of being held still, the two controls and the
     * horizon descend a tracking cost together. It is only meant to be scaled and added onto a
     * state.
     *
     * The cost is the one KinematicsFlowController::tangent() builds over its horizon, here over
     * the horizon tau the state carries, with the residuals listed there:
     *
     *     J = w0 (Rx/L)^2 + w1 (Ry/L)^2 + w2 Ryaw^2 + w3 Rdelta^2 + w4 (tau Ru/L)^2
     *
     * and the controls and the horizon follow the preconditioned negative gradient
     *
     *     da/dt = -(L^2/(2 tau^5)) dJ/da,   domega/dt = -(1/(2 tau^3)) dJ/domega,
     *     dtau/dt = -(tau/2) dJ/dtau.
     *
     * The horizon is preconditioned by itself, which keeps it positive along the flow,
     * tau(t) = tau(0) exp(-int dJ/dtau dt / 2), though a step of an integrator still crosses zero
     * once h dJ/dtau reaches two. dJ/dtau also takes in the horizon the u term is scaled by, which
     * pulls the horizon down for as long as the speed residual stays off zero.
     *
     * Nothing in the cost ties the horizon to the time the target actually lies ahead. Driving
     * straight at the target with its speed u and both controls at zero, a distance d short of it,
     * with tau = d / u zeroes every residual and all three rates whatever d, so the loop does not
     * close an along track offset: the horizon takes it up instead, and the vehicle settles
     * wherever the transient leaves it along the reference.
     *
     * Both the plant and the cost act on the two controls of the given state clamped into the
     * bounds the model carries: whatever the caller hands in, and however far an intermediate stage
     * of rungeKutta() has wandered, the flow comes out as it would at the state within the bounds.
     * The horizon is taken as it is and has to be positive.
     */
    [[using gnu: flatten, leaf, hot]]
    auto tangent(const state_type& state, const state_type& target_state) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (!(std::abs(state.delta) < std::numbers::pi_v<value_type> * 0.5)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::tangent() requires the "
                    "state.delta to lie in (-pi/2, pi/2): state.delta = {0:f}.",
                    state.delta
                )
            );
        }
        if (!(state.tau > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::tangent() requires a "
                    "positive horizon: state.tau = {0:f}.",
                    state.tau
                )
            );
        }
#endif
        const value_type x{state.x};
        const value_type y{state.y};
        const value_type yaw{state.yaw};
        const value_type delta{state.delta};
        const value_type u{state.u};
        const value_type u2{u * u};
        const value_type a{std::clamp(state.a, m_model.a_lower_bound(), m_model.a_upper_bound())};
        const value_type omega{
            std::clamp(state.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound())
        };
        const value_type tau{state.tau};
        const value_type tau2{tau * tau};
        const value_type lf{m_model.front_wheelbase()};
        const value_type lr{m_model.rear_wheelbase()};
        const value_type L{m_model.wheelbase()};
        const value_type L2{L * L};
        const value_type L3{L2 * L};
        const value_type sin_yaw{std::sin(yaw)};
        const value_type cos_yaw{std::cos(yaw)};
        const value_type sin_delta{std::sin(delta)};
        const value_type cos_delta{std::cos(delta)};
        // std::tan() rather than sin/cos keeps the plant flow bit for bit equal to the model's.
        const value_type tan_delta{std::tan(delta)};
        const value_type sec_delta{static_cast<value_type>(1.0) / cos_delta};
        const value_type sec2_delta{sec_delta * sec_delta};
        const value_type sin_yaw_plus_delta{sin_yaw * cos_delta + cos_yaw * sin_delta};
        const value_type cos_yaw_plus_delta{cos_yaw * cos_delta - sin_yaw * sin_delta};
        // The gradients derived in Mathematica, negated into descent directions, with each
        // denominator doubled to halve the preconditioners.
        const value_type a_rate{
            -(static_cast<value_type>(4.0) * tau * (u + a * tau - target_state.u) * m_weights[4] +
              static_cast<value_type>(4.0) * m_weights[2] * tan_delta *
                  (u * tau2 * omega * sec2_delta + L * (yaw - target_state.yaw) +
                   tau * (u + a * tau) * tan_delta) +
              (static_cast<value_type>(4.0) * (sin_yaw * lf + sec_delta * sin_yaw_plus_delta * lr) *
               m_weights[1] *
               (L * (u * tau2 * omega * cos_yaw * sec2_delta * lr +
                     tau * (u + a * tau) * sin_yaw * L + L * (y - target_state.y)) +
                tau *
                    (cos_yaw * lf * (u2 * tau + (u + a * tau) * lr) +
                     lr * (u2 * tau * cos_yaw_plus_delta * sec_delta +
                           (u + a * tau) * cos_yaw * lr)) *
                    tan_delta)) /
                  L3 +
              ((cos_yaw * lf + cos_yaw_plus_delta * sec_delta * lr) * m_weights[0] *
               (static_cast<value_type>(-2.0) * tau * sec2_delta * sin_yaw * L *
                    (static_cast<value_type>(2.0) * u * tau * omega * lr +
                     (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                         (u2 * tau + (u + a * tau) * lr)) +
                static_cast<value_type>(4.0) * L2 * (x - target_state.x) +
                static_cast<value_type>(4.0) * tau * cos_yaw *
                    ((u + a * tau) * L2 - u2 * tau * lr * tan_delta * tan_delta))) /
                  L3) /
            (static_cast<value_type>(4.0) * tau2 * tau)
        };
        const value_type omega_rate{
            -(static_cast<value_type>(4.0) * L3 * L2 * m_weights[3] *
                  (delta + tau * omega - target_state.delta) +
              static_cast<value_type>(4.0) * u * tau * sec2_delta * L3 * m_weights[2] *
                  (u * tau2 * omega * sec2_delta + L * (yaw - target_state.yaw) +
                   tau * (u + a * tau) * tan_delta) +
              u * tau * sec2_delta * sin_yaw * lr * m_weights[0] *
                  (static_cast<value_type>(2.0) * tau * sec2_delta * sin_yaw * L *
                       (static_cast<value_type>(2.0) * u * tau * omega * lr +
                        (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                            (u2 * tau + (u + a * tau) * lr)) -
                   static_cast<value_type>(4.0) * L2 * (x - target_state.x) -
                   static_cast<value_type>(4.0) * tau * cos_yaw *
                       ((u + a * tau) * L2 - u2 * tau * lr * tan_delta * tan_delta)) +
              static_cast<value_type>(4.0) * u * tau * cos_yaw * sec2_delta * lr * m_weights[1] *
                  (lf * lf * (y + tau * (u + a * tau) * sin_yaw - target_state.y) +
                   lf * (u2 * tau2 * cos_yaw * tan_delta +
                         lr * (static_cast<value_type>(2.0) * y +
                               static_cast<value_type>(2.0) * tau * (u + a * tau) * sin_yaw -
                               static_cast<value_type>(2.0) * target_state.y +
                               tau * cos_yaw *
                                   (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta))) +
                   lr * (u2 * tau2 * cos_yaw_plus_delta * sec_delta * tan_delta +
                         lr * (y + tau * (u + a * tau) * sin_yaw - target_state.y +
                               tau * cos_yaw *
                                   (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta))))) /
            (static_cast<value_type>(4.0) * tau2 * L3 * L2)
        };
        const value_type tau_rate{
            -(tau *
              (static_cast<value_type>(2.0) * a * tau2 * L2 * L2 * (u + a * tau - target_state.u) *
                   m_weights[4] +
               static_cast<value_type>(2.0) * tau * L2 * L2 * (u + a * tau - target_state.u) *
                   (u + a * tau - target_state.u) * m_weights[4] +
               static_cast<value_type>(2.0) * omega * L3 * L3 * m_weights[3] *
                   (delta + tau * omega - target_state.delta) +
               static_cast<value_type>(2.0) * L2 * L2 * m_weights[2] *
                   (u * tau2 * omega * sec2_delta + L * (yaw - target_state.yaw) +
                    tau * (u + a * tau) * tan_delta) *
                   (static_cast<value_type>(2.0) * u * tau * omega * sec2_delta +
                    (u + static_cast<value_type>(2.0) * a * tau) * tan_delta) +
               static_cast<value_type>(2.0) * m_weights[1] *
                   (lf * lf * (y + tau * (u + a * tau) * sin_yaw - target_state.y) +
                    lf * (u2 * tau2 * cos_yaw * tan_delta +
                          lr * (static_cast<value_type>(2.0) * y +
                                static_cast<value_type>(2.0) * tau * (u + a * tau) * sin_yaw -
                                static_cast<value_type>(2.0) * target_state.y +
                                tau * cos_yaw *
                                    (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta))) +
                    lr * (u2 * tau2 * cos_yaw_plus_delta * sec_delta * tan_delta +
                          lr * (y + tau * (u + a * tau) * sin_yaw - target_state.y +
                                tau * cos_yaw *
                                    (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta)))) *
                   ((u + static_cast<value_type>(2.0) * a * tau) * sin_yaw * lf * lf +
                    lf * (static_cast<value_type>(2.0) * u2 * tau * cos_yaw * tan_delta +
                          lr * (static_cast<value_type>(2.0) * u * tau * omega * cos_yaw *
                                    sec2_delta +
                                static_cast<value_type>(2.0) *
                                    (u + static_cast<value_type>(2.0) * a * tau) * sin_yaw +
                                (u + static_cast<value_type>(2.0) * a * tau) * cos_yaw *
                                    tan_delta)) +
                    lr * (static_cast<value_type>(2.0) * u2 * tau * cos_yaw_plus_delta * sec_delta *
                              tan_delta +
                          lr * ((u + static_cast<value_type>(2.0) * a * tau) * sin_yaw +
                                cos_yaw *
                                    (static_cast<value_type>(2.0) * u * tau * omega * sec2_delta +
                                     (u + static_cast<value_type>(2.0) * a * tau) * tan_delta)))) +
               (m_weights[0] *
                (static_cast<value_type>(-2.0) * tau * sec2_delta * sin_yaw * L *
                     (static_cast<value_type>(2.0) * u * tau * omega * lr +
                      (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                          (u2 * tau + (u + a * tau) * lr)) +
                 static_cast<value_type>(4.0) * L2 * (x - target_state.x) +
                 static_cast<value_type>(4.0) * tau * cos_yaw *
                     ((u + a * tau) * L2 - u2 * tau * lr * tan_delta * tan_delta)) *
                ((u + static_cast<value_type>(2.0) * a * tau) * cos_yaw * lf * lf +
                 sec_delta * lr *
                     (((u + static_cast<value_type>(2.0) * a * tau) * cos_yaw_plus_delta -
                       static_cast<value_type>(2.0) * u * tau * omega * sec_delta * sin_yaw) *
                          lr -
                      static_cast<value_type>(2.0) * u2 * tau * sin_yaw_plus_delta * tan_delta) +
                 lf * (static_cast<value_type>(-2.0) * u2 * tau * sin_yaw * tan_delta +
                       lr * (static_cast<value_type>(2.0) *
                                 (u + static_cast<value_type>(2.0) * a * tau) * cos_yaw -
                             sin_yaw *
                                 (static_cast<value_type>(2.0) * u * tau * omega * sec2_delta +
                                  (u + static_cast<value_type>(2.0) * a * tau) * tan_delta))))) /
                   static_cast<value_type>(2.0))) /
            (static_cast<value_type>(2.0) * L3 * L3)
        };
        const value_type r{u * tan_delta / L};
        const value_type v{lr * r};
        return state_type{u * cos_yaw - v * sin_yaw,
                          u * sin_yaw + v * cos_yaw,
                          r,
                          omega,
                          a,
                          a_rate,
                          omega_rate,
                          tau_rate};
    }

    /**
     * @brief Tracking cost J that tangent() carries the two controls and the horizon down, at the
     * given state against the target.
     *
     * J is the one KinematicsFlowController::cost() evaluates, here over the horizon tau the state
     * carries, and it ignores the a, omega and tau of the target. As in tangent(), the two controls
     * of the given state enter clamped into the bounds the model carries, while the horizon is
     * taken as it is and has to be positive.
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
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::cost() requires the "
                    "state.delta to lie in (-pi/2, pi/2): state.delta = {0:f}.",
                    state.delta
                )
            );
        }
        if (!(state.tau > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsAdaptiveFlowController::cost() requires a "
                    "positive horizon: state.tau = {0:f}.",
                    state.tau
                )
            );
        }
#endif
        const value_type x{state.x};
        const value_type y{state.y};
        const value_type yaw{state.yaw};
        const value_type delta{state.delta};
        const value_type u{state.u};
        const value_type u2{u * u};
        const value_type a{std::clamp(state.a, m_model.a_lower_bound(), m_model.a_upper_bound())};
        const value_type omega{
            std::clamp(state.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound())
        };
        const value_type tau{state.tau};
        const value_type tau2{tau * tau};
        const value_type lf{m_model.front_wheelbase()};
        const value_type lr{m_model.rear_wheelbase()};
        const value_type L{m_model.wheelbase()};
        const value_type L2{L * L};
        const value_type L3{L2 * L};
        const value_type sin_yaw{std::sin(yaw)};
        const value_type cos_yaw{std::cos(yaw)};
        const value_type sin_delta{std::sin(delta)};
        const value_type cos_delta{std::cos(delta)};
        const value_type tan_delta{std::tan(delta)};
        const value_type sec_delta{static_cast<value_type>(1.0) / cos_delta};
        const value_type sec2_delta{sec_delta * sec_delta};
        const value_type cos_yaw_plus_delta{cos_yaw * cos_delta - sin_yaw * sin_delta};
        // The cost derived in Mathematica.
        return (tau2 * L2 * L2 * (u + a * tau - target_state.u) * (u + a * tau - target_state.u) *
                    m_weights[4] +
                L3 * L3 * m_weights[3] * (delta + tau * omega - target_state.delta) *
                    (delta + tau * omega - target_state.delta) +
                L2 * L2 * m_weights[2] *
                    (u * tau2 * omega * sec2_delta + L * (yaw - target_state.yaw) +
                     tau * (u + a * tau) * tan_delta) *
                    (u * tau2 * omega * sec2_delta + L * (yaw - target_state.yaw) +
                     tau * (u + a * tau) * tan_delta) +
                (m_weights[0] *
                 (static_cast<value_type>(-2.0) * tau * sec2_delta * sin_yaw * L *
                      (static_cast<value_type>(2.0) * u * tau * omega * lr +
                       (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                           (u2 * tau + (u + a * tau) * lr)) +
                  static_cast<value_type>(4.0) * L2 * (x - target_state.x) +
                  static_cast<value_type>(4.0) * tau * cos_yaw *
                      ((u + a * tau) * L2 - u2 * tau * lr * tan_delta * tan_delta)) *
                 (static_cast<value_type>(-2.0) * tau * sec2_delta * sin_yaw * L *
                      (static_cast<value_type>(2.0) * u * tau * omega * lr +
                       (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                           (u2 * tau + (u + a * tau) * lr)) +
                  static_cast<value_type>(4.0) * L2 * (x - target_state.x) +
                  static_cast<value_type>(4.0) * tau * cos_yaw *
                      ((u + a * tau) * L2 - u2 * tau * lr * tan_delta * tan_delta))) /
                    static_cast<value_type>(16.0) +
                m_weights[1] *
                    (lf * lf * (y + tau * (u + a * tau) * sin_yaw - target_state.y) +
                     lf * (u2 * tau2 * cos_yaw * tan_delta +
                           lr * (static_cast<value_type>(2.0) * y +
                                 static_cast<value_type>(2.0) * tau * (u + a * tau) * sin_yaw -
                                 static_cast<value_type>(2.0) * target_state.y +
                                 tau * cos_yaw *
                                     (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta))) +
                     lr * (u2 * tau2 * cos_yaw_plus_delta * sec_delta * tan_delta +
                           lr * (y + tau * (u + a * tau) * sin_yaw - target_state.y +
                                 tau * cos_yaw *
                                     (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta)))) *
                    (lf * lf * (y + tau * (u + a * tau) * sin_yaw - target_state.y) +
                     lf * (u2 * tau2 * cos_yaw * tan_delta +
                           lr * (static_cast<value_type>(2.0) * y +
                                 static_cast<value_type>(2.0) * tau * (u + a * tau) * sin_yaw -
                                 static_cast<value_type>(2.0) * target_state.y +
                                 tau * cos_yaw *
                                     (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta))) +
                     lr *
                         (u2 * tau2 * cos_yaw_plus_delta * sec_delta * tan_delta +
                          lr * (y + tau * (u + a * tau) * sin_yaw - target_state.y +
                                tau * cos_yaw *
                                    (u * tau * omega * sec2_delta + (u + a * tau) * tan_delta))))) /
               (L3 * L3);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<2>;

    model_type m_model{};
    std::array<value_type, 5> m_weights{
        static_cast<value_type>(1.0), static_cast<value_type>(1.0), static_cast<value_type>(1.0),
        static_cast<value_type>(1.0), static_cast<value_type>(1.0)
    };
};

} // namespace basjoo::bicycle
