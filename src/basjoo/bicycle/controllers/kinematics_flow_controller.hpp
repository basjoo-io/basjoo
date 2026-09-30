/**
 * @file kinematics_flow_controller.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
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
 * @brief Plant state of KinematicsModel together with the two controls driving it.
 *
 * x, y, yaw, delta and u are the plant state, a is the longitudinal acceleration du/dt and
 * omega is the steering rate d(delta)/dt. The controls evolve over time alongside the plant,
 * which is what makes this a controller rather than a model.
 */
template <std::floating_point T>
struct KinematicsFlowControllerState final : public ::basjoo::math::DenseArithmeticBase {
    using value_type = T;
    using size_type = std::size_t;
    using rank_type = std::size_t;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr KinematicsFlowControllerState() noexcept = default;

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
    static constexpr auto extent([[maybe_unused]] rank_type r) noexcept -> size_type {
        return size();
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 7;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return &x;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return &x;
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

} // namespace detail

template <std::floating_point T>
class KinematicsFlowController final {
  public:
    using value_type = T;
    using model_type = ::basjoo::bicycle::KinematicsModel<value_type>;

    using state_type = detail::KinematicsFlowControllerState<value_type>;

    KinematicsFlowController() noexcept = default;
    KinematicsFlowController(const KinematicsFlowController& other) noexcept = delete;
    auto operator=(const KinematicsFlowController& other) noexcept
        -> KinematicsFlowController& = delete;
    KinematicsFlowController(KinematicsFlowController&& other) noexcept = default;
    auto operator=(KinematicsFlowController&& other) noexcept
        -> KinematicsFlowController& = default;
    ~KinematicsFlowController() noexcept = default;

    /**
     * @brief Builds a controller from the plant model, the five cost weights and the horizon.
     *
     * The model carries the geometry and the hard bounds on the two controls, and model() hands it
     * back. The weights follow the state layout and weigh in turn the x, y, yaw, delta and u
     * residuals of the one step prediction over the horizon tau against the target, so the a and
     * omega of any target are ignored. Every term is made dimensionless before it is weighed, so
     * the five weights are pure numbers and directly comparable with each other. See tangent() for
     * the cost they enter.
     */
    explicit KinematicsFlowController(
        model_type model, std::span<const value_type, 5> weights, value_type tau
    ) noexcept(!BASJOO_CHECK_PARAMS)
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

    [[using gnu: pure, always_inline]]
    constexpr auto tau() const noexcept -> value_type {
        return m_tau;
    }

    /**
     * @brief Advances the closed loop by one step of length h with forward Euler.
     *
     * The plant and the controls both take one Euler step along tangent() evaluated at the start
     * of the step, which leaves the plant where KinematicsModel::forwardEuler() puts it under the
     * controls the step starts with. The controls are then clamped into the bounds the model
     * carries. The horizon has to span at least the step.
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
        if (!(m_tau >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires the "
                    "horizon to span at least the time step: tau = {0:f} while h = {1:f}.",
                    m_tau, h
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
     * starts with, and the controls move along tangent() evaluated at the start of the step and
     * are then clamped into the bounds the model carries. When the horizon equals h, that staging
     * is also the prediction the tracking cost is built on, up to O(h^2) in the lateral velocity
     * and the yaw rate: the prediction extrapolates those two linearly, while the step recomputes
     * them from the fresh u and delta. As with forwardEuler(), the horizon has to span at least the
     * step.
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
                    "positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(m_tau >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires the "
                    "horizon to span at least the time step: tau = {0:f} while h = {1:f}.",
                    m_tau, h
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
            )
        };
    }

    /**
     * @brief Advances the closed loop by one step of length h with the classic fourth order
     * Runge-Kutta scheme.
     *
     * Unlike the two Euler overloads, the controls keep moving inside the step: every stage
     * evaluates tangent() afresh, so the plant at each stage runs under the controls that stage has
     * reached, clamped into the bounds the model carries, as is the result of the step. The target
     * is the point of the reference the horizon looks ahead to from the start of the step, and it
     * is held over the step while the vehicle moves on towards it, so every stage looks ahead over
     * the horizon less the stage time, which is what is left to reach that point from where the
     * stage sits. The full horizon would have the later stages predict the vehicle past that point
     * and brake for it, leaving it trailing the reference. The horizon therefore has to span more
     * than the step, which keeps the target ahead of the last stage as well.
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
        if (!(m_tau > h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires the horizon "
                    "to span more than the time step: tau = {0:f} while h = {1:f}.",
                    m_tau, h
                )
            );
        }
#endif
        constexpr value_type kHalf{static_cast<value_type>(0.5)};
        constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
        constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
        const state_type k1{tangent(initial_state, target_state, m_tau) * h};
        const state_type k2{
            tangent(initial_state + k1 * kHalf, target_state, m_tau - h * kHalf) * h
        };
        const state_type k3{
            tangent(initial_state + k2 * kHalf, target_state, m_tau - h * kHalf) * h
        };
        const state_type k4{tangent(initial_state + k3, target_state, m_tau - h) * h};
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
     * the two controls of the given state. Its a and omega fields are what makes this a controller
     * rather than a model: instead of being held still they carry the two controls down a tracking
     * cost. It is only meant to be scaled and added onto a state.
     *
     * The cost compares a one step symplectic Euler prediction over the horizon tau against the
     * target state, whose a and omega it ignores. Every term is divided by the wheelbase L and by
     * tau until it is dimensionless, which is what leaves the five weights as pure, mutually
     * comparable numbers:
     *
     *     J = w0 (Rx/L)^2 + w1 (Ry/L)^2 + w2 Ryaw^2 + w3 Rdelta^2 + w4 (tau Ru/L)^2
     *
     * and the controls follow the preconditioned negative gradient
     *
     *     da/dt = -(L^2/(2 tau^5)) dJ/da,   domega/dt = -(1/(2 tau^3)) dJ/domega:
     *
     * for the delta and u terms, an Euler step of length h moves the control by exactly w h / tau
     * times the Newton step of that term alone, so a unit weight is a full Newton step once the
     * step spans the whole horizon. The horizon sets the loop gain and the step only how finely
     * the resulting flow is followed, and shortening the horizon tightens the loop. Likewise,
     * scaling all five weights together scales the gain with them instead of leaving the loop
     * unchanged. Nothing holds the controls back but the residuals they feed into, so an Euler step
     * needs the horizon to outlast it: along a straight line with unit weights on x and u, the
     * longitudinal error decays under forwardEuler() only for tau > h and under symplecticEuler()
     * only for tau > 4h/3.
     *
     * The prediction holds the speed and the heading it reaches at the end of the horizon over the
     * whole horizon and takes the heading change to first order, which leaves every residual
     * affine in a and omega:
     *
     *     u1 = u + a tau,   r = u tan(delta) / L,   v = l_r r,
     *     r1 = (u1 tan(delta) + tau u omega sec^2(delta)) / L,   v1 = l_r r1,
     *     Rx = x - x_ref + tau (u1 cos(yaw) - v1 sin(yaw) - tau r (u sin(yaw) + v cos(yaw))),
     *     Ry = y - y_ref + tau (u1 sin(yaw) + v1 cos(yaw) + tau r (u cos(yaw) - v sin(yaw))),
     *     Ryaw = yaw - yaw_ref + tau r1,   Rdelta = delta - delta_ref + tau omega,
     *     Ru = u1 - u_ref.
     *
     * So even from a state on the reference it misses where the reference goes over tau, and the
     * loop settles off it. Along a reference that accelerates at a and turns at the yaw rate r, the
     * offset is of the order of a tau^2/2 + u r^2 tau^3/2 behind it and u r tau^2/2 to the outside
     * of the turn. Only a shorter horizon shrinks it: the weights merely trade its components
     * against each other, and scaling them all together leaves it where it is.
     *
     * Both the plant and the cost act on the two controls of the given state clamped into the
     * bounds the model carries: whatever the caller hands in, and however far an intermediate stage
     * of rungeKutta() has wandered, the flow comes out as it would at the state within the bounds.
     */
    [[using gnu: always_inline]]
    auto tangent(const state_type& state, const state_type& target_state) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> state_type {
        return tangent(state, target_state, m_tau);
    }

    /**
     * @brief Tracking cost J that tangent() carries the two controls down, at the given state
     * against the target.
     *
     * J compares the one step prediction over the horizon tau against the target state, whose a
     * and omega it ignores, and tangent() lists its terms and the residuals they weigh. As there,
     * the two controls of the given state enter clamped into the bounds the model carries.
     */
    [[using gnu: always_inline]]
    auto cost(const state_type& state, const state_type& target_state) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> value_type {
        return cost(state, target_state, m_tau);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<3>;

    /**
     * @brief cost() over tau instead of the horizon of the controller, as tangent() has it for the
     * later stages of rungeKutta().
     */
    [[using gnu: flatten, leaf]]
    auto cost(
        const state_type& state, const state_type& target_state, value_type tau
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
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

    /**
     * @brief tangent() looking ahead over tau instead of the horizon of the controller, so that the
     * later stages of rungeKutta() look ahead over what is left of it.
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
        // The gradients derived in Mathematica, negated into descent directions, with the 2 of each
        // denominator doubled to halve the preconditioners.
        const value_type a_rate{
            -(static_cast<value_type>(4.0) * tau * L3 * (a * tau + u - target_state.u) *
                  m_weights[4] +
              static_cast<value_type>(4.0) * L3 * m_weights[2] * tan_delta *
                  (tau2 * u * omega * sec2_delta + L * (yaw - target_state.yaw) +
                   tau * (a * tau + u) * tan_delta) +
              (static_cast<value_type>(4.0) * (sin_yaw * lf + sec_delta * sin_yaw_plus_delta * lr) *
               m_weights[1] *
               (L * (tau2 * u * omega * cos_yaw * sec2_delta * lr +
                     tau * (a * tau + u) * sin_yaw * L + L * (y - target_state.y)) +
                tau *
                    (cos_yaw * lf * (tau * u2 + (a * tau + u) * lr) +
                     lr * (tau * u2 * cos_yaw_plus_delta * sec_delta +
                           (a * tau + u) * cos_yaw * lr)) *
                    tan_delta)) +
              ((cos_yaw * lf + cos_yaw_plus_delta * sec_delta * lr) * m_weights[0] *
               (static_cast<value_type>(-2.0) * tau * sec2_delta * sin_yaw * L *
                    (static_cast<value_type>(2.0) * tau * u * omega * lr +
                     (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                         (tau * u2 + (a * tau + u) * lr)) +
                static_cast<value_type>(4.0) * L2 * (x - target_state.x) +
                static_cast<value_type>(4.0) * tau * cos_yaw *
                    ((a * tau + u) * L2 - tau * u2 * lr * tan_delta * tan_delta)))) /
            (static_cast<value_type>(4.0) * tau2 * tau * L3)
        };
        const value_type omega_rate{
            -(static_cast<value_type>(4.0) * L3 * L2 * m_weights[3] *
                  (delta + tau * omega - target_state.delta) +
              static_cast<value_type>(4.0) * tau * u * sec2_delta * L3 * m_weights[2] *
                  (tau2 * u * omega * sec2_delta + L * (yaw - target_state.yaw) +
                   tau * (a * tau + u) * tan_delta) +
              static_cast<value_type>(4.0) * tau * u * cos_yaw * sec2_delta * lr * m_weights[1] *
                  (lf * lf * (y + tau * (a * tau + u) * sin_yaw - target_state.y) +
                   lf * (tau2 * u2 * cos_yaw * tan_delta +
                         lr * (static_cast<value_type>(2.0) * y +
                               static_cast<value_type>(2.0) * tau * (a * tau + u) * sin_yaw -
                               static_cast<value_type>(2.0) * target_state.y +
                               tau * cos_yaw *
                                   (tau * u * omega * sec2_delta + (a * tau + u) * tan_delta))) +
                   lr * (tau2 * u2 * cos_yaw_plus_delta * sec_delta * tan_delta +
                         lr * (y + tau * (a * tau + u) * sin_yaw - target_state.y +
                               tau * cos_yaw *
                                   (tau * u * omega * sec2_delta + (a * tau + u) * tan_delta)))) +
              tau * u * sec2_delta * sin_yaw * lr * m_weights[0] *
                  (static_cast<value_type>(-4.0) * lf * lf *
                       (x + tau * (a * tau + u) * cos_yaw - target_state.x) +
                   static_cast<value_type>(4.0) * lr *
                       (tau2 * u2 * sec_delta * sin_yaw_plus_delta * tan_delta +
                        lr * (-x - tau * (a * tau + u) * cos_yaw + target_state.x +
                              tau * sin_yaw *
                                  (tau * u * omega * sec2_delta + (a * tau + u) * tan_delta))) +
                   static_cast<value_type>(4.0) * lf *
                       (tau2 * u2 * sin_yaw * tan_delta +
                        lr * (static_cast<value_type>(-2.0) * x -
                              static_cast<value_type>(2.0) * tau * (a * tau + u) * cos_yaw +
                              static_cast<value_type>(2.0) * target_state.x +
                              tau * sin_yaw *
                                  (tau * u * omega * sec2_delta + (a * tau + u) * tan_delta))))) /
            (static_cast<value_type>(4.0) * tau2 * L3 * L2)
        };
        const value_type r{u * tan_delta / L};
        const value_type v{lr * r};
        return state_type{
            u * cos_yaw - v * sin_yaw, u * sin_yaw + v * cos_yaw, r, omega, a, a_rate, omega_rate
        };
    }

    model_type m_model{};
    std::array<value_type, 5> m_weights{
        static_cast<value_type>(1.0), static_cast<value_type>(1.0), static_cast<value_type>(1.0),
        static_cast<value_type>(1.0), static_cast<value_type>(1.0)
    };
    value_type m_tau{1.0};
};

} // namespace basjoo::bicycle
