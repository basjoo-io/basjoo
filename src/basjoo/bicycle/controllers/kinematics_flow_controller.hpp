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
#include <iterator>
#include <numbers>
#include <span>
#include <stdexcept>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/bicycle/models/kinematics_model.hpp"
#include "basjoo/bicycle/models/state.hpp"

namespace basjoo::bicycle {

template <std::floating_point T>
class KinematicsFlowController final {
  public:
    using value_type = T;
    using state_type = State<value_type>;
    using model_type = ::basjoo::bicycle::KinematicsModel<value_type>;

    KinematicsFlowController() noexcept = default;
    KinematicsFlowController(const KinematicsFlowController& other) noexcept = delete;
    auto operator=(const KinematicsFlowController& other) noexcept
        -> KinematicsFlowController& = delete;
    KinematicsFlowController(KinematicsFlowController&& other) noexcept = default;
    auto operator=(KinematicsFlowController&& other) noexcept
        -> KinematicsFlowController& = default;
    ~KinematicsFlowController() noexcept = default;

    /**
     * @brief Builds a controller from the plant model and the five cost weights.
     *
     * The model carries the geometry and the hard bounds on the two controls, and model() hands it
     * back. The weights follow the state layout and weigh in turn the x, y, yaw, delta and u
     * residuals of the one step prediction against the target, so the a and omega of any target
     * are ignored. Every term is made dimensionless before it is weighed, so the five weights are
     * pure numbers and directly comparable with each other. See tangent() for the cost they enter.
     */
    explicit KinematicsFlowController(
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
     * The plant takes the step KinematicsModel::forwardEuler() takes under the controls it starts
     * with, and the controls move along tangent() evaluated at the start of the step and are then
     * clamped into the bounds the model carries. The horizon has to span at least the step.
     */
    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, const state_type& target_state, value_type horizon, value_type h
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
        if (!(horizon >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires the "
                    "horizon "
                    "to span at least the time step: horizon = {0:f} while h = {1:f}.",
                    horizon, h
                )
            );
        }
#endif
        const state_type k{tangent(initial_state, target_state, horizon)};
        const value_type a{
            std::clamp(initial_state.a + k.a * h, m_model.a_lower_bound(), m_model.a_upper_bound())
        };
        const value_type omega{std::clamp(
            initial_state.omega + k.omega * h, m_model.omega_lower_bound(),
            m_model.omega_upper_bound()
        )};
        initial_state = m_model.forwardEuler(std::move(initial_state), h);
        initial_state.a = a;
        initial_state.omega = omega;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, std::span<const state_type> target_states,
        std::span<const value_type> horizons, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (target_states.size() != hs.size() || horizons.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires the three "
                    "input spans to have identical sizes: target_states.size() = {0:d}, "
                    "horizons.size() = {1:d}, hs.size() = {2:d}.",
                    target_states.size(), horizons.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state =
                forwardEuler(std::move(initial_state), target_states[i], horizons[i], hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, const state_type& target_state,
        std::span<const value_type> horizons, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (horizons.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::forwardEuler() requires the two "
                    "input spans to have identical sizes: horizons.size() = {0:d}, hs.size() "
                    "= {1:d}.",
                    horizons.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state =
                forwardEuler(std::move(initial_state), target_state, horizons[i], hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    /**
     * @brief Advances the closed loop by one step of length h with symplectic Euler.
     *
     * The plant takes the step KinematicsModel::symplecticEuler() takes under the controls it
     * starts with, and the controls move along tangent() evaluated at the start of the step and
     * are then clamped into the bounds the model carries. When
     * the horizon equals h, that staging is also the prediction the tracking cost is built on, up
     * to O(h^2) in v and r: the prediction still extrapolates those two linearly, while the model
     * recomputes them from the fresh u and delta.
     */
    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, const state_type& target_state, value_type horizon, value_type h
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
        if (!(horizon >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires the "
                    "horizon "
                    "to span at least the time step: horizon = {0:f} while h = {1:f}.",
                    horizon, h
                )
            );
        }
#endif
        const state_type k{tangent(initial_state, target_state, horizon)};
        const value_type a{
            std::clamp(initial_state.a + k.a * h, m_model.a_lower_bound(), m_model.a_upper_bound())
        };
        const value_type omega{std::clamp(
            initial_state.omega + k.omega * h, m_model.omega_lower_bound(),
            m_model.omega_upper_bound()
        )};
        initial_state = m_model.symplecticEuler(std::move(initial_state), h);
        initial_state.a = a;
        initial_state.omega = omega;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, std::span<const state_type> target_states,
        std::span<const value_type> horizons, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (target_states.size() != hs.size() || horizons.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires the "
                    "three "
                    "input spans to have identical sizes: target_states.size() = {0:d}, "
                    "horizons.size() = {1:d}, hs.size() = {2:d}.",
                    target_states.size(), horizons.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state =
                symplecticEuler(std::move(initial_state), target_states[i], horizons[i], hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto symplecticEuler(
        state_type initial_state, const state_type& target_state,
        std::span<const value_type> horizons, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (horizons.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::symplecticEuler() requires the two "
                    "input spans to have identical sizes: horizons.size() = {0:d}, hs.size() "
                    "= {1:d}.",
                    horizons.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state =
                symplecticEuler(std::move(initial_state), target_state, horizons[i], hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    /**
     * @brief Advances the closed loop by one step of length h with the classic fourth order
     * Runge-Kutta scheme.
     *
     * Unlike the two Euler overloads, the controls keep moving inside the step: every stage
     * evaluates tangent() afresh, so the plant at each stage runs under the controls that stage has
     * reached, clamped into the bounds the model carries, as is the result of the step. The target
     * is the point of the reference the horizon looks ahead to from the start of the step, and it
     * is held over the step while the vehicle moves on towards it, so every stage hands tangent()
     * the horizon less the stage time, which is what is left to reach that point from where the
     * stage sits. The full horizon would have the later stages predict the vehicle past that point
     * and brake for it, leaving it trailing the reference. The horizon therefore has to span more
     * than the step, which keeps the target ahead of the last stage as well. v and r are
     * recomputed from u and delta at the end of the step, as KinematicsModel does.
     *
     * Only one target comes in per step, so against a reference that moves on over the step the
     * closed loop is followed to first order in h only, as it is by the two Euler overloads:
     * fourth order would take the reference points the later stages look ahead to as well.
     */
    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, const state_type& target_state, value_type horizon, value_type h
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
        if (!(horizon > h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires the horizon "
                    "to span more than the time step: horizon = {0:f} while h = {1:f}.",
                    horizon, h
                )
            );
        }
#endif
        constexpr value_type kHalf{static_cast<value_type>(0.5)};
        constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
        constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
        const state_type k1{tangent(initial_state, target_state, horizon) * h};
        const state_type k2{
            tangent(initial_state + k1 * kHalf, target_state, horizon - h * kHalf) * h
        };
        const state_type k3{
            tangent(initial_state + k2 * kHalf, target_state, horizon - h * kHalf) * h
        };
        const state_type k4{tangent(initial_state + k3, target_state, horizon - h) * h};
        initial_state += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
        initial_state.a =
            std::clamp(initial_state.a, m_model.a_lower_bound(), m_model.a_upper_bound());
        initial_state.omega = std::clamp(
            initial_state.omega, m_model.omega_lower_bound(), m_model.omega_upper_bound()
        );
        initial_state.r = initial_state.u * std::tan(initial_state.delta) / m_model.wheelbase();
        initial_state.v = m_model.rear_wheelbase() * initial_state.r;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, std::span<const state_type> target_states,
        std::span<const value_type> horizons, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (target_states.size() != hs.size() || horizons.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires the three "
                    "input spans to have identical sizes: target_states.size() = {0:d}, "
                    "horizons.size() = {1:d}, hs.size() = {2:d}.",
                    target_states.size(), horizons.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state =
                rungeKutta(std::move(initial_state), target_states[i], horizons[i], hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, const state_type& target_state,
        std::span<const value_type> horizons, std::span<const value_type> hs,
        std::output_iterator<state_type> auto trace
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if (horizons.size() != hs.size()) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::rungeKutta() requires the two "
                    "input spans to have identical sizes: horizons.size() = {0:d}, hs.size() "
                    "= {1:d}.",
                    horizons.size(), hs.size()
                )
            );
        }
#endif
        const std::size_t size{hs.size()};
        for (std::size_t i{0}; i < size; ++i) {
            initial_state = rungeKutta(std::move(initial_state), target_state, horizons[i], hs[i]);
            *trace = initial_state;
            ++trace;
        }
        return initial_state;
    }

    /**
     * @brief Tangent vector of the closed loop flow at the given state.
     *
     * State doubles as its own tangent space here, so the returned object is not a state. Its
     * first seven fields are the plant flow, the same as KinematicsModel::tangent() returns: v
     * and r are not integrated but recomputed from u and delta after every step, so those two
     * fields are zero. Its a and omega fields are what makes this a controller rather than a
     * model: instead of being held still they carry the two controls down a tracking cost. It is
     * only meant to be scaled and added onto a state.
     *
     * The cost compares a one step symplectic Euler prediction over the horizon H against the
     * target state, whose a and omega it ignores. Every term is divided by the wheelbase L and by
     * H until it is dimensionless, which is what leaves the five weights as pure, mutually
     * comparable numbers:
     *
     *     J = w0 (Rx/L)^2 + w1 (Ry/L)^2 + w2 Ryaw^2 + w3 Rdelta^2 + w4 (H Ru/L)^2
     *
     * and the controls follow the preconditioned negative gradient
     *
     *     da/dt = -(L^2/H^5) dJ/da,   domega/dt = -(1/H^3) dJ/domega:
     *
     * for the delta and u terms, an Euler step of length h moves the control by exactly 2 w h / H
     * times the Newton step of that term alone, so a weight of one half is a full Newton step once
     * the step spans the whole horizon. The horizon sets the loop gain and the step only how
     * finely the resulting flow is followed, and shortening the horizon tightens the loop.
     * Likewise, scaling all five weights together scales the gain with them instead of leaving the
     * loop unchanged. Nothing holds the controls back but the residuals they feed into, so an Euler
     * step needs the horizon to outlast it: along a straight line with unit weights on x and u, the
     * longitudinal error decays under forwardEuler() only for H > 1.4196h and under
     * symplecticEuler() only for H > 8h/7.
     *
     * The prediction holds the speed and the heading it reaches at the end of the horizon over the
     * whole horizon, and takes the heading change to first order, so even from a state on the
     * reference it misses where the reference goes over H, and the loop settles off it. Along a
     * reference that accelerates at a and turns at the yaw rate r, the offset is of the order of
     * a H^2/2 + u r^2 H^3/2 behind it and u r H^2/2 to the outside of the turn. Only a shorter
     * horizon shrinks it: the weights merely trade its components against each other, and scaling
     * them all together leaves it where it is.
     *
     * Neither the plant nor the cost reads the v and r of the given state, and both act on its two
     * controls clamped into the bounds the model carries: whatever the caller hands in, and however
     * far an intermediate stage of rungeKutta() has wandered, the flow comes out as it would at the
     * state that is on the constraint and within the bounds.
     */
    [[using gnu: always_inline]]
    auto tangent(
        const state_type& state, const state_type& target_state, value_type horizon
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
        if (!(horizon > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::bicycle::KinematicsFlowController::tangent() requires a positive "
                    "horizon: horizon = {0:f}.",
                    horizon
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
        const value_type horizon2{horizon * horizon};
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
        const value_type sin_yaw_plus_delta{sin_yaw * cos_delta + cos_yaw * sin_delta};
        const value_type cos_yaw_plus_delta{cos_yaw * cos_delta - sin_yaw * sin_delta};
        const value_type a_rate{
            -(static_cast<value_type>(4.0) * horizon * L3 * (a * horizon + u - target_state.u) *
                  m_weights[4] +
              static_cast<value_type>(4.0) * L3 * m_weights[2] * tan_delta *
                  (horizon2 * u * omega * sec2_delta + L * (yaw - target_state.yaw) +
                   horizon * (a * horizon + u) * tan_delta) +
              (static_cast<value_type>(4.0) * (sin_yaw * lf + sec_delta * sin_yaw_plus_delta * lr) *
               m_weights[1] *
               (L * (horizon2 * u * omega * cos_yaw * sec2_delta * lr +
                     horizon * (a * horizon + u) * sin_yaw * L + L * (y - target_state.y)) +
                horizon *
                    (cos_yaw * lf * (horizon * u2 + (a * horizon + u) * lr) +
                     lr * (horizon * u2 * cos_yaw_plus_delta * sec_delta +
                           (a * horizon + u) * cos_yaw * lr)) *
                    tan_delta)) +
              ((cos_yaw * lf + cos_yaw_plus_delta * sec_delta * lr) * m_weights[0] *
               (static_cast<value_type>(-2.0) * horizon * sec2_delta * sin_yaw * L *
                    (static_cast<value_type>(2.0) * horizon * u * omega * lr +
                     (static_cast<value_type>(2.0) * sin_delta * cos_delta) *
                         (horizon * u2 + (a * horizon + u) * lr)) +
                static_cast<value_type>(4.0) * L2 * (x - target_state.x) +
                static_cast<value_type>(4.0) * horizon * cos_yaw *
                    ((a * horizon + u) * L2 - horizon * u2 * lr * tan_delta * tan_delta)))) /
            (static_cast<value_type>(2.0) * horizon2 * horizon * L3)
        };
        const value_type omega_rate{
            -(static_cast<value_type>(4.0) * L3 * L2 * m_weights[3] *
                  (delta + horizon * omega - target_state.delta) +
              static_cast<value_type>(4.0) * horizon * u * sec2_delta * L3 * m_weights[2] *
                  (horizon2 * u * omega * sec2_delta + L * (yaw - target_state.yaw) +
                   horizon * (a * horizon + u) * tan_delta) +
              static_cast<value_type>(4.0) * horizon * u * cos_yaw * sec2_delta * lr *
                  m_weights[1] *
                  (lf * lf * (y + horizon * (a * horizon + u) * sin_yaw - target_state.y) +
                   lf *
                       (horizon2 * u2 * cos_yaw * tan_delta +
                        lr * (static_cast<value_type>(2.0) * y +
                              static_cast<value_type>(2.0) * horizon * (a * horizon + u) * sin_yaw -
                              static_cast<value_type>(2.0) * target_state.y +
                              horizon * cos_yaw *
                                  (horizon * u * omega * sec2_delta +
                                   (a * horizon + u) * tan_delta))) +
                   lr * (horizon2 * u2 * cos_yaw_plus_delta * sec_delta * tan_delta +
                         lr * (y + horizon * (a * horizon + u) * sin_yaw - target_state.y +
                               horizon * cos_yaw *
                                   (horizon * u * omega * sec2_delta +
                                    (a * horizon + u) * tan_delta)))) +
              horizon * u * sec2_delta * sin_yaw * lr * m_weights[0] *
                  (static_cast<value_type>(-4.0) * lf * lf *
                       (x + horizon * (a * horizon + u) * cos_yaw - target_state.x) +
                   static_cast<value_type>(4.0) * lr *
                       (horizon2 * u2 * sec_delta * sin_yaw_plus_delta * tan_delta +
                        lr * (-x - horizon * (a * horizon + u) * cos_yaw + target_state.x +
                              horizon * sin_yaw *
                                  (horizon * u * omega * sec2_delta +
                                   (a * horizon + u) * tan_delta))) +
                   static_cast<value_type>(4.0) * lf *
                       (horizon2 * u2 * sin_yaw * tan_delta +
                        lr * (static_cast<value_type>(-2.0) * x -
                              static_cast<value_type>(2.0) * horizon * (a * horizon + u) * cos_yaw +
                              static_cast<value_type>(2.0) * target_state.x +
                              horizon * sin_yaw *
                                  (horizon * u * omega * sec2_delta +
                                   (a * horizon + u) * tan_delta))))) /
            (static_cast<value_type>(2.0) * horizon2 * L3 * L2)
        };
        const value_type r{u * tan_delta / L};
        const value_type v{lr * r};
        return state_type{
            .x{u * cos_yaw - v * sin_yaw},
            .y{u * sin_yaw + v * cos_yaw},
            .yaw{r},
            .delta{omega},
            .u{a},
            .v{static_cast<value_type>(0.0)},
            .r{static_cast<value_type>(0.0)},
            .a{a_rate},
            .omega{omega_rate}
        };
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
