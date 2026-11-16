/**
 * @file quadrotor/controllers/dynamics_flow_controller.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Flow controller of DynamicsModel with a fixed or an adaptive preview horizon.
 * @version 0.1
 * @date 2026-11-13
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
#include <limits>
#include <mdspan>
#include <span>
#include <stdexcept>
#include <utility>

#include "zpp_bits.h"

#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/integrate_mode.hpp"
#include "basjoo/quadrotor/models/dynamics_model.hpp"

namespace basjoo::quadrotor {

/**
 * @brief Plant state of DynamicsModel together with the four controls driving it and, when
 * Adaptive is true, the horizon they are steered over.
 *
 * x, y, z, u, v, w, qx, qy, qz, qw, omega_x, omega_y and omega_z are the plant state, the fields of
 * DynamicsModelState in its order, and torque_x, torque_y, torque_z and thrust the controls, the
 * fields of DynamicsModelControl. tau, present only when Adaptive is true, is the horizon of the
 * prediction the controls are steered by (see DynamicsFlowController::tangent()). The controls, and
 * the horizon where there is one, evolve over time alongside the plant, along the flow of
 * DynamicsFlowController::tangent(), which is what makes this a controller rather than a model.
 *
 * The fields are stored flat, so that the state is a vector in the sense of VecArithmetic and the
 * integrators and the piecewise functions combine it field by field. Default construction gives
 * the default plant state, at rest at the origin under the identity attitude, with all four
 * controls zero and, where there is one, a horizon of one.
 */
template <std::floating_point T, bool Adaptive = false>
class DynamicsFlowControllerState;

/**
 * @brief State of a DynamicsFlowController whose horizon is fixed: the plant state and the four
 * controls.
 */
template <std::floating_point T>
class DynamicsFlowControllerState<T, false> final : public ::basjoo::math::DenseArithmeticBase {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using extents_type = std::extents<index_type, 17>;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr DynamicsFlowControllerState() noexcept = default;

    // Every field set to v, the attitude included. Generic code builds the zero of a value type as
    // value_type{0.0}.
    constexpr explicit DynamicsFlowControllerState(value_type v) noexcept
        : m_data{v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v} {}

    // The fields in storage order, one parameter each.
    // NOLINTBEGIN(readability-function-size)
    constexpr DynamicsFlowControllerState(
        value_type x, value_type y, value_type z, value_type u, value_type v, value_type w,
        value_type qx, value_type qy, value_type qz, value_type qw, value_type omega_x,
        value_type omega_y, value_type omega_z, value_type torque_x, value_type torque_y,
        value_type torque_z, value_type thrust
    ) noexcept
        : m_data{x,  y,       z,       u,       v,        w,        qx,       qy,    qz,
                 qw, omega_x, omega_y, omega_z, torque_x, torque_y, torque_z, thrust} {}
    // NOLINTEND(readability-function-size)

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
        return 17;
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
    constexpr auto z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[2];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto u(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[3];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto v(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[4];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto w(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[5];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qx(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[6];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qy(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[7];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qz(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[8];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qw(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[9];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega_x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[10];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega_y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[11];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega_z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[12];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[13];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[14];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[15];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto thrust(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[16];
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // x, y, z, u, v, w, qx, qy, qz, qw, omega_x, omega_y, omega_z, torque_x, torque_y, torque_z and
    // thrust, in this order.
    std::array<value_type, 17> m_data{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief State of a DynamicsFlowController whose horizon adapts: the plant state, the four
 * controls and the horizon.
 */
template <std::floating_point T>
class DynamicsFlowControllerState<T, true> final : public ::basjoo::math::DenseArithmeticBase {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using extents_type = std::extents<index_type, 18>;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr DynamicsFlowControllerState() noexcept = default;

    // Every field set to v, the attitude and the horizon included. Generic code builds the zero of
    // a value type as value_type{0.0}.
    constexpr explicit DynamicsFlowControllerState(value_type v) noexcept
        : m_data{v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v, v} {}

    // The fields in storage order, one parameter each.
    // NOLINTBEGIN(readability-function-size)
    constexpr DynamicsFlowControllerState(
        value_type x, value_type y, value_type z, value_type u, value_type v, value_type w,
        value_type qx, value_type qy, value_type qz, value_type qw, value_type omega_x,
        value_type omega_y, value_type omega_z, value_type torque_x, value_type torque_y,
        value_type torque_z, value_type thrust, value_type tau
    ) noexcept
        : m_data{x,  y,       z,       u,       v,        w,        qx,       qy,     qz,
                 qw, omega_x, omega_y, omega_z, torque_x, torque_y, torque_z, thrust, tau} {}
    // NOLINTEND(readability-function-size)

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
        return 18;
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
    constexpr auto z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[2];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto u(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[3];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto v(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[4];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto w(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[5];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qx(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[6];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qy(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[7];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qz(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[8];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto qw(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[9];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega_x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[10];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega_y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[11];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto omega_z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[12];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[13];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[14];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[15];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto thrust(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[16];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto tau(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[17];
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // x, y, z, u, v, w, qx, qy, qz, qw, omega_x, omega_y, omega_z, torque_x, torque_y, torque_z,
    // thrust and tau, in this order.
    std::array<value_type, 18> m_data{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0};
};

/**
 * @brief Tracking controller of DynamicsModel whose four controls flow down a tracking cost
 * alongside the plant.
 *
 * Every integrator steps the plant under the controls the state carries and steps the controls
 * along the preconditioned negative gradient of a cost that compares a one step prediction of the
 * plant over a horizon tau against a target (see tangent()). With Adaptive false the horizon is a
 * constant of the controller. With Adaptive true it is a field of the state, kept within bounds
 * the controller carries, and descends the same cost alongside the controls, and at the horizon a
 * state carries the controls are steered exactly as by the controller with that fixed horizon.
 */
template <std::floating_point T, bool Adaptive = false>
class DynamicsFlowController final {
  public:
    using value_type = T;
    using model_type = ::basjoo::quadrotor::DynamicsModel<value_type>;

    using state_type = DynamicsFlowControllerState<value_type, Adaptive>;

    DynamicsFlowController() noexcept = default;
    DynamicsFlowController(const DynamicsFlowController& other) noexcept = delete;
    auto operator=(const DynamicsFlowController& other) noexcept
        -> DynamicsFlowController& = delete;
    DynamicsFlowController(DynamicsFlowController&& other) noexcept = default;
    auto operator=(DynamicsFlowController&& other) noexcept -> DynamicsFlowController& = default;
    ~DynamicsFlowController() noexcept = default;

    /**
     * @brief Builds a controller from the plant model, the four cost weights and the horizon.
     *
     * The model carries the mass, the inertia, gravity and the hard bounds on the four controls,
     * and model() hands it back. The weights weigh in turn the position, body velocity, attitude
     * and angular velocity residuals of the prediction against the target, so the controls of any
     * target are ignored. Every term is made dimensionless by the length L0 = sqrt(J_xx / m) and
     * the time T0 = sqrt(L0 / g) of the model before it is weighed, so the four weights are pure
     * numbers and directly comparable with each other, which is why gravity has to be positive.
     * One weight covers the three components of a vector together, which leaves the cost
     * independent of how the world frame is turned about its vertical. See tangent() for the cost
     * they enter.
     */
    explicit DynamicsFlowController(
        model_type model, std::span<const value_type, 4> weights, value_type tau
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires(!Adaptive)
        : m_model{std::move(model)}, m_tau{tau} {
#if BASJOO_CHECK_PARAMS == 1
        if (!(tau > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsFlowController "
                    "requires a positive horizon: tau = {0:f}.",
                    tau
                )
            );
        }
        if (!(m_model.gravity() > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsFlowController "
                    "requires a model with a positive gravity, which the unit time of its cost "
                    "T0 = sqrt(L0 / g) is taken from: gravity = {0:f}.",
                    m_model.gravity()
                )
            );
        }
#endif
        std::ranges::copy_n(weights.begin(), 4, m_weights.begin());
    }

    /**
     * @brief Builds a controller whose horizon adapts from the plant model, the four cost weights
     * and the bounds the horizon is kept within.
     *
     * The horizon comes with the state instead, and the controls and the horizon of any target are
     * ignored. The weights are those of the controller with a fixed horizon. The upper bound may be
     * infinite, while a lower bound of zero leaves the horizon free to sink as far as the checks of
     * the integrators let it.
     */
    explicit DynamicsFlowController(
        model_type model, std::span<const value_type, 4> weights, value_type tau_lower_bound,
        value_type tau_upper_bound
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires Adaptive
        : m_model{std::move(model)}, m_tau_lower{tau_lower_bound}, m_tau_upper{tau_upper_bound} {
#if BASJOO_CHECK_PARAMS == 1
        if (!(0.0 <= tau_lower_bound && tau_lower_bound <= tau_upper_bound)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsFlowController "
                    "requires horizon bounds with 0 <= tau_lower_bound <= tau_upper_bound: "
                    "tau_lower_bound = {0:f} while tau_upper_bound = {1:f}.",
                    tau_lower_bound, tau_upper_bound
                )
            );
        }
        if (!(m_model.gravity() > 0.0)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsFlowController "
                    "requires a model with a positive gravity, which the unit time of its cost "
                    "T0 = sqrt(L0 / g) is taken from: gravity = {0:f}.",
                    m_model.gravity()
                )
            );
        }
#endif
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
     * @brief Lowest horizon a state is clamped to. Only a controller whose horizon adapts has one.
     */
    [[using gnu: pure, always_inline]]
    constexpr auto tau_lower_bound() const noexcept -> value_type
        requires Adaptive
    {
        return m_tau_lower;
    }

    /**
     * @brief Highest horizon a state is clamped to. Only a controller whose horizon adapts has one.
     */
    [[using gnu: pure, always_inline]]
    constexpr auto tau_upper_bound() const noexcept -> value_type
        requires Adaptive
    {
        return m_tau_upper;
    }

    /**
     * @brief Advances the closed loop by one step of length h with forward Euler.
     *
     * The plant, the controls and, when Adaptive is true, the horizon all take one Euler step
     * along tangent() evaluated at the start of the step, which leaves the plant where
     * DynamicsModel::forwardEuler() puts it under the controls the step starts with, its attitude
     * normalized at the end of the step as there. The controls are then clamped into the bounds the
     * model carries, and an adaptive horizon into the bounds of the controller. The horizon the
     * step starts with has to span at least the step.
     */
    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, const state_type& target_state, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        // |q|^2 within sqrt(epsilon) of one, about half the digits.
        const value_type attitude_sqr{
            initial_state.qx() * initial_state.qx() + initial_state.qy() * initial_state.qy() +
            initial_state.qz() * initial_state.qz() + initial_state.qw() * initial_state.qw()
        };
        if (!(std::abs(attitude_sqr - static_cast<value_type>(1.0)) <=
              std::sqrt(std::numeric_limits<value_type>::epsilon()))) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsFlowController::forwardEuler() requires the "
                    "attitude of initial_state to be a unit quaternion: |q|^2 = {0:f}.",
                    attitude_sqr
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsFlowController::forwardEuler() requires a positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(horizonOf(initial_state) >= h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsFlowController::forwardEuler() requires the "
                    "horizon to span at least the time step: tau = {0:f} while h = {1:f}.",
                    horizonOf(initial_state), h
                )
            );
        }
#endif
        initial_state += tangent(initial_state, target_state, horizonOf(initial_state)) * h;
        const value_type norm{std::sqrt(
            initial_state.qx() * initial_state.qx() + initial_state.qy() * initial_state.qy() +
            initial_state.qz() * initial_state.qz() + initial_state.qw() * initial_state.qw()
        )};
        initial_state.qx() /= norm;
        initial_state.qy() /= norm;
        initial_state.qz() /= norm;
        initial_state.qw() /= norm;
        initial_state.torque_x() = std::clamp(
            initial_state.torque_x(), m_model.control_lower_bound().torque_x(),
            m_model.control_upper_bound().torque_x()
        );
        initial_state.torque_y() = std::clamp(
            initial_state.torque_y(), m_model.control_lower_bound().torque_y(),
            m_model.control_upper_bound().torque_y()
        );
        initial_state.torque_z() = std::clamp(
            initial_state.torque_z(), m_model.control_lower_bound().torque_z(),
            m_model.control_upper_bound().torque_z()
        );
        initial_state.thrust() = std::clamp(
            initial_state.thrust(), m_model.control_lower_bound().thrust(),
            m_model.control_upper_bound().thrust()
        );
        if constexpr (Adaptive) {
            initial_state.tau() = std::clamp(initial_state.tau(), m_tau_lower, m_tau_upper);
        }
        return initial_state;
    }

    /**
     * @brief Advances the closed loop by one step of length h with the classic fourth order
     * Runge-Kutta scheme.
     *
     * Unlike forwardEuler(), the controls and, when Adaptive is true, the horizon keep moving
     * inside the step: every stage evaluates tangent() afresh on the state that stage has reached,
     * so the plant at each stage runs under the controls of that stage, clamped into the bounds the
     * model carries, as is the result of the step. The target is the point of the reference the
     * horizon looks ahead to from the start of the step, and it is held over the step while the
     * vehicle moves on towards it, so every stage looks ahead over its horizon, clamped into the
     * bounds where it adapts, less its stage time, which is what is left to reach that point from
     * where the stage sits. The stage time enters the stages alone: an adaptive horizon itself
     * moves by its rate only, and is clamped into its bounds at the end of the step. The stages are
     * left off unit norm, which tangent() takes, and only the attitude of the step is normalized.
     * The horizon the step starts with has to span more than the step, which keeps the target
     * ahead of the last stage as well.
     *
     * Only one target comes in per step, so against a reference that moves on over the step the
     * closed loop is followed to first order in h only, as it is by forwardEuler(): fourth order
     * would take the reference points the later stages look ahead to as well.
     */
    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, const state_type& target_state, value_type h
    ) const noexcept(!BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        // |q|^2 within sqrt(epsilon) of one, about half the digits.
        const value_type attitude_sqr{
            initial_state.qx() * initial_state.qx() + initial_state.qy() * initial_state.qy() +
            initial_state.qz() * initial_state.qz() + initial_state.qw() * initial_state.qw()
        };
        if (!(std::abs(attitude_sqr - static_cast<value_type>(1.0)) <=
              std::sqrt(std::numeric_limits<value_type>::epsilon()))) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsFlowController::rungeKutta() requires the attitude "
                    "of initial_state to be a unit quaternion: |q|^2 = {0:f}.",
                    attitude_sqr
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsFlowController::rungeKutta() requires a positive "
                    "time step: h = {0:f}.",
                    h
                )
            );
        }
        if (!(horizonOf(initial_state) > h)) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsFlowController::rungeKutta() requires the horizon "
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
        const value_type norm{std::sqrt(
            initial_state.qx() * initial_state.qx() + initial_state.qy() * initial_state.qy() +
            initial_state.qz() * initial_state.qz() + initial_state.qw() * initial_state.qw()
        )};
        initial_state.qx() /= norm;
        initial_state.qy() /= norm;
        initial_state.qz() /= norm;
        initial_state.qw() /= norm;
        initial_state.torque_x() = std::clamp(
            initial_state.torque_x(), m_model.control_lower_bound().torque_x(),
            m_model.control_upper_bound().torque_x()
        );
        initial_state.torque_y() = std::clamp(
            initial_state.torque_y(), m_model.control_lower_bound().torque_y(),
            m_model.control_upper_bound().torque_y()
        );
        initial_state.torque_z() = std::clamp(
            initial_state.torque_z(), m_model.control_lower_bound().torque_z(),
            m_model.control_upper_bound().torque_z()
        );
        initial_state.thrust() = std::clamp(
            initial_state.thrust(), m_model.control_lower_bound().thrust(),
            m_model.control_upper_bound().thrust()
        );
        if constexpr (Adaptive) {
            initial_state.tau() = std::clamp(initial_state.tau(), m_tau_lower, m_tau_upper);
        }
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
                    "basjoo::quadrotor::DynamicsFlowController::evolve() requires the two input "
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
                    "basjoo::quadrotor::DynamicsFlowController::evolve() requires the trace to be "
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
     * first thirteen fields are the plant flow, the same as DynamicsModel::tangent() returns under
     * the four controls of the given state. Its four control fields, and its tau field when
     * Adaptive is true, carry the controller down the tracking cost
     *
     *     J = (w0 |Rr|^2 / L0^2 + w1 T0^2 |Rv|^2 / L0^2 + w2 |Rq|^2 + w3 T0^2 |Rw|^2) / 2,
     *
     * which compares a one step prediction of the plant over the horizon tau against the target
     * state, ignoring the controls and the horizon of the target. Lengths are measured in
     * L0 = sqrt(J_xx / m), speeds in L0 / T0 and angular velocities in 1 / T0, where
     * T0 = sqrt(L0 / g), so every term is dimensionless: L0 and T0 are the length and the time the
     * mass, the roll inertia and gravity make up, and L0 = g T0^2. Neither unit moves with the
     * horizon. The prediction is the Taylor expansion of the flow of DynamicsModel under the four
     * controls held over the horizon, each residual taken to the order at which every control that
     * reaches it has appeared: the position to tau^4 and the body velocity to tau^3, where the
     * torques first tilt the thrust, the attitude to tau^2 and the angular velocity to tau. The
     * expansion is exact through those orders, the terms in which the thrust multiplies a torque
     * included, which leaves every residual affine in the thrust for given torques and in the
     * torques for a given thrust:
     *
     *     alpha = J^-1 (torque - omega x J omega),
     *     alpha' = J^-1 (-alpha x J omega - omega x J alpha),
     *     lift = thrust / m,   b = R^T e_z,
     *     A = lift R e_z - g e_z,   A' = lift R (omega x e_z),
     *     A'' = lift R (omega x (omega x e_z) + alpha x e_z),
     *     v' = -omega x v + lift e_z - g b,   v'' = -alpha x v - omega x v' + g omega x b,
     *     v''' = -alpha' x v - 2 alpha x v' - omega x v'' + g alpha x b - g omega x (omega x b),
     *     q' = q (omega, 0) / 2,   q'' = q (alpha, 0) / 2 - |omega|^2 q / 4,
     *     Rr = r - r_ref + tau R v + tau^2/2 A + tau^3/6 A' + tau^4/24 A'',
     *     Rv = v - v_ref + tau v' + tau^2/2 v'' + tau^3/6 v''',
     *     Rq = 2 (q + tau q' + tau^2/2 q'' - s q_ref),   Rw = omega - omega_ref + tau alpha.
     *
     * r is the position in the world frame, v the velocity in the body frame, q the attitude,
     * omega the angular velocity in the body frame and J the inertia tensor. The velocities are
     * compared field by field, in the body frames of the prediction and of the target. s, the sign
     * of q . q_ref, picks out of q_ref and -q_ref, one rotation either way, the one nearer the
     * attitude, so that Rq is the difference of two neighbouring quaternions, about the angle
     * between them in radians while that is small. Under a fixed attitude and a constant thrust
     * the prediction is exact outright.
     *
     * The controls follow the preconditioned negative gradient
     *
     *     d(torque_i)/dt = -(J_ii^2 / (T0^2 tau^3)) dJ/d(torque_i),
     *     d(thrust)/dt = -(m^2 g^2 T0^2 / tau^3) dJ/d(thrust):
     *
     * an Euler step of length h moves a torque by exactly w3 h / tau times the Newton step of the
     * angular velocity term alone, and, while the vehicle neither rotates nor is torqued, the
     * thrust by w1 h / tau times that of the velocity term alone, so a unit weight is a full Newton
     * step once the step spans the whole horizon. In the dimensionless controls torque_i T0^2 /
     * J_ii and thrust / (m g) and the dimensionless time t / T0, all four preconditioners read (T0
     * / tau)^3. Measured in L0 and T0, the closed loop about hover depends on the weights and on
     * tau / T0 alone: the inertias cancel against the preconditioners, and gravity comes to one.
     * Its horizontal part runs from the torques through the angular velocity and the attitude into
     * the position, which the position term reaches at tau^4 / 24 only, so the velocity and
     * attitude terms are what damp it. At tau = 2.5 T0, unit weights leave the loop all but
     * undamped, its slowest mode decaying as exp(-0.008 t / T0), while the weights 0.1, 1.5, 6.5
     * and 0.2 leave its slowest mode decaying as exp(-0.061 t / T0) and no pair damped at a ratio
     * below 0.59. Nothing holds the controls back but the residuals they feed into, so an Euler
     * step needs the horizon to outlast it: along a vertical line with the velocity weighed alone,
     * the vertical velocity error decays under forwardEuler() only for tau > h, which is why that
     * takes no horizon shorter than the step.
     *
     * When Adaptive is true the horizon descends the same cost,
     *
     *     dtau/dt = -tau dJ/dtau,
     *
     * preconditioned by itself, and every integrator clamps it into [tau_lower_bound(),
     * tau_upper_bound()] at the end of a step, as tangent() and cost() clamp the horizon of the
     * state before they use it. As the units do not move with the horizon, dJ/dtau weighs the
     * residuals against their derivatives by the horizon alone. Off the reference those are large,
     * so the horizon runs to a bound within a fraction of a second and tends to stay on one while
     * the offset closes: there the bounds set the horizon rather than the flow. Nothing in the cost
     * ties the horizon to the time the target actually lies ahead: flying level and straight at a
     * target with its velocity and attitude, without rotating, at the hover thrust and without
     * torques, a distance d short of it, tau = d / |v| zeroes every residual and all five rates
     * whatever d. The loop therefore does not close an along track offset, and the horizon takes it
     * up instead, as far as its bounds let it. A caller that takes the target the horizon of the
     * state ahead along the reference, rather than a fixed time ahead, ties the two together, and
     * the loop then does close the offset.
     *
     * Waypoints sparser than the step are to be interpolated at the time the horizon looks ahead
     * to: the nearest waypoint lies up to half their spacing off that time, which shifts the target
     * along track, and an adaptive horizon takes that offset up like any other.
     *
     * Both the plant and the cost act on the four controls of the given state clamped into the
     * bounds the model carries and, when Adaptive is true, on its horizon clamped into the bounds
     * of the controller: whatever the caller hands in, and however far an intermediate stage of
     * rungeKutta() has wandered, the flow comes out as it would at the state within the bounds.
     */
    [[using gnu: always_inline]]
    auto tangent(const state_type& state, const state_type& target_state) const noexcept(
        !Adaptive || !BASJOO_CHECK_PARAMS
    ) -> state_type {
        return tangent(state, target_state, horizonOf(state));
    }

    /**
     * @brief Tracking cost J that tangent() carries the controller down, at the given state against
     * the target.
     *
     * J compares the one step prediction over the horizon, the controller's own or, when Adaptive
     * is true, the one the state carries, against the target state, whose controls and horizon it
     * ignores. tangent() lists its terms and the residuals they weigh. As there, the four controls
     * of the given state enter clamped into the bounds the model carries, and an adaptive horizon
     * clamped into the bounds of the controller.
     */
    [[using gnu: flatten, leaf]]
    auto cost(const state_type& state, const state_type& target_state) const noexcept(
        !Adaptive || !BASJOO_CHECK_PARAMS
    ) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if constexpr (Adaptive) {
            if (!(horizonOf(state) > 0.0)) [[unlikely]] {
                throw std::invalid_argument(
                    std::format(
                        "Invalid arguments detected! "
                        "basjoo::quadrotor::DynamicsFlowController::cost() requires a positive "
                        "horizon: tau = {0:f}.",
                        horizonOf(state)
                    )
                );
            }
        }
#endif
        const value_type tau{horizonOf(state)};
        const Expansion expansion{expand(state, target_state, tau)};
        return kHalf * weigh(expansion.value, expansion.value);
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<5>;

    static constexpr value_type kHalf{static_cast<value_type>(0.5)};
    static constexpr value_type kQuarter{static_cast<value_type>(0.25)};
    static constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
    static constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
    static constexpr value_type kTwentyFourth{static_cast<value_type>(1.0 / 24.0)};
    static constexpr value_type kOne{static_cast<value_type>(1.0)};
    static constexpr value_type kTwo{static_cast<value_type>(2.0)};

    /**
     * @brief One value per plant field: a residual of the prediction, a partial derivative of the
     * residuals, or the plant flow.
     */
    struct Residuals final {
        value_type x{0.0};
        value_type y{0.0};
        value_type z{0.0};
        value_type u{0.0};
        value_type v{0.0};
        value_type w{0.0};
        value_type qx{0.0};
        value_type qy{0.0};
        value_type qz{0.0};
        value_type qw{0.0};
        value_type omega_x{0.0};
        value_type omega_y{0.0};
        value_type omega_z{0.0};
    };

    /**
     * @brief What expand() derives at a state: the plant flow, the residuals at the horizon and
     * their partial derivatives by the four controls and by the horizon, the last left at zero
     * unless Adaptive is true.
     */
    struct Expansion final {
        Residuals flow{};
        Residuals value{};
        Residuals by_torque_x{};
        Residuals by_torque_y{};
        Residuals by_torque_z{};
        Residuals by_thrust{};
        Residuals by_tau{};
    };

    /**
     * @brief Horizon the prediction runs over at the given state: the controller's own, or the one
     * the state carries, clamped into the bounds of the controller, when Adaptive is true.
     */
    [[using gnu: pure, always_inline]]
    constexpr auto horizonOf([[maybe_unused]] const state_type& state) const noexcept
        -> value_type {
        if constexpr (Adaptive) {
            return std::clamp(state.tau(), m_tau_lower, m_tau_upper);
        } else {
            return m_tau;
        }
    }

    /**
     * @brief Weighted inner product of two sets of residuals, lengths measured in L0 and times in
     * T0 of the model.
     *
     * weigh(R, R) is twice the cost J. Paired with a partial derivative of the residuals by a
     * control, or by the horizon, it is the matching partial derivative of J, as neither unit
     * moves with the horizon.
     */
    [[using gnu: pure, always_inline]]
    auto weigh(const Residuals& lhs, const Residuals& rhs) const noexcept -> value_type {
        const value_type m{m_model.mass()};
        const value_type g{m_model.gravity()};
        const value_type jxx{m_model.inertia_xx()};
        const value_type length{std::sqrt(jxx / m)};
        // 1 / L0^2 = m / J_xx, 1 / (L0 / T0)^2 = 1 / (g L0) and T0^2 = L0 / g.
        return m_weights[0] * m / jxx * (lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z) +
               m_weights[1] / (g * length) * (lhs.u * rhs.u + lhs.v * rhs.v + lhs.w * rhs.w) +
               m_weights[2] *
                   (lhs.qx * rhs.qx + lhs.qy * rhs.qy + lhs.qz * rhs.qz + lhs.qw * rhs.qw) +
               m_weights[3] * length / g *
                   (lhs.omega_x * rhs.omega_x + lhs.omega_y * rhs.omega_y +
                    lhs.omega_z * rhs.omega_z);
    }

    /**
     * @brief Residuals of the prediction over tau against the target, their partial derivatives
     * and the plant flow, all under the four controls of the state clamped into the bounds.
     *
     * Every residual is a polynomial of degree four at most in tau whose coefficients are affine in
     * the thrust for given torques and in the torques for a given thrust, so its partial derivative
     * by a control follows from those of the coefficients, taken at the other controls of the
     * state, and its derivative by the horizon, which only an adaptive horizon needs, from the
     * powers of tau. The coefficients of tau make up the plant flow itself, which tangent() hands
     * on as it is. The rotation and the plant flow are spelled out as DynamicsModel::tangent()
     * spells them.
     */
    // NOLINTBEGIN(readability-function-size)
    [[using gnu: pure, always_inline]]
    auto expand(
        const state_type& state, const state_type& target_state, value_type tau
    ) const noexcept -> Expansion {
        const value_type torque_x{std::clamp(
            state.torque_x(), m_model.control_lower_bound().torque_x(),
            m_model.control_upper_bound().torque_x()
        )};
        const value_type torque_y{std::clamp(
            state.torque_y(), m_model.control_lower_bound().torque_y(),
            m_model.control_upper_bound().torque_y()
        )};
        const value_type torque_z{std::clamp(
            state.torque_z(), m_model.control_lower_bound().torque_z(),
            m_model.control_upper_bound().torque_z()
        )};
        const value_type thrust{std::clamp(
            state.thrust(), m_model.control_lower_bound().thrust(),
            m_model.control_upper_bound().thrust()
        )};
        const value_type m{m_model.mass()};
        const value_type jxx{m_model.inertia_xx()};
        const value_type jyy{m_model.inertia_yy()};
        const value_type jzz{m_model.inertia_zz()};
        const value_type g{m_model.gravity()};
        const value_type qx{state.qx()};
        const value_type qy{state.qy()};
        const value_type qz{state.qz()};
        const value_type qw{state.qw()};
        const value_type r00{static_cast<value_type>(1.0) - kTwo * (qy * qy + qz * qz)};
        const value_type r01{kTwo * (qx * qy - qz * qw)};
        const value_type r02{kTwo * (qx * qz + qy * qw)};
        const value_type r10{kTwo * (qx * qy + qz * qw)};
        const value_type r11{static_cast<value_type>(1.0) - kTwo * (qx * qx + qz * qz)};
        const value_type r12{kTwo * (qy * qz - qx * qw)};
        const value_type r20{kTwo * (qx * qz - qy * qw)};
        const value_type r21{kTwo * (qy * qz + qx * qw)};
        const value_type r22{static_cast<value_type>(1.0) - kTwo * (qx * qx + qy * qy)};
        const value_type u{state.u()};
        const value_type v{state.v()};
        const value_type w{state.w()};
        const value_type omega_x{state.omega_x()};
        const value_type omega_y{state.omega_y()};
        const value_type omega_z{state.omega_z()};

        // The plant flow, as DynamicsModel::tangent() has it: the world velocity R v, the body
        // acceleration v', dq/dt = q', and the angular acceleration alpha.
        const value_type vx{r00 * u + r01 * v + r02 * w};
        const value_type vy{r10 * u + r11 * v + r12 * w};
        const value_type vz{r20 * u + r21 * v + r22 * w};
        const value_type v1_x{omega_z * v - omega_y * w - g * r20};
        const value_type v1_y{omega_x * w - omega_z * u - g * r21};
        const value_type v1_z{omega_y * u - omega_x * v + thrust / m - g * r22};
        const value_type q1_x{
            static_cast<value_type>(0.5) * (qw * omega_x + qy * omega_z - qz * omega_y)
        };
        const value_type q1_y{
            static_cast<value_type>(0.5) * (qw * omega_y + qz * omega_x - qx * omega_z)
        };
        const value_type q1_z{
            static_cast<value_type>(0.5) * (qw * omega_z + qx * omega_y - qy * omega_x)
        };
        const value_type q1_w{
            static_cast<value_type>(-0.5) * (qx * omega_x + qy * omega_y + qz * omega_z)
        };
        const value_type alpha_x{(torque_x + (jyy - jzz) * omega_y * omega_z) / jxx};
        const value_type alpha_y{(torque_y + (jzz - jxx) * omega_z * omega_x) / jyy};
        const value_type alpha_z{(torque_z + (jxx - jyy) * omega_x * omega_y) / jzz};

        // alpha' under held torques: the derivative of the gyroscopic terms alone.
        const value_type alpha1_x{(jyy - jzz) * (alpha_y * omega_z + omega_y * alpha_z) / jxx};
        const value_type alpha1_y{(jzz - jxx) * (alpha_z * omega_x + omega_z * alpha_x) / jyy};
        const value_type alpha1_z{(jxx - jyy) * (alpha_x * omega_y + omega_x * alpha_y) / jzz};

        // The world acceleration A and its derivatives are lift times the world images of e_z,
        // omega x e_z = (omega_y, -omega_x, 0) and omega x (omega x e_z) + alpha x e_z = e2, the
        // last less g e_z in A.
        const value_type lift{thrust / m};
        const value_type e2_x{omega_x * omega_z + alpha_y};
        const value_type e2_y{omega_y * omega_z - alpha_x};
        const value_type e2_z{-(omega_x * omega_x + omega_y * omega_y)};
        const value_type c1_x{r00 * omega_y - r01 * omega_x};
        const value_type c1_y{r10 * omega_y - r11 * omega_x};
        const value_type c1_z{r20 * omega_y - r21 * omega_x};
        const value_type c2_x{r00 * e2_x + r01 * e2_y + r02 * e2_z};
        const value_type c2_y{r10 * e2_x + r11 * e2_y + r12 * e2_z};
        const value_type c2_z{r20 * e2_x + r21 * e2_y + r22 * e2_z};

        // The body velocity: b = R^T e_z = (r20, r21, r22), and the cross products of v'' and v'''.
        const value_type wb_x{omega_y * r22 - omega_z * r21};
        const value_type wb_y{omega_z * r20 - omega_x * r22};
        const value_type wb_z{omega_x * r21 - omega_y * r20};
        const value_type v2_x{
            -(alpha_y * w - alpha_z * v) - (omega_y * v1_z - omega_z * v1_y) + g * wb_x
        };
        const value_type v2_y{
            -(alpha_z * u - alpha_x * w) - (omega_z * v1_x - omega_x * v1_z) + g * wb_y
        };
        const value_type v2_z{
            -(alpha_x * v - alpha_y * u) - (omega_x * v1_y - omega_y * v1_x) + g * wb_z
        };
        const value_type v3_x{
            -(alpha1_y * w - alpha1_z * v) - kTwo * (alpha_y * v1_z - alpha_z * v1_y) -
            (omega_y * v2_z - omega_z * v2_y) + g * (alpha_y * r22 - alpha_z * r21) -
            g * (omega_y * wb_z - omega_z * wb_y)
        };
        const value_type v3_y{
            -(alpha1_z * u - alpha1_x * w) - kTwo * (alpha_z * v1_x - alpha_x * v1_z) -
            (omega_z * v2_x - omega_x * v2_z) + g * (alpha_z * r20 - alpha_x * r22) -
            g * (omega_z * wb_x - omega_x * wb_z)
        };
        const value_type v3_z{
            -(alpha1_x * v - alpha1_y * u) - kTwo * (alpha_x * v1_y - alpha_y * v1_x) -
            (omega_x * v2_y - omega_y * v2_x) + g * (alpha_x * r21 - alpha_y * r20) -
            g * (omega_x * wb_y - omega_y * wb_x)
        };

        // The attitude: q'' = q (alpha, 0) / 2 - |omega|^2 q / 4, and the sign of the target.
        const value_type omega_sqr{omega_x * omega_x + omega_y * omega_y + omega_z * omega_z};
        const value_type q2_x{
            kHalf * (qw * alpha_x + qy * alpha_z - qz * alpha_y) - kQuarter * omega_sqr * qx
        };
        const value_type q2_y{
            kHalf * (qw * alpha_y + qz * alpha_x - qx * alpha_z) - kQuarter * omega_sqr * qy
        };
        const value_type q2_z{
            kHalf * (qw * alpha_z + qx * alpha_y - qy * alpha_x) - kQuarter * omega_sqr * qz
        };
        const value_type q2_w{
            -kHalf * (qx * alpha_x + qy * alpha_y + qz * alpha_z) - kQuarter * omega_sqr * qw
        };
        const value_type alignment{
            qx * target_state.qx() + qy * target_state.qy() + qz * target_state.qz() +
            qw * target_state.qw()
        };
        const value_type sign{alignment < static_cast<value_type>(0.0) ? -kOne : kOne};

        // The partial derivatives of alpha by the torques are 1 / J_ii on the diagonal, and those
        // of alpha' follow from its gyroscopic products.
        const value_type dalpha_x{kOne / jxx};
        const value_type dalpha_y{kOne / jyy};
        const value_type dalpha_z{kOne / jzz};
        const value_type dalpha1_y_by_torque_x{(jzz - jxx) * omega_z * dalpha_x / jyy};
        const value_type dalpha1_z_by_torque_x{(jxx - jyy) * omega_y * dalpha_x / jzz};
        const value_type dalpha1_x_by_torque_y{(jyy - jzz) * omega_z * dalpha_y / jxx};
        const value_type dalpha1_z_by_torque_y{(jxx - jyy) * omega_x * dalpha_y / jzz};
        const value_type dalpha1_x_by_torque_z{(jyy - jzz) * omega_y * dalpha_z / jxx};
        const value_type dalpha1_y_by_torque_z{(jzz - jxx) * omega_x * dalpha_z / jyy};

        // torque_x: dalpha = (dalpha_x, 0, 0), so dv'' = -dalpha x v = (0, dalpha_x w,
        // -dalpha_x v) and dv''' = -dalpha' x v - 2 dalpha x v' - omega x dv'' + g dalpha x b.
        const value_type dv2_y_by_torque_x{dalpha_x * w};
        const value_type dv2_z_by_torque_x{-dalpha_x * v};
        const value_type dv3_x_by_torque_x{
            -(dalpha1_y_by_torque_x * w - dalpha1_z_by_torque_x * v) -
            (omega_y * dv2_z_by_torque_x - omega_z * dv2_y_by_torque_x)
        };
        const value_type dv3_y_by_torque_x{
            -dalpha1_z_by_torque_x * u + kTwo * dalpha_x * v1_z - (-omega_x * dv2_z_by_torque_x) -
            g * dalpha_x * r22
        };
        const value_type dv3_z_by_torque_x{
            dalpha1_y_by_torque_x * u - kTwo * dalpha_x * v1_y - (omega_x * dv2_y_by_torque_x) +
            g * dalpha_x * r21
        };
        // torque_y: dalpha = (0, dalpha_y, 0), so dv'' = (-dalpha_y w, 0, dalpha_y u).
        const value_type dv2_x_by_torque_y{-dalpha_y * w};
        const value_type dv2_z_by_torque_y{dalpha_y * u};
        const value_type dv3_x_by_torque_y{
            dalpha1_z_by_torque_y * v - kTwo * dalpha_y * v1_z - (omega_y * dv2_z_by_torque_y) +
            g * dalpha_y * r22
        };
        const value_type dv3_y_by_torque_y{
            -(dalpha1_z_by_torque_y * u - dalpha1_x_by_torque_y * w) -
            (omega_z * dv2_x_by_torque_y - omega_x * dv2_z_by_torque_y)
        };
        const value_type dv3_z_by_torque_y{
            -dalpha1_x_by_torque_y * v + kTwo * dalpha_y * v1_x - (-omega_y * dv2_x_by_torque_y) -
            g * dalpha_y * r20
        };
        // torque_z: dalpha = (0, 0, dalpha_z), so dv'' = (dalpha_z v, -dalpha_z u, 0).
        const value_type dv2_x_by_torque_z{dalpha_z * v};
        const value_type dv2_y_by_torque_z{-dalpha_z * u};
        const value_type dv3_x_by_torque_z{
            -dalpha1_y_by_torque_z * w + kTwo * dalpha_z * v1_y - (-omega_z * dv2_y_by_torque_z) -
            g * dalpha_z * r21
        };
        const value_type dv3_y_by_torque_z{
            dalpha1_x_by_torque_z * w - kTwo * dalpha_z * v1_x - (omega_z * dv2_x_by_torque_z) +
            g * dalpha_z * r20
        };
        const value_type dv3_z_by_torque_z{
            -(dalpha1_x_by_torque_z * v - dalpha1_y_by_torque_z * u) -
            (omega_x * dv2_y_by_torque_z - omega_y * dv2_x_by_torque_z)
        };

        const value_type tau2{tau * tau};
        const value_type tau4{tau2 * tau2};
        Expansion expansion{
            .flow =
                {vx, vy, vz, v1_x, v1_y, v1_z, q1_x, q1_y, q1_z, q1_w, alpha_x, alpha_y, alpha_z},
            .value =
                {state.x() - target_state.x() +
                     tau * (vx + tau * (kHalf * lift * r02 +
                                        tau * lift * (kSixth * c1_x + tau * kTwentyFourth * c2_x))),
                 state.y() - target_state.y() +
                     tau * (vy + tau * (kHalf * lift * r12 +
                                        tau * lift * (kSixth * c1_y + tau * kTwentyFourth * c2_y))),
                 state.z() - target_state.z() +
                     tau * (vz + tau * (kHalf * (lift * r22 - g) +
                                        tau * lift * (kSixth * c1_z + tau * kTwentyFourth * c2_z))),
                 u - target_state.u() + tau * (v1_x + tau * (kHalf * v2_x + tau * kSixth * v3_x)),
                 v - target_state.v() + tau * (v1_y + tau * (kHalf * v2_y + tau * kSixth * v3_y)),
                 w - target_state.w() + tau * (v1_z + tau * (kHalf * v2_z + tau * kSixth * v3_z)),
                 kTwo * (qx - sign * target_state.qx() + tau * (q1_x + tau * kHalf * q2_x)),
                 kTwo * (qy - sign * target_state.qy() + tau * (q1_y + tau * kHalf * q2_y)),
                 kTwo * (qz - sign * target_state.qz() + tau * (q1_z + tau * kHalf * q2_z)),
                 kTwo * (qw - sign * target_state.qw() + tau * (q1_w + tau * kHalf * q2_w)),
                 omega_x - target_state.omega_x() + tau * alpha_x,
                 omega_y - target_state.omega_y() + tau * alpha_y,
                 omega_z - target_state.omega_z() + tau * alpha_z},
            // The torques reach the position through alpha x e_z in A'' alone, which torque_z
            // leaves at zero, and the attitude through q (dalpha, 0) / 2 in q''.
            .by_torque_x =
                {-tau4 * kTwentyFourth * lift * r01 * dalpha_x,
                 -tau4 * kTwentyFourth * lift * r11 * dalpha_x,
                 -tau4 * kTwentyFourth * lift * r21 * dalpha_x,
                 tau2 * tau * kSixth * dv3_x_by_torque_x,
                 tau2 * (kHalf * dv2_y_by_torque_x + tau * kSixth * dv3_y_by_torque_x),
                 tau2 * (kHalf * dv2_z_by_torque_x + tau * kSixth * dv3_z_by_torque_x),
                 tau2 * kHalf * dalpha_x * qw, tau2 * kHalf * dalpha_x * qz,
                 -tau2 * kHalf * dalpha_x * qy, -tau2 * kHalf * dalpha_x * qx, tau * dalpha_x,
                 static_cast<value_type>(0.0), static_cast<value_type>(0.0)},
            .by_torque_y =
                {tau4 * kTwentyFourth * lift * r00 * dalpha_y,
                 tau4 * kTwentyFourth * lift * r10 * dalpha_y,
                 tau4 * kTwentyFourth * lift * r20 * dalpha_y,
                 tau2 * (kHalf * dv2_x_by_torque_y + tau * kSixth * dv3_x_by_torque_y),
                 tau2 * tau * kSixth * dv3_y_by_torque_y,
                 tau2 * (kHalf * dv2_z_by_torque_y + tau * kSixth * dv3_z_by_torque_y),
                 -tau2 * kHalf * dalpha_y * qz, tau2 * kHalf * dalpha_y * qw,
                 tau2 * kHalf * dalpha_y * qx, -tau2 * kHalf * dalpha_y * qy,
                 static_cast<value_type>(0.0), tau * dalpha_y, static_cast<value_type>(0.0)},
            .by_torque_z =
                {static_cast<value_type>(0.0), static_cast<value_type>(0.0),
                 static_cast<value_type>(0.0),
                 tau2 * (kHalf * dv2_x_by_torque_z + tau * kSixth * dv3_x_by_torque_z),
                 tau2 * (kHalf * dv2_y_by_torque_z + tau * kSixth * dv3_y_by_torque_z),
                 tau2 * tau * kSixth * dv3_z_by_torque_z, tau2 * kHalf * dalpha_z * qy,
                 -tau2 * kHalf * dalpha_z * qx, tau2 * kHalf * dalpha_z * qw,
                 -tau2 * kHalf * dalpha_z * qz, static_cast<value_type>(0.0),
                 static_cast<value_type>(0.0), tau * dalpha_z},
            // The thrust: dv' = e_z / m, dv'' = -omega x e_z / m and
            // dv''' = (omega x (omega x e_z) - 2 alpha x e_z) / m; neither the attitude nor the
            // angular velocity depends on it.
            .by_thrust =
                {tau2 * (kHalf * r02 + tau * (kSixth * c1_x + tau * kTwentyFourth * c2_x)) / m,
                 tau2 * (kHalf * r12 + tau * (kSixth * c1_y + tau * kTwentyFourth * c2_y)) / m,
                 tau2 * (kHalf * r22 + tau * (kSixth * c1_z + tau * kTwentyFourth * c2_z)) / m,
                 tau2 * (-kHalf * omega_y + tau * kSixth * (omega_x * omega_z - kTwo * alpha_y)) /
                     m,
                 tau2 * (kHalf * omega_x + tau * kSixth * (omega_y * omega_z + kTwo * alpha_x)) / m,
                 tau * (kOne + tau2 * kSixth * e2_z) / m, static_cast<value_type>(0.0),
                 static_cast<value_type>(0.0), static_cast<value_type>(0.0),
                 static_cast<value_type>(0.0), static_cast<value_type>(0.0),
                 static_cast<value_type>(0.0), static_cast<value_type>(0.0)},
            .by_tau = {}
        };
        if constexpr (Adaptive) {
            // The derivatives of the four polynomials in tau, term by term.
            expansion.by_tau = {
                vx + tau * (lift * r02 + tau * lift * (kHalf * c1_x + tau * kSixth * c2_x)),
                vy + tau * (lift * r12 + tau * lift * (kHalf * c1_y + tau * kSixth * c2_y)),
                vz + tau * (lift * r22 - g + tau * lift * (kHalf * c1_z + tau * kSixth * c2_z)),
                v1_x + tau * (v2_x + tau * kHalf * v3_x),
                v1_y + tau * (v2_y + tau * kHalf * v3_y),
                v1_z + tau * (v2_z + tau * kHalf * v3_z),
                kTwo * (q1_x + tau * q2_x),
                kTwo * (q1_y + tau * q2_y),
                kTwo * (q1_z + tau * q2_z),
                kTwo * (q1_w + tau * q2_w),
                alpha_x,
                alpha_y,
                alpha_z
            };
        }
        return expansion;
    }
    // NOLINTEND(readability-function-size)

    /**
     * @brief tangent() looking ahead over tau instead of the horizon at the given state, so that
     * the later stages of rungeKutta() look ahead over what is left of it.
     */
    [[using gnu: flatten, leaf, hot]]
    auto tangent(
        const state_type& state, const state_type& target_state, value_type tau
    ) const noexcept(!Adaptive || !BASJOO_CHECK_PARAMS) -> state_type {
#if BASJOO_CHECK_PARAMS == 1
        if constexpr (Adaptive) {
            if (!(tau > 0.0)) [[unlikely]] {
                throw std::invalid_argument(
                    std::format(
                        "Invalid arguments detected! "
                        "basjoo::quadrotor::DynamicsFlowController::tangent() requires a positive "
                        "horizon: tau = {0:f}.",
                        tau
                    )
                );
            }
        }
#endif
        const Expansion expansion{expand(state, target_state, tau)};
        const value_type m{m_model.mass()};
        const value_type g{m_model.gravity()};
        const value_type jxx{m_model.inertia_xx()};
        const value_type jyy{m_model.inertia_yy()};
        const value_type jzz{m_model.inertia_zz()};
        const value_type length{std::sqrt(jxx / m)};
        const value_type tau3{tau * tau * tau};
        const value_type torque_scale{length / g * tau3}; // T0^2 tau^3, T0^2 = L0 / g
        const value_type torque_x_rate{
            -jxx * jxx * weigh(expansion.value, expansion.by_torque_x) / torque_scale
        };
        const value_type torque_y_rate{
            -jyy * jyy * weigh(expansion.value, expansion.by_torque_y) / torque_scale
        };
        const value_type torque_z_rate{
            -jzz * jzz * weigh(expansion.value, expansion.by_torque_z) / torque_scale
        };
        const value_type thrust_rate{
            -m * m * g * length * weigh(expansion.value, expansion.by_thrust) / tau3
        };
        const Residuals& flow{expansion.flow};
        if constexpr (Adaptive) {
            const value_type tau_rate{-tau * weigh(expansion.value, expansion.by_tau)};
            return state_type{flow.x,       flow.y,        flow.z,        flow.u,
                              flow.v,       flow.w,        flow.qx,       flow.qy,
                              flow.qz,      flow.qw,       flow.omega_x,  flow.omega_y,
                              flow.omega_z, torque_x_rate, torque_y_rate, torque_z_rate,
                              thrust_rate,  tau_rate};
        } else {
            return state_type{flow.x,       flow.y,        flow.z,        flow.u,
                              flow.v,       flow.w,        flow.qx,       flow.qy,
                              flow.qz,      flow.qw,       flow.omega_x,  flow.omega_y,
                              flow.omega_z, torque_x_rate, torque_y_rate, torque_z_rate,
                              thrust_rate};
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
    // The bounds an adaptive horizon is kept within, by default none at all. NaN and unused when
    // Adaptive is false.
    value_type m_tau_lower{
        Adaptive ? static_cast<value_type>(0.0) : std::numeric_limits<value_type>::quiet_NaN()
    };
    value_type m_tau_upper{
        Adaptive ? std::numeric_limits<value_type>::infinity()
                 : std::numeric_limits<value_type>::quiet_NaN()
    };
};

} // namespace basjoo::quadrotor
