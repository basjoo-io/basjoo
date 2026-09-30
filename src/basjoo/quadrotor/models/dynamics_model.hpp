/**
 * @file quadrotor/models/dynamics_model.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Rigid body dynamics of a quadrotor driven by body torques and a collective thrust.
 * @version 0.1
 * @date 2026-11-12
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

namespace basjoo::quadrotor {

/**
 * @brief Position, body velocity, attitude and body angular velocity of the quadrotor.
 *
 * x, y and z give the center of mass in the world frame, u, v and w the velocity in the body frame,
 * qx, qy, qz and qw the attitude and omega_x, omega_y and omega_z the angular velocity in the body
 * frame. The world frame is ENU and the body frame FLU, as REP-103 has them: x forward, y left and
 * z up. The attitude is the Hamilton quaternion qw + qx i + qy j + qz k taking body frame vectors
 * into the world frame as q v q*, stored vector part first.
 *
 * The thirteen fields are stored flat, so that the state is a vector in the sense of VecArithmetic
 * and the integrators and the piecewise functions combine it field by field. Default construction
 * gives a state at rest at the origin under the identity attitude.
 */
template <std::floating_point T>
class DynamicsModelState final : public ::basjoo::math::DenseArithmeticBase {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using extents_type = std::extents<index_type, 13>;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr DynamicsModelState() noexcept = default;

    // Every field set to v, the attitude included. Generic code builds the zero of a value type as
    // value_type{0.0}.
    constexpr explicit DynamicsModelState(value_type v) noexcept
        : m_data{v, v, v, v, v, v, v, v, v, v, v, v, v} {}

    // The fields in storage order, one parameter each.
    // NOLINTBEGIN(readability-function-size)
    constexpr DynamicsModelState(
        value_type x, value_type y, value_type z, value_type u, value_type v, value_type w,
        value_type qx, value_type qy, value_type qz, value_type qw, value_type omega_x,
        value_type omega_y, value_type omega_z
    ) noexcept
        : m_data{x, y, z, u, v, w, qx, qy, qz, qw, omega_x, omega_y, omega_z} {}
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
        return 13;
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

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // x, y, z, u, v, w, qx, qy, qz, qw, omega_x, omega_y and omega_z, in this order.
    std::array<value_type, 13> m_data{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      0.0, 0.0, 1.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Body torques and collective thrust driving the quadrotor.
 *
 * torque_x, torque_y and torque_z are the torques about the body axes and thrust is the collective
 * thrust of the rotors along the body z axis. The four fields are stored flat like those of
 * DynamicsModelState, so that control sequences combine and interpolate field by field. Default
 * construction gives all four zero.
 */
template <std::floating_point T>
class DynamicsModelControl final : public ::basjoo::math::DenseArithmeticBase {
  public:
    using value_type = T;
    using size_type = std::size_t;
    using index_type = std::size_t;
    using rank_type = std::size_t;
    using extents_type = std::extents<index_type, 4>;
    using pointer = value_type*;
    using const_pointer = const value_type*;

    constexpr DynamicsModelControl() noexcept = default;

    // Every field set to v. Generic code builds the zero of a value type as value_type{0.0}.
    constexpr explicit DynamicsModelControl(value_type v) noexcept : m_data{v, v, v, v} {}

    constexpr DynamicsModelControl(
        value_type torque_x, value_type torque_y, value_type torque_z, value_type thrust
    ) noexcept
        : m_data{torque_x, torque_y, torque_z, thrust} {}

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
        return 4;
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
    constexpr auto torque_x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[0];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[1];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto torque_z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[2];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto thrust(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[3];
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    // torque_x, torque_y, torque_z and thrust, in this order.
    std::array<value_type, 4> m_data{0.0, 0.0, 0.0, 0.0};
};

/**
 * @brief Rigid body dynamics of a quadrotor driven by body torques and a collective thrust.
 *
 * The Newton-Euler equations of a rigid body under gravity, its body axes along the principal axes
 * of inertia, so that the inertia tensor is J = diag(J_xx, J_yy, J_zz):
 *
 *     d(x, y, z)/dt = R (u, v, w)
 *     d(u, v, w)/dt = -omega x (u, v, w) + (0, 0, thrust / m) - g R^T (0, 0, 1)
 *     dq/dt         = q (omega, 0) / 2
 *     J d(omega)/dt = torque - omega x J omega
 *
 * R is the rotation of the attitude q and (omega, 0) the pure quaternion of the angular velocity.
 * The thrust pushes along the body z axis and gravity pulls along -z of the world frame. Drag,
 * rotor dynamics and the allocation of the controls to the motors are left out. The controls are
 * handed to every integrator alongside the state and held constant over a step.
 *
 * The attitude is integrated in R^4, which lets |q| drift off one, so every integrator normalizes
 * it at the end of its step. forwardEuler() and rungeKutta() are the plain explicit schemes, linear
 * combinations of tangents over all thirteen fields. tangent() takes an attitude of any non-zero
 * norm and rotates through q / |q|, so the stages of rungeKutta() and interpolated states, which
 * are off unit norm, are evaluated as their normalized attitude.
 *
 * The four controls are hard bounded, each on its own. tangent(), and with it every integrator,
 * clamps them into [control_lower_bound(), control_upper_bound()] before it acts on them, so the
 * model never applies a control outside its bounds. The bounds default to infinite, which is no
 * constraint at all. A box only approximates what the rotors can deliver: their thrusts are bounded
 * one by one, so near full thrust the torques left over shrink, which no box captures.
 */
template <std::floating_point T>
class DynamicsModel final {
  public:
    using value_type = T;

    using state_type = DynamicsModelState<value_type>;
    using control_type = DynamicsModelControl<value_type>;

    DynamicsModel() noexcept = default;
    DynamicsModel(const DynamicsModel& other) noexcept = default;
    auto operator=(const DynamicsModel& other) noexcept -> DynamicsModel& = default;
    DynamicsModel(DynamicsModel&& other) noexcept = default;
    auto operator=(DynamicsModel&& other) noexcept -> DynamicsModel& = default;
    ~DynamicsModel() noexcept = default;

    // gravity is the magnitude of the gravitational acceleration, which pulls along -z of the world
    // frame: 9.80665 is the standard one, in m / s^2.
    explicit DynamicsModel(
        value_type mass, value_type inertia_xx, value_type inertia_yy, value_type inertia_zz,
        control_type control_lower_bound =
            control_type{-std::numeric_limits<value_type>::infinity()},
        control_type control_upper_bound =
            control_type{std::numeric_limits<value_type>::infinity()},
        value_type gravity = static_cast<value_type>(9.80665)
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_mass{mass}, m_jxx{inertia_xx}, m_jyy{inertia_yy}, m_jzz{inertia_zz},
          m_control_lo{control_lower_bound}, m_control_up{control_upper_bound}, m_gravity{gravity} {
#if BASJOO_CHECK_PARAMS == 1
        if (mass <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel requires a "
                    "positive mass: mass = {0:f}.",
                    mass
                )
            );
        }
        if (inertia_xx <= 0.0 || inertia_yy <= 0.0 || inertia_zz <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel requires the "
                    "three principal moments of inertia to be positive: inertia_xx = {0:f}, "
                    "inertia_yy = {1:f}, inertia_zz = {2:f}.",
                    inertia_xx, inertia_yy, inertia_zz
                )
            );
        }
        if (gravity < 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel requires a "
                    "non-negative gravity, the magnitude of the gravitational acceleration along "
                    "-z of the world frame: gravity = {0:f}.",
                    gravity
                )
            );
        }
        if (!(m_control_lo.torque_x() <= 0.0 && 0.0 <= m_control_up.torque_x() &&
              m_control_lo.torque_y() <= 0.0 && 0.0 <= m_control_up.torque_y() &&
              m_control_lo.torque_z() <= 0.0 && 0.0 <= m_control_up.torque_z())) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel requires the "
                    "torque bounds to bracket zero: [{0:f}, {1:f}] about x, [{2:f}, {3:f}] about "
                    "y and [{4:f}, {5:f}] about z.",
                    m_control_lo.torque_x(), m_control_up.torque_x(), m_control_lo.torque_y(),
                    m_control_up.torque_y(), m_control_lo.torque_z(), m_control_up.torque_z()
                )
            );
        }
        if (!(m_control_lo.thrust() <= m_control_up.thrust())) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel requires the "
                    "thrust lower bound not to exceed the upper bound: thrust lower bound = "
                    "{0:f} while thrust upper bound = {1:f}.",
                    m_control_lo.thrust(), m_control_up.thrust()
                )
            );
        }
#endif
    }

    [[using gnu: pure, always_inline]]
    constexpr auto mass() const noexcept -> value_type {
        return m_mass;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto inertia_xx() const noexcept -> value_type {
        return m_jxx;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto inertia_yy() const noexcept -> value_type {
        return m_jyy;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto inertia_zz() const noexcept -> value_type {
        return m_jzz;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto control_lower_bound() const noexcept -> const control_type& {
        return m_control_lo;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto control_upper_bound() const noexcept -> const control_type& {
        return m_control_up;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto gravity() const noexcept -> value_type {
        return m_gravity;
    }

    [[using gnu: always_inline]]
    auto forwardEuler(
        state_type initial_state, const control_type& control, value_type h
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
                    "basjoo::quadrotor::DynamicsModel::forwardEuler() requires the attitude of "
                    "initial_state to be a unit quaternion: |q|^2 = {0:f}.",
                    attitude_sqr
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! "
                    "basjoo::quadrotor::DynamicsModel::forwardEuler() requires a positive time "
                    "step: h = {0:f}.",
                    h
                )
            );
        }
#endif
        initial_state += tangent(initial_state, control) * h;
        const value_type norm{std::sqrt(
            initial_state.qx() * initial_state.qx() + initial_state.qy() * initial_state.qy() +
            initial_state.qz() * initial_state.qz() + initial_state.qw() * initial_state.qw()
        )};
        initial_state.qx() /= norm;
        initial_state.qy() /= norm;
        initial_state.qz() /= norm;
        initial_state.qw() /= norm;
        return initial_state;
    }

    [[using gnu: always_inline]]
    auto rungeKutta(
        state_type initial_state, const control_type& control, value_type h
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
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel::rungeKutta() "
                    "requires the attitude of initial_state to be a unit quaternion: |q|^2 = "
                    "{0:f}.",
                    attitude_sqr
                )
            );
        }
        if (h <= 0.0) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel::rungeKutta() "
                    "requires a positive time step: h = {0:f}.",
                    h
                )
            );
        }
#endif
        // The stages are left off unit norm, which tangent() takes; only the step is normalized.
        constexpr value_type kHalf{static_cast<value_type>(0.5)};
        constexpr value_type kSixth{static_cast<value_type>(1.0 / 6.0)};
        constexpr value_type kThird{static_cast<value_type>(1.0 / 3.0)};
        const state_type k1{tangent(initial_state, control) * h};
        const state_type k2{tangent(initial_state + k1 * kHalf, control) * h};
        const state_type k3{tangent(initial_state + k2 * kHalf, control) * h};
        const state_type k4{tangent(initial_state + k3, control) * h};
        initial_state += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
        const value_type norm{std::sqrt(
            initial_state.qx() * initial_state.qx() + initial_state.qy() * initial_state.qy() +
            initial_state.qz() * initial_state.qz() + initial_state.qw() * initial_state.qw()
        )};
        initial_state.qx() /= norm;
        initial_state.qy() /= norm;
        initial_state.qz() /= norm;
        initial_state.qw() /= norm;
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
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel::evolve() "
                    "requires the two input spans to have identical sizes: controls.size() = "
                    "{0:d}, hs.size() = {1:d}.",
                    controls.size(), hs.size()
                )
            );
        }
        if (!trace.empty() && trace.size() != controls.size() + 1) [[unlikely]] {
            throw std::invalid_argument(
                std::format(
                    "Invalid arguments detected! basjoo::quadrotor::DynamicsModel::evolve() "
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
     * State doubles as its own tangent space here, so the returned object is not a state: its x,
     * y and z fields hold the world frame velocity, its u, v and w fields the derivative of the
     * body velocity, its attitude fields dq/dt and its omega fields the angular acceleration. It is
     * only meant to be scaled and added onto a state.
     *
     * The attitude may have any non-zero norm. The rotation is taken of q / |q|, with the entries
     * of R written as forms of degree two in q scaled by 2 / |q|^2, while dq/dt = q (omega, 0) / 2
     * scales with q, which is what keeps the flow on the sphere |q| = const. The controls are
     * clamped into their bounds here, which is where every integrator picks up the clamp.
     */
    [[using gnu: pure, always_inline]]
    auto tangent(const state_type& state, const control_type& control) const noexcept
        -> state_type {
        const value_type torque_x{
            std::clamp(control.torque_x(), m_control_lo.torque_x(), m_control_up.torque_x())
        };
        const value_type torque_y{
            std::clamp(control.torque_y(), m_control_lo.torque_y(), m_control_up.torque_y())
        };
        const value_type torque_z{
            std::clamp(control.torque_z(), m_control_lo.torque_z(), m_control_up.torque_z())
        };
        const value_type thrust{
            std::clamp(control.thrust(), m_control_lo.thrust(), m_control_up.thrust())
        };
        const value_type qx{state.qx()};
        const value_type qy{state.qy()};
        const value_type qz{state.qz()};
        const value_type qw{state.qw()};
        const value_type s{static_cast<value_type>(2.0) / (qx * qx + qy * qy + qz * qz + qw * qw)};
        const value_type r00{static_cast<value_type>(1.0) - s * (qy * qy + qz * qz)};
        const value_type r01{s * (qx * qy - qz * qw)};
        const value_type r02{s * (qx * qz + qy * qw)};
        const value_type r10{s * (qx * qy + qz * qw)};
        const value_type r11{static_cast<value_type>(1.0) - s * (qx * qx + qz * qz)};
        const value_type r12{s * (qy * qz - qx * qw)};
        const value_type r20{s * (qx * qz - qy * qw)};
        const value_type r21{s * (qy * qz + qx * qw)};
        const value_type r22{static_cast<value_type>(1.0) - s * (qx * qx + qy * qy)};
        const value_type u{state.u()};
        const value_type v{state.v()};
        const value_type w{state.w()};
        const value_type omega_x{state.omega_x()};
        const value_type omega_y{state.omega_y()};
        const value_type omega_z{state.omega_z()};
        // The last row of R is R^T (0, 0, 1), the world z axis seen from the body. dq/dt is the
        // Hamilton product q (omega, 0) / 2 written out: its vector part is
        // (qw omega + (qx, qy, qz) x omega) / 2 and its scalar part -(qx, qy, qz) . omega / 2.
        return state_type{
            r00 * u + r01 * v + r02 * w,
            r10 * u + r11 * v + r12 * w,
            r20 * u + r21 * v + r22 * w,
            omega_z * v - omega_y * w - m_gravity * r20,
            omega_x * w - omega_z * u - m_gravity * r21,
            omega_y * u - omega_x * v + thrust / m_mass - m_gravity * r22,
            static_cast<value_type>(0.5) * (qw * omega_x + qy * omega_z - qz * omega_y),
            static_cast<value_type>(0.5) * (qw * omega_y + qz * omega_x - qx * omega_z),
            static_cast<value_type>(0.5) * (qw * omega_z + qx * omega_y - qy * omega_x),
            static_cast<value_type>(-0.5) * (qx * omega_x + qy * omega_y + qz * omega_z),
            (torque_x + (m_jyy - m_jzz) * omega_y * omega_z) / m_jxx,
            (torque_y + (m_jzz - m_jxx) * omega_z * omega_x) / m_jyy,
            (torque_z + (m_jxx - m_jyy) * omega_x * omega_y) / m_jzz
        };
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<7>;

    value_type m_mass{1.0};
    value_type m_jxx{1.0};
    value_type m_jyy{1.0};
    value_type m_jzz{1.0};
    control_type m_control_lo{-std::numeric_limits<value_type>::infinity()};
    control_type m_control_up{std::numeric_limits<value_type>::infinity()};
    value_type m_gravity{9.80665};
};

} // namespace basjoo::quadrotor
