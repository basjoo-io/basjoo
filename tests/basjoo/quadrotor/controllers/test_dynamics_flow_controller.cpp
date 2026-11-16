/**
 * @file quadrotor/controllers/test_dynamics_flow_controller.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit test of DynamicsFlowController, with a fixed and with an adaptive horizon.
 * @version 0.1
 * @date 2026-11-13
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/quadrotor/controllers/dynamics_flow_controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <initializer_list>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <type_traits>
#include <utility>
#include <vector>

#include "cxxopts.hpp"
#include "matplot/matplot.h"
#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/mat33.hpp"
#include "basjoo/math/dense/quaternion.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/functions/function_proxy.hpp"
#include "basjoo/math/functions/piecewise_cubic_function.hpp"
#include "basjoo/math/functions/piecewise_linear_function.hpp"
#include "basjoo/math/functions/piecewise_quintic_function.hpp"
#include "basjoo/math/integrate_mode.hpp"
#include "basjoo/math/type_traits.hpp"
#include "basjoo/quadrotor/models/dynamics_model.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

namespace {

bool plot_graph{false};

auto createFigureHandle() noexcept -> matplot::figure_handle {
    matplot::figure_handle fig = matplot::figure(true);
    fig->backend()->run_command("unset warnings");
    fig->size(1800, 1800);
    return fig;
}

/**
 * @brief The nineteen subplots of a figure on a 5 x 4 grid, in reading order.
 *
 * The 3D path spans the first two cells of the top row, followed by the position and the attitude
 * errors. The rows below hold the body velocity and its error, the attitude, the angular velocity
 * and the horizon, and the four controls against time, a field per subplot. Every error is taken
 * against the simultaneous point. The labels are gnuplot's enhanced text, in which _ and ^ set
 * subscripts and superscripts.
 */
auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    const std::array<const char*, 18> ylabels{
        "log_{10}‖r − r^*‖ [m]",
        "log_{10}∠(q, q^*) [rad]",
        "u [m/s]",
        "v [m/s]",
        "w [m/s]",
        "log_{10}‖v_B − v_B^*‖ [m/s]",
        "q_x",
        "q_y",
        "q_z",
        "q_w",
        "ω_x [rad/s]",
        "ω_y [rad/s]",
        "ω_z [rad/s]",
        "τ [s]",
        "M_x [N·m]",
        "M_y [N·m]",
        "M_z [N·m]",
        "T [N]"
    };
    std::vector<matplot::axes_handle> axes;
    axes.reserve(ylabels.size() + 1);
    axes.emplace_back(fig->add_subplot(5, 4, {0, 1}));
    for (std::size_t i = 0; i < ylabels.size(); ++i) {
        axes.emplace_back(fig->add_subplot(5, 4, i + 2));
    }
    for (const matplot::axes_handle& ax : axes) {
        ax->grid(matplot::on);
        ax->hold(matplot::on);
        ax->font_size(12.0);
        ax->x_axis().label_font_size(12.0);
        ax->y_axis().label_font_size(12.0);
    }
    axes[0]->z_axis().label_font_size(12.0);
    axes[0]->xlabel("x [m]");
    axes[0]->ylabel("y [m]");
    axes[0]->zlabel("z [m]");
    for (std::size_t i = 0; i < ylabels.size(); ++i) {
        axes[i + 1]->xlabel("t [s]");
        axes[i + 1]->ylabel(ylabels[i]);
    }
    return axes;
}

} // namespace

namespace basjoo::quadrotor {

namespace {

using ::basjoo::math::IntegrateMode;
using ::basjoo::math::Mat33;
using ::basjoo::math::Quaternion;
using ::basjoo::math::Vec3;

template <std::floating_point T>
inline constexpr T kMass{static_cast<T>(1.5)};
template <std::floating_point T>
inline constexpr T kInertiaXx{static_cast<T>(0.03)};
template <std::floating_point T>
inline constexpr T kInertiaYy{static_cast<T>(0.035)};
template <std::floating_point T>
inline constexpr T kInertiaZz{static_cast<T>(0.06)};
template <std::floating_point T>
inline constexpr T kGravity{static_cast<T>(9.80665)};
template <std::floating_point T>
inline constexpr T kHoverThrust{kMass<T> * kGravity<T>};
// No two bounds agree, so that a clamp that read the wrong field cannot pass. The thrust bounds
// leave zero out on purpose: an idle thrust above zero is what real rotors deliver.
template <std::floating_point T>
inline constexpr T kTorqueXLo{static_cast<T>(-0.4)};
template <std::floating_point T>
inline constexpr T kTorqueXUp{static_cast<T>(0.5)};
template <std::floating_point T>
inline constexpr T kTorqueYLo{static_cast<T>(-0.3)};
template <std::floating_point T>
inline constexpr T kTorqueYUp{static_cast<T>(0.6)};
template <std::floating_point T>
inline constexpr T kTorqueZLo{static_cast<T>(-0.1)};
template <std::floating_point T>
inline constexpr T kTorqueZUp{static_cast<T>(0.2)};
template <std::floating_point T>
inline constexpr T kThrustLo{static_cast<T>(2.0)};
template <std::floating_point T>
inline constexpr T kThrustUp{static_cast<T>(30.0)};
// The horizon of a fixed controller and the one an adaptive closed loop run starts with: 2.5 T0 of
// the vehicle above, whose L0 = sqrt(J_xx / m) comes to 0.141 m and T0 = sqrt(L0 / g) to 0.120 s.
template <std::floating_point T>
inline constexpr T kHorizon{static_cast<T>(0.3)};
// The bounds an adaptive controller keeps its horizon within, half as long again as kHorizon
// either way.
template <std::floating_point T>
inline constexpr T kTauLowerBound{static_cast<T>(0.2)};
template <std::floating_point T>
inline constexpr T kTauUpperBound{static_cast<T>(0.45)};
// The longest horizon a step can start with, and so how far past the last step a reference has to
// reach.
template <std::floating_point T>
inline constexpr T kMaxPreview{kTauUpperBound<T>};
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};
// The mean spacing of the waypoints of a reference, ten steps of kH, from which every interval
// strays by up to half either way (see waypointTimesOf()).
inline constexpr double kWaypointSpacing{0.1};
/**
 * @brief Four cost weights that differ from each other and from one, and damp the loop well.
 *
 * A default constructed controller weighs every term by one, so it takes weights like these to
 * tell a weight that went missing, or landed on the wrong term, apart from one that did not. At
 * kHorizon they leave the slowest mode of the loop about hover decaying as exp(-0.061 t / T0) and
 * no pair damped at a ratio below 0.59, where unit weights leave the loop all but undamped.
 */
template <std::floating_point T>
inline constexpr std::array<T, 4> kWeights{
    static_cast<T>(0.1), static_cast<T>(1.5), static_cast<T>(6.5), static_cast<T>(0.2)
};

inline constexpr std::array<IntegrateMode, 2> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::RUNGE_KUTTA
};

/**
 * @brief Scalar type and horizon mode a test case shared by both modes is instantiated for.
 */
template <std::floating_point T, bool A>
struct Mode final {
    using value_type = T;
    static constexpr bool kAdaptive{A};
};

using FixedFloat = Mode<float, false>;
using AdaptiveFloat = Mode<float, true>;
using FixedDouble = Mode<double, false>;
using AdaptiveDouble = Mode<double, true>;

template <std::floating_point T, bool A>
using State = DynamicsFlowControllerState<T, A>;
template <std::floating_point T>
using ModelState = DynamicsModelState<T>;
template <std::floating_point T>
using ModelControl = DynamicsModelControl<T>;
template <std::floating_point T, bool A>
using Controller = DynamicsFlowController<T, A>;
// A reference trajectory: target states at timestamped waypoints, interpolated linearly between.
template <std::floating_point T, bool A>
using Reference = ::basjoo::math::PiecewiseLinearFunction<State<T, A>>;

// The array storage lets the state arithmetic and the accessors run in constant expressions.
consteval auto constantState() -> bool {
    State<double, false> s{1.0, 2.0,   3.0,   0.5, 0.25, -0.5, 0.0,  0.0, 0.0,
                           1.0, 0.125, -0.25, 2.0, 0.5,  -0.5, 0.25, 15.0};
    s += State<double, false>{1.0};
    s *= 2.0;
    s.thrust() -= 2.0;
    s.qx() = 0.5;
    State<double, true> a{1.0, 2.0,   3.0,   0.5, 0.25, -0.5, 0.0,  0.0,  0.0,
                          1.0, 0.125, -0.25, 2.0, 0.5,  -0.5, 0.25, 15.0, 0.3};
    a += a;
    return s.x() == 4.0 && s.qx() == 0.5 && s.qw() == 4.0 && s[12] == 6.0 && s.torque_z() == 2.5 &&
           s.thrust() == 30.0 && s - s == State<double, false>{0.0} &&
           State<double, false>{}.qw() == 1.0 && a.tau() == 0.6 && a[17] == 0.6 &&
           State<double, true>{}.tau() == 1.0 && State<double, true>{}.qw() == 1.0;
}
static_assert(constantState());

template <std::floating_point T>
[[nodiscard]] constexpr auto lowerBound() noexcept -> ModelControl<T> {
    return ModelControl<T>{kTorqueXLo<T>, kTorqueYLo<T>, kTorqueZLo<T>, kThrustLo<T>};
}

template <std::floating_point T>
[[nodiscard]] constexpr auto upperBound() noexcept -> ModelControl<T> {
    return ModelControl<T>{kTorqueXUp<T>, kTorqueYUp<T>, kTorqueZUp<T>, kThrustUp<T>};
}

template <std::floating_point T>
[[nodiscard]] auto boundedModel() -> DynamicsModel<T> {
    return DynamicsModel<T>{kMass<T>,      kInertiaXx<T>,   kInertiaYy<T>,
                            kInertiaZz<T>, lowerBound<T>(), upperBound<T>()};
}

template <std::floating_point T>
[[nodiscard]] auto unboundedModel() -> DynamicsModel<T> {
    return DynamicsModel<T>{kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>};
}

/**
 * @brief Controller of either mode. tau is the horizon of a fixed one and the two bounds those of
 * an adaptive one, whose horizon comes with the state, each going unused by the other mode.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto controllerOf(
    const DynamicsModel<T>& model,
    const std::type_identity_t<std::array<T, 4>>& weights = kWeights<T>,
    [[maybe_unused]] std::type_identity_t<T> tau = kHorizon<T>,
    [[maybe_unused]] std::type_identity_t<T> tau_lower_bound = kTauLowerBound<T>,
    [[maybe_unused]] std::type_identity_t<T> tau_upper_bound = kTauUpperBound<T>
) -> Controller<T, A> {
    if constexpr (A) {
        return Controller<T, A>{
            model, std::span<const T, 4>{weights}, tau_lower_bound, tau_upper_bound
        };
    } else {
        return Controller<T, A>{model, std::span<const T, 4>{weights}, tau};
    }
}

/**
 * @brief A state of the fixed mode carried over into the given mode, with the horizon tau when
 * that adapts.
 */
template <bool A, std::floating_point T>
[[nodiscard]] constexpr auto inMode(
    const State<T, false>& state, [[maybe_unused]] std::type_identity_t<T> tau
) noexcept -> State<T, A> {
    if constexpr (A) {
        State<T, A> result{};
        for (std::size_t i{0}; i < State<T, false>::size(); ++i) {
            result[i] = state[i];
        }
        result.tau() = tau;
        return result;
    } else {
        return state;
    }
}

/**
 * @brief A state of the given mode taken back to the fixed mode, its horizon dropped.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto fixedOf(const State<T, A>& state) noexcept -> State<T, false> {
    State<T, false> result{};
    for (std::size_t i{0}; i < State<T, false>::size(); ++i) {
        result[i] = state[i];
    }
    return result;
}

/**
 * @brief Horizon the prediction runs over at the given state: the one of a fixed controller, or
 * the one the state carries, clamped into the bounds of an adaptive controller.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto horizonOf(
    [[maybe_unused]] const Controller<T, A>& controller, [[maybe_unused]] const State<T, A>& state
) noexcept -> T {
    if constexpr (A) {
        return std::clamp(state.tau(), controller.tau_lower_bound(), controller.tau_upper_bound());
    } else {
        return controller.tau();
    }
}

/**
 * @brief The given state with its horizon set to tau where it carries one.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto withHorizon(State<T, A> state, [[maybe_unused]] T tau) noexcept
    -> State<T, A> {
    if constexpr (A) {
        state.tau() = tau;
    }
    return state;
}

/**
 * @brief A vector of the scenario precision from double constants, so that the scenarios below
 * read as their numbers.
 */
template <std::floating_point T>
[[nodiscard]] auto makeVec3(double x, double y, double z) noexcept -> Vec3<T> {
    return Vec3<T>{static_cast<T>(x), static_cast<T>(y), static_cast<T>(z)};
}

/**
 * @brief The attitude of REP-103's roll, pitch and yaw, of the scenario precision.
 */
template <std::floating_point T>
[[nodiscard]] auto attitudeFromRpy(double roll, double pitch, double yaw) noexcept
    -> Quaternion<T> {
    return Quaternion<T>::fromRpy(static_cast<T>(roll), static_cast<T>(pitch), static_cast<T>(yaw));
}

/**
 * @brief Builds a state from, and takes it apart into, the vectors of physical space it is made of
 * and the controls.
 *
 * The controller takes its attitude as four flat scalars, while the scenarios below are written in
 * terms of positions, velocities, attitudes and angular velocities. math::Quaternion and math::Vec3
 * are an implementation of their own of the rotations the controller writes out field by field,
 * which makes them the yardstick the controller is held to.
 */
template <std::floating_point T>
[[nodiscard]] auto stateOf(
    const Vec3<T>& position, const Vec3<T>& velocity, const Quaternion<T>& attitude,
    const Vec3<T>& omega, const ModelControl<T>& control
) noexcept -> State<T, false> {
    return State<T, false>{position.x(),       position.y(),       position.z(),
                           velocity.x(),       velocity.y(),       velocity.z(),
                           attitude.x(),       attitude.y(),       attitude.z(),
                           attitude.w(),       omega.x(),          omega.y(),
                           omega.z(),          control.torque_x(), control.torque_y(),
                           control.torque_z(), control.thrust()};
}

/**
 * @brief A target state: the controller ignores the controls and the horizon of a target, so they
 * are left NaN, here and wherever a target is carried over into the adaptive mode, which any use
 * of them would carry into the result.
 */
template <std::floating_point T>
[[nodiscard]] auto targetAt(
    const Vec3<T>& position, const Vec3<T>& velocity, const Quaternion<T>& attitude,
    const Vec3<T>& omega
) noexcept -> State<T, false> {
    constexpr T kNaN{std::numeric_limits<T>::quiet_NaN()};
    return stateOf(position, velocity, attitude, omega, ModelControl<T>{kNaN});
}

template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto plantOf(const State<T, A>& state) noexcept -> ModelState<T> {
    return ModelState<T>{state.x(),       state.y(),       state.z(),      state.u(),  state.v(),
                         state.w(),       state.qx(),      state.qy(),     state.qz(), state.qw(),
                         state.omega_x(), state.omega_y(), state.omega_z()};
}

template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto controlOf(const State<T, A>& state) noexcept -> ModelControl<T> {
    return ModelControl<T>{state.torque_x(), state.torque_y(), state.torque_z(), state.thrust()};
}

template <std::floating_point T, bool A>
[[nodiscard]] auto positionOf(const State<T, A>& state) noexcept -> Vec3<T> {
    return Vec3<T>{state.x(), state.y(), state.z()};
}

template <std::floating_point T, bool A>
[[nodiscard]] auto velocityOf(const State<T, A>& state) noexcept -> Vec3<T> {
    return Vec3<T>{state.u(), state.v(), state.w()};
}

template <std::floating_point T, bool A>
[[nodiscard]] auto attitudeOf(const State<T, A>& state) noexcept -> Quaternion<T> {
    return Quaternion<T>{state.qx(), state.qy(), state.qz(), state.qw()};
}

template <std::floating_point T, bool A>
[[nodiscard]] auto omegaOf(const State<T, A>& state) noexcept -> Vec3<T> {
    return Vec3<T>{state.omega_x(), state.omega_y(), state.omega_z()};
}

/**
 * @brief A vehicle climbing through a banked turn, every control inside the bounds and off zero.
 *
 * An adaptive horizon starts inside its bounds but off both of them, off its default of one and
 * off kHorizon, so that a horizon that went missing somewhere, or was read from somewhere else,
 * shows.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto startOf() -> State<T, A> {
    return inMode<A>(
        stateOf(
            makeVec3<T>(0.4, -0.3, 2.0), makeVec3<T>(1.2, -0.7, 0.4),
            attitudeFromRpy<T>(0.3, -0.2, 0.7), makeVec3<T>(0.6, -0.9, 0.5),
            ModelControl<T>{
                static_cast<T>(0.05), static_cast<T>(-0.08), static_cast<T>(0.03),
                static_cast<T>(1.3) * kHoverThrust<T>
            }
        ),
        static_cast<T>(0.35)
    );
}

/**
 * @brief A target ahead of startOf(), off by enough in every residual that the rates at the start
 * are far from zero.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto targetOf() -> State<T, A> {
    return inMode<A>(
        targetAt(
            makeVec3<T>(1.1, 0.2, 2.4), makeVec3<T>(1.5, 0.1, -0.2),
            attitudeFromRpy<T>(0.1, 0.05, 0.6), makeVec3<T>(0.1, 0.2, -0.1)
        ),
        std::numeric_limits<T>::quiet_NaN()
    );
}

/**
 * @brief Climbing through a banked turn, sinking backwards rolled the other way, and pitched
 * forward while yawing fast, each with all four controls inside the bounds and, when adaptive, a
 * horizon inside its bounds.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto statesOf() -> std::array<State<T, A>, 3> {
    return {
        startOf<T, A>(),
        inMode<A>(
            stateOf(
                makeVec3<T>(3.0, -2.0, 1.0), makeVec3<T>(-0.8, 1.1, -0.3),
                attitudeFromRpy<T>(-0.4, 0.25, -1.1), makeVec3<T>(-0.7, 0.4, -0.3),
                ModelControl<T>{
                    static_cast<T>(-0.12), static_cast<T>(0.2), static_cast<T>(-0.05),
                    static_cast<T>(0.9) * kHoverThrust<T>
                }
            ),
            static_cast<T>(0.25)
        ),
        inMode<A>(
            stateOf(
                makeVec3<T>(-1.0, 4.0, 5.0), makeVec3<T>(0.3, 0.2, 1.5),
                attitudeFromRpy<T>(0.1, 0.5, 2.5), makeVec3<T>(0.2, -0.3, 1.1),
                ModelControl<T>{
                    static_cast<T>(0.3), static_cast<T>(-0.2), static_cast<T>(0.15),
                    static_cast<T>(1.6) * kHoverThrust<T>
                }
            ),
            static_cast<T>(0.4)
        )
    };
}

/**
 * @brief Raises maximum to value, a NaN included.
 *
 * std::max() skips a NaN in its second argument, which would let a diverging integrator pass a
 * bound on its largest error.
 */
template <std::floating_point T>
auto raiseTo(T& maximum, T value) noexcept -> void {
    if (!(value <= maximum)) {
        maximum = value;
    }
}

/**
 * @brief Largest per field difference between two values of the same flat type: states of the
 * controller or of the model.
 */
template <typename Value>
[[nodiscard]] auto distanceOf(const Value& lhs, const Value& rhs) noexcept ->
    typename Value::value_type {
    using value_type = typename Value::value_type;
    value_type result{static_cast<value_type>(0.0)};
    for (std::size_t i{0}; i < Value::size(); ++i) {
        raiseTo(result, std::abs(lhs.data()[i] - rhs.data()[i]));
    }
    return result;
}

/**
 * @brief Bound on the drift between two code paths that walk the same arithmetic.
 *
 * The compiler is free to fuse a multiply and an add into one FMA at one call site and not at
 * another, which moves the last bit or so of a result. Every defect a comparison against this bound
 * is meant to catch lands at least an order of magnitude above it, which the tests require
 * explicitly wherever that is not obvious.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto roundoffTolerance() noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(1.0e-5) : static_cast<T>(1.0e-12);
}

/**
 * @brief Relative tolerance between two computations of one quantity that round differently, such
 * as a rate and a finite difference of the cost, with well over an order of magnitude of headroom.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto derivativeTolerance() noexcept -> double {
    return std::same_as<T, float> ? 1.0e-4 : 1.0e-10;
}

/**
 * @brief All four controls within the bounds, which also fails on a NaN.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto areControlsWithinBounds(const State<T, A>& state) noexcept -> bool {
    return kTorqueXLo<T> <= state.torque_x() && state.torque_x() <= kTorqueXUp<T> &&
           kTorqueYLo<T> <= state.torque_y() && state.torque_y() <= kTorqueYUp<T> &&
           kTorqueZLo<T> <= state.torque_z() && state.torque_z() <= kTorqueZUp<T> &&
           kThrustLo<T> <= state.thrust() && state.thrust() <= kThrustUp<T>;
}

/**
 * @brief All four controls within the bounds and, when adaptive, the horizon within the bounds of
 * the controllers controllerOf() builds by default, which also fails on a NaN.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto isWithinBounds(const State<T, A>& state) noexcept -> bool {
    if constexpr (A) {
        return areControlsWithinBounds(state) && kTauLowerBound<T> <= state.tau() &&
               state.tau() <= kTauUpperBound<T>;
    } else {
        return areControlsWithinBounds(state);
    }
}

/**
 * @brief Smallest magnitude among the rates a tangent gives the controller fields.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto smallestRateOf(const State<T, A>& k) noexcept -> T {
    const T controls{std::min(
        {std::abs(k.torque_x()), std::abs(k.torque_y()), std::abs(k.torque_z()),
         std::abs(k.thrust())}
    )};
    if constexpr (A) {
        return std::min(controls, std::abs(k.tau()));
    } else {
        return controls;
    }
}

template <std::floating_point T, bool A>
[[nodiscard]] auto stepWith(
    IntegrateMode mode, const Controller<T, A>& controller, const State<T, A>& state,
    const State<T, A>& target_state, std::type_identity_t<T> h
) -> State<T, A> {
    switch (mode) {
    case IntegrateMode::FORWARD_EULER:
        return controller.forwardEuler(state, target_state, h);
    case IntegrateMode::RUNGE_KUTTA:
        return controller.rungeKutta(state, target_state, h);
    }
    std::unreachable();
}

template <std::floating_point T, bool A>
auto evolveWith(
    IntegrateMode mode, const Controller<T, A>& controller, const State<T, A>& initial_state,
    std::type_identity_t<std::span<const State<T, A>>> target_states,
    std::type_identity_t<std::span<const T>> hs,
    std::type_identity_t<std::span<State<T, A>>> trace = {}
) -> State<T, A> {
    switch (mode) {
    case IntegrateMode::FORWARD_EULER:
        return controller.template evolve<IntegrateMode::FORWARD_EULER>(
            initial_state, target_states, hs, trace
        );
    case IntegrateMode::RUNGE_KUTTA:
        return controller.template evolve<IntegrateMode::RUNGE_KUTTA>(
            initial_state, target_states, hs, trace
        );
    }
    std::unreachable();
}

/**
 * @brief tangent() looking ahead over the horizon at the given state less shortening: through the
 * horizon of the state when that adapts, and through a second controller built with the shorter
 * horizon when it is fixed.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto tangentOverShorterHorizon(
    const Controller<T, A>& controller, State<T, A> state, const State<T, A>& target_state,
    T shortening
) -> State<T, A> {
    if constexpr (A) {
        state.tau() -= shortening;
        return controller.tangent(state, target_state);
    } else {
        std::array<T, 4> weights{};
        std::ranges::copy(controller.weights(), weights.begin());
        const Controller<T, A> shorter{
            controllerOf<T, A>(controller.model(), weights, controller.tau() - shortening)
        };
        return shorter.tangent(state, target_state);
    }
}

/**
 * @brief One classic Runge-Kutta step spelled out through tangent(), the later stages looking ahead
 * over the horizon less stage_times times their stage time, its attitude normalized and its
 * controls, and an adaptive horizon, clamped at the end.
 *
 * With stage_times at one it walks the same arithmetic as rungeKutta(), so the two part only by
 * roundoff, and at zero every stage looks ahead over the full horizon. An adaptive horizon has to
 * stay inside its bounds over the stages, where shortening it before or after its clamp comes to
 * the same.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto rungeKuttaThroughTangent(
    const Controller<T, A>& controller, const State<T, A>& state, const State<T, A>& target_state,
    T h, T stage_times
) -> State<T, A> {
    constexpr T kHalf{static_cast<T>(0.5)};
    constexpr T kSixth{static_cast<T>(1.0 / 6.0)};
    constexpr T kThird{static_cast<T>(1.0 / 3.0)};
    const DynamicsModel<T>& model{controller.model()};
    const State<T, A> k1{controller.tangent(state, target_state) * h};
    const State<T, A> k2{
        tangentOverShorterHorizon(
            controller, state + k1 * kHalf, target_state, stage_times * h * kHalf
        ) *
        h
    };
    const State<T, A> k3{
        tangentOverShorterHorizon(
            controller, state + k2 * kHalf, target_state, stage_times * h * kHalf
        ) *
        h
    };
    const State<T, A> k4{
        tangentOverShorterHorizon(controller, state + k3, target_state, stage_times * h) * h
    };
    State<T, A> result{state};
    result += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
    const T norm{attitudeOf(result).euclidean()};
    result.qx() /= norm;
    result.qy() /= norm;
    result.qz() /= norm;
    result.qw() /= norm;
    result.torque_x() = std::clamp(
        result.torque_x(), model.control_lower_bound().torque_x(),
        model.control_upper_bound().torque_x()
    );
    result.torque_y() = std::clamp(
        result.torque_y(), model.control_lower_bound().torque_y(),
        model.control_upper_bound().torque_y()
    );
    result.torque_z() = std::clamp(
        result.torque_z(), model.control_lower_bound().torque_z(),
        model.control_upper_bound().torque_z()
    );
    result.thrust() = std::clamp(
        result.thrust(), model.control_lower_bound().thrust(), model.control_upper_bound().thrust()
    );
    if constexpr (A) {
        result.tau() =
            std::clamp(result.tau(), controller.tau_lower_bound(), controller.tau_upper_bound());
    }
    return result;
}

/**
 * @brief Derivative of f along the field of the state that `field` returns a reference to, from
 * the central difference.
 *
 * The central difference is exact for quadratics, and the cost is quadratic in each control on its
 * own, so it recovers their derivatives up to rounding whatever the step, and a wide step keeps the
 * rounding small.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto derivativeOf(
    const auto& f, const State<T, A>& state, const auto& field, std::type_identity_t<T> step
) -> T {
    const auto at = [&](T shift) -> T {
        State<T, A> shifted{state};
        field(shifted) += shift;
        return f(shifted);
    };
    return (at(step) - at(-step)) / (static_cast<T>(2.0) * step);
}

/**
 * @brief Derivative of f along the field of the state that `field` returns a reference to, from
 * the nine point stencil.
 *
 * The stencil is exact for polynomials up to degree eight, which covers the cost along the
 * horizon: the position residuals are quartic in it, and the units do not move with it. So it
 * recovers that derivative up to rounding whatever the step.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto ninePointDerivativeOf(
    const auto& f, const State<T, A>& state, const auto& field, std::type_identity_t<T> step
) -> T {
    const auto at = [&](T shift) -> T {
        State<T, A> shifted{state};
        field(shifted) += shift;
        return f(shifted);
    };
    return (static_cast<T>(3.0) *
                (at(static_cast<T>(-4.0) * step) - at(static_cast<T>(4.0) * step)) +
            static_cast<T>(32.0) *
                (at(static_cast<T>(3.0) * step) - at(static_cast<T>(-3.0) * step)) +
            static_cast<T>(168.0) *
                (at(static_cast<T>(-2.0) * step) - at(static_cast<T>(2.0) * step)) +
            static_cast<T>(672.0) * (at(step) - at(-step))) /
           (static_cast<T>(840.0) * step);
}

/**
 * @brief Times of the waypoints of a reference: zero, then on to the first at or past duration.
 *
 * The k-th interval spans kWaypointSpacing (0.5 + frac(k phi)), phi the golden ratio, which spreads
 * the intervals evenly over half to one and a half times the mean spacing without them ever
 * settling into a pattern. Only a multiply and a floor in double go into them, which every
 * platform rounds alike.
 */
template <std::floating_point T>
[[nodiscard]] auto waypointTimesOf(T duration) -> std::vector<T> {
    std::vector<T> ts{static_cast<T>(0.0)};
    double t{0.0};
    for (std::size_t k{1}; t < static_cast<double>(duration); ++k) {
        const double phase{static_cast<double>(k) * std::numbers::phi};
        t += kWaypointSpacing * (0.5 + (phase - std::floor(phase)));
        ts.push_back(static_cast<T>(t));
    }
    return ts;
}

// The reference flies a figure of eight, its two horizontal frequencies an octave apart, while it
// climbs and sinks slowly and turns its nose to and fro.
inline constexpr std::array<double, 3> kReferenceAmplitudes{10.0, 5.0, 1.5};
inline constexpr std::array<double, 3> kReferenceFrequencies{0.2, 0.4, 0.15};
inline constexpr double kReferenceHeight{2.0};

/**
 * @brief The derivative of the given order of the reference position, in closed form.
 */
[[nodiscard]] auto referencePositionOf(double t, int order) -> Vec3<double> {
    Vec3<double> result{};
    for (std::size_t i{0}; i < 3; ++i) {
        result[i] = kReferenceAmplitudes[i] * std::pow(kReferenceFrequencies[i], order) *
                    std::sin(kReferenceFrequencies[i] * t + order * std::numbers::pi / 2.0);
    }
    if (order == 0) {
        result.z() += kReferenceHeight;
    }
    return result;
}

[[nodiscard]] auto referenceYawOf(double t) -> double { return 0.3 + 0.4 * std::sin(0.1 * t); }

/**
 * @brief Attitude of the reference: the body z axis along the thrust its acceleration takes, the
 * nose turned towards its yaw.
 *
 * A quadrotor is differentially flat in its position and yaw, so the attitude, the angular velocity
 * and the controls all follow from the reference position and yaw, which is what makes the
 * reference a trajectory of the model. The body x axis is the heading of the yaw projected off the
 * thrust axis.
 */
[[nodiscard]] auto referenceAttitudeOf(double t) -> Quaternion<double> {
    const Vec3<double> thrust_axis{
        (referencePositionOf(t, 2) + Vec3<double>{0.0, 0.0, kGravity<double>}).normalized()
    };
    const double yaw{referenceYawOf(t)};
    const Vec3<double> heading{std::cos(yaw), std::sin(yaw), 0.0};
    const Vec3<double> left{thrust_axis.cross(heading).normalized()};
    const Vec3<double> forward{left.cross(thrust_axis)};
    return Quaternion<double>::fromRotationMatrix(
        Mat33<double>{
            forward.x(), left.x(), thrust_axis.x(), forward.y(), left.y(), thrust_axis.y(),
            forward.z(), left.z(), thrust_axis.z()
        }
    );
}

/**
 * @brief Angular velocity of the reference in the body frame, 2 q* dq/dt, with dq/dt from a central
 * difference of the attitude.
 */
[[nodiscard]] auto referenceOmegaOf(double t) -> Vec3<double> {
    constexpr double kStep{1.0e-5};
    const Quaternion<double> rate{
        (referenceAttitudeOf(t + kStep) - referenceAttitudeOf(t - kStep)) * (0.5 / kStep)
    };
    const Quaternion<double> product{referenceAttitudeOf(t).conjugated() * rate};
    return Vec3<double>{2.0 * product.x(), 2.0 * product.y(), 2.0 * product.z()};
}

/**
 * @brief Controls of the reference: the thrust its acceleration takes, and the torques that turn
 * it as it turns, J alpha + omega x J omega, with alpha from a central difference of omega.
 */
[[nodiscard]] auto referenceControlOf(double t) -> ModelControl<double> {
    constexpr double kStep{1.0e-4};
    const Vec3<double> omega{referenceOmegaOf(t)};
    const Vec3<double> alpha{
        (referenceOmegaOf(t + kStep) - referenceOmegaOf(t - kStep)) * (0.5 / kStep)
    };
    const Vec3<double> inertia{kInertiaXx<double>, kInertiaYy<double>, kInertiaZz<double>};
    const Vec3<double> momentum{
        inertia.x() * omega.x(), inertia.y() * omega.y(), inertia.z() * omega.z()
    };
    const Vec3<double> torque{
        Vec3<double>{inertia.x() * alpha.x(), inertia.y() * alpha.y(), inertia.z() * alpha.z()} +
        omega.cross(momentum)
    };
    return ModelControl<double>{
        torque.x(), torque.y(), torque.z(),
        kMass<double> *
            (referencePositionOf(t, 2) + Vec3<double>{0.0, 0.0, kGravity<double>}).euclidean()
    };
}

/**
 * @brief The reference at time t as a target state of the given precision and mode, worked out in
 * double.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto referenceTargetAt(double t) -> State<T, A> {
    const Vec3<double> position{referencePositionOf(t, 0)};
    const Quaternion<double> attitude{referenceAttitudeOf(t)};
    const Vec3<double> velocity{attitude.conjugated().rotate(referencePositionOf(t, 1))};
    const Vec3<double> omega{referenceOmegaOf(t)};
    return inMode<A>(
        targetAt(
            makeVec3<T>(position.x(), position.y(), position.z()),
            makeVec3<T>(velocity.x(), velocity.y(), velocity.z()), Quaternion<T>{attitude},
            makeVec3<T>(omega.x(), omega.y(), omega.z())
        ),
        std::numeric_limits<T>::quiet_NaN()
    );
}

/**
 * @brief The reference up to duration, handed over as waypoints at the uneven times of
 * waypointTimesOf().
 *
 * Every waypoint is a target state, so that the reference interpolated at any time can be handed
 * to the controller as a preview point (see marchAlong()).
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto referenceOf(T duration) -> Reference<T, A> {
    const std::vector<T> ts{waypointTimesOf(duration)};
    std::vector<State<T, A>> waypoints;
    waypoints.reserve(ts.size());
    for (const T t : ts) {
        waypoints.push_back(referenceTargetAt<T, A>(static_cast<double>(t)));
    }
    return Reference<T, A>{ts, waypoints};
}

/**
 * @brief The start of the reference under its own controls, an adaptive horizon at kHorizon.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto onTheReference() -> State<T, A> {
    State<T, A> state{withHorizon(referenceTargetAt<T, A>(0.0), kHorizon<T>)};
    const ModelControl<double> control{referenceControlOf(0.0)};
    state.torque_x() = static_cast<T>(control.torque_x());
    state.torque_y() = static_cast<T>(control.torque_y());
    state.torque_z() = static_cast<T>(control.torque_z());
    state.thrust() = static_cast<T>(control.thrust());
    return state;
}

/**
 * @brief Marches one integrator num_steps steps of h along the reference from the initial state
 * at time zero, the trace led by that state.
 *
 * Step i starts at t_i = i h and is handed as its target the preview point, the reference
 * interpolated between the two waypoints around t_i plus the horizon the step starts with: the
 * horizon of the controller when that is fixed, the one the state carries when it adapts. No
 * horizon gets past kMaxPreview, so the reference has to reach that far past the last step.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto marchAlong(
    IntegrateMode mode, const Controller<T, A>& controller, const State<T, A>& initial_state,
    const Reference<T, A>& reference, T h, std::size_t num_steps
) -> std::vector<State<T, A>> {
    REQUIRE_GE(reference.maxT(), h * static_cast<T>(num_steps) + kMaxPreview<T>);
    std::vector<State<T, A>> trace;
    trace.reserve(num_steps + 1);
    trace.push_back(initial_state);
    for (std::size_t i{0}; i < num_steps; ++i) {
        const State<T, A> preview_point{
            reference(h * static_cast<T>(i) + horizonOf(controller, trace.back()))
        };
        trace.push_back(stepWith(mode, controller, trace.back(), preview_point, h));
    }
    return trace;
}

/**
 * @brief One closed loop run of every integrator, each trace led by the initial state so that it
 * lines up with ts and with the simultaneous points, the reference interpolated at the same times,
 * which the run is measured and plotted against.
 */
template <std::floating_point T, bool A>
struct ClosedLoop final {
    std::vector<T> ts;
    std::vector<State<T, A>> simultaneous_points;
    std::array<std::vector<State<T, A>>, kIntegrateModes.size()> traces;
    T horizon{};

    [[nodiscard]] auto trace(IntegrateMode mode) const noexcept -> const std::vector<State<T, A>>& {
        return traces[static_cast<std::size_t>(mode)];
    }
};

/**
 * @brief Marches every integrator along the reference (see marchAlong()), each of the num_steps
 * steps to be measured against its simultaneous point, the reference at the time the step starts.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto runClosedLoop(
    const Controller<T, A>& controller, const State<T, A>& initial_state,
    const Reference<T, A>& reference, T h, std::size_t num_steps
) -> ClosedLoop<T, A> {
    ClosedLoop<T, A> loop;
    loop.horizon = horizonOf(controller, initial_state);
    for (std::size_t i{0}; i <= num_steps; ++i) {
        const T t{h * static_cast<T>(i)};
        loop.ts.push_back(t);
        loop.simultaneous_points.push_back(reference(t));
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        loop.traces[static_cast<std::size_t>(mode)] =
            marchAlong(mode, controller, initial_state, reference, h, num_steps);
    }
    return loop;
}

/**
 * @brief Largest distance to the simultaneous points over the steps first through last, with a
 * NaN propagating into the result as in distanceOf().
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto settledDistanceOf(
    const ClosedLoop<T, A>& loop, IntegrateMode mode, std::size_t first, std::size_t last
) -> T {
    const std::vector<State<T, A>>& trace{loop.trace(mode)};
    T result{static_cast<T>(0.0)};
    for (std::size_t i{first}; i <= last; ++i) {
        raiseTo(result, positionOf(trace[i]).euclideanTo(positionOf(loop.simultaneous_points[i])));
    }
    return result;
}

/**
 * @brief Plots the path of every integrator in 3D and each field of its state against time, the
 * path and the plant fields over those of the reference, with the decimal logarithm of the
 * position, attitude and body velocity errors against the simultaneous points. A fixed horizon
 * shows as a flat line. Every curve keeps its color and line style across the nineteen subplots,
 * so that one legend across the top of the figure covers them all.
 *
 * The errors go through log10() rather than onto a log axis because matplot hands its data to
 * gnuplot in fixed point with ten decimals, which rounds small errors to zero, and a log axis drops
 * zeros.
 */
template <bool A>
auto plotClosedLoop(const std::string& title, const ClosedLoop<double, A>& loop) -> void {
    // The subplot of every plant field past the position, as createAxesHandles() lays them out:
    // u, v, w, the attitude and the angular velocity.
    constexpr std::array<std::size_t, 10> kPlantAxes{3, 4, 5, 7, 8, 9, 10, 11, 12, 13};
    // The subplots of the horizon and of the four controls.
    constexpr std::size_t kHorizonAxes{14};
    constexpr std::array<std::size_t, 4> kControlAxes{15, 16, 17, 18};
    // The subplots of the position, attitude and body velocity errors.
    constexpr std::array<std::size_t, 3> kErrorAxes{1, 2, 6};
    const std::array<float, 4> reference_color{matplot::default_color(0)};
    const std::array<float, 4> last_reference_color{matplot::default_color(1)};
    const std::array<std::array<float, 4>, kIntegrateModes.size()> mode_colors{
        matplot::default_color(2), matplot::default_color(4)
    };
    const std::array<const char*, kIntegrateModes.size()> line_specs{":", "--"};

    matplot::figure_handle fig = createFigureHandle();
    fig->title(title);
    std::vector<matplot::axes_handle> axes = createAxesHandles(fig);
    for (std::size_t i{1}; i < axes.size(); ++i) {
        axes[i]->xlim({loop.ts.front(), loop.ts.back()});
    }

    // The controller fields of a target are NaN, so only its plant fields are gathered.
    constexpr std::size_t kNumPlantFields{ModelState<double>::size()};
    constexpr std::size_t kNumControlFields{ModelControl<double>::size()};
    std::array<std::vector<double>, kNumPlantFields> reference_fields;
    for (const State<double, A>& point : loop.simultaneous_points) {
        for (std::size_t f{0}; f < kNumPlantFields; ++f) {
            reference_fields[f].push_back(point[f]);
        }
    }
    axes[0]
        ->plot3(reference_fields[0], reference_fields[1], reference_fields[2], "-")
        ->line_width(2.0)
        .color(reference_color);
    axes[0]
        ->plot3(
            std::vector<double>{reference_fields[0].back()},
            std::vector<double>{reference_fields[1].back()},
            std::vector<double>{reference_fields[2].back()}, "o"
        )
        ->line_width(2.0)
        .color(last_reference_color);
    for (std::size_t f{3}; f < 13; ++f) {
        axes[kPlantAxes[f - 3]]
            ->plot(loop.ts, reference_fields[f], "-")
            ->line_width(2.0)
            .color(reference_color);
    }

    for (const IntegrateMode mode : kIntegrateModes) {
        const auto m{static_cast<std::size_t>(mode)};
        const std::vector<State<double, A>>& trace{loop.trace(mode)};
        std::array<std::vector<double>, kNumPlantFields + kNumControlFields> fields;
        std::vector<double> horizons;
        std::array<std::vector<double>, kErrorAxes.size()> errors;
        for (std::size_t i{0}; i < trace.size(); ++i) {
            for (std::size_t f{0}; f < fields.size(); ++f) {
                fields[f].push_back(trace[i][f]);
            }
            if constexpr (A) {
                horizons.push_back(trace[i].tau());
            } else {
                horizons.push_back(loop.horizon);
            }
            const State<double, A>& point{loop.simultaneous_points[i]};
            errors[0].push_back(positionOf(trace[i]).euclideanTo(positionOf(point)));
            errors[1].push_back(attitudeOf(trace[i]).angularDistance(attitudeOf(point)));
            errors[2].push_back(velocityOf(trace[i]).euclideanTo(velocityOf(point)));
        }
        const std::array<float, 4>& color{mode_colors[m]};
        axes[0]
            ->plot3(fields[0], fields[1], fields[2], line_specs[m])
            ->line_width(2.0)
            .color(color);
        for (std::size_t f{3}; f < 13; ++f) {
            axes[kPlantAxes[f - 3]]
                ->plot(loop.ts, fields[f], line_specs[m])
                ->line_width(2.0)
                .color(color);
        }
        for (std::size_t f{kNumPlantFields}; f < fields.size(); ++f) {
            axes[kControlAxes[f - kNumPlantFields]]
                ->plot(loop.ts, fields[f], line_specs[m])
                ->line_width(2.0)
                .color(color);
        }
        axes[kHorizonAxes]->plot(loop.ts, horizons, line_specs[m])->line_width(2.0).color(color);
        for (std::size_t e{0}; e < errors.size(); ++e) {
            // An error of exactly zero has no logarithm and is left out as a NaN, which gnuplot
            // skips, such as the errors at the start of a run on the reference.
            for (double& error : errors[e]) {
                error = error > 0.0 ? std::log10(error) : std::numeric_limits<double>::quiet_NaN();
            }
            axes[kErrorAxes[e]]
                ->plot(loop.ts, errors[e], line_specs[m])
                ->line_width(2.0)
                .color(color);
        }
    }

    // A matplot figure has no legend of its own, so the legend gets an axes to itself: a strip
    // across the top whose limits keep its curves out of sight, so that only their keys show.
    const matplot::axes_handle key{fig->add_axes({0.05F, 0.945F, 0.9F, 0.025F})};
    key->hold(matplot::on);
    const std::vector<double> out_of_sight{0.0, 1.0};
    key->plot(out_of_sight, out_of_sight, "-")->line_width(2.0).color(reference_color);
    key->plot(out_of_sight, out_of_sight, "o")->line_width(2.0).color(last_reference_color);
    for (const IntegrateMode mode : kIntegrateModes) {
        const auto m{static_cast<std::size_t>(mode)};
        key->plot(out_of_sight, out_of_sight, line_specs[m])->line_width(2.0).color(mode_colors[m]);
    }
    key->xlim({2.0, 3.0});
    key->ylim({2.0, 3.0});
    key->box(false);
    key->x_axis().visible(false);
    key->y_axis().visible(false);
    key->legend({"reference", "last reference", "forward Euler", "Runge–Kutta"});
    key->legend()->num_columns(kIntegrateModes.size() + 2);
    key->legend()->box(false);
    key->legend()->location(matplot::legend::general_alignment::center);
    key->legend()->font_size(12.0);
    fig->show();
}

} // namespace

} // namespace basjoo::quadrotor

TYPE_TO_STRING_AS("FixedFloat", basjoo::quadrotor::FixedFloat);
TYPE_TO_STRING_AS("AdaptiveFloat", basjoo::quadrotor::AdaptiveFloat);
TYPE_TO_STRING_AS("FixedDouble", basjoo::quadrotor::FixedDouble);
TYPE_TO_STRING_AS("AdaptiveDouble", basjoo::quadrotor::AdaptiveDouble);

namespace basjoo::quadrotor {

TEST_CASE_TEMPLATE(
    "ConstructorKeepsTheModelTheWeightsAndTheHorizon", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    // The controller has no mass, inertia or bounds of its own, so model() has to hand back the
    // very model it was built with.
    const DynamicsModel<T>& model{controller.model()};
    CHECK_EQ(model.mass(), kMass<T>);
    CHECK_EQ(model.inertia_xx(), kInertiaXx<T>);
    CHECK_EQ(model.inertia_yy(), kInertiaYy<T>);
    CHECK_EQ(model.inertia_zz(), kInertiaZz<T>);
    CHECK_EQ(model.gravity(), kGravity<T>);
    CHECK_EQ(model.control_lower_bound(), lowerBound<T>());
    CHECK_EQ(model.control_upper_bound(), upperBound<T>());
    // No two weights agree, so a shuffle cannot pass.
    CHECK(std::ranges::equal(controller.weights(), kWeights<T>));
    if constexpr (kAdaptive) {
        CHECK_EQ(controller.tau_lower_bound(), kTauLowerBound<T>);
        CHECK_EQ(controller.tau_upper_bound(), kTauUpperBound<T>);
    } else {
        CHECK_EQ(controller.tau(), kHorizon<T>);
    }
}

TEST_CASE_TEMPLATE(
    "ControllerSerializationRoundtrip", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(controller).code == std::errc{});
    // A default constructed controller has unit weights and a unit, unbounded model, a fixed one a
    // unit horizon and an adaptive one no bounds on its horizon, none of which agrees with the
    // original, so every field checked below has to have come through the archive.
    Controller<T, kAdaptive> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.model().mass(), kMass<T>);
    CHECK_EQ(restored.model().inertia_xx(), kInertiaXx<T>);
    CHECK_EQ(restored.model().inertia_yy(), kInertiaYy<T>);
    CHECK_EQ(restored.model().inertia_zz(), kInertiaZz<T>);
    CHECK_EQ(restored.model().control_lower_bound(), lowerBound<T>());
    CHECK_EQ(restored.model().control_upper_bound(), upperBound<T>());
    CHECK(std::ranges::equal(restored.weights(), kWeights<T>));
    if constexpr (kAdaptive) {
        CHECK_EQ(restored.tau_lower_bound(), kTauLowerBound<T>);
        CHECK_EQ(restored.tau_upper_bound(), kTauUpperBound<T>);
    } else {
        CHECK_EQ(restored.tau(), kHorizon<T>);
    }
    // Same arithmetic along the same code path, so the restored controller has to reproduce the
    // cost of the original to within roundoff: the two calls are inlined separately, and the
    // compiler may fuse them differently.
    CHECK_EQ(
        restored.cost(startOf<T, kAdaptive>(), targetOf<T, kAdaptive>()),
        doctest::Approx(controller.cost(startOf<T, kAdaptive>(), targetOf<T, kAdaptive>()))
            .epsilon(roundoffTolerance<T>())
    );
}

TEST_CASE_TEMPLATE(
    "StateSerializationRoundtrip", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    for (const State<T, kAdaptive>& state : {State<T, kAdaptive>{}, startOf<T, kAdaptive>()}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(state).code == std::errc{});
        State<T, kAdaptive> restored{static_cast<T>(0.0)};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, state);
    }
}

TEST_CASE_TEMPLATE(
    "StateIndexingAndFill", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    using S = State<T, kAdaptive>;
    const S state{startOf<T, kAdaptive>()};

    SUBCASE("IndexingWalksTheFieldsInOrder") {
        // operator[] reaches every field in the order data() lays them out in, and writes through
        // as well as it reads, and the field constructor takes them in that order.
        S written{};
        for (std::size_t i{0}; i < S::size(); ++i) {
            CHECK_EQ(&state[i], state.data() + i);
            written[i] = state[i];
        }
        CHECK_EQ(written, state);
        const S counted{[] -> S {
            if constexpr (kAdaptive) {
                return S{static_cast<T>(0.0),  static_cast<T>(1.0),  static_cast<T>(2.0),
                         static_cast<T>(3.0),  static_cast<T>(4.0),  static_cast<T>(5.0),
                         static_cast<T>(6.0),  static_cast<T>(7.0),  static_cast<T>(8.0),
                         static_cast<T>(9.0),  static_cast<T>(10.0), static_cast<T>(11.0),
                         static_cast<T>(12.0), static_cast<T>(13.0), static_cast<T>(14.0),
                         static_cast<T>(15.0), static_cast<T>(16.0), static_cast<T>(17.0)};
            } else {
                return S{static_cast<T>(0.0),  static_cast<T>(1.0),  static_cast<T>(2.0),
                         static_cast<T>(3.0),  static_cast<T>(4.0),  static_cast<T>(5.0),
                         static_cast<T>(6.0),  static_cast<T>(7.0),  static_cast<T>(8.0),
                         static_cast<T>(9.0),  static_cast<T>(10.0), static_cast<T>(11.0),
                         static_cast<T>(12.0), static_cast<T>(13.0), static_cast<T>(14.0),
                         static_cast<T>(15.0), static_cast<T>(16.0)};
            }
        }()};
        std::vector<T> accessed{counted.x(),        counted.y(),        counted.z(),
                                counted.u(),        counted.v(),        counted.w(),
                                counted.qx(),       counted.qy(),       counted.qz(),
                                counted.qw(),       counted.omega_x(),  counted.omega_y(),
                                counted.omega_z(),  counted.torque_x(), counted.torque_y(),
                                counted.torque_z(), counted.thrust()};
        if constexpr (kAdaptive) {
            accessed.push_back(counted.tau());
        }
        REQUIRE_EQ(accessed.size(), S::size());
        for (std::size_t i{0}; i < S::size(); ++i) {
            CHECK_EQ(accessed[i], static_cast<T>(i));
        }
    }

    SUBCASE("FillConstructorSetsEveryField") {
        const S filled{static_cast<T>(0.75)};
        for (std::size_t i{0}; i < S::size(); ++i) {
            CHECK_EQ(filled[i], static_cast<T>(0.75));
        }
        // Filled with zero, the attitude and the horizon included, a state is the zero generic
        // code adds, which the default state is not, as its attitude defaults to the identity.
        CHECK_EQ(state + S{static_cast<T>(0.0)}, state);
        CHECK_NE(state + S{}, state);
    }

    SUBCASE("DefaultIsAtRestUnderTheIdentityAttitude") {
        // The attitude defaults to the identity and, where there is one, the horizon to one.
        const S rest{};
        for (std::size_t i{0}; i < S::size(); ++i) {
            CHECK_EQ(rest[i], i == 9 || i == 17 ? static_cast<T>(1.0) : static_cast<T>(0.0));
        }
    }
}

TEST_CASE_TEMPLATE(
    "StateTrajectoriesInterpolate", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    // A state is a vector in the sense of VecArithmetic, which is all the piecewise functions and
    // their proxy ask of a value type. Every field below is affine in t, which linear, natural
    // cubic and quintic splines all reproduce exactly, so what is left is the roundoff of the
    // spline solves.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    using S = State<T, kAdaptive>;
    using ::basjoo::math::FunctionProxy;
    using ::basjoo::math::makeFunctionProxy;
    using ::basjoo::math::PiecewiseCubicFunction;
    using ::basjoo::math::PiecewiseLinearFunction;
    using ::basjoo::math::PiecewiseQuinticFunction;
    static_assert(::basjoo::math::VecArithmetic<S>);
    static_assert(::basjoo::math::isVecArithmeticV<S>);
    static_assert(!::basjoo::math::MatArithmetic<S>);
    static_assert(!::basjoo::math::isMatArithmeticV<S>);
    constexpr T kTolerance{
        std::same_as<T, float> ? static_cast<T>(1.0e-4) : static_cast<T>(1.0e-10)
    };

    const S origin{startOf<T, kAdaptive>()};
    S rate{};
    for (std::size_t i{0}; i < S::size(); ++i) {
        rate[i] = static_cast<T>(0.25) * static_cast<T>(i) - static_cast<T>(1.5);
    }
    const auto exact = [&](T t) -> S { return origin + rate * t; };
    // Unevenly spaced, so that no spline solve gets to lean on equal intervals.
    const std::vector<T> ts{
        static_cast<T>(0.0), static_cast<T>(0.5), static_cast<T>(1.25), static_cast<T>(2.0),
        static_cast<T>(3.0)
    };
    std::vector<S> ys;
    ys.reserve(ts.size());
    for (const T t : ts) {
        ys.push_back(exact(t));
    }
    const T lower{static_cast<T>(0.3)};
    const T upper{static_cast<T>(2.6)};
    const S area{
        origin * (upper - lower) + rate * ((upper * upper - lower * lower) * static_cast<T>(0.5))
    };

    std::vector<FunctionProxy<S>> functions;
    functions.emplace_back(makeFunctionProxy(PiecewiseLinearFunction<S>{ts, ys}));
    functions.emplace_back(makeFunctionProxy(PiecewiseCubicFunction<S>{ts, ys}));
    functions.emplace_back(makeFunctionProxy(PiecewiseQuinticFunction<S>{ts, ys}));
    for (const FunctionProxy<S>& function : functions) {
        for (const T t :
             {static_cast<T>(0.2), static_cast<T>(0.9), static_cast<T>(1.6), static_cast<T>(2.7)}) {
            CHECK_LE(distanceOf(function->eval(t), exact(t)), kTolerance);
            CHECK_LE(distanceOf(function->derivative(t), rate), kTolerance);
        }
        CHECK_LE(distanceOf(function->integral(lower, upper), area), kTolerance);

        // Restored into an empty proxy, so the concrete function has to come through the archive.
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(function).code == std::errc{});
        FunctionProxy<S> restored{};
        REQUIRE(in(restored).code == std::errc{});
        REQUIRE(restored.has_value());
        CHECK(proxy_typeid(*restored) == proxy_typeid(*function));
        CHECK_EQ(restored->eval(lower), function->eval(lower));
    }
}

TEST_CASE_TEMPLATE(
    "TangentCarriesThePlantFlowOfTheModel", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const DynamicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const auto loose = [](State<T, kAdaptive> state) -> State<T, kAdaptive> {
        state.torque_x() = static_cast<T>(5.0);
        state.torque_y() = static_cast<T>(-3.0);
        state.torque_z() = static_cast<T>(1.0);
        state.thrust() = static_cast<T>(100.0);
        return state;
    };

    SUBCASE("PlantFieldsEqualTheModelTangent") {
        // The controller spells the plant flow out itself instead of calling the model, so that the
        // flow and the cost can share their rotation. This is what keeps that copy honest: any
        // change to DynamicsModel::tangent() that is not mirrored in the controller fails here, and
        // so does a clamp that went missing. The two spell the same arithmetic out at different
        // call sites, which the compiler fuses into FMAs differently, so they agree to roundoff
        // rather than bit for bit.
        for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
            for (const State<T, kAdaptive>& handed : {state, loose(state)}) {
                CHECK_LE(
                    distanceOf(
                        plantOf(controller.tangent(handed, target)),
                        model.tangent(plantOf(handed), controlOf(handed))
                    ),
                    roundoffTolerance<T>()
                );
            }
        }
    }

    SUBCASE("LooseControlsChangeNothing") {
        // The controls enter the flow and the cost only through their bounds, so controls beyond
        // the bounds may not move a single bit of the tangent.
        for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
            State<T, kAdaptive> clamped{state};
            clamped.torque_x() = kTorqueXUp<T>;
            clamped.torque_y() = kTorqueYLo<T>;
            clamped.torque_z() = kTorqueZUp<T>;
            clamped.thrust() = kThrustUp<T>;
            CHECK_EQ(controller.tangent(loose(state), target), controller.tangent(clamped, target));
        }
    }
}

TEST_CASE_TEMPLATE(
    "CostIsHalfTheWeightedSumOfTheResiduals", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // cost() and the rates share their residuals, so the residuals are spelled out here once more,
    // straight from the formulas documented with DynamicsFlowController::tangent() and in terms of
    // math::Vec3 and math::Quaternion. The two sides arrange the arithmetic differently and round
    // differently.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const T g{kGravity<T>};
    // The units of the cost, L0 = sqrt(J_xx / m) and T0 = sqrt(L0 / g).
    const T length{std::sqrt(kInertiaXx<T> / kMass<T>)};
    const T time{std::sqrt(length / g)};
    const Vec3<T> up{makeVec3<T>(0.0, 0.0, 1.0)};
    const auto times_inertia = [](const Vec3<T>& a) -> Vec3<T> {
        return Vec3<T>{kInertiaXx<T> * a.x(), kInertiaYy<T> * a.y(), kInertiaZz<T> * a.z()};
    };
    const auto over_inertia = [](const Vec3<T>& a) -> Vec3<T> {
        return Vec3<T>{a.x() / kInertiaXx<T>, a.y() / kInertiaYy<T>, a.z() / kInertiaZz<T>};
    };
    for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
        const T tau{horizonOf(controller, state)};
        const Quaternion<T> q{attitudeOf(state)};
        const Vec3<T> velocity{velocityOf(state)};
        const Vec3<T> omega{omegaOf(state)};
        const Vec3<T> torque{state.torque_x(), state.torque_y(), state.torque_z()};
        const T lift{state.thrust() / kMass<T>};
        const Vec3<T> alpha{over_inertia(torque - omega.cross(times_inertia(omega)))};
        const Vec3<T> alpha_dot{
            over_inertia(-(alpha.cross(times_inertia(omega)) + omega.cross(times_inertia(alpha))))
        };
        const Vec3<T> b{q.conjugated().rotate(up)};
        const Vec3<T> a0{q.rotate(up) * lift - up * g};
        const Vec3<T> a1{q.rotate(omega.cross(up)) * lift};
        const Vec3<T> a2{q.rotate(omega.cross(omega.cross(up)) + alpha.cross(up)) * lift};
        const Vec3<T> rr{
            positionOf(state) - positionOf(target) + q.rotate(velocity) * tau +
            a0 * (tau * tau / static_cast<T>(2.0)) + a1 * (tau * tau * tau / static_cast<T>(6.0)) +
            a2 * (tau * tau * tau * tau / static_cast<T>(24.0))
        };
        const Vec3<T> v1{-omega.cross(velocity) + up * lift - b * g};
        const Vec3<T> v2{-alpha.cross(velocity) - omega.cross(v1) + omega.cross(b) * g};
        const Vec3<T> v3{
            -alpha_dot.cross(velocity) - alpha.cross(v1) * static_cast<T>(2.0) - omega.cross(v2) +
            alpha.cross(b) * g - omega.cross(omega.cross(b)) * g
        };
        const Vec3<T> rv{
            velocity - velocityOf(target) + v1 * tau + v2 * (tau * tau / static_cast<T>(2.0)) +
            v3 * (tau * tau * tau / static_cast<T>(6.0))
        };
        const Quaternion<T> q1{
            q * Quaternion<T>{omega.x(), omega.y(), omega.z(), static_cast<T>(0.0)} *
            static_cast<T>(0.5)
        };
        const Quaternion<T> q2{
            q * Quaternion<T>{alpha.x(), alpha.y(), alpha.z(), static_cast<T>(0.0)} *
                static_cast<T>(0.5) -
            q * (omega.euclideanSqr() / static_cast<T>(4.0))
        };
        const T sign{
            q.dot(attitudeOf(target)) < static_cast<T>(0.0) ? static_cast<T>(-1.0)
                                                            : static_cast<T>(1.0)
        };
        const Quaternion<T> rq{
            (q + q1 * tau + q2 * (tau * tau / static_cast<T>(2.0)) - attitudeOf(target) * sign) *
            static_cast<T>(2.0)
        };
        const Vec3<T> rw{omega - omegaOf(target) + alpha * tau};
        const T expected{
            (kWeights<T>[0] * rr.euclideanSqr() / (length * length) +
             kWeights<T>[1] * time * time * rv.euclideanSqr() / (length * length) +
             kWeights<T>[2] * rq.euclideanSqr() +
             kWeights<T>[3] * time * time * rw.euclideanSqr()) /
            static_cast<T>(2.0)
        };
        CHECK_EQ(
            controller.cost(state, target),
            doctest::Approx(expected).epsilon(derivativeTolerance<T>())
        );
    }
}

TEST_CASE_TEMPLATE(
    "PredictionIsExactUnderAConstantThrustAtFixedAttitude", P, FixedFloat, AdaptiveFloat,
    FixedDouble, AdaptiveDouble
) {
    // Without torques and without a rotation the attitude stays put, so the body acceleration is
    // constant and the vehicle moves by a polynomial of degree two in time, which the prediction
    // takes in exactly: every residual against where the vehicle gets to vanishes. A target a
    // centimetre off sets the scale the cost is held to. The horizon lies inside the bounds of an
    // adaptive controller, which would clamp it otherwise.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    constexpr T kTau{static_cast<T>(0.4)};
    const Controller<T, kAdaptive> controller{
        controllerOf<T, kAdaptive>(unboundedModel<T>(), kWeights<T>, kTau)
    };
    const Vec3<T> position{makeVec3<T>(1.0, -0.5, 3.0)};
    const Vec3<T> velocity{makeVec3<T>(1.0, -0.5, 0.25)};
    const Quaternion<T> attitude{attitudeFromRpy<T>(0.2, -0.3, 0.5)};
    const Vec3<T> rest{makeVec3<T>(0.0, 0.0, 0.0)};
    const T thrust{static_cast<T>(1.3) * kHoverThrust<T>};
    const State<T, kAdaptive> state{inMode<kAdaptive>(
        stateOf(
            position, velocity, attitude, rest,
            ModelControl<T>{static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), thrust}
        ),
        kTau
    )};
    const Vec3<T> up{makeVec3<T>(0.0, 0.0, 1.0)};
    const Vec3<T> acceleration{
        up * (thrust / kMass<T>)-attitude.conjugated().rotate(up) * kGravity<T>
    };
    const State<T, kAdaptive> target{inMode<kAdaptive>(
        targetAt(
            position + attitude.rotate(
                           velocity * kTau + acceleration * (static_cast<T>(0.5) * kTau * kTau)
                       ),
            velocity + acceleration * kTau, attitude, rest
        ),
        std::numeric_limits<T>::quiet_NaN()
    )};
    State<T, kAdaptive> off{target};
    off.x() += static_cast<T>(0.01);
    const T off_cost{controller.cost(state, off)};
    REQUIRE_GT(off_cost, static_cast<T>(0.0));
    CHECK_LE(controller.cost(state, target), roundoffTolerance<T>() * off_cost);
}

TEST_CASE_TEMPLATE(
    "PredictionErrorIsOfFifthOrderOnASteadyClimbingTurn", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // In a steady climbing turn the controls and the body velocities stay constant, and the
    // prediction is exact through tau^4, so the position residual against where the vehicle gets
    // to is of fifth order in the horizon: halving the horizon divides it by 32. Only the position
    // is weighed, so the cost is half that residual squared over L0^2. The turn is the one of
    // SteadyClimbingTurn in the model test: banked by atan(speed * turn rate / g), its torques
    // balancing the gyroscopic term, its nose 0.3 rad off the direction of flight. Both horizons
    // lie within the bounds of an adaptive controller.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const DynamicsModel<T> model{unboundedModel<T>()};
    const std::array<T, 4> position_only{
        static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0)
    };
    constexpr T kSpeed{static_cast<T>(3.0)};
    constexpr T kTurnRate{static_cast<T>(1.0)};
    constexpr T kClimbRate{static_cast<T>(0.5)};
    constexpr T kRadius{kSpeed / kTurnRate};
    const Vec3<T> up{makeVec3<T>(0.0, 0.0, 1.0)};
    const Vec3<T> start{kRadius, static_cast<T>(0.0), static_cast<T>(2.0)};
    const Vec3<T> world_velocity{static_cast<T>(0.0), kSpeed, kClimbRate};
    const Vec3<T> centripetal{-kSpeed * kTurnRate, static_cast<T>(0.0), static_cast<T>(0.0)};
    const Vec3<T> lift{centripetal + up * kGravity<T>};
    const Quaternion<T> attitude{
        Quaternion<T>::fromTwoVectors(up, lift) *
        attitudeFromRpy<T>(0.0, 0.0, std::numbers::pi / 2.0 + 0.3)
    };
    const Vec3<T> omega{attitude.conjugated().rotate(up * kTurnRate)};
    const Vec3<T> torque{omega.cross(
        Vec3<T>{kInertiaXx<T> * omega.x(), kInertiaYy<T> * omega.y(), kInertiaZz<T> * omega.z()}
    )};
    const Vec3<T> velocity{attitude.conjugated().rotate(world_velocity)};
    const State<T, false> state{stateOf(
        start, velocity, attitude, omega,
        ModelControl<T>{torque.x(), torque.y(), torque.z(), kMass<T> * lift.euclidean()}
    )};
    const T length{std::sqrt(kInertiaXx<T> / kMass<T>)};
    const auto residual = [&](T tau) -> T {
        const Controller<T, kAdaptive> controller{
            controllerOf<T, kAdaptive>(model, position_only, tau)
        };
        const T angle{kTurnRate * tau};
        const State<T, kAdaptive> target{inMode<kAdaptive>(
            targetAt(
                Vec3<T>{
                    kRadius * std::cos(angle), kRadius * std::sin(angle),
                    start.z() + kClimbRate * tau
                },
                velocity,
                Quaternion<T>::fromRotationVector(static_cast<T>(0.0), static_cast<T>(0.0), angle) *
                    attitude,
                omega
            ),
            std::numeric_limits<T>::quiet_NaN()
        )};
        return length *
               std::sqrt(
                   static_cast<T>(2.0) * controller.cost(inMode<kAdaptive>(state, tau), target)
               );
    };
    const T ratio{residual(static_cast<T>(0.4)) / residual(static_cast<T>(0.2))};
    CHECK_GT(ratio, static_cast<T>(30.0));
    CHECK_LT(ratio, static_cast<T>(34.0));
}

TEST_CASE_TEMPLATE(
    "RatesAreThePreconditionedNegativeGradientOfTheCost", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // The rates are d(torque_i)/dt = -(J_ii^2 / (T0^2 tau^3)) dJ/d(torque_i),
    // d(thrust)/dt = -(m^2 g^2 T0^2 / tau^3) dJ/d(thrust) and, when adaptive,
    // dtau/dt = -tau dJ/dtau. They take the derivatives of the residuals in closed form, while the
    // finite differences recover those of cost() up to rounding, so a slip in the closed forms
    // shows here. The model is unbounded, so no clamp cuts into the differences, and the horizons
    // of statesOf() lie far enough inside their bounds that none cuts into the widest shift of the
    // horizon either.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(unboundedModel<T>())};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const auto cost = [&](const State<T, kAdaptive>& state) -> T {
        return controller.cost(state, target);
    };
    const T time{std::sqrt(std::sqrt(kInertiaXx<T> / kMass<T>) / kGravity<T>)};
    constexpr T kTorqueStep{static_cast<T>(1.0)};
    constexpr T kThrustStep{static_cast<T>(10.0)};
    constexpr T kTauStep{static_cast<T>(0.01)};
    const double tolerance{derivativeTolerance<T>()};
    for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
        const T tau{horizonOf(controller, state)};
        const T tau3{tau * tau * tau};
        const T torque_scale{time * time * tau3};
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        CHECK_EQ(
            k.torque_x(),
            doctest::Approx(
                -kInertiaXx<T> * kInertiaXx<T> / torque_scale *
                derivativeOf(
                    cost, state, [](auto& s) -> auto& { return s.torque_x(); }, kTorqueStep
                )
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            k.torque_y(),
            doctest::Approx(
                -kInertiaYy<T> * kInertiaYy<T> / torque_scale *
                derivativeOf(
                    cost, state, [](auto& s) -> auto& { return s.torque_y(); }, kTorqueStep
                )
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            k.torque_z(),
            doctest::Approx(
                -kInertiaZz<T> * kInertiaZz<T> / torque_scale *
                derivativeOf(
                    cost, state, [](auto& s) -> auto& { return s.torque_z(); }, kTorqueStep
                )
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            k.thrust(), doctest::Approx(
                            -kMass<T> * kMass<T> * kGravity<T> * kGravity<T> * time * time / tau3 *
                            derivativeOf(
                                cost, state, [](auto& s) -> auto& { return s.thrust(); },
                                kThrustStep
                            )
                        )
                            .epsilon(tolerance)
        );
        if constexpr (kAdaptive) {
            CHECK_EQ(
                k.tau(),
                doctest::Approx(
                    -tau * ninePointDerivativeOf(
                               cost, state, [](auto& s) -> auto& { return s.tau(); }, kTauStep
                           )
                )
                    .epsilon(tolerance)
            );
        }
    }
}

TEST_CASE_TEMPLATE(
    "ControlRatesDescendThePreconditionedGradient", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const DynamicsModel<T> model{boundedModel<T>()};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    constexpr T kWeight{static_cast<T>(0.6)};
    constexpr T kZero{static_cast<T>(0.0)};

    SUBCASE("AngularVelocityTermAlone") {
        // Left with its angular velocity term alone, the cost reads J = w3 T0^2 |R|^2 / 2 with
        // R = omega + tau alpha - omega_target, whose gradient dJ/d(torque_i) = w3 T0^2 tau R_i /
        // J_ii has a closed form, and every torque descends it with the preconditioner
        // J_ii^2 / (T0^2 tau^3). Put differently, an Euler step of length h moves a torque by
        // w3 h / tau times the Newton step -J_ii R_i / tau. The thrust has nothing left to descend.
        const Controller<T, kAdaptive> controller{
            controllerOf<T, kAdaptive>(model, {kZero, kZero, kZero, kWeight})
        };
        const State<T, kAdaptive> state{startOf<T, kAdaptive>()};
        const T tau{horizonOf(controller, state)};
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        const Vec3<T> alpha{
            model.tangent(plantOf(state), controlOf(state)).omega_x(),
            model.tangent(plantOf(state), controlOf(state)).omega_y(),
            model.tangent(plantOf(state), controlOf(state)).omega_z()
        };
        const Vec3<T> residual{omegaOf(state) + alpha * tau - omegaOf(target)};
        const T tau2{tau * tau};
        CHECK_EQ(
            k.torque_x(), doctest::Approx(-kWeight * kInertiaXx<T> * residual.x() / tau2)
                              .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(
            k.torque_y(), doctest::Approx(-kWeight * kInertiaYy<T> * residual.y() / tau2)
                              .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(
            k.torque_z(), doctest::Approx(-kWeight * kInertiaZz<T> * residual.z() / tau2)
                              .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(k.thrust(), kZero);
    }

    SUBCASE("VelocityTermAlone") {
        // Left with its velocity term alone, the cost reads J = w1 T0^2 |R|^2 / (2 L0^2), and while
        // the vehicle neither rotates nor is torqued the thrust enters R = v - v_target +
        // tau (thrust / m e_z - g R^T e_z) through tau / m along the body z axis alone. Its
        // gradient dJ/d(thrust) = w1 T0^2 tau R_w / (m L0^2) then has a closed form, and the thrust
        // descends it with the preconditioner m^2 g^2 T0^2 / tau^3, which, as L0 = g T0^2, again
        // comes to w1 h / tau times the Newton step -m R_w / tau per Euler step.
        const Controller<T, kAdaptive> controller{
            controllerOf<T, kAdaptive>(model, {kZero, kWeight, kZero, kZero})
        };
        const Quaternion<T> attitude{attitudeFromRpy<T>(0.3, -0.2, 0.7)};
        const State<T, kAdaptive> state{inMode<kAdaptive>(
            stateOf(
                makeVec3<T>(0.4, -0.3, 2.0), makeVec3<T>(1.2, -0.7, 0.4), attitude,
                makeVec3<T>(0.0, 0.0, 0.0),
                ModelControl<T>{kZero, kZero, kZero, static_cast<T>(1.3) * kHoverThrust<T>}
            ),
            static_cast<T>(0.35)
        )};
        const T tau{horizonOf(controller, state)};
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        const T body_up_gravity{attitude.conjugated().rotate(makeVec3<T>(0.0, 0.0, 1.0)).z()};
        const T residual{
            state.w() - target.w() +
            tau * (state.thrust() / kMass<T> - kGravity<T> * body_up_gravity)
        };
        CHECK_EQ(
            k.thrust(), doctest::Approx(-kWeight * kMass<T> * residual / (tau * tau))
                            .epsilon(roundoffTolerance<T>())
        );
    }

    SUBCASE("MixedPartialsMatchThePreconditioners") {
        // The rates are preconditioned partial derivatives of one and the same cost, so the cross
        // derivatives of the thrust rate by a torque and of that torque rate by the thrust are the
        // one mixed partial of J scaled by the two preconditioners, whose ratio is
        // m^2 g^2 T0^4 / J_ii^2 whatever the weights, the state, the target and the horizon. Every
        // rate is quadratic in each control on its own, so central differences recover those
        // derivatives up to rounding whatever the step. The model is unbounded so that no clamp
        // cuts into them.
        const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(unboundedModel<T>())};
        const State<T, kAdaptive> state{startOf<T, kAdaptive>()};
        const T time{std::sqrt(std::sqrt(kInertiaXx<T> / kMass<T>) / kGravity<T>)};
        constexpr T kTorqueStep{static_cast<T>(1.0)};
        constexpr T kThrustStep{static_cast<T>(10.0)};
        const auto rates_at = [&](std::size_t field, T shift) -> State<T, kAdaptive> {
            State<T, kAdaptive> shifted{state};
            shifted[field] += shift;
            return controller.tangent(shifted, target);
        };
        const std::array<T, 3> inertias{kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>};
        constexpr std::size_t kThrustField{16};
        for (std::size_t i{0}; i < 3; ++i) {
            CAPTURE(i);
            const std::size_t torque_field{13 + i};
            const T thrust_rate_by_torque{
                (rates_at(torque_field, kTorqueStep)[kThrustField] -
                 rates_at(torque_field, -kTorqueStep)[kThrustField]) /
                (static_cast<T>(2.0) * kTorqueStep)
            };
            const T torque_rate_by_thrust{
                (rates_at(kThrustField, kThrustStep)[torque_field] -
                 rates_at(kThrustField, -kThrustStep)[torque_field]) /
                (static_cast<T>(2.0) * kThrustStep)
            };
            REQUIRE_GT(std::abs(torque_rate_by_thrust), static_cast<T>(1.0e-3));
            CHECK_EQ(
                thrust_rate_by_torque / torque_rate_by_thrust,
                doctest::Approx(
                    kMass<T> * kMass<T> * kGravity<T> * kGravity<T> * time * time * time * time /
                    (inertias[i] * inertias[i])
                )
                    .epsilon(derivativeTolerance<T>())
            );
        }
    }
}

TEST_CASE_TEMPLATE(
    "CostAndRatesAreInvariantUnderMotionsAboutTheVertical", P, FixedFloat, AdaptiveFloat,
    FixedDouble, AdaptiveDouble
) {
    // One weight covers the three components of a vector together and the body frame quantities
    // do not see the world frame, so turning the state and the target together about the vertical,
    // the axis gravity leaves free, and shifting them together moves neither the cost nor the rates
    // of the controller fields.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const Quaternion<T> turn{Quaternion<T>::fromRotationVector(
        static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.7)
    )};
    const Vec3<T> shift{makeVec3<T>(5.0, -3.0, 2.0)};
    const auto moved = [&](State<T, kAdaptive> state) -> State<T, kAdaptive> {
        const Vec3<T> position{turn.rotate(positionOf(state)) + shift};
        const Quaternion<T> attitude{turn * attitudeOf(state)};
        state.x() = position.x();
        state.y() = position.y();
        state.z() = position.z();
        state.qx() = attitude.x();
        state.qy() = attitude.y();
        state.qz() = attitude.z();
        state.qw() = attitude.w();
        return state;
    };
    const double tolerance{derivativeTolerance<T>()};
    for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        const State<T, kAdaptive> k_moved{controller.tangent(moved(state), moved(target))};
        CHECK_EQ(
            controller.cost(moved(state), moved(target)),
            doctest::Approx(controller.cost(state, target)).epsilon(tolerance)
        );
        CHECK_EQ(k_moved.torque_x(), doctest::Approx(k.torque_x()).epsilon(tolerance));
        CHECK_EQ(k_moved.torque_y(), doctest::Approx(k.torque_y()).epsilon(tolerance));
        CHECK_EQ(k_moved.torque_z(), doctest::Approx(k.torque_z()).epsilon(tolerance));
        CHECK_EQ(k_moved.thrust(), doctest::Approx(k.thrust()).epsilon(tolerance));
        if constexpr (kAdaptive) {
            CHECK_EQ(k_moved.tau(), doctest::Approx(k.tau()).epsilon(tolerance));
        }
    }
}

TEST_CASE_TEMPLATE(
    "SingleStepIntegratorsDriveThePlantThroughTheModel", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const DynamicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    const T tolerance{roundoffTolerance<T>()};
    const T h{kH<T>};
    const State<T, kAdaptive> start{startOf<T, kAdaptive>()};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const State<T, kAdaptive> k{controller.tangent(start, target)};
    // The comparisons below hold the integrators to a tolerance, so the rates at this start have to
    // be large enough that holding a control still over a step, or stepping the plant under the
    // updated controls instead of the held ones, lands far outside it. The latter moves the
    // angular velocity by h^2 times a torque rate over an inertia and the body velocity by h^2
    // times the thrust rate over the mass, which outweighs every inertia here.
    REQUIRE_GT(h * h * smallestRateOf(k) / kMass<T>, static_cast<T>(10.0) * tolerance);

    SUBCASE("ForwardEulerPlantFieldsFollowTheModel") {
        // The plant half of a forward Euler step is the model's own step under the controls the
        // step starts with, its attitude normalized as there, so the thirteen plant fields have to
        // land where the model puts them.
        CHECK_LE(
            distanceOf(
                plantOf(controller.forwardEuler(start, target, h)),
                model.forwardEuler(plantOf(start), controlOf(start), h)
            ),
            tolerance
        );
    }

    SUBCASE("ForwardEulerControllerFieldsStepAlongTheRates") {
        // The controls, and an adaptive horizon, take one Euler step along the rates tangent()
        // gives at the start of the step, clamped into their bounds. The horizon of startOf()
        // falls so fast towards targetOf() that the step takes it to its lower bound.
        const State<T, kAdaptive> stepped{controller.forwardEuler(start, target, h)};
        CHECK_EQ(
            stepped.torque_x(),
            doctest::Approx(
                std::clamp(start.torque_x() + k.torque_x() * h, kTorqueXLo<T>, kTorqueXUp<T>)
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            stepped.torque_y(),
            doctest::Approx(
                std::clamp(start.torque_y() + k.torque_y() * h, kTorqueYLo<T>, kTorqueYUp<T>)
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            stepped.torque_z(),
            doctest::Approx(
                std::clamp(start.torque_z() + k.torque_z() * h, kTorqueZLo<T>, kTorqueZUp<T>)
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            stepped.thrust(),
            doctest::Approx(std::clamp(start.thrust() + k.thrust() * h, kThrustLo<T>, kThrustUp<T>))
                .epsilon(tolerance)
        );
        if constexpr (kAdaptive) {
            REQUIRE_LT(start.tau() + k.tau() * h, kTauLowerBound<T>);
            CHECK_EQ(stepped.tau(), kTauLowerBound<T>);
        }
    }

    SUBCASE("RungeKuttaStagesSeeTheHorizonLessTheirStageTime") {
        // The target is held over the step while the vehicle moves on towards it, so the two
        // midpoint stages look ahead over the horizon less h/2 and the last one over the horizon
        // less h. Handing every stage the full horizon instead is required to land well outside
        // the tolerance, or the comparison could not tell the two apart. An adaptive horizon gets
        // bounds wide enough for its stages to stay inside, where rungeKuttaThroughTangent()
        // shortens it the way rungeKutta() does.
        const Controller<T, kAdaptive> wide{controllerOf<T, kAdaptive>(
            model, kWeights<T>, kHorizon<T>, static_cast<T>(0.05), static_cast<T>(0.6)
        )};
        const State<T, kAdaptive> expected{
            rungeKuttaThroughTangent(wide, start, target, h, static_cast<T>(1.0))
        };
        REQUIRE_GT(
            distanceOf(
                expected, rungeKuttaThroughTangent(wide, start, target, h, static_cast<T>(0.0))
            ),
            static_cast<T>(10.0) * tolerance
        );
        CHECK_LE(distanceOf(wide.rungeKutta(start, target, h), expected), tolerance);
    }

    SUBCASE("ControlsNeverLeaveTheBounds") {
        // A target far out of reach, looked ahead to over a twentieth of a second, drives every
        // control rate hard enough that a single unclamped step overshoots a bound, so the clamp at
        // the end of every integrator is all that keeps the controls in. The overshoot is required
        // first, or the checks below would hold trivially. An adaptive horizon is pinned to the
        // twentieth of a second by bounds that coincide.
        const State<T, kAdaptive> distant{inMode<kAdaptive>(
            targetAt(
                makeVec3<T>(40.0, -30.0, 20.0), makeVec3<T>(10.0, -8.0, 5.0),
                attitudeFromRpy<T>(-1.0, 0.8, 2.0), makeVec3<T>(-3.0, 2.0, 4.0)
            ),
            std::numeric_limits<T>::quiet_NaN()
        )};
        constexpr T kShortHorizon{static_cast<T>(0.05)};
        const Controller<T, kAdaptive> short_sighted{controllerOf<T, kAdaptive>(
            model, kWeights<T>, kShortHorizon, kShortHorizon, kShortHorizon
        )};
        const State<T, kAdaptive> pull{short_sighted.tangent(start, distant)};
        const State<T, kAdaptive> unclamped{start + pull * h};
        REQUIRE_FALSE((
            kTorqueXLo<T> <= unclamped.torque_x() && unclamped.torque_x() <= kTorqueXUp<T>
        ));
        REQUIRE_FALSE((
            kTorqueYLo<T> <= unclamped.torque_y() && unclamped.torque_y() <= kTorqueYUp<T>
        ));
        REQUIRE_FALSE((
            kTorqueZLo<T> <= unclamped.torque_z() && unclamped.torque_z() <= kTorqueZUp<T>
        ));
        REQUIRE_FALSE((kThrustLo<T> <= unclamped.thrust() && unclamped.thrust() <= kThrustUp<T>));
        // Controls handed in beyond the bounds are only held to come back inside them. Where they
        // land in there is left open on purpose: the controller steps from the control as handed
        // in and clamps afterwards while the plant flow clamps first, and the two only part ways
        // for a control that was out of bounds to begin with.
        State<T, kAdaptive> loose{start};
        loose.torque_x() = static_cast<T>(5.0);
        loose.torque_y() = static_cast<T>(-3.0);
        loose.torque_z() = static_cast<T>(1.0);
        loose.thrust() = static_cast<T>(100.0);
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const State<T, kAdaptive>& handed : {start, loose}) {
                CHECK(areControlsWithinBounds(stepWith(mode, short_sighted, handed, distant, h)));
            }
        }
    }
}

TEST_CASE_TEMPLATE(
    "EvolveMatchesMarchingSingleSteps", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    const State<T, kAdaptive> start{startOf<T, kAdaptive>()};
    const State<T, kAdaptive> goal{targetOf<T, kAdaptive>()};
    // No two entries agree, within a span or across the spans, so an evolve() that read the wrong
    // index cannot pass. Every step stays shorter than the horizon, as rungeKutta() requires.
    const std::vector<T> hs{
        static_cast<T>(0.010), static_cast<T>(0.012), static_cast<T>(0.008), static_cast<T>(0.011),
        static_cast<T>(0.009)
    };
    std::vector<State<T, kAdaptive>> targets;
    for (std::size_t i{0}; i < hs.size(); ++i) {
        const T shift{static_cast<T>(0.1) * static_cast<T>(i)};
        State<T, kAdaptive> target{goal};
        target.x() += shift;
        target.y() -= shift;
        target.z() += static_cast<T>(0.5) * shift;
        target.u() -= static_cast<T>(0.3) * shift;
        target.omega_z() += static_cast<T>(0.2) * shift;
        targets.push_back(target);
    }

    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        // Marching the single step integrator by hand gives the state after every step, which is
        // what the trace is supposed to hold behind the initial state.
        std::vector<State<T, kAdaptive>> expected{start};
        for (std::size_t i{0}; i < hs.size(); ++i) {
            expected.push_back(stepWith(mode, controller, expected.back(), targets[i], hs[i]));
        }
        std::vector<State<T, kAdaptive>> trace(hs.size() + 1);
        const State<T, kAdaptive> returned{evolveWith(mode, controller, start, targets, hs, trace)};
        for (std::size_t i{0}; i < trace.size(); ++i) {
            CHECK_LE(distanceOf(trace[i], expected[i]), roundoffTolerance<T>());
        }
        CHECK_EQ(returned, trace.back());
        // An empty trace records nothing and leaves the run as it is.
        CHECK_LE(
            distanceOf(evolveWith(mode, controller, start, targets, hs), returned),
            roundoffTolerance<T>()
        );
    }
}

TEST_CASE_TEMPLATE(
    "FirstOrderConvergenceAlongWaypoints", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    // Every step previews the horizon it starts with ahead along a moving reference (see
    // marchAlong()), so its target is the preview point at t_i + tau, never the simultaneous point
    // at t_i. As h shrinks, both integrators close in on the loop whose target slides along the
    // reference continuously, but only to first order in h: the target comes in once per step and
    // is held while the step is taken, which caps rungeKutta() at first order as well. Halving h
    // therefore halves the gap to the next resolution, and the ratio below sits near 2 for both.
    // Every resolution interpolates one and the same reference, its waypoints several times sparser
    // than the coarsest step, so the runs differ in the step alone. The model is unbounded, since
    // a clamp kicking in would kink the flow and blur the order. An adaptive horizon runs into
    // both of its bounds here and is clamped back at the end of every step, which keeps the order:
    // the clamp moves a horizon that left the bounds by no more than it left them.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(unboundedModel<T>())};
    constexpr T kDuration{static_cast<T>(4.0)};
    constexpr std::size_t kFineSteps{1280};
    const T fine_h{kDuration / static_cast<T>(kFineSteps)};
    const Reference<T, kAdaptive> reference{
        referenceOf<T, kAdaptive>(kDuration + kMaxPreview<T> + static_cast<T>(0.5))
    };
    // Off the reference and slower than it, under its attitude and at hover.
    const State<T, kAdaptive>& on{reference.ys().front()};
    const State<T, kAdaptive> start{inMode<kAdaptive>(
        stateOf(
            positionOf(on) + makeVec3<T>(0.5, -0.6, 0.3), velocityOf(on) * static_cast<T>(0.9),
            attitudeOf(on), makeVec3<T>(0.0, 0.0, 0.0),
            ModelControl<T>{
                static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
            }
        ),
        kHorizon<T>
    )};
    // The state a run ends in when every step spans stride fine steps.
    const auto run = [&](IntegrateMode mode, std::size_t stride) -> State<T, kAdaptive> {
        return marchAlong(
                   mode, controller, start, reference, fine_h * static_cast<T>(stride),
                   kFineSteps / stride
        )
            .back();
    };

    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        const State<T, kAdaptive> coarse{run(mode, 4)};
        const State<T, kAdaptive> medium{run(mode, 2)};
        const State<T, kAdaptive> fine{run(mode, 1)};
        const T ratio{distanceOf(coarse, medium) / distanceOf(medium, fine)};
        CHECK_GT(ratio, static_cast<T>(1.5));
        CHECK_LT(ratio, static_cast<T>(3.0));
    }
}

TEST_CASE_TEMPLATE(
    "ClosedLoopAlongAReferenceTrajectory", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    // The reference comes as waypoints at uneven times, ten steps apart on average (see
    // referenceOf()). Every step is measured against its simultaneous point and handed the preview
    // point the horizon it starts with past that one, both interpolated between the waypoints
    // around them (see marchAlong()). Every run lasts a minute, for --plot-graph to show it settle
    // and an adaptive horizon move, and is checked from five to 20 seconds.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const DynamicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    constexpr std::size_t kNumSteps{6000};
    constexpr std::size_t kSettlingSteps{500};
    constexpr std::size_t kCheckedSteps{2000};
    const T h{kH<T>};
    const Reference<T, kAdaptive> reference{referenceOf<T, kAdaptive>(
        h * static_cast<T>(kNumSteps) + kMaxPreview<T> + static_cast<T>(0.5)
    )};

    SUBCASE("FromOffTheReference") {
        // The vehicle starts at rest under the identity attitude and the hover thrust, a few metres
        // off the reference, which moves off at nearly 3 m/s. The loop then stays within 35 cm of
        // the simultaneous points from five seconds on with a fixed horizon and within 25 cm with
        // an adaptive one, for every integrator, and closes in on them to a couple of centimetres
        // past 20 seconds, with no control ever reaching a bound. An adaptive horizon runs to its
        // upper bound within half a second and stays there for some five seconds while the offset
        // closes, which is what gets it closer than the fixed one, then drops to its lower bound
        // and wanders between the two along the reference.
        const State<T, kAdaptive>& on{reference.ys().front()};
        const State<T, kAdaptive> start{inMode<kAdaptive>(
            stateOf(
                positionOf(on) + makeVec3<T>(1.5, -2.0, 1.0), makeVec3<T>(0.0, 0.0, 0.0),
                Quaternion<T>::identity(), makeVec3<T>(0.0, 0.0, 0.0),
                ModelControl<T>{
                    static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
                }
            ),
            kHorizon<T>
        )};
        const T settled_distance{kAdaptive ? static_cast<T>(0.25) : static_cast<T>(0.35)};
        const ClosedLoop<T, kAdaptive> loop{
            runClosedLoop(controller, start, reference, h, kNumSteps)
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            CAPTURE(mode);
            CHECK(std::ranges::all_of(loop.trace(mode), isWithinBounds<T, kAdaptive>));
            CHECK_LT(
                settledDistanceOf(loop, mode, kSettlingSteps, kCheckedSteps), settled_distance
            );
        }
        if (plot_graph) {
            if constexpr (std::same_as<T, double>) {
                plotClosedLoop(
                    kAdaptive ? "adaptive horizon, starting off the reference"
                              : "fixed horizon, starting off the reference",
                    loop
                );
            }
        }
    }

    SUBCASE("FromOnTheReference") {
        // The vehicle starts on the reference under the controls of the reference, an adaptive
        // horizon at kHorizon, so there is no offset to close. The loop then stays within 2 cm of
        // the simultaneous points with either horizon, for every integrator. What it leaves is
        // mostly the reference itself: between two waypoints the interpolation cuts the corner of
        // the path they were taken off, and it kinks at every waypoint, neither of which the
        // vehicle can follow exactly.
        const State<T, kAdaptive> start{onTheReference<T, kAdaptive>()};
        const T settled_distance{static_cast<T>(0.02)};
        const ClosedLoop<T, kAdaptive> loop{
            runClosedLoop(controller, start, reference, h, kNumSteps)
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            CAPTURE(mode);
            CHECK(std::ranges::all_of(loop.trace(mode), isWithinBounds<T, kAdaptive>));
            CHECK_LT(
                settledDistanceOf(loop, mode, kSettlingSteps, kCheckedSteps), settled_distance
            );
        }
        if (plot_graph) {
            if constexpr (std::same_as<T, double>) {
                plotClosedLoop(
                    kAdaptive ? "adaptive horizon, starting on the reference"
                              : "fixed horizon, starting on the reference",
                    loop
                );
            }
        }
    }
}

TEST_CASE_TEMPLATE("StabilityBoundOfTheForwardEulerStep", T, float, double) {
    // Along a vertical line with the velocity weighed alone, the torques see no residual to
    // descend, so the vehicle stays level on the line, and the closed loop is linear in the
    // vertical velocity error and the thrust: in units of the horizon its characteristic
    // polynomial is s^2 + s + 1, whatever L0 and T0, so a forward Euler step of it is stable only
    // for tau > h, the bound documented with DynamicsFlowController::tangent() and the shortest
    // horizon forwardEuler() takes when it checks its arguments. Every step takes as
    // its preview point the point of the line the horizon past where the reference is as the step
    // starts, and the error is taken against the velocity of the line. Three percent to either
    // side of the bound leaves the spectral radius about one and a half percent off one, which
    // shrinks or grows the error by some six orders of magnitude over a thousand steps. The
    // envelope over the last hundred steps keeps a zero crossing of the oscillating error from
    // passing for decay.
    const DynamicsModel<T> model{unboundedModel<T>()};
    const std::array<T, 4> weights{
        static_cast<T>(0.0), static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0)
    };
    const T h{kH<T>};
    constexpr T kSpeed{static_cast<T>(2.0)};
    constexpr T kSpeedOffset{static_cast<T>(0.1)};
    constexpr std::size_t kNumSteps{1000};
    constexpr std::size_t kWindow{100};
    const Vec3<T> rest{makeVec3<T>(0.0, 0.0, 0.0)};
    const auto envelope = [&](T tau) -> T {
        const Controller<T, false> controller{controllerOf<T, false>(model, weights, tau)};
        State<T, false> state{stateOf(
            makeVec3<T>(0.0, 0.0, 0.0), makeVec3<T>(0.0, 0.0, kSpeed + kSpeedOffset),
            Quaternion<T>::identity(), rest,
            ModelControl<T>{
                static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
            }
        )};
        T result{static_cast<T>(0.0)};
        for (std::size_t i{0}; i < kNumSteps; ++i) {
            const T t{h * static_cast<T>(i)};
            state = controller.forwardEuler(
                state,
                targetAt(
                    makeVec3<T>(0.0, 0.0, kSpeed * (t + tau)), makeVec3<T>(0.0, 0.0, kSpeed),
                    Quaternion<T>::identity(), rest
                ),
                h
            );
            if (i + kWindow >= kNumSteps) {
                raiseTo(result, std::abs(state.w() - kSpeed));
            }
        }
        return result;
    };
    CHECK_LT(envelope(static_cast<T>(1.03) * h), static_cast<T>(0.01) * kSpeedOffset);
#if BASJOO_CHECK_PARAMS == 1
    // Below the bound the horizon no longer spans the step, which forwardEuler() rejects.
    CHECK_THROWS_AS(static_cast<void>(envelope(static_cast<T>(0.97) * h)), std::invalid_argument);
#else
    CHECK_GT(envelope(static_cast<T>(0.97) * h), static_cast<T>(100.0) * kSpeedOffset);
#endif
}

TEST_CASE_TEMPLATE("AdaptiveMatchesTheFixedHorizonControllerAtItsHorizon", T, float, double) {
    // At the horizon a state carries within its bounds, the adaptive controller walks the
    // arithmetic of the fixed one built with that horizon, so the plant fields, the four control
    // rates and the cost agree to within roundoff: the two are inlined separately, and the
    // compiler may fuse them differently.
    const DynamicsModel<T> model{boundedModel<T>()};
    const Controller<T, true> adaptive{controllerOf<T, true>(model)};
    const State<T, true> target{targetOf<T, true>()};
    const T tolerance{roundoffTolerance<T>()};
    for (const State<T, true>& state : statesOf<T, true>()) {
        const Controller<T, false> fixed{controllerOf<T, false>(model, kWeights<T>, state.tau())};
        const State<T, false> expected{fixed.tangent(fixedOf(state), fixedOf(target))};
        const State<T, true> k{adaptive.tangent(state, target)};
        CHECK_LE(distanceOf(plantOf(k), plantOf(expected)), tolerance);
        CHECK_EQ(k.torque_x(), doctest::Approx(expected.torque_x()).epsilon(tolerance));
        CHECK_EQ(k.torque_y(), doctest::Approx(expected.torque_y()).epsilon(tolerance));
        CHECK_EQ(k.torque_z(), doctest::Approx(expected.torque_z()).epsilon(tolerance));
        CHECK_EQ(k.thrust(), doctest::Approx(expected.thrust()).epsilon(tolerance));
        CHECK_EQ(
            adaptive.cost(state, target),
            doctest::Approx(fixed.cost(fixedOf(state), fixedOf(target))).epsilon(tolerance)
        );
    }
}

TEST_CASE_TEMPLATE("AdaptiveAlongTrackOffsetsCostNothing", T, float, double) {
    // Flying level and straight at a target with its velocity and attitude, without rotating, at
    // the hover thrust and without torques, a distance d short of it, tau = d / |v| zeroes every
    // residual whatever d (see DynamicsFlowController::tangent()). The cost vanishes there, and
    // since it never drops below zero, so does its gradient, which leaves the five rates nothing
    // to descend: the horizon takes up an along track offset instead of the loop closing it. The
    // cost is measured against the one at a horizon a fifth longer, still within the bounds,
    // which is required to be positive, or the check could not tell that horizon apart from any
    // other.
    const Controller<T, true> controller{controllerOf<T, true>(boundedModel<T>())};
    constexpr T kYaw{static_cast<T>(0.3)};
    constexpr T kSpeed{static_cast<T>(4.0)};
    const Quaternion<T> attitude{attitudeFromRpy<T>(0.0, 0.0, kYaw)};
    const Vec3<T> heading{std::cos(kYaw), std::sin(kYaw), static_cast<T>(0.0)};
    const Vec3<T> position{makeVec3<T>(1.0, -2.0, 3.0)};
    const Vec3<T> velocity{makeVec3<T>(kSpeed, 0.0, 0.0)};
    const Vec3<T> rest{makeVec3<T>(0.0, 0.0, 0.0)};
    for (const T distance : {static_cast<T>(1.0), static_cast<T>(1.2), static_cast<T>(1.4)}) {
        const State<T, true> target{inMode<true>(
            targetAt(position + heading * distance, velocity, attitude, rest),
            std::numeric_limits<T>::quiet_NaN()
        )};
        const State<T, true> state{inMode<true>(
            stateOf(
                position, velocity, attitude, rest,
                ModelControl<T>{
                    static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
                }
            ),
            distance / kSpeed
        )};
        State<T, true> off{state};
        off.tau() *= static_cast<T>(1.2);
        REQUIRE_LE(off.tau(), kTauUpperBound<T>);
        const T off_cost{controller.cost(off, target)};
        REQUIRE_GT(off_cost, static_cast<T>(0.0));
        CHECK_LE(controller.cost(state, target), roundoffTolerance<T>() * off_cost);
    }
}

TEST_CASE_TEMPLATE("HorizonStaysWithinItsBounds", T, float, double) {
    const Controller<T, true> controller{controllerOf<T, true>(boundedModel<T>())};
    const State<T, true> start{startOf<T, true>()};
    const State<T, true> target{targetOf<T, true>()};
    const T h{kH<T>};

    SUBCASE("LooseHorizonChangesNothing") {
        // The horizon of a state enters the flow and the cost only through its bounds, so a
        // horizon beyond them may not move a single bit of either.
        for (const auto& [loose, bound] :
             {std::pair{static_cast<T>(0.9), kTauUpperBound<T>},
              std::pair{static_cast<T>(0.05), kTauLowerBound<T>}}) {
            CHECK_EQ(
                controller.tangent(withHorizon(start, loose), target),
                controller.tangent(withHorizon(start, bound), target)
            );
            CHECK_EQ(
                controller.cost(withHorizon(start, loose), target),
                controller.cost(withHorizon(start, bound), target)
            );
        }
    }

    SUBCASE("IntegratorsClampTheHorizon") {
        // Towards targetOf() the horizon of startOf() falls, and towards a target far out of reach
        // it climbs, fast enough in either case that a single unclamped step from the bound it
        // heads for overshoots that bound, so the clamp at the end of every integrator is all that
        // keeps the horizon in. The overshoot is required first, or the checks below would hold
        // trivially.
        const State<T, true> distant{inMode<true>(
            targetAt(
                makeVec3<T>(40.0, -30.0, 20.0), makeVec3<T>(10.0, -8.0, 5.0),
                attitudeFromRpy<T>(-1.0, 0.8, 2.0), makeVec3<T>(-3.0, 2.0, 4.0)
            ),
            std::numeric_limits<T>::quiet_NaN()
        )};
        const State<T, true> at_lower{withHorizon(start, kTauLowerBound<T>)};
        const State<T, true> at_upper{withHorizon(start, kTauUpperBound<T>)};
        REQUIRE_LT(
            at_lower.tau() + controller.tangent(at_lower, target).tau() * h, kTauLowerBound<T>
        );
        REQUIRE_GT(
            at_upper.tau() + controller.tangent(at_upper, distant).tau() * h, kTauUpperBound<T>
        );
        for (const IntegrateMode mode : kIntegrateModes) {
            CAPTURE(mode);
            CHECK(isWithinBounds(stepWith(mode, controller, at_lower, target, h)));
            CHECK(isWithinBounds(stepWith(mode, controller, at_upper, distant, h)));
        }
    }
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE(
    "RejectsInvalidArguments", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const DynamicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    const State<T, kAdaptive> start{startOf<T, kAdaptive>()};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    constexpr T kNaN{std::numeric_limits<T>::quiet_NaN()};
    constexpr T kInfinity{std::numeric_limits<T>::infinity()};

    SUBCASE("ControllerRejectsAnInvalidHorizon") {
        // A fixed horizon has to be positive. The bounds of an adaptive one have to be ordered
        // and the lower one not negative, while infinity above and zero below are fine.
        if constexpr (kAdaptive) {
            for (const auto& [lower, upper] :
                 {std::pair{-kH<T>, kTauUpperBound<T>},
                  std::pair{kTauUpperBound<T>, kTauLowerBound<T>},
                  std::pair{kNaN, kTauUpperBound<T>}, std::pair{kTauLowerBound<T>, kNaN}}) {
                CHECK_THROWS_AS(
                    static_cast<void>(
                        controllerOf<T, kAdaptive>(model, kWeights<T>, kHorizon<T>, lower, upper)
                    ),
                    std::invalid_argument
                );
            }
            CHECK_NOTHROW(
                static_cast<void>(controllerOf<T, kAdaptive>(
                    model, kWeights<T>, kHorizon<T>, static_cast<T>(0.0), kInfinity
                ))
            );
        } else {
            for (const T tau : {static_cast<T>(0.0), -kH<T>, kNaN}) {
                CHECK_THROWS_AS(
                    static_cast<void>(controllerOf<T, kAdaptive>(model, kWeights<T>, tau)),
                    std::invalid_argument
                );
            }
        }
    }

    SUBCASE("ControllerRejectsAModelWithoutGravity") {
        // The unit time of the cost, T0 = sqrt(L0 / g), is taken from gravity.
        const DynamicsModel<T> weightless{kMass<T>,           kInertiaXx<T>,   kInertiaYy<T>,
                                          kInertiaZz<T>,      lowerBound<T>(), upperBound<T>(),
                                          static_cast<T>(0.0)};
        CHECK_THROWS_AS(
            static_cast<void>(controllerOf<T, kAdaptive>(weightless)), std::invalid_argument
        );
    }

    SUBCASE("IntegratorsRejectAttitudeOffUnitNorm") {
        // tangent() takes an attitude off unit norm as it comes, as the stages of rungeKutta()
        // need, but a step has to start from a unit one.
        State<T, kAdaptive> state{start};
        state.qx() *= static_cast<T>(1.1);
        state.qy() *= static_cast<T>(1.1);
        state.qz() *= static_cast<T>(1.1);
        state.qw() *= static_cast<T>(1.1);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(stepWith(mode, controller, state, target, kH<T>)),
                std::invalid_argument
            );
        }
    }

    SUBCASE("IntegratorsRejectNonPositiveTimeStep") {
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const T h : {static_cast<T>(0.0), static_cast<T>(-0.01)}) {
                CHECK_THROWS_AS(
                    static_cast<void>(stepWith(mode, controller, start, target, h)),
                    std::invalid_argument
                );
            }
        }
    }

    SUBCASE("HorizonHasToOutlastTheStep") {
        // A forward Euler step is fine with a horizon that spans exactly the step, while the last
        // stage of rungeKutta() looks ahead over the horizon less the step, which has to stay
        // positive for the target to lie ahead of that stage, so it needs strictly more. An
        // adaptive horizon is pinned to tau by bounds that coincide.
        const T h{kH<T>};
        const auto step_over = [&](IntegrateMode mode, T tau) -> State<T, kAdaptive> {
            const Controller<T, kAdaptive> horizon_controller{
                controllerOf<T, kAdaptive>(model, kWeights<T>, tau, tau, tau)
            };
            return stepWith(mode, horizon_controller, withHorizon(start, tau), target, h);
        };
        CHECK_NOTHROW(static_cast<void>(step_over(IntegrateMode::FORWARD_EULER, h)));
        CHECK_THROWS_AS(
            static_cast<void>(step_over(IntegrateMode::FORWARD_EULER, h * static_cast<T>(0.5))),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(step_over(IntegrateMode::RUNGE_KUTTA, h)), std::invalid_argument
        );
        CHECK_NOTHROW(
            static_cast<void>(step_over(IntegrateMode::RUNGE_KUTTA, h * static_cast<T>(1.01)))
        );
    }

    SUBCASE("AdaptiveHorizonHasToBePositive") {
        // Bounds that let the horizon sink to zero leave tangent() and cost() to reject a state
        // whose horizon got there, or never was a number.
        if constexpr (kAdaptive) {
            const Controller<T, kAdaptive> unbounded{controllerOf<T, kAdaptive>(
                model, kWeights<T>, kHorizon<T>, static_cast<T>(0.0), kInfinity
            )};
            for (const T tau : {static_cast<T>(0.0), -kH<T>, kNaN}) {
                const State<T, kAdaptive> state{withHorizon(start, tau)};
                CHECK_THROWS_AS(
                    static_cast<void>(unbounded.tangent(state, target)), std::invalid_argument
                );
                CHECK_THROWS_AS(
                    static_cast<void>(unbounded.cost(state, target)), std::invalid_argument
                );
            }
        }
    }

    SUBCASE("EvolveRejectsMismatchedSpans") {
        const std::vector<T> hs(4, kH<T>);
        const std::vector<State<T, kAdaptive>> targets(4, target);
        const std::vector<State<T, kAdaptive>> short_targets(3, target);
        std::vector<State<T, kAdaptive>> short_trace(hs.size());
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(evolveWith(mode, controller, start, short_targets, hs)),
                std::invalid_argument
            );
            CHECK_THROWS_AS(
                static_cast<void>(evolveWith(mode, controller, start, targets, hs, short_trace)),
                std::invalid_argument
            );
        }
    }
}

#endif

} // namespace basjoo::quadrotor

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options(
        "dynamics_flow_controller_test", "unit test of DynamicsFlowController class"
    );
    options.allow_unrecognised_options(); // doctest reads its own flags from argv below
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
