/**
 * @file quadrotor/models/test_dynamics_model.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-11-12
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/quadrotor/models/dynamics_model.hpp"

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
#include "basjoo/math/dense/quaternion.hpp"
#include "basjoo/math/dense/vec3.hpp"
#include "basjoo/math/functions/function_proxy.hpp"
#include "basjoo/math/functions/piecewise_cubic_function.hpp"
#include "basjoo/math/functions/piecewise_linear_function.hpp"
#include "basjoo/math/functions/piecewise_quintic_function.hpp"
#include "basjoo/math/integrate_mode.hpp"
#include "basjoo/math/type_traits.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

namespace {

bool plot_graph{false};

auto createFigureHandle() noexcept -> matplot::figure_handle {
    matplot::figure_handle fig = matplot::figure(true);
    fig->backend()->run_command("unset warnings");
    fig->size(1800, 1500);
    return fig;
}

/**
 * @brief The fifteen subplots of a figure on a 4 x 4 grid, in reading order.
 *
 * The 3D trajectory spans the first two cells of the top row, followed by the position and the
 * attitude errors. The rows below hold the body velocity, the attitude and the angular velocity
 * against time, a field per subplot, the two velocities followed by their errors. The labels are
 * gnuplot's enhanced text, in which _ and ^ set subscripts and superscripts.
 */
auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    const std::array<const char*, 14> ylabels{
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
        "log_{10}‖ω − ω^*‖ [rad/s]"
    };
    std::vector<matplot::axes_handle> axes;
    axes.reserve(ylabels.size() + 1);
    axes.emplace_back(fig->add_subplot(4, 4, {0, 1}));
    for (std::size_t i = 0; i < ylabels.size(); ++i) {
        axes.emplace_back(fig->add_subplot(4, 4, i + 2));
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
// The axial moment of the body with equal x and y moments, see axisymmetricModel().
template <std::floating_point T>
inline constexpr T kAxialInertia{static_cast<T>(0.05)};
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
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};
// Every scenario runs this long, which gives the errors of the integrators time to build up.
template <std::floating_point T>
inline constexpr T kDuration{static_cast<T>(20.0)};
// A free fall over kDuration drops g kDuration^2 / 2 = 1961 m, so the scenarios that fall start
// this high to end their run near the ground.
template <std::floating_point T>
inline constexpr T kDropHeight{static_cast<T>(2000.0)};

inline constexpr std::array<IntegrateMode, 2> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::RUNGE_KUTTA
};

// The errors maxErrorsOf() measures, in the order it returns them.
inline constexpr std::array<const char*, 4> kErrorNames{
    "position", "body velocity", "attitude", "angular velocity"
};

template <std::floating_point T>
using State = DynamicsModelState<T>;

template <std::floating_point T>
using Control = DynamicsModelControl<T>;

// The array storage lets the state and control arithmetic and the accessors run in constant
// expressions.
consteval auto constantState() -> bool {
    State<double> s{1.0, 2.0, 3.0, 0.5, 0.25, -0.5, 0.0, 0.0, 0.0, 1.0, 0.125, -0.25, 2.0};
    s += State<double>{1.0};
    s *= 2.0;
    s.u() -= 1.0;
    s.qx() = 0.5;
    return s.x() == 4.0 && s.u() == 2.0 && s.qx() == 0.5 && s.qw() == 4.0 && s[12] == 6.0 &&
           s - s == State<double>{0.0} && State<double>{}.qw() == 1.0;
}
static_assert(constantState());

consteval auto constantControl() -> bool {
    Control<double> c{0.5, -0.25, 0.125, 15.0};
    c += Control<double>{1.0};
    c *= 2.0;
    c.thrust() -= 2.0;
    return c.torque_x() == 3.0 && c.torque_y() == 1.5 && c[2] == 2.25 && c.thrust() == 30.0 &&
           c - c == Control<double>{} && Control<double>{0.0} == Control<double>{};
}
static_assert(constantControl());

template <std::floating_point T>
[[nodiscard]] constexpr auto lowerBound() noexcept -> Control<T> {
    return Control<T>{kTorqueXLo<T>, kTorqueYLo<T>, kTorqueZLo<T>, kThrustLo<T>};
}

template <std::floating_point T>
[[nodiscard]] constexpr auto upperBound() noexcept -> Control<T> {
    return Control<T>{kTorqueXUp<T>, kTorqueYUp<T>, kTorqueZUp<T>, kThrustUp<T>};
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
 * @brief A body whose x and y moments of inertia agree.
 *
 * The torque free motion of such a body is the one rigid body motion with an elementary closed form
 * beyond a spin about a principal axis: see TorqueFreeAxisymmetricPrecession.
 */
template <std::floating_point T>
[[nodiscard]] auto axisymmetricModel() -> DynamicsModel<T> {
    return DynamicsModel<T>{kMass<T>, kInertiaXx<T>, kInertiaXx<T>, kAxialInertia<T>};
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
 * @brief Builds a state from, and takes it apart into, the vectors of physical space it is made of.
 *
 * The model takes its attitude as four flat scalars, while the scenarios below are written in terms
 * of positions, velocities, attitudes and angular velocities. math::Quaternion and math::Vec3 are
 * an implementation of their own of the rotations the model writes out field by field, which makes
 * them the yardstick the model is held to.
 */
template <std::floating_point T>
[[nodiscard]] auto stateOf(
    const Vec3<T>& position, const Vec3<T>& velocity, const Quaternion<T>& attitude,
    const Vec3<T>& omega
) noexcept -> State<T> {
    return State<T>{position.x(), position.y(), position.z(), velocity.x(), velocity.y(),
                    velocity.z(), attitude.x(), attitude.y(), attitude.z(), attitude.w(),
                    omega.x(),    omega.y(),    omega.z()};
}

template <std::floating_point T>
[[nodiscard]] auto positionOf(const State<T>& state) noexcept -> Vec3<T> {
    return Vec3<T>{state.x(), state.y(), state.z()};
}

template <std::floating_point T>
[[nodiscard]] auto velocityOf(const State<T>& state) noexcept -> Vec3<T> {
    return Vec3<T>{state.u(), state.v(), state.w()};
}

template <std::floating_point T>
[[nodiscard]] auto attitudeOf(const State<T>& state) noexcept -> Quaternion<T> {
    return Quaternion<T>{state.qx(), state.qy(), state.qz(), state.qw()};
}

template <std::floating_point T>
[[nodiscard]] auto omegaOf(const State<T>& state) noexcept -> Vec3<T> {
    return Vec3<T>{state.omega_x(), state.omega_y(), state.omega_z()};
}

template <std::floating_point T>
[[nodiscard]] auto worldVelocityOf(const State<T>& state) -> Vec3<T> {
    return attitudeOf(state).rotate(velocityOf(state));
}

/**
 * @brief Angular momentum about the center of mass, in the world frame.
 */
template <std::floating_point T>
[[nodiscard]] auto angularMomentumOf(const DynamicsModel<T>& model, const State<T>& state)
    -> Vec3<T> {
    return attitudeOf(state).rotate(
        Vec3<T>{
            model.inertia_xx() * state.omega_x(), model.inertia_yy() * state.omega_y(),
            model.inertia_zz() * state.omega_z()
        }
    );
}

template <std::floating_point T>
[[nodiscard]] auto rotationalEnergyOf(const DynamicsModel<T>& model, const State<T>& state) noexcept
    -> T {
    return static_cast<T>(0.5) * (model.inertia_xx() * state.omega_x() * state.omega_x() +
                                  model.inertia_yy() * state.omega_y() * state.omega_y() +
                                  model.inertia_zz() * state.omega_z() * state.omega_z());
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
 * @brief Largest per field difference between two values of the same flat type: states, controls
 * or quaternions.
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
 * @brief Accuracy the fourth order Runge-Kutta integrator keeps over a run of a couple of seconds.
 *
 * Tight enough that an integrator that silently degraded to first order, whose error is orders of
 * magnitude larger, would fail immediately.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto rungeKuttaTolerance() noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(5.0e-3) : static_cast<T>(1.0e-6);
}

/**
 * @brief The error below which an integrator is as accurate as a scenario of kDuration can check.
 *
 * double holds every error to 1e-6. float rounds its positions near kDropHeight and its speeds near
 * 200 m / s by about 1e-4 a step, which builds up to errors of about 0.1 m and 0.1 m / s over a run
 * of kDuration whatever the integrator does, so float is held to 1e-2 relative to the scale of the
 * quantity: its largest magnitude along the run, though at least one.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto accuracyFloor(T scale) noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(1.0e-2) * std::max(scale, static_cast<T>(1.0))
                                  : static_cast<T>(1.0e-6);
}

/**
 * @brief Bound on the drift between two calls that walk the same arithmetic.
 *
 * The compiler may fuse a multiply and an add into an FMA in one inlined copy of a step and not in
 * the other, which moves the last bit or so, so two calls that are meant to agree cannot be held to
 * bitwise equality. Whatever such a comparison is meant to catch lands orders of magnitude above
 * the bound.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto roundoffTolerance() noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(1.0e-5) : static_cast<T>(1.0e-12);
}

/**
 * @brief Piecewise constant controls: step i holds controls[i] over hs[i].
 */
template <std::floating_point T>
struct Schedule final {
    std::vector<T> hs;
    std::vector<Control<T>> controls;
};

template <std::floating_point T>
[[nodiscard]] auto constantSchedule(std::size_t num_steps, T h, const Control<T>& control)
    -> Schedule<T> {
    return Schedule<T>{std::vector<T>(num_steps, h), std::vector<Control<T>>(num_steps, control)};
}

/**
 * @brief The same schedule in twice as many steps of half the length.
 *
 * Each control is held over both halves of its step, so the controls switch at the same instants
 * as in the original schedule.
 */
template <std::floating_point T>
[[nodiscard]] auto halvedSteps(const Schedule<T>& schedule) -> Schedule<T> {
    Schedule<T> result;
    result.hs.reserve(schedule.hs.size() * 2);
    result.controls.reserve(schedule.controls.size() * 2);
    for (std::size_t i{0}; i < schedule.hs.size(); ++i) {
        for (std::size_t half{0}; half < 2; ++half) {
            result.hs.push_back(schedule.hs[i] * static_cast<T>(0.5));
            result.controls.push_back(schedule.controls[i]);
        }
    }
    return result;
}

/**
 * @brief One sample of the closed form solution of a scenario.
 *
 * velocity is in the body frame, like the u, v and w fields of a state.
 */
template <std::floating_point T>
struct Sample final {
    Vec3<T> position;
    Vec3<T> velocity;
    Quaternion<T> attitude;
    Vec3<T> omega;
};

/**
 * @brief The motion of the center of mass without thrust, along whatever rotation the body takes.
 *
 * With the thrust off, gravity is the only force, so the center of mass flies the same parabola in
 * the world frame whatever the attitude does: the body velocity is that world velocity seen
 * through the attitude at time t, which is the part the rotation decides.
 */
template <std::floating_point T>
[[nodiscard]] auto freeFallAt(
    const State<T>& initial_state, T t, const Quaternion<T>& attitude, const Vec3<T>& omega
) -> Sample<T> {
    const Vec3<T> world_velocity{worldVelocityOf(initial_state)};
    const Vec3<T> gravity{static_cast<T>(0.0), static_cast<T>(0.0), -kGravity<T>};
    return Sample<T>{
        positionOf(initial_state) + world_velocity * t + gravity * (static_cast<T>(0.5) * t * t),
        attitude.conjugated().rotate(world_velocity + gravity * t), attitude, omega
    };
}

template <std::floating_point T>
[[nodiscard]] auto positionErrorOf(const State<T>& state, const Sample<T>& reference) -> T {
    return positionOf(state).euclideanTo(reference.position);
}

template <std::floating_point T>
[[nodiscard]] auto velocityErrorOf(const State<T>& state, const Sample<T>& reference) -> T {
    return velocityOf(state).euclideanTo(reference.velocity);
}

template <std::floating_point T>
[[nodiscard]] auto attitudeErrorOf(const State<T>& state, const Sample<T>& reference) -> T {
    return attitudeOf(state).angularDistance(reference.attitude);
}

template <std::floating_point T>
[[nodiscard]] auto omegaErrorOf(const State<T>& state, const Sample<T>& reference) -> T {
    return omegaOf(state).euclideanTo(reference.omega);
}

template <std::floating_point T>
[[nodiscard]] auto stepWith(
    IntegrateMode mode, const DynamicsModel<T>& model, const State<T>& state,
    const Control<T>& control, std::type_identity_t<T> h
) -> State<T> {
    switch (mode) {
    case IntegrateMode::FORWARD_EULER:
        return model.forwardEuler(state, control, h);
    case IntegrateMode::RUNGE_KUTTA:
        return model.rungeKutta(state, control, h);
    }
    std::unreachable();
}

template <std::floating_point T>
auto evolveWith(
    IntegrateMode mode, const DynamicsModel<T>& model, const State<T>& initial_state,
    const Schedule<T>& schedule, std::type_identity_t<std::span<State<T>>> trace = {}
) -> State<T> {
    switch (mode) {
    case IntegrateMode::FORWARD_EULER:
        return model.template evolve<IntegrateMode::FORWARD_EULER>(
            initial_state, schedule.controls, schedule.hs, trace
        );
    case IntegrateMode::RUNGE_KUTTA:
        return model.template evolve<IntegrateMode::RUNGE_KUTTA>(
            initial_state, schedule.controls, schedule.hs, trace
        );
    }
    std::unreachable();
}

/**
 * @brief One run of every integrator through the same schedule, each trace led by the initial
 * state so that it lines up with ts and with the samples of the closed form solution, if any.
 */
template <std::floating_point T>
struct Trajectory final {
    std::vector<T> ts;
    std::array<std::vector<State<T>>, kIntegrateModes.size()> traces;
    std::vector<Sample<T>> reference;

    [[nodiscard]] auto trace(IntegrateMode mode) const noexcept -> const std::vector<State<T>>& {
        return traces[static_cast<std::size_t>(mode)];
    }
};

/**
 * @brief Marches every integrator through the schedule with evolve().
 *
 * The time is summed in double: a float sum of a thousand steps drifts off the instants the
 * integrators actually reach by enough to show up in the errors against the closed form.
 */
template <std::floating_point T>
[[nodiscard]] auto simulate(
    const DynamicsModel<T>& model, const State<T>& initial_state, const Schedule<T>& schedule
) -> Trajectory<T> {
    const std::size_t size{schedule.hs.size()};
    Trajectory<T> trajectory;
    trajectory.ts.reserve(size + 1);
    trajectory.ts.push_back(static_cast<T>(0.0));
    double elapsed{0.0};
    for (const T h : schedule.hs) {
        elapsed += static_cast<double>(h);
        trajectory.ts.push_back(static_cast<T>(elapsed));
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        std::vector<State<T>>& trace{trajectory.traces[static_cast<std::size_t>(mode)]};
        trace.resize(size + 1);
        static_cast<void>(evolveWith(mode, model, initial_state, schedule, trace));
    }
    return trajectory;
}

/**
 * @brief Samples the closed form solution alongside an already marched trajectory.
 *
 * @param reference_at maps a time in the run onto the closed form solution there.
 */
template <std::floating_point T>
auto fillReference(Trajectory<T>& trajectory, const auto& reference_at) -> void {
    trajectory.reference.reserve(trajectory.ts.size());
    for (const T t : trajectory.ts) {
        trajectory.reference.push_back(reference_at(t));
    }
}

/**
 * @brief Largest position, body velocity, attitude and angular velocity errors along a trace, in
 * the order of kErrorNames.
 */
template <std::floating_point T>
[[nodiscard]] auto maxErrorsOf(
    const std::vector<State<T>>& trace, const std::vector<Sample<T>>& reference
) -> std::array<T, kErrorNames.size()> {
    std::array<T, kErrorNames.size()> result{};
    for (std::size_t i{0}; i < trace.size(); ++i) {
        raiseTo(result[0], positionErrorOf(trace[i], reference[i]));
        raiseTo(result[1], velocityErrorOf(trace[i], reference[i]));
        raiseTo(result[2], attitudeErrorOf(trace[i], reference[i]));
        raiseTo(result[3], omegaErrorOf(trace[i], reference[i]));
    }
    return result;
}

/**
 * @brief Asserts that an error an integrator makes at the step h, and again at h / 2, shrinks the
 * way its order says it should.
 *
 * An error within accuracyFloor() of the scale of its quantity only has to stay there: either the
 * integrator is exact up to roundoff, such as forwardEuler() on the body velocity under a fixed
 * attitude, or the error is the rounding of float rather than the integrator's. Above the floor
 * forwardEuler(), being first order, has to at least halve its error along with the step, a ratio
 * above 1.5, and the fourth order rungeKutta() has to cut it sixteen fold, a ratio above 12. In
 * float rungeKutta() is held to the floor alone: its error is rounding through and through, which a
 * ratio says nothing about. The ratio of forwardEuler() has no upper bound: it turns the attitude
 * to second order under a constant angular velocity, which quarters that error.
 */
template <std::floating_point T>
auto checkErrorShrinks(IntegrateMode mode, T coarse_error, T fine_error, T scale) -> void {
    const bool fourth_order{mode != IntegrateMode::FORWARD_EULER};
    const T limit{accuracyFloor(scale)};
    if ((std::same_as<T, float> && fourth_order) || coarse_error <= limit) {
        CHECK_LE(coarse_error, limit);
        CHECK_LE(fine_error, limit);
    } else {
        CHECK_GT(
            coarse_error, (fourth_order ? static_cast<T>(12.0) : static_cast<T>(1.5)) * fine_error
        );
    }
}

/**
 * @brief Asserts that every integrator hands back a unit attitude after every step.
 *
 * The attitude is integrated in R^4, where forward Euler alone grows |q|^2 by h^2 |omega|^2 / 4 a
 * step, so this is what the normalization at the end of each step is there for. A normalized
 * quaternion is still a few roundings off unit norm, hence the bound of a few epsilon.
 */
template <std::floating_point T>
auto checkUnitAttitude(const Trajectory<T>& trajectory) -> void {
    constexpr T kTolerance{static_cast<T>(16.0) * std::numeric_limits<T>::epsilon()};
    for (const IntegrateMode mode : kIntegrateModes) {
        const std::vector<State<T>>& trace{trajectory.trace(mode)};
        T max_error{static_cast<T>(0.0)};
        for (std::size_t i{1}; i < trace.size(); ++i) {
            raiseTo(max_error, std::abs(attitudeOf(trace[i]).euclideanSqr() - static_cast<T>(1.0)));
        }
        CHECK_LE(max_error, kTolerance);
    }
}

/**
 * @brief Runs a scenario with a closed form solution at the step h and at h / 2, and holds every
 * integrator to that solution.
 *
 * Every error of every integrator goes through checkErrorShrinks(). Returns the run at the step h
 * with the closed form sampled alongside, for the checks particular to the scenario and the plot.
 */
template <std::floating_point T>
auto checkClosedForm(
    const DynamicsModel<T>& model, const State<T>& initial_state, const Schedule<T>& schedule,
    const auto& reference_at
) -> Trajectory<T> {
    Trajectory<T> coarse{simulate(model, initial_state, schedule)};
    Trajectory<T> fine{simulate(model, initial_state, halvedSteps(schedule))};
    fillReference(coarse, reference_at);
    fillReference(fine, reference_at);
    // The scale of every error is the largest magnitude of its quantity along the closed form; the
    // attitude error is an angle, which leaves it the scale of one accuracyFloor() falls back to.
    std::array<T, kErrorNames.size()> scales{};
    for (const Sample<T>& sample : coarse.reference) {
        raiseTo(scales[0], sample.position.euclidean());
        raiseTo(scales[1], sample.velocity.euclidean());
        raiseTo(scales[3], sample.omega.euclidean());
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        const std::array<T, kErrorNames.size()> coarse_errors{
            maxErrorsOf(coarse.trace(mode), coarse.reference)
        };
        const std::array<T, kErrorNames.size()> fine_errors{
            maxErrorsOf(fine.trace(mode), fine.reference)
        };
        for (std::size_t i{0}; i < kErrorNames.size(); ++i) {
            const std::string error_name{kErrorNames[i]};
            CAPTURE(error_name);
            checkErrorShrinks(mode, coarse_errors[i], fine_errors[i], scales[i]);
        }
    }
    checkUnitAttitude(coarse);
    checkUnitAttitude(fine);
    return coarse;
}

/**
 * @brief Estimates the observed order of convergence of the integrators.
 *
 * Halving the step three times and comparing consecutive results estimates the order without
 * needing a solution to compare against, which is what the scenarios whose rotation has no closed
 * form require. Fourth order cuts the step error sixteen fold, first order only halves it, so
 * running forwardEuler() through the same estimator as rungeKutta() both pins down the claim of
 * rungeKutta() and proves the estimator itself measures what it says it does.
 *
 * The controls are addressed by the fraction of the run rather than by the elapsed time so that a
 * piecewise constant schedule switches on exactly the same step boundary at every resolution. A
 * discontinuity landing inside a step at one resolution and on its boundary at another would
 * degrade the order for reasons that have nothing to do with the integrator.
 */
template <std::floating_point T>
auto checkConvergenceOrder(
    const DynamicsModel<T>& model, const State<T>& initial_state, T duration,
    std::size_t base_steps, const auto& control_at
) -> void {
    const auto final_state = [&](IntegrateMode mode, std::size_t steps) -> State<T> {
        Schedule<T> schedule{
            constantSchedule(steps, duration / static_cast<T>(steps), Control<T>{})
        };
        for (std::size_t i{0}; i < steps; ++i) {
            schedule.controls[i] = control_at(static_cast<T>(i) / static_cast<T>(steps));
        }
        return evolveWith(mode, model, initial_state, schedule);
    };
    const auto ratio_of = [&](IntegrateMode mode) -> T {
        const State<T> coarse{final_state(mode, base_steps)};
        const State<T> medium{final_state(mode, base_steps * 2)};
        const State<T> fine{final_state(mode, base_steps * 4)};
        return distanceOf(coarse, medium) / distanceOf(medium, fine);
    };

    const T runge_kutta_ratio{ratio_of(IntegrateMode::RUNGE_KUTTA)};
    const T forward_euler_ratio{ratio_of(IntegrateMode::FORWARD_EULER)};
    CHECK_GT(runge_kutta_ratio, static_cast<T>(12.0));
    CHECK_LT(runge_kutta_ratio, static_cast<T>(24.0));
    CHECK_GT(forward_euler_ratio, static_cast<T>(1.5));
    CHECK_LT(forward_euler_ratio, static_cast<T>(3.0));
}

/**
 * @brief Plots the trajectory of every integrator in 3D and each field of its state against time,
 * over the closed form solution where the scenario has one, with the decimal logarithm of the
 * errors against that solution; the error subplots of a scenario without one stay empty. Every
 * curve keeps its color and line style across the fifteen subplots, so that one legend across the
 * top of the figure covers them all.
 *
 * The errors go through log10() rather than onto a log axis because matplot hands its data to
 * gnuplot in fixed point with ten decimals, which rounds the errors of Runge-Kutta, down to 1e-13,
 * to zero, and a log axis drops zeros.
 */
auto plotTrajectory(const std::string& title, const Trajectory<double>& trajectory) -> void {
    // The subplot of every field of the state, as createAxesHandles() lays them out: x, y and z go
    // to the 3D trajectory together.
    constexpr std::array<std::size_t, State<double>::size()> kFieldAxes{0, 0, 0,  3,  4,  5, 7,
                                                                        8, 9, 10, 11, 12, 13};
    // The subplot of every error, in the order of kErrorNames.
    constexpr std::array<std::size_t, kErrorNames.size()> kErrorAxes{1, 6, 2, 14};
    const std::array<float, 4> reference_color{matplot::default_color(0)};
    const std::array<std::array<float, 4>, kIntegrateModes.size()> mode_colors{
        matplot::default_color(2), matplot::default_color(4)
    };
    const std::array<const char*, kIntegrateModes.size()> mode_names{
        "forward Euler", "Runge–Kutta"
    };
    const std::array<const char*, kIntegrateModes.size()> line_specs{":", "--"};
    const bool has_reference{!trajectory.reference.empty()};

    matplot::figure_handle fig = createFigureHandle();
    fig->title(title);
    std::vector<matplot::axes_handle> axes = createAxesHandles(fig);
    for (std::size_t i{1}; i < axes.size(); ++i) {
        axes[i]->xlim({trajectory.ts.front(), trajectory.ts.back()});
    }
    const auto plot_states = [&](const std::vector<State<double>>& states, const char* line_spec,
                                 const std::array<float, 4>& color) -> void {
        std::array<std::vector<double>, State<double>::size()> fields;
        for (const State<double>& state : states) {
            for (std::size_t f{0}; f < State<double>::size(); ++f) {
                fields[f].push_back(state[f]);
            }
        }
        axes[0]->plot3(fields[0], fields[1], fields[2], line_spec)->line_width(2.0).color(color);
        for (std::size_t f{3}; f < State<double>::size(); ++f) {
            axes[kFieldAxes[f]]
                ->plot(trajectory.ts, fields[f], line_spec)
                ->line_width(2.0)
                .color(color);
        }
    };

    if (has_reference) {
        std::vector<State<double>> reference_states;
        reference_states.reserve(trajectory.reference.size());
        for (const Sample<double>& sample : trajectory.reference) {
            reference_states.push_back(
                stateOf(sample.position, sample.velocity, sample.attitude, sample.omega)
            );
        }
        plot_states(reference_states, "-", reference_color);
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        const auto m{static_cast<std::size_t>(mode)};
        const std::vector<State<double>>& trace{trajectory.trace(mode)};
        plot_states(trace, line_specs[m], mode_colors[m]);
        if (!has_reference) {
            continue;
        }
        std::array<std::vector<double>, kErrorNames.size()> errors;
        for (std::size_t i{0}; i < trace.size(); ++i) {
            errors[0].push_back(positionErrorOf(trace[i], trajectory.reference[i]));
            errors[1].push_back(velocityErrorOf(trace[i], trajectory.reference[i]));
            errors[2].push_back(attitudeErrorOf(trace[i], trajectory.reference[i]));
            errors[3].push_back(omegaErrorOf(trace[i], trajectory.reference[i]));
        }
        for (std::size_t e{0}; e < errors.size(); ++e) {
            // An error of exactly zero has no logarithm and is left out as a NaN, which gnuplot
            // skips; an error that is zero all along, such as that of an angular velocity an
            // integrator keeps exactly, leaves nothing to plot at all.
            bool any_error{false};
            for (double& error : errors[e]) {
                any_error = any_error || error > 0.0;
                error = error > 0.0 ? std::log10(error) : std::numeric_limits<double>::quiet_NaN();
            }
            if (any_error) {
                axes[kErrorAxes[e]]
                    ->plot(trajectory.ts, errors[e], line_specs[m])
                    ->line_width(2.0)
                    .color(mode_colors[m]);
            }
        }
    }

    // A matplot figure has no legend of its own, so the legend gets an axes to itself: a strip
    // across the top whose limits keep its curves out of sight, so that only their keys show.
    const matplot::axes_handle key{fig->add_axes({0.05F, 0.935F, 0.9F, 0.03F})};
    key->hold(matplot::on);
    const std::vector<double> out_of_sight{0.0, 1.0};
    std::vector<std::string> names;
    if (has_reference) {
        key->plot(out_of_sight, out_of_sight, "-")->line_width(2.0).color(reference_color);
        names.emplace_back("closed form");
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        const auto m{static_cast<std::size_t>(mode)};
        key->plot(out_of_sight, out_of_sight, line_specs[m])->line_width(2.0).color(mode_colors[m]);
        names.emplace_back(mode_names[m]);
    }
    key->xlim({2.0, 3.0});
    key->ylim({2.0, 3.0});
    key->box(false);
    key->x_axis().visible(false);
    key->y_axis().visible(false);
    key->legend(names);
    key->legend()->num_columns(names.size());
    key->legend()->box(false);
    key->legend()->location(matplot::legend::general_alignment::center);
    key->legend()->font_size(12.0);
    fig->show();
}

} // namespace

// Scenarios whose rotation is a spin about a fixed axis, the precession of a symmetric body or a
// steady turn have a closed form solution to compare against, and so does the translation whenever
// either the attitude is fixed, the thrust is off or the motion is steady in the body frame.

TEST_CASE_TEMPLATE("ConstantThrustAtFixedTilt", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    // Without torques and without an initial rotation the attitude stays put, so the body
    // acceleration is constant: the thrust along the body z axis and gravity seen from the body.
    const Quaternion<T> attitude{attitudeFromRpy<T>(0.2, -0.3, 0.5)};
    const State<T> initial_state{stateOf(
        makeVec3<T>(0.5, -1.0, 2.0), makeVec3<T>(1.0, -0.5, 0.25), attitude,
        makeVec3<T>(0.0, 0.0, 0.0)
    )};
    const T thrust{static_cast<T>(1.3) * kHoverThrust<T>};
    const Schedule<T> schedule{constantSchedule(
        kNumSteps, h,
        Control<T>{static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), thrust}
    )};

    const Vec3<T> up{makeVec3<T>(0.0, 0.0, 1.0)};
    const Vec3<T> acceleration{
        up * (thrust / kMass<T>)-attitude.conjugated().rotate(up) * kGravity<T>
    };
    const Vec3<T> initial_velocity{velocityOf(initial_state)};
    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            return Sample<T>{
                positionOf(initial_state) +
                    attitude.rotate(
                        initial_velocity * t + acceleration * (static_cast<T>(0.5) * t * t)
                    ),
                initial_velocity + acceleration * t, attitude, makeVec3<T>(0.0, 0.0, 0.0)
            };
        })
    };

    // In the world frame the acceleration is the tilted thrust less gravity, which leaves the
    // world velocity growing along a fixed direction.
    const Vec3<T> world_acceleration{attitude.rotate(up) * (thrust / kMass<T>)-up * kGravity<T>};
    const Vec3<T> final_velocity{
        worldVelocityOf(initial_state) + world_acceleration * kDuration<T>
    };
    const State<T>& final_state{trajectory.trace(IntegrateMode::RUNGE_KUTTA).back()};
    CHECK_LE(
        worldVelocityOf(final_state).euclideanTo(final_velocity),
        accuracyFloor(final_velocity.euclidean())
    );

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("constant thrust at fixed tilt", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("PrincipalAxisSpin", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr T kRate{static_cast<T>(2.5)};
    constexpr std::size_t kNumSteps{4000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};
    const Quaternion<T> attitude{attitudeFromRpy<T>(0.3, -0.2, 0.1)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, Control<T>{})};

    // A spin about a principal axis leaves the gyroscopic term zero, so without torques the
    // angular velocity stays put and the attitude turns about that body axis at a constant rate.
    // The intermediate axis is the unstable one, but nothing perturbs the two zero components.
    for (const std::size_t axis : {std::size_t{0}, std::size_t{1}, std::size_t{2}}) {
        CAPTURE(axis);
        Vec3<T> omega{static_cast<T>(0.0)};
        omega[axis] = kRate;
        const State<T> initial_state{stateOf(
            Vec3<T>{static_cast<T>(0.0), static_cast<T>(0.0), kDropHeight<T>},
            makeVec3<T>(2.0, 1.0, -0.5), attitude, omega
        )};
        const Trajectory<T> trajectory{
            checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
                const Vec3<T> turn{omega * t};
                return freeFallAt(
                    initial_state, t,
                    attitude * Quaternion<T>::fromRotationVector(turn.x(), turn.y(), turn.z()),
                    omega
                );
            })
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const State<T>& state : trajectory.trace(mode)) {
                CHECK_EQ(omegaOf(state), omega);
            }
        }

        if (plot_graph) {
            if constexpr (std::same_as<T, double>) {
                if (axis == 2) {
                    plotTrajectory("principal axis spin", trajectory);
                }
            }
        }
    }
}

TEST_CASE_TEMPLATE("ConstantTorqueFromRest", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr T kTorque{static_cast<T>(0.003)};
    constexpr std::size_t kNumSteps{4000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};
    const Quaternion<T> attitude{attitudeFromRpy<T>(-0.1, 0.2, 0.7)};
    const State<T> initial_state{stateOf(
        Vec3<T>{static_cast<T>(1.0), static_cast<T>(2.0), kDropHeight<T>},
        makeVec3<T>(0.5, -0.25, 1.0), attitude, makeVec3<T>(0.0, 0.0, 0.0)
    )};
    const std::array<T, 3> inertias{kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>};

    // A torque about one principal axis, from rest, keeps the angular velocity along that axis,
    // where the gyroscopic term vanishes: the rate grows linearly with the moment of that axis
    // alone and the angle turned quadratically. Each axis is run on its own, so a model that mixed
    // up two moments of inertia cannot pass. The torque is small enough that the spin ends at 1 to
    // 2 rad / s: a faster one would have forwardEuler() grow the body velocity a little more every
    // step, by sqrt(1 + (h omega)^2), into errors of kilometers over kDuration.
    for (const std::size_t axis : {std::size_t{0}, std::size_t{1}, std::size_t{2}}) {
        CAPTURE(axis);
        Control<T> control{};
        control[axis] = kTorque;
        Vec3<T> direction{static_cast<T>(0.0)};
        direction[axis] = static_cast<T>(1.0);
        const T angular_acceleration{kTorque / inertias[axis]};
        const Schedule<T> schedule{constantSchedule(kNumSteps, h, control)};
        const Trajectory<T> trajectory{
            checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
                const Vec3<T> turn{
                    direction * (static_cast<T>(0.5) * angular_acceleration * t * t)
                };
                return freeFallAt(
                    initial_state, t,
                    attitude * Quaternion<T>::fromRotationVector(turn.x(), turn.y(), turn.z()),
                    direction * (angular_acceleration * t)
                );
            })
        };

        if (plot_graph) {
            if constexpr (std::same_as<T, double>) {
                if (axis == 0) {
                    plotTrajectory("constant torque from rest", trajectory);
                }
            }
        }
    }
}

TEST_CASE_TEMPLATE("TorqueFreeAxisymmetricPrecession", T, float, double) {
    const DynamicsModel<T> model{axisymmetricModel<T>()};
    const Vec3<T> initial_omega{makeVec3<T>(0.6, -0.35, 3.0)};
    const Quaternion<T> initial_attitude{attitudeFromRpy<T>(0.4, -0.25, 1.0)};
    const State<T> initial_state{stateOf(
        Vec3<T>{static_cast<T>(0.0), static_cast<T>(0.0), kDropHeight<T>},
        makeVec3<T>(1.0, 2.0, 3.0), initial_attitude, initial_omega
    )};

    // With equal x and y moments J and the axial moment J_z, Euler's equations keep omega_z fixed
    // and turn the transverse rate about the body z axis at lambda = (J_z - J) / J * omega_z. The
    // body itself rotates about the fixed angular momentum L at |L| / J, on top of a rotation about
    // its own z axis at -lambda: q(t) = Exp(|L| / J * t * L / |L|) q0 Exp(-lambda * t * e_z).
    const T lambda{(kAxialInertia<T> - kInertiaXx<T>) / kInertiaXx<T> * initial_omega.z()};
    const Vec3<T> momentum{initial_attitude.rotate(
        Vec3<T>{
            kInertiaXx<T> * initial_omega.x(), kInertiaXx<T> * initial_omega.y(),
            kAxialInertia<T> * initial_omega.z()
        }
    )};
    const Vec3<T> precession{momentum / kInertiaXx<T>};
    constexpr std::size_t kNumSteps{10000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, Control<T>{})};

    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            const T c{std::cos(lambda * t)};
            const T s{std::sin(lambda * t)};
            const Vec3<T> turn{precession * t};
            return freeFallAt(
                initial_state, t,
                Quaternion<T>::fromRotationVector(turn.x(), turn.y(), turn.z()) * initial_attitude *
                    Quaternion<T>::fromRotationVector(
                        static_cast<T>(0.0), static_cast<T>(0.0), -lambda * t
                    ),
                Vec3<T>{
                    initial_omega.x() * c - initial_omega.y() * s,
                    initial_omega.x() * s + initial_omega.y() * c, initial_omega.z()
                }
            );
        })
    };

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("torque free axisymmetric precession", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("LevelSpinWhileTranslating", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr T kYawRate{static_cast<T>(1.5)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};
    const Quaternion<T> attitude{attitudeFromRpy<T>(0.0, 0.0, 0.4)};
    const Vec3<T> omega{static_cast<T>(0.0), static_cast<T>(0.0), kYawRate};
    const State<T> initial_state{
        stateOf(makeVec3<T>(0.0, 0.0, 5.0), makeVec3<T>(3.0, -1.0, 0.5), attitude, omega)
    };
    const Schedule<T> schedule{constantSchedule(
        kNumSteps, h,
        Control<T>{static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>}
    )};

    // Standing level the thrust cancels gravity, so no force acts and the center of mass flies a
    // straight line while the body spins underneath it. The body velocity has to turn the other
    // way round to keep the world velocity fixed, which is the work of the -omega x (u, v, w) term:
    // without it the world velocity would swing round with the body and bend the path into a
    // circle.
    const Vec3<T> world_velocity{worldVelocityOf(initial_state)};
    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            const Quaternion<T> attitude_at{
                attitude * Quaternion<T>::fromRotationVector(
                               static_cast<T>(0.0), static_cast<T>(0.0), kYawRate * t
                           )
            };
            return Sample<T>{
                positionOf(initial_state) + world_velocity * t,
                attitude_at.conjugated().rotate(world_velocity), attitude_at, omega
            };
        })
    };

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("level spin while translating", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("SteadyClimbingTurn", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr T kSpeed{static_cast<T>(3.0)};
    constexpr T kTurnRate{static_cast<T>(1.0)};
    constexpr T kClimbRate{static_cast<T>(0.5)};
    constexpr T kRadius{kSpeed / kTurnRate};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    // The center of mass circles the world z axis at a constant speed and climbs at a constant
    // rate, while the body turns about the world z axis along with it, so that nothing changes in
    // the body frame: the body velocity, the angular velocity and the controls stay constant. The
    // thrust has to carry the centripetal acceleration plus gravity, which banks the body by
    // atan(speed * turn rate / g), about 17 degrees, and the torques have to balance the gyroscopic
    // term omega x J omega, as the angular velocity lies off the principal axes of the banked
    // body. The turn leaves the heading about the thrust axis free: the nose points 0.3 rad off the
    // direction of flight. Every term of the equations of motion takes part.
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
    const Control<T> control{torque.x(), torque.y(), torque.z(), kMass<T> * lift.euclidean()};
    const State<T> initial_state{
        stateOf(start, attitude.conjugated().rotate(world_velocity), attitude, omega)
    };
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, control)};

    // Steady in the body frame: neither velocity changes.
    const State<T> k{model.tangent(initial_state, control)};
    CHECK_LE(velocityOf(k).euclidean(), roundoffTolerance<T>());
    CHECK_LE(omegaOf(k).euclidean(), roundoffTolerance<T>());

    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            const T angle{kTurnRate * t};
            return Sample<T>{
                Vec3<T>{
                    kRadius * std::cos(angle), kRadius * std::sin(angle), start.z() + kClimbRate * t
                },
                velocityOf(initial_state),
                Quaternion<T>::fromRotationVector(static_cast<T>(0.0), static_cast<T>(0.0), angle) *
                    attitude,
                omega
            };
        })
    };

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("steady climbing turn", trajectory);
        }
    }
}

// Scenarios whose rotation has no elementary closed form lean on what the mechanics conserves, on
// the free fall the center of mass keeps to without thrust and on the observed order of
// convergence instead. With no solution to compare against, their plots leave the errors out.

TEST_CASE_TEMPLATE("TorqueFreeAsymmetricInvariants", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr std::size_t kNumSteps{4000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};
    const State<T> initial_state{stateOf(
        Vec3<T>{static_cast<T>(0.0), static_cast<T>(0.0), kDropHeight<T>},
        makeVec3<T>(0.5, 0.5, 0.5), attitudeFromRpy<T>(0.5, 0.3, -0.8), makeVec3<T>(1.5, -2.0, 1.0)
    )};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, Control<T>{})};
    const Trajectory<T> coarse{simulate(model, initial_state, schedule)};
    const Trajectory<T> fine{simulate(model, initial_state, halvedSteps(schedule))};
    checkUnitAttitude(coarse);
    checkUnitAttitude(fine);

    // Without torques the rotational energy and the angular momentum in the world frame are
    // constant, the latter as a vector, which ties the attitude to the angular velocity at every
    // step. forwardEuler() lets the energy grow and the momentum drift by first order, rungeKutta()
    // by fourth order. The drifts are relative to begin with, which makes their scale one. The
    // translation is left to TumblingFreeFall.
    const T energy{rotationalEnergyOf(model, initial_state)};
    const Vec3<T> momentum{angularMomentumOf(model, initial_state)};
    const auto drifts_of = [&](const std::vector<State<T>>& trace) -> std::array<T, 2> {
        std::array<T, 2> drifts{};
        for (const State<T>& state : trace) {
            raiseTo(drifts[0], std::abs(rotationalEnergyOf(model, state) - energy) / energy);
            raiseTo(
                drifts[1],
                angularMomentumOf(model, state).euclideanTo(momentum) / momentum.euclidean()
            );
        }
        return drifts;
    };
    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        const std::array<T, 2> coarse_drifts{drifts_of(coarse.trace(mode))};
        const std::array<T, 2> fine_drifts{drifts_of(fine.trace(mode))};
        checkErrorShrinks(mode, coarse_drifts[0], fine_drifts[0], static_cast<T>(1.0));
        checkErrorShrinks(mode, coarse_drifts[1], fine_drifts[1], static_cast<T>(1.0));
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("torque free asymmetric invariants", coarse);
        }
    }
}

TEST_CASE_TEMPLATE("TumblingFreeFall", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr std::size_t kNumSteps{4000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    // The torques switch sign about all three axes at different points of the run, so the body
    // tumbles with no closed form to it, while without thrust the center of mass still has to fall
    // freely: the body velocity, the attitude and the world velocity they make up have to stay
    // consistent through every step for that. The torques are small enough to keep the tumbling
    // below a couple of rad / s over kDuration.
    const auto control_at = [](T fraction) -> Control<T> {
        return Control<T>{
            fraction < static_cast<T>(0.3) ? static_cast<T>(0.004) : static_cast<T>(-0.003),
            fraction < static_cast<T>(0.6) ? static_cast<T>(-0.002) : static_cast<T>(0.005),
            fraction < static_cast<T>(0.45) ? static_cast<T>(0.001) : static_cast<T>(-0.0016),
            static_cast<T>(0.0)
        };
    };
    Schedule<T> schedule{constantSchedule(kNumSteps, h, Control<T>{})};
    for (std::size_t i{0}; i < kNumSteps; ++i) {
        schedule.controls[i] = control_at(static_cast<T>(i) / static_cast<T>(kNumSteps));
    }
    const State<T> initial_state{stateOf(
        Vec3<T>{static_cast<T>(0.0), static_cast<T>(0.0), kDropHeight<T>},
        makeVec3<T>(2.0, 0.0, 1.0), attitudeFromRpy<T>(0.1, 0.2, 0.3), makeVec3<T>(0.5, -0.4, 0.3)
    )};
    const Trajectory<T> coarse{simulate(model, initial_state, schedule)};
    const Trajectory<T> fine{simulate(model, initial_state, halvedSteps(schedule))};
    checkUnitAttitude(coarse);
    checkUnitAttitude(fine);

    // The largest position and world velocity errors against the free fall along a trace, each on
    // the scale of the largest position and speed of the fall. forwardEuler() is first order in
    // both, rungeKutta() fourth order.
    const Vec3<T> gravity{static_cast<T>(0.0), static_cast<T>(0.0), -kGravity<T>};
    const Vec3<T> world_velocity{worldVelocityOf(initial_state)};
    const auto free_fall_position_at = [&](T t) -> Vec3<T> {
        return positionOf(initial_state) + world_velocity * t +
               gravity * (static_cast<T>(0.5) * t * t);
    };
    std::array<T, 2> scales{};
    for (const T t : coarse.ts) {
        raiseTo(scales[0], free_fall_position_at(t).euclidean());
        raiseTo(scales[1], (world_velocity + gravity * t).euclidean());
    }
    const auto free_fall_errors_of = [&](const Trajectory<T>& trajectory,
                                         IntegrateMode mode) -> std::array<T, 2> {
        std::array<T, 2> errors{};
        for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
            const State<T>& state{trajectory.trace(mode)[i]};
            const T t{trajectory.ts[i]};
            raiseTo(errors[0], positionOf(state).euclideanTo(free_fall_position_at(t)));
            raiseTo(errors[1], worldVelocityOf(state).euclideanTo(world_velocity + gravity * t));
        }
        return errors;
    };
    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        const std::array<T, 2> coarse_errors{free_fall_errors_of(coarse, mode)};
        const std::array<T, 2> fine_errors{free_fall_errors_of(fine, mode)};
        checkErrorShrinks(mode, coarse_errors[0], fine_errors[0], scales[0]);
        checkErrorShrinks(mode, coarse_errors[1], fine_errors[1], scales[1]);
    }

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder(model, initial_state, kDuration<T>, kNumSteps, control_at);
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("tumbling free fall", coarse);
        }
    }
}

TEST_CASE_TEMPLATE("ThrustLeavesTheRotationAlone", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    constexpr std::size_t kNumSteps{4000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    // Thrust and torques both vary, so neither the rotation nor the translation has a closed form.
    // The rotation never reads the translation, though: the same torques with the thrust off have
    // to turn the body exactly the same way, step for step and integrator for integrator. The
    // torques are small enough to keep the tumbling below a couple of rad / s over kDuration.
    const auto control_at = [](T fraction) -> Control<T> {
        const T thrust_scale{
            fraction < static_cast<T>(0.25)  ? static_cast<T>(1.2)
            : fraction < static_cast<T>(0.7) ? static_cast<T>(0.8)
                                             : static_cast<T>(1.05)
        };
        return Control<T>{
            fraction < static_cast<T>(0.5) ? static_cast<T>(0.002) : static_cast<T>(-0.0024),
            fraction < static_cast<T>(0.35) ? static_cast<T>(-0.0016) : static_cast<T>(0.0012),
            fraction < static_cast<T>(0.8) ? static_cast<T>(0.0004) : static_cast<T>(-0.0008),
            thrust_scale * kHoverThrust<T>
        };
    };
    Schedule<T> schedule{constantSchedule(kNumSteps, h, Control<T>{})};
    Schedule<T> without_thrust{schedule};
    for (std::size_t i{0}; i < kNumSteps; ++i) {
        schedule.controls[i] = control_at(static_cast<T>(i) / static_cast<T>(kNumSteps));
        without_thrust.controls[i] = schedule.controls[i];
        without_thrust.controls[i].thrust() = static_cast<T>(0.0);
    }
    // Tilted as it tumbles, the thrust carries less than the weight on balance, so the body sinks
    // about 480 m over kDuration; it starts high enough to end near the ground.
    const State<T> initial_state{stateOf(
        makeVec3<T>(1.0, -1.0, 500.0), makeVec3<T>(1.5, -0.5, 0.2),
        attitudeFromRpy<T>(-0.2, 0.15, 1.2), makeVec3<T>(0.3, 0.2, -0.5)
    )};
    const Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const Trajectory<T> tumbling{simulate(model, initial_state, without_thrust)};
    checkUnitAttitude(trajectory);
    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        T max_attitude_error{static_cast<T>(0.0)};
        T max_omega_error{static_cast<T>(0.0)};
        for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
            const State<T>& state{trajectory.trace(mode)[i]};
            const State<T>& reference{tumbling.trace(mode)[i]};
            raiseTo(max_attitude_error, distanceOf(attitudeOf(state), attitudeOf(reference)));
            raiseTo(max_omega_error, omegaOf(state).euclideanTo(omegaOf(reference)));
        }
        CHECK_LE(max_attitude_error, roundoffTolerance<T>());
        CHECK_LE(max_omega_error, roundoffTolerance<T>());
    }

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder(model, initial_state, kDuration<T>, kNumSteps, control_at);
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("thrust leaves the rotation alone", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("EvolveMatchesMarchingSingleSteps", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    const T tolerance{roundoffTolerance<T>()};

    SUBCASE("AlongASchedule") {
        const State<T> initial_state{stateOf(
            makeVec3<T>(1.0, -2.0, 3.0), makeVec3<T>(1.5, -0.5, 0.25),
            attitudeFromRpy<T>(0.1, -0.2, 0.3), makeVec3<T>(0.4, -0.3, 0.2)
        )};
        // No two entries agree within a span, so an evolve() that read the wrong index cannot
        // pass.
        const Schedule<T> schedule{
            {static_cast<T>(0.010), static_cast<T>(0.012), static_cast<T>(0.008),
             static_cast<T>(0.011), static_cast<T>(0.009)},
            {Control<T>{
                 static_cast<T>(0.02), static_cast<T>(-0.01), static_cast<T>(0.005),
                 static_cast<T>(16.0)
             },
             Control<T>{
                 static_cast<T>(-0.02), static_cast<T>(0.015), static_cast<T>(0.0),
                 static_cast<T>(12.0)
             },
             Control<T>{
                 static_cast<T>(0.01), static_cast<T>(0.0), static_cast<T>(-0.004),
                 static_cast<T>(18.0)
             },
             Control<T>{
                 static_cast<T>(0.0), static_cast<T>(-0.02), static_cast<T>(0.003),
                 static_cast<T>(14.5)
             },
             Control<T>{
                 static_cast<T>(0.005), static_cast<T>(0.01), static_cast<T>(-0.002),
                 static_cast<T>(15.5)
             }}
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            // Marching the single step integrator by hand gives the state after every step, which
            // is what the trace is supposed to hold behind the initial state.
            std::vector<State<T>> expected{initial_state};
            for (std::size_t i{0}; i < schedule.hs.size(); ++i) {
                expected.push_back(
                    stepWith(mode, model, expected.back(), schedule.controls[i], schedule.hs[i])
                );
            }
            std::vector<State<T>> trace(schedule.hs.size() + 1);
            const State<T> returned{evolveWith(mode, model, initial_state, schedule, trace)};
            for (std::size_t i{0}; i < trace.size(); ++i) {
                CHECK_LE(distanceOf(trace[i], expected[i]), tolerance);
            }
            CHECK_EQ(trace.front(), initial_state);
            CHECK_EQ(returned, trace.back());
            // An empty trace records nothing and leaves the run as it is.
            CHECK_LE(
                distanceOf(evolveWith(mode, model, initial_state, schedule), returned), tolerance
            );
        }
    }

    SUBCASE("WithoutSteps") {
        // Without a step to take, the initial state comes back untouched, traced or not.
        const State<T> initial_state{stateOf(
            makeVec3<T>(1.0, 2.0, 3.0), makeVec3<T>(4.0, 5.0, 6.0),
            attitudeFromRpy<T>(0.1, 0.2, 0.3), makeVec3<T>(0.7, 0.8, 0.9)
        )};
        const Schedule<T> none{};
        std::vector<State<T>> trace(1);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_EQ(evolveWith(mode, model, initial_state, none), initial_state);
            CHECK_EQ(evolveWith(mode, model, initial_state, none, trace), initial_state);
            CHECK_EQ(trace.front(), initial_state);
        }
    }
}

TEST_CASE_TEMPLATE("TangentFollowsTheEquationsOfMotion", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    const Quaternion<T> attitude{attitudeFromRpy<T>(0.3, -0.4, 1.1)};
    const State<T> state{stateOf(
        makeVec3<T>(1.0, -2.0, 3.0), makeVec3<T>(2.0, -1.5, 0.5), attitude,
        makeVec3<T>(0.7, -1.2, 0.9)
    )};
    const Control<T> control{
        static_cast<T>(0.03), static_cast<T>(-0.02), static_cast<T>(0.01), static_cast<T>(17.0)
    };
    const T tolerance{roundoffTolerance<T>()};

    SUBCASE("EveryFieldMatchesTheNewtonEulerEquations") {
        // Written out from the vectors here, through Quaternion and Vec3, rather than from the
        // scalar expansions tangent() is made of.
        const State<T> k{model.tangent(state, control)};
        const Vec3<T> velocity{velocityOf(state)};
        const Vec3<T> omega{omegaOf(state)};
        const Vec3<T> up{makeVec3<T>(0.0, 0.0, 1.0)};
        const Vec3<T> momentum{
            kInertiaXx<T> * omega.x(), kInertiaYy<T> * omega.y(), kInertiaZz<T> * omega.z()
        };
        const Vec3<T> torque_left{
            Vec3<T>{control.torque_x(), control.torque_y(), control.torque_z()} -
            omega.cross(momentum)
        };
        CHECK_LE(positionOf(k).euclideanTo(attitude.rotate(velocity)), tolerance);
        CHECK_LE(
            velocityOf(k).euclideanTo(
                -omega.cross(velocity) +
                up * (control.thrust() / kMass<T>)-attitude.conjugated().rotate(up) * kGravity<T>
            ),
            tolerance
        );
        // dq/dt = q (omega, 0) / 2, the Hamilton product of Quaternion.
        CHECK_LE(
            distanceOf(
                attitudeOf(k),
                attitude * Quaternion<T>{omega.x(), omega.y(), omega.z(), static_cast<T>(0.0)} *
                    static_cast<T>(0.5)
            ),
            tolerance
        );
        CHECK_LE(
            omegaOf(k).euclideanTo(
                Vec3<T>{
                    torque_left.x() / kInertiaXx<T>, torque_left.y() / kInertiaYy<T>,
                    torque_left.z() / kInertiaZz<T>
                }
            ),
            tolerance
        );
    }

    SUBCASE("AttitudeOfAnyNormRotatesAsItsUnitOne") {
        // The stages of rungeKutta() and interpolated states carry an attitude off unit norm.
        // Scaling it has to leave every field alone but dq/dt, which scales along with it.
        constexpr T kScale{static_cast<T>(1.7)};
        State<T> scaled{state};
        scaled.qx() *= kScale;
        scaled.qy() *= kScale;
        scaled.qz() *= kScale;
        scaled.qw() *= kScale;
        const State<T> k{model.tangent(state, control)};
        const State<T> k_scaled{model.tangent(scaled, control)};
        CHECK_LE(positionOf(k_scaled).euclideanTo(positionOf(k)), tolerance);
        CHECK_LE(velocityOf(k_scaled).euclideanTo(velocityOf(k)), tolerance);
        CHECK_LE(omegaOf(k_scaled).euclideanTo(omegaOf(k)), tolerance);
        CHECK_LE(distanceOf(attitudeOf(k_scaled), attitudeOf(k) * kScale), tolerance);
    }

    SUBCASE("TheFlowKeepsTheNormOfTheAttitude") {
        // dq/dt is orthogonal to q, which is what keeps |q| constant along the exact flow and
        // leaves only the integrators' own error for the normalization to take out.
        const State<T> k{model.tangent(state, control)};
        CHECK_LE(std::abs(attitudeOf(k).dot(attitude)), tolerance);
    }

    SUBCASE("HoverIsAnEquilibrium") {
        // Standing level and at rest, with the thrust carrying the weight exactly, whichever way
        // the body faces, nothing moves: the tangent vanishes and every integrator leaves the
        // state where it is.
        const State<T> standing{stateOf(
            makeVec3<T>(1.0, -2.0, 3.0), makeVec3<T>(0.0, 0.0, 0.0),
            attitudeFromRpy<T>(0.0, 0.0, 0.3), makeVec3<T>(0.0, 0.0, 0.0)
        )};
        const Control<T> hover{
            static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
        };
        CHECK_LE(distanceOf(model.tangent(standing, hover), State<T>{0.0}), tolerance);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_LE(
                distanceOf(stepWith(mode, model, standing, hover, kH<T>), standing), tolerance
            );
        }
    }
}

// The four controls are the one place the model says no: everything else it takes at face value,
// while the torques and the thrust are clamped into the bounds it was built with.

TEST_CASE_TEMPLATE("ControlBoundsClampEveryEntryPoint", T, float, double) {
    const DynamicsModel<T> model{boundedModel<T>()};
    const DynamicsModel<T> unbounded{unboundedModel<T>()};
    const State<T> start{stateOf(
        makeVec3<T>(0.0, 0.0, 5.0), makeVec3<T>(1.0, 0.5, -0.25),
        attitudeFromRpy<T>(0.1, 0.2, -0.3), makeVec3<T>(0.4, -0.2, 0.3)
    )};

    SUBCASE("BoundsComeBackOutAndDefaultToInfinite") {
        CHECK_EQ(model.control_lower_bound(), lowerBound<T>());
        CHECK_EQ(model.control_upper_bound(), upperBound<T>());
        CHECK_EQ(unbounded.control_lower_bound(), Control<T>{-std::numeric_limits<T>::infinity()});
        CHECK_EQ(unbounded.control_upper_bound(), Control<T>{std::numeric_limits<T>::infinity()});
    }

    SUBCASE("EveryEntryPointClamps") {
        // Controls beyond the bounds have to act exactly as the bounds they are clamped to, each
        // field against its own bound.
        const Control<T> loose{
            static_cast<T>(5.0), static_cast<T>(-4.0), static_cast<T>(3.0), static_cast<T>(0.5)
        };
        const Control<T> clamped{kTorqueXUp<T>, kTorqueYLo<T>, kTorqueZUp<T>, kThrustLo<T>};
        CHECK_EQ(model.tangent(start, loose), model.tangent(start, clamped));
        CHECK_NE(unbounded.tangent(start, loose), unbounded.tangent(start, clamped));
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_LE(
                distanceOf(
                    stepWith(mode, model, start, loose, kH<T>),
                    stepWith(mode, model, start, clamped, kH<T>)
                ),
                roundoffTolerance<T>()
            );
        }
    }

    SUBCASE("ClampingEqualsSteppingTheClampedSchedule") {
        // However the clamp is spelled, it has to come to the same thing as handing an unbounded
        // model the already clamped schedule, step for step. The two walk the same arithmetic but
        // not the same code path, so they are only held to the accuracy the integrator itself is
        // guaranteed: the extra clamps shift which multiply adds the compiler fuses, which moves
        // the last bits of every step and compounds over hundreds of them.
        constexpr std::size_t kNumSteps{200};
        Schedule<T> schedule{constantSchedule(kNumSteps, kH<T>, Control<T>{})};
        Schedule<T> clamped{schedule};
        for (std::size_t i{0}; i < kNumSteps; ++i) {
            const T t{static_cast<T>(i) * kH<T>};
            schedule.controls[i] = Control<T>{
                static_cast<T>(0.8) * std::sin(t), static_cast<T>(0.7) * std::cos(t),
                static_cast<T>(0.3) * std::sin(static_cast<T>(2.0) * t),
                static_cast<T>(20.0) + static_cast<T>(25.0) * std::sin(t)
            };
            for (std::size_t j{0}; j < Control<T>::size(); ++j) {
                clamped.controls[i][j] =
                    std::clamp(schedule.controls[i][j], lowerBound<T>()[j], upperBound<T>()[j]);
            }
        }
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_LE(
                distanceOf(
                    evolveWith(mode, model, start, schedule),
                    evolveWith(mode, unbounded, start, clamped)
                ),
                rungeKuttaTolerance<T>()
            );
        }
    }

    SUBCASE("BoundsNothingEverReachesChangeNothing") {
        // Same reasoning as above: bounds nothing reaches must not move the trajectory, but the
        // clamps still stand between the two call sites.
        const DynamicsModel<T> wide{
            kMass<T>,
            kInertiaXx<T>,
            kInertiaYy<T>,
            kInertiaZz<T>,
            Control<T>{static_cast<T>(-1.0e3)},
            Control<T>{static_cast<T>(1.0e3)}
        };
        const Schedule<T> schedule{constantSchedule(
            200, kH<T>,
            Control<T>{
                static_cast<T>(0.01), static_cast<T>(-0.02), static_cast<T>(0.005),
                static_cast<T>(16.0)
            }
        )};
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_LE(
                distanceOf(
                    evolveWith(mode, wide, start, schedule),
                    evolveWith(mode, unbounded, start, schedule)
                ),
                rungeKuttaTolerance<T>()
            );
        }
    }
}

TEST_CASE_TEMPLATE("IntegratorsKeepTheAttitudeUnit", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};

    SUBCASE("UnderFastRotation") {
        // A fast spin over large steps, h |omega| = 0.5: forward Euler alone would grow |q|^2 by a
        // factor of 1 + h^2 |omega|^2 / 4 = 1.0625 a step. The spin is about a principal axis, so
        // the angular velocity stays exactly put and the steps stay stable, which a spin off the
        // principal axes this coarse would not: the gyroscopic term blows forward Euler up.
        const State<T> initial_state{stateOf(
            makeVec3<T>(0.0, 0.0, 0.0), makeVec3<T>(0.0, 0.0, 0.0),
            attitudeFromRpy<T>(0.2, 0.4, 0.6), makeVec3<T>(0.0, 0.0, 10.0)
        )};
        const Schedule<T> schedule{constantSchedule(500, static_cast<T>(0.05), Control<T>{})};
        checkUnitAttitude(simulate(model, initial_state, schedule));
    }

    SUBCASE("FromAnAttitudeJustOffUnitNorm") {
        // An attitude off unit norm by less than the integrators check against is accepted and
        // comes back normalized.
        const State<T> initial_state{stateOf(
            makeVec3<T>(0.0, 0.0, 0.0), makeVec3<T>(0.0, 0.0, 0.0),
            attitudeFromRpy<T>(0.2, 0.4, 0.6) *
                (static_cast<T>(1.0) +
                 static_cast<T>(0.25) * std::sqrt(std::numeric_limits<T>::epsilon())),
            makeVec3<T>(0.0, 0.0, 0.0)
        )};
        const Control<T> control{
            static_cast<T>(0.01), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            const State<T> next{stepWith(mode, model, initial_state, control, kH<T>)};
            CHECK_LE(
                std::abs(attitudeOf(next).euclideanSqr() - static_cast<T>(1.0)),
                static_cast<T>(16.0) * std::numeric_limits<T>::epsilon()
            );
        }
    }
}

TEST_CASE_TEMPLATE("StateArithmetic", T, float, double) {
    // Binary fractions keep every expected result exact for float as well as for double, and no
    // two fields agree, so an operator that mixed two of them up cannot hide behind a coincidence.
    const State<T> lhs{static_cast<T>(1.5),   static_cast<T>(-2.5),   static_cast<T>(0.75),
                       static_cast<T>(0.125), static_cast<T>(7.5),    static_cast<T>(-3.0),
                       static_cast<T>(0.5),   static_cast<T>(-0.25),  static_cast<T>(0.375),
                       static_cast<T>(2.0),   static_cast<T>(-0.625), static_cast<T>(4.5),
                       static_cast<T>(-1.75)};
    const State<T> rhs{static_cast<T>(0.5),   static_cast<T>(1.5),  static_cast<T>(-0.25),
                       static_cast<T>(-0.75), static_cast<T>(2.5),  static_cast<T>(1.25),
                       static_cast<T>(-1.0),  static_cast<T>(0.75), static_cast<T>(0.0625),
                       static_cast<T>(-0.5),  static_cast<T>(3.0),  static_cast<T>(-0.5),
                       static_cast<T>(0.25)};

    SUBCASE("LayoutCoversEveryField") {
        static_assert(State<T>::size() == 13);
        CHECK_EQ(sizeof(State<T>), State<T>::size() * sizeof(T));
        CHECK_EQ(lhs.data(), &lhs.x());
        const std::array<const T*, 13> fields{&lhs.x(),      &lhs.y(),       &lhs.z(),
                                              &lhs.u(),      &lhs.v(),       &lhs.w(),
                                              &lhs.qx(),     &lhs.qy(),      &lhs.qz(),
                                              &lhs.qw(),     &lhs.omega_x(), &lhs.omega_y(),
                                              &lhs.omega_z()};
        for (std::size_t i{0}; i < fields.size(); ++i) {
            CHECK_EQ(fields[i], lhs.data() + i);
        }
    }

    SUBCASE("ConstructorTakesTheFieldsInStorageOrder") {
        const State<T> counted{static_cast<T>(0.0), static_cast<T>(1.0),  static_cast<T>(2.0),
                               static_cast<T>(3.0), static_cast<T>(4.0),  static_cast<T>(5.0),
                               static_cast<T>(6.0), static_cast<T>(7.0),  static_cast<T>(8.0),
                               static_cast<T>(9.0), static_cast<T>(10.0), static_cast<T>(11.0),
                               static_cast<T>(12.0)};
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            CHECK_EQ(counted[i], static_cast<T>(i));
        }
    }

    SUBCASE("DefaultIsAtRestUnderTheIdentityAttitude") {
        const State<T> standing{};
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            CHECK_EQ(standing[i], i == 9 ? static_cast<T>(1.0) : static_cast<T>(0.0));
        }
    }

    SUBCASE("EveryFieldTakesPartInEquality") {
        CHECK_EQ(lhs, State<T>{lhs});
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            State<T> perturbed{lhs};
            perturbed.data()[i] += static_cast<T>(1.0);
            CHECK_NE(perturbed, lhs);
        }
    }

    SUBCASE("IndexingWalksTheFieldsInOrder") {
        // operator[] reaches every field in the order data() lays them out in, and writes through
        // as well as it reads.
        State<T> written{};
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            CHECK_EQ(&lhs[i], lhs.data() + i);
            written[i] = lhs[i];
        }
        CHECK_EQ(written, lhs);
    }

    SUBCASE("FillConstructorSetsEveryField") {
        const State<T> filled{static_cast<T>(0.75)};
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            CHECK_EQ(filled[i], static_cast<T>(0.75));
        }
        CHECK_EQ(lhs + State<T>{static_cast<T>(0.0)}, lhs);
    }

    SUBCASE("OperatorsTouchEveryField") {
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            CHECK_EQ((lhs + rhs)[i], lhs[i] + rhs[i]);
            CHECK_EQ((lhs - rhs)[i], lhs[i] - rhs[i]);
            CHECK_EQ((lhs * static_cast<T>(2.0))[i], lhs[i] * static_cast<T>(2.0));
            CHECK_EQ((static_cast<T>(2.0) * lhs)[i], lhs[i] * static_cast<T>(2.0));
            CHECK_EQ((lhs / static_cast<T>(2.0))[i], lhs[i] * static_cast<T>(0.5));
            CHECK_EQ((-lhs)[i], -lhs[i]);
        }
    }

    SUBCASE("RvalueOperandsAgreeWithLvalueOnes") {
        // An rvalue right hand side of a subtraction is overwritten in place, so it is the one
        // overload whose operand order can flip.
        CHECK_EQ(lhs - State<T>{rhs}, lhs - rhs);
        CHECK_EQ(State<T>{lhs} - rhs, lhs - rhs);
        CHECK_EQ(lhs + State<T>{rhs}, lhs + rhs);
        CHECK_EQ(static_cast<T>(2.0) * State<T>{lhs}, lhs * static_cast<T>(2.0));
        CHECK_EQ(-State<T>{lhs}, -lhs);
    }
}

TEST_CASE_TEMPLATE("ControlArithmetic", T, float, double) {
    const Control<T> lhs{
        static_cast<T>(0.5), static_cast<T>(-0.25), static_cast<T>(0.125), static_cast<T>(15.0)
    };
    const Control<T> rhs{
        static_cast<T>(-1.5), static_cast<T>(0.75), static_cast<T>(2.5), static_cast<T>(-4.0)
    };

    SUBCASE("LayoutCoversEveryField") {
        static_assert(Control<T>::size() == 4);
        CHECK_EQ(sizeof(Control<T>), Control<T>::size() * sizeof(T));
        const std::array<const T*, 4> fields{
            &lhs.torque_x(), &lhs.torque_y(), &lhs.torque_z(), &lhs.thrust()
        };
        for (std::size_t i{0}; i < fields.size(); ++i) {
            CHECK_EQ(fields[i], lhs.data() + i);
            CHECK_EQ(&lhs[i], lhs.data() + i);
        }
        CHECK_EQ(Control<T>{}, Control<T>{static_cast<T>(0.0)});
    }

    SUBCASE("EveryFieldTakesPartInEquality") {
        for (std::size_t i{0}; i < Control<T>::size(); ++i) {
            Control<T> perturbed{lhs};
            perturbed[i] += static_cast<T>(1.0);
            CHECK_NE(perturbed, lhs);
        }
    }

    SUBCASE("OperatorsTouchEveryField") {
        for (std::size_t i{0}; i < Control<T>::size(); ++i) {
            CHECK_EQ((lhs + rhs)[i], lhs[i] + rhs[i]);
            CHECK_EQ((lhs - rhs)[i], lhs[i] - rhs[i]);
            CHECK_EQ((static_cast<T>(2.0) * lhs)[i], lhs[i] * static_cast<T>(2.0));
            CHECK_EQ((lhs / static_cast<T>(2.0))[i], lhs[i] * static_cast<T>(0.5));
            CHECK_EQ((-lhs)[i], -lhs[i]);
        }
        CHECK_EQ(lhs - Control<T>{rhs}, lhs - rhs);
    }
}

TEST_CASE_TEMPLATE("StateSerializationRoundtrip", T, float, double) {
    for (const State<T>& state :
         {State<T>{},
          stateOf(
              makeVec3<T>(1.5, -2.5, 0.3), makeVec3<T>(0.15, 7.9, -0.6),
              attitudeFromRpy<T>(0.1, 0.2, 0.3), makeVec3<T>(0.4, -0.5, 0.6)
          ),
          State<T>{static_cast<T>(-3.2)}}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(state).code == std::errc{});
        State<T> restored{static_cast<T>(9.0)};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, state);
    }
}

TEST_CASE_TEMPLATE("ControlSerializationRoundtrip", T, float, double) {
    for (const Control<T>& control :
         {Control<T>{}, lowerBound<T>(), upperBound<T>(),
          Control<T>{std::numeric_limits<T>::infinity()}}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(control).code == std::errc{});
        Control<T> restored{static_cast<T>(9.0)};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, control);
    }
}

/**
 * @brief Interpolates a trajectory of values, states or controls, through every piecewise function.
 *
 * A state and a control are vectors in the sense of VecArithmetic, which is all the piecewise
 * functions and their proxy ask of a value type. Every field below is affine in t, which linear,
 * natural cubic and quintic splines all reproduce exactly, so what is left is the roundoff of the
 * spline solves.
 */
template <typename Value>
auto checkInterpolation(const Value& origin, const Value& rate) -> void {
    using T = typename Value::value_type;
    using ::basjoo::math::FunctionProxy;
    using ::basjoo::math::makeFunctionProxy;
    using ::basjoo::math::PiecewiseCubicFunction;
    using ::basjoo::math::PiecewiseLinearFunction;
    using ::basjoo::math::PiecewiseQuinticFunction;
    static_assert(::basjoo::math::VecArithmetic<Value>);
    static_assert(::basjoo::math::isVecArithmeticV<Value>);
    static_assert(!::basjoo::math::MatArithmetic<Value>);
    static_assert(!::basjoo::math::isMatArithmeticV<Value>);
    constexpr T kTolerance{
        std::same_as<T, float> ? static_cast<T>(1.0e-4) : static_cast<T>(1.0e-10)
    };

    const auto exact = [&](T t) -> Value { return origin + rate * t; };
    // Unevenly spaced, so that no spline solve gets to lean on equal intervals.
    const std::vector<T> ts{
        static_cast<T>(0.0), static_cast<T>(0.5), static_cast<T>(1.25), static_cast<T>(2.0),
        static_cast<T>(3.0)
    };
    std::vector<Value> ys;
    ys.reserve(ts.size());
    for (const T t : ts) {
        ys.push_back(exact(t));
    }
    const T lower{static_cast<T>(0.3)};
    const T upper{static_cast<T>(2.6)};
    const Value area{
        origin * (upper - lower) + rate * ((upper * upper - lower * lower) * static_cast<T>(0.5))
    };

    std::vector<FunctionProxy<Value>> functions;
    functions.emplace_back(makeFunctionProxy(PiecewiseLinearFunction<Value>{ts, ys}));
    functions.emplace_back(makeFunctionProxy(PiecewiseCubicFunction<Value>{ts, ys}));
    functions.emplace_back(makeFunctionProxy(PiecewiseQuinticFunction<Value>{ts, ys}));
    for (const FunctionProxy<Value>& function : functions) {
        for (const T t :
             {static_cast<T>(0.2), static_cast<T>(0.9), static_cast<T>(1.6), static_cast<T>(2.7)}) {
            CHECK_LE(distanceOf(function->eval(t), exact(t)), kTolerance);
            CHECK_LE(distanceOf(function->derivative(t), rate), kTolerance);
        }
        CHECK_LE(distanceOf(function->integral(lower, upper), area), kTolerance);

        // Restored into an empty proxy, so the concrete function has to come through the archive.
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(function).code == std::errc{});
        FunctionProxy<Value> restored{};
        REQUIRE(in(restored).code == std::errc{});
        REQUIRE(restored.has_value());
        CHECK(proxy_typeid(*restored) == proxy_typeid(*function));
        CHECK_EQ(restored->eval(lower), function->eval(lower));
    }
}

TEST_CASE_TEMPLATE("StateTrajectoriesInterpolate", T, float, double) {
    // The attitude fields are interpolated field by field like the others, which takes them off
    // unit norm between the samples; tangent() takes such states as their normalized attitude.
    checkInterpolation(
        stateOf(
            makeVec3<T>(1.5, -2.5, 0.3), makeVec3<T>(0.15, 7.5, -1.0),
            attitudeFromRpy<T>(0.1, 0.2, 0.3), makeVec3<T>(0.4, -0.5, 0.6)
        ),
        State<T>{
            static_cast<T>(7.5), static_cast<T>(1.25), static_cast<T>(-0.4), static_cast<T>(0.05),
            static_cast<T>(-1.5), static_cast<T>(0.75), static_cast<T>(0.02), static_cast<T>(-0.03),
            static_cast<T>(0.04), static_cast<T>(-0.01), static_cast<T>(0.3), static_cast<T>(-0.2),
            static_cast<T>(0.1)
        }
    );
}

TEST_CASE_TEMPLATE("ControlTrajectoriesInterpolate", T, float, double) {
    checkInterpolation(
        Control<T>{
            static_cast<T>(0.02), static_cast<T>(-0.01), static_cast<T>(0.005), static_cast<T>(14.0)
        },
        Control<T>{
            static_cast<T>(-0.004), static_cast<T>(0.003), static_cast<T>(0.001),
            static_cast<T>(1.5)
        }
    );
}

TEST_CASE_TEMPLATE("ModelSerializationRoundtrip", T, float, double) {
    const DynamicsModel<T> model{
        kMass<T>,        kInertiaXx<T>,   kInertiaYy<T>,       kInertiaZz<T>,
        lowerBound<T>(), upperBound<T>(), static_cast<T>(3.71)
    };
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(model).code == std::errc{});
    DynamicsModel<T> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.mass(), model.mass());
    CHECK_EQ(restored.inertia_xx(), model.inertia_xx());
    CHECK_EQ(restored.inertia_yy(), model.inertia_yy());
    CHECK_EQ(restored.inertia_zz(), model.inertia_zz());
    CHECK_EQ(restored.control_lower_bound(), model.control_lower_bound());
    CHECK_EQ(restored.control_upper_bound(), model.control_upper_bound());
    CHECK_EQ(restored.gravity(), model.gravity());
    // The controls are well beyond the bounds, so a restored model that lost the bounds, the
    // inertia or the gravity would step somewhere else.
    const State<T> state{stateOf(
        makeVec3<T>(0.0, 0.0, 1.0), makeVec3<T>(1.0, -1.0, 0.5), attitudeFromRpy<T>(0.1, -0.2, 0.3),
        makeVec3<T>(0.5, -0.6, 0.7)
    )};
    const Control<T> control{
        static_cast<T>(9.0), static_cast<T>(-9.0), static_cast<T>(9.0), static_cast<T>(90.0)
    };
    CHECK_LE(
        distanceOf(
            restored.rungeKutta(state, control, kH<T>), model.rungeKutta(state, control, kH<T>)
        ),
        roundoffTolerance<T>()
    );
}

TEST_CASE_TEMPLATE("DefaultModel", T, float, double) {
    // A unit body under standard gravity with no bounds: what a deserialization starts from.
    const DynamicsModel<T> model{};
    CHECK_EQ(model.mass(), static_cast<T>(1.0));
    CHECK_EQ(model.inertia_xx(), static_cast<T>(1.0));
    CHECK_EQ(model.inertia_yy(), static_cast<T>(1.0));
    CHECK_EQ(model.inertia_zz(), static_cast<T>(1.0));
    CHECK_EQ(model.gravity(), kGravity<T>);
    CHECK_EQ(model.control_lower_bound(), Control<T>{-std::numeric_limits<T>::infinity()});
    CHECK_EQ(model.control_upper_bound(), Control<T>{std::numeric_limits<T>::infinity()});
    CHECK_EQ(
        (DynamicsModel<T>{kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>}.gravity()),
        kGravity<T>
    );
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE("RejectsInvalidArguments", T, float, double) {
    const DynamicsModel<T> model{unboundedModel<T>()};
    const State<T> state{stateOf(
        makeVec3<T>(0.0, 0.0, 1.0), makeVec3<T>(1.0, 0.0, 0.0), attitudeFromRpy<T>(0.1, 0.2, 0.3),
        makeVec3<T>(0.0, 0.0, 0.1)
    )};
    const Control<T> hover{
        static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kHoverThrust<T>
    };

    SUBCASE("ModelRejectsNonPositiveMassOrInertia") {
        CHECK_THROWS_AS(
            (DynamicsModel<T>{static_cast<T>(0.0), kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>}),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            (DynamicsModel<T>{kMass<T>, static_cast<T>(-0.01), kInertiaYy<T>, kInertiaZz<T>}),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            (DynamicsModel<T>{kMass<T>, kInertiaXx<T>, static_cast<T>(0.0), kInertiaZz<T>}),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            (DynamicsModel<T>{kMass<T>, kInertiaXx<T>, kInertiaYy<T>, static_cast<T>(-1.0)}),
            std::invalid_argument
        );
    }

    SUBCASE("ModelRejectsNegativeGravity") {
        // A negative gravity is what a habit of NED frames leads to; here it pulls along -z.
        CHECK_THROWS_AS(
            (DynamicsModel<T>{
                kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>, lowerBound<T>(),
                upperBound<T>(), -kGravity<T>
            }),
            std::invalid_argument
        );
        CHECK_NOTHROW(
            static_cast<void>(DynamicsModel<T>{
                kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>, lowerBound<T>(),
                upperBound<T>(), static_cast<T>(0.0)
            })
        );
    }

    SUBCASE("ModelRejectsTorqueBoundsThatDoNotBracketZero") {
        for (std::size_t axis{0}; axis < 3; ++axis) {
            CAPTURE(axis);
            Control<T> lower{lowerBound<T>()};
            lower[axis] = static_cast<T>(0.05);
            CHECK_THROWS_AS(
                (DynamicsModel<T>{
                    kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>, lower, upperBound<T>()
                }),
                std::invalid_argument
            );
            Control<T> upper{upperBound<T>()};
            upper[axis] = static_cast<T>(-0.05);
            CHECK_THROWS_AS(
                (DynamicsModel<T>{
                    kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>, lowerBound<T>(), upper
                }),
                std::invalid_argument
            );
        }
    }

    SUBCASE("ModelRejectsThrustBoundsOutOfOrder") {
        // The thrust bounds need not bracket zero, an idle thrust above it is fine, but they have
        // to be in order.
        CHECK_NOTHROW(static_cast<void>(boundedModel<T>()));
        Control<T> lower{lowerBound<T>()};
        lower.thrust() = kThrustUp<T> + static_cast<T>(1.0);
        CHECK_THROWS_AS(
            (DynamicsModel<T>{
                kMass<T>, kInertiaXx<T>, kInertiaYy<T>, kInertiaZz<T>, lower, upperBound<T>()
            }),
            std::invalid_argument
        );
    }

    SUBCASE("IntegratorsRejectNonPositiveTimeStep") {
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const T h : {static_cast<T>(0.0), static_cast<T>(-0.01)}) {
                CHECK_THROWS_AS(
                    static_cast<void>(stepWith(mode, model, state, hover, h)), std::invalid_argument
                );
            }
        }
    }

    SUBCASE("IntegratorsRejectAttitudeOffUnitNorm") {
        // The zero of the state, which generic code builds, is no attitude at all, and one scaled
        // off unit norm by more than sqrt(epsilon) is taken for a mistake.
        State<T> scaled{state};
        scaled.qx() *= static_cast<T>(1.01);
        scaled.qy() *= static_cast<T>(1.01);
        scaled.qz() *= static_cast<T>(1.01);
        scaled.qw() *= static_cast<T>(1.01);
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const State<T>& rejected : {State<T>{static_cast<T>(0.0)}, scaled}) {
                CHECK_THROWS_AS(
                    static_cast<void>(stepWith(mode, model, rejected, hover, kH<T>)),
                    std::invalid_argument
                );
            }
        }
    }

    SUBCASE("EvolveRejectsMismatchedSpanSizes") {
        const Schedule<T> short_controls{
            std::vector<T>(4, kH<T>), std::vector<Control<T>>(3, hover)
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(evolveWith(mode, model, state, short_controls)),
                std::invalid_argument
            );
        }
    }

    SUBCASE("EvolveRejectsTraceOfWrongLength") {
        const Schedule<T> schedule{constantSchedule(4, kH<T>, hover)};
        std::vector<State<T>> too_short(4);
        std::vector<State<T>> too_long(6);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(evolveWith(mode, model, state, schedule, too_short)),
                std::invalid_argument
            );
            CHECK_THROWS_AS(
                static_cast<void>(evolveWith(mode, model, state, schedule, too_long)),
                std::invalid_argument
            );
        }
    }
}

#endif

} // namespace basjoo::quadrotor

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options("dynamics_model_test", "unit test of DynamicsModel class");
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
