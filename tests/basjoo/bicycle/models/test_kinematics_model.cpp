/**
 * @file test_kinematics_model.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-08-08
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/bicycle/models/kinematics_model.hpp"

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
#include "basjoo/math/dense/vec2.hpp"
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
 * @brief The nine subplots of a figure on a 3 x 3 grid, in reading order.
 *
 * The path in the plane leads the top row, followed by the position and the yaw errors. The rows
 * below hold the five fields of the state and the yaw rate against time, a quantity per subplot.
 * The labels are gnuplot's enhanced text, in which _ and ^ set subscripts and superscripts.
 */
auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    const std::array<const char*, 8> ylabels{
        "log_{10}‖r − r^*‖ [m]",
        "log_{10}|ψ − ψ^*| [rad]",
        "x [m]",
        "y [m]",
        "ψ [rad]",
        "u [m/s]",
        "δ [rad]",
        "dψ/dt [rad/s]"
    };
    std::vector<matplot::axes_handle> axes;
    axes.reserve(ylabels.size() + 1);
    for (std::size_t i = 0; i <= ylabels.size(); ++i) {
        axes.emplace_back(fig->add_subplot(3, 3, i));
    }
    for (const matplot::axes_handle& ax : axes) {
        ax->grid(matplot::on);
        ax->hold(matplot::on);
        ax->font_size(12.0);
        ax->x_axis().label_font_size(12.0);
        ax->y_axis().label_font_size(12.0);
    }
    axes[0]->xlabel("x [m]");
    axes[0]->ylabel("y [m]");
    for (std::size_t i = 0; i < ylabels.size(); ++i) {
        axes[i + 1]->xlabel("t [s]");
        axes[i + 1]->ylabel(ylabels[i]);
    }
    return axes;
}

} // namespace

namespace basjoo::bicycle {

namespace {

using ::basjoo::math::IntegrateMode;
using ::basjoo::math::Vec2;

template <std::floating_point T>
inline constexpr T kFrontWheelbase{static_cast<T>(1.2)};
template <std::floating_point T>
inline constexpr T kRearWheelbase{static_cast<T>(1.4)};
template <std::floating_point T>
inline constexpr T kWheelbase{kFrontWheelbase<T> + kRearWheelbase<T>};
template <std::floating_point T>
inline constexpr T kALo{static_cast<T>(-5.0)};
template <std::floating_point T>
inline constexpr T kAUp{static_cast<T>(3.0)};
template <std::floating_point T>
inline constexpr T kOmegaLo{static_cast<T>(-0.4)};
template <std::floating_point T>
inline constexpr T kOmegaUp{static_cast<T>(0.5)};
template <std::floating_point T>
inline constexpr T kInitialYaw{static_cast<T>(0.3)};
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};
// Every scenario runs this long, which gives the errors of the integrators time to build up.
template <std::floating_point T>
inline constexpr T kDuration{static_cast<T>(20.0)};

inline constexpr std::array<IntegrateMode, 2> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::RUNGE_KUTTA
};

// The errors maxErrorsOf() measures, in the order it returns them.
inline constexpr std::array<const char*, 3> kErrorNames{"position", "yaw", "speed"};

template <std::floating_point T>
using State = KinematicsModelState<T>;
template <std::floating_point T>
using Control = KinematicsModelControl<T>;

// The array storage lets the state arithmetic and the accessors run in constant expressions.
consteval auto constantState() -> bool {
    State<double> s{1.0, 2.0, 0.5, 3.0, 0.25};
    s += State<double>{1.0};
    s *= 2.0;
    s.u() -= 1.0;
    return s.x() == 4.0 && s.u() == 7.0 && s[2] == 3.0 && s - s == State<double>{0.0};
}
static_assert(constantState());

consteval auto constantControl() -> bool {
    Control<double> c{0.5, -0.25};
    c += Control<double>{1.0};
    c *= 2.0;
    c.omega() -= 1.0;
    return c.a() == 3.0 && c.omega() == 0.5 && c[1] == 0.5 && c - c == Control<double>{} &&
           Control<double>{0.0} == Control<double>{};
}
static_assert(constantControl());

template <std::floating_point T>
[[nodiscard]] auto boundedModel() -> KinematicsModel<T> {
    return KinematicsModel<T>{
        kFrontWheelbase<T>, kRearWheelbase<T>, Control<T>{kALo<T>, kOmegaLo<T>},
        Control<T>{kAUp<T>, kOmegaUp<T>}
    };
}

template <std::floating_point T>
[[nodiscard]] auto unboundedModel() -> KinematicsModel<T> {
    return KinematicsModel<T>{kFrontWheelbase<T>, kRearWheelbase<T>};
}

/**
 * @brief A vehicle at the origin, heading kInitialYaw, with the given steering angle and speed.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto startAt(T delta, T u) noexcept -> State<T> {
    return State<T>{static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, u, delta};
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
 * @brief Largest per field difference between two states or two controls.
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
 * double holds every error to 1e-6. float rounds the heading, which winds up past 20 rad, by up to
 * 1e-6 rad a step, which builds up to errors of up to 5e-4 rad in the heading and 8e-3 m in the
 * position over a run of kDuration whatever the integrator does, so float is held to 1e-2 relative
 * to the scale of the quantity: its largest magnitude along the run, though at least one.
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
[[nodiscard]] auto constantSchedule(std::size_t num_steps, T h, T accel, T omega) -> Schedule<T> {
    return Schedule<T>{
        std::vector<T>(num_steps, h), std::vector<Control<T>>(num_steps, Control<T>{accel, omega})
    };
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
 * speed is the signed magnitude of the velocity of the center of gravity, what speedOf() answers,
 * rather than the longitudinal component u the acceleration control acts on: the closed form
 * follows the path the center of gravity traces. The steering angle stays put along it.
 */
template <std::floating_point T>
struct Sample final {
    Vec2<T> center;
    T yaw;
    T speed;
    T delta;
};

/**
 * @brief Side slip angle written as beta = atan(l_r * tan(delta) / (l_f + l_r)).
 *
 * Deliberately spelled out from the literature rather than derived from the body velocity, so that
 * the reference solution stays independent of the implementation under test.
 */
template <std::floating_point T>
[[nodiscard]] auto slipAngleOf(T delta) noexcept -> T {
    return std::atan(kRearWheelbase<T> * std::tan(delta) / kWheelbase<T>);
}

/**
 * @brief Path curvature written as kappa = sin(beta) / l_r.
 *
 * This is the d(yaw)/dt = speed * sin(beta) / l_r form given by Kong et al., which is an algebraic
 * rearrangement of the cos(beta) * tan(delta) / wheelbase form the implementation uses.
 */
template <std::floating_point T>
[[nodiscard]] auto curvatureOf(T delta) noexcept -> T {
    return std::sin(slipAngleOf(delta)) / kRearWheelbase<T>;
}

/**
 * @brief Yaw rate and lateral velocity the kinematic constraint ties to u and delta.
 *
 * The state carries neither, so the assertions below rebuild them as r = u * tan(delta) / L and
 * v = l_r * r.
 */
template <std::floating_point T>
[[nodiscard]] auto yawRateOf(const State<T>& state) noexcept -> T {
    return state.u() * std::tan(state.delta()) / kWheelbase<T>;
}

template <std::floating_point T>
[[nodiscard]] auto lateralVelocityOf(const State<T>& state) noexcept -> T {
    return kRearWheelbase<T> * yawRateOf(state);
}

/**
 * @brief Signed magnitude of the body frame velocity, signed by the longitudinal component so that
 * reversing keeps it negative.
 */
template <std::floating_point T>
[[nodiscard]] auto speedOf(const State<T>& state) noexcept -> T {
    return std::copysign(std::hypot(state.u(), lateralVelocityOf(state)), state.u());
}

/**
 * @brief Gathers the flat position and velocity fields of a state back into a Vec2.
 *
 * State stores its scalars flat and keeps its velocity in the body frame, while the reference
 * solution below is written in terms of planar vectors in the world frame, so these two keep the
 * assertions reading the way the geometry does.
 */
template <std::floating_point T>
[[nodiscard]] auto centerOf(const State<T>& state) noexcept -> Vec2<T> {
    return Vec2<T>{state.x(), state.y()};
}

template <std::floating_point T>
[[nodiscard]] auto worldVelocityOf(const State<T>& state) noexcept -> Vec2<T> {
    const T v{lateralVelocityOf(state)};
    return Vec2<T>{
        state.u() * std::cos(state.yaw()) - v * std::sin(state.yaw()),
        state.u() * std::sin(state.yaw()) + v * std::cos(state.yaw())
    };
}

/**
 * @brief The sample of the closed form a state starts it from, and the state a sample stands for.
 */
template <std::floating_point T>
[[nodiscard]] auto sampleOf(const State<T>& state) noexcept -> Sample<T> {
    return Sample<T>{centerOf(state), state.yaw(), speedOf(state), state.delta()};
}

template <std::floating_point T>
[[nodiscard]] auto stateOf(const Sample<T>& sample) noexcept -> State<T> {
    return State<T>{
        sample.center.x(), sample.center.y(), sample.yaw,
        sample.speed * std::cos(slipAngleOf(sample.delta)), sample.delta
    };
}

/**
 * @brief Closed form solution under a constant acceleration and the constant steering angle of
 * the initial sample.
 *
 * Only valid while omega is zero: a constant steering angle gives a constant path curvature that
 * does not depend on the speed, so the center of gravity always traces a circular arc of radius
 * 1 / kappa and the acceleration only changes how fast that arc is traversed. Note that accel is
 * the derivative of the longitudinal component u rather than of the total speed, and the two
 * differ by the constant factor cos(beta).
 */
template <std::floating_point T>
[[nodiscard]] auto analyticAt(const Sample<T>& initial, T accel, T t) noexcept -> Sample<T> {
    const T beta{slipAngleOf(initial.delta)};
    const T kappa{curvatureOf(initial.delta)};
    const T speed_rate{accel / std::cos(beta)};
    const T arc_length{initial.speed * t + static_cast<T>(0.5) * speed_rate * t * t};
    const T course{initial.yaw + beta};
    const T speed{initial.speed + speed_rate * t};
    if (kappa == static_cast<T>(0.0)) {
        return Sample<T>{
            {initial.center.x() + arc_length * std::cos(course),
             initial.center.y() + arc_length * std::sin(course)},
            initial.yaw,
            speed,
            initial.delta
        };
    }
    return Sample<T>{
        {initial.center.x() + (std::sin(course + kappa * arc_length) - std::sin(course)) / kappa,
         initial.center.y() - (std::cos(course + kappa * arc_length) - std::cos(course)) / kappa},
        initial.yaw + kappa * arc_length,
        speed,
        initial.delta
    };
}

template <std::floating_point T>
[[nodiscard]] auto positionErrorOf(const State<T>& state, const Sample<T>& reference) noexcept
    -> T {
    return std::hypot(state.x() - reference.center.x(), state.y() - reference.center.y());
}

template <std::floating_point T>
[[nodiscard]] auto yawErrorOf(const State<T>& state, const Sample<T>& reference) noexcept -> T {
    return std::abs(state.yaw() - reference.yaw);
}

template <std::floating_point T>
[[nodiscard]] auto speedErrorOf(const State<T>& state, const Sample<T>& reference) noexcept -> T {
    return std::abs(speedOf(state) - reference.speed);
}

template <std::floating_point T>
[[nodiscard]] auto stepWith(
    IntegrateMode mode, const KinematicsModel<T>& model, const State<T>& state,
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
    IntegrateMode mode, const KinematicsModel<T>& model, const State<T>& initial_state,
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
    const KinematicsModel<T>& model, const State<T>& initial_state, const Schedule<T>& schedule
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
 * @brief Largest position, yaw and speed errors along a trace, in the order of kErrorNames.
 */
template <std::floating_point T>
[[nodiscard]] auto maxErrorsOf(
    const std::vector<State<T>>& trace, const std::vector<Sample<T>>& reference
) -> std::array<T, kErrorNames.size()> {
    std::array<T, kErrorNames.size()> result{};
    for (std::size_t i{0}; i < trace.size(); ++i) {
        raiseTo(result[0], positionErrorOf(trace[i], reference[i]));
        raiseTo(result[1], yawErrorOf(trace[i], reference[i]));
        raiseTo(result[2], speedErrorOf(trace[i], reference[i]));
    }
    return result;
}

/**
 * @brief Asserts that an error an integrator makes at the step h, and again at h / 2, shrinks the
 * way its order says it should.
 *
 * An error within accuracyFloor() of the scale of its quantity only has to stay there: either the
 * integrator is exact up to roundoff, such as forwardEuler() on the yaw at a constant speed, whose
 * rate it holds exactly, or the error is the rounding of float rather than the integrator's. Above
 * the floor forwardEuler(), being first order, has to at least halve its error along with the
 * step, a ratio above 1.5, and the fourth order rungeKutta() has to cut it sixteen fold, a ratio
 * above 12. In float rungeKutta() is held to the floor alone: its error is rounding through and
 * through, which a ratio says nothing about.
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
 * @brief Asserts that every integrator drives the steering angle and the longitudinal velocity as
 * the controls say.
 *
 * The two controls are the rates of delta and u and are held constant over a step, so every
 * integrator has to bring both back where accumulating the piecewise constant controls by hand
 * puts them, up to roundoff. This is what carries the scenarios that have no closed form solution.
 */
template <std::floating_point T>
auto checkInvariants(
    const Trajectory<T>& trajectory, const State<T>& initial_state, const Schedule<T>& schedule
) -> void {
    const std::size_t size{schedule.hs.size()};
    std::vector<T> us{initial_state.u()};
    std::vector<T> deltas{initial_state.delta()};
    us.reserve(size + 1);
    deltas.reserve(size + 1);
    T u_scale{std::abs(initial_state.u())};
    T delta_scale{std::abs(initial_state.delta())};
    for (std::size_t i{0}; i < size; ++i) {
        us.push_back(us.back() + schedule.controls[i].a() * schedule.hs[i]);
        deltas.push_back(deltas.back() + schedule.controls[i].omega() * schedule.hs[i]);
        raiseTo(u_scale, std::abs(us.back()));
        raiseTo(delta_scale, std::abs(deltas.back()));
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        const std::vector<State<T>>& trace{trajectory.trace(mode)};
        T max_u_error{static_cast<T>(0.0)};
        T max_delta_error{static_cast<T>(0.0)};
        for (std::size_t i{0}; i <= size; ++i) {
            raiseTo(max_u_error, std::abs(trace[i].u() - us[i]));
            raiseTo(max_delta_error, std::abs(trace[i].delta() - deltas[i]));
        }
        CHECK_LE(max_u_error, accuracyFloor(u_scale));
        CHECK_LE(max_delta_error, accuracyFloor(delta_scale));
    }
}

/**
 * @brief Runs a scenario with a closed form solution at the step h and at h / 2, and holds every
 * integrator to that solution.
 *
 * Every error of every integrator goes through checkErrorShrinks(), and both runs through
 * checkInvariants(). Returns the run at the step h with the closed form sampled alongside, for the
 * checks particular to the scenario and the plot.
 */
template <std::floating_point T>
auto checkClosedForm(
    const KinematicsModel<T>& model, const State<T>& initial_state, const Schedule<T>& schedule,
    const auto& reference_at
) -> Trajectory<T> {
    const Schedule<T> halved{halvedSteps(schedule)};
    Trajectory<T> coarse{simulate(model, initial_state, schedule)};
    Trajectory<T> fine{simulate(model, initial_state, halved)};
    fillReference(coarse, reference_at);
    fillReference(fine, reference_at);
    // The scale of every error is the largest magnitude of its quantity along the closed form; the
    // yaw winds up without wrapping, and its rounding grows along with it.
    std::array<T, kErrorNames.size()> scales{};
    for (const Sample<T>& sample : coarse.reference) {
        raiseTo(scales[0], sample.center.euclidean());
        raiseTo(scales[1], std::abs(sample.yaw));
        raiseTo(scales[2], std::abs(sample.speed));
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
    checkInvariants(coarse, initial_state, schedule);
    checkInvariants(fine, initial_state, halved);
    return coarse;
}

/**
 * @brief Estimates the observed order of convergence of the integrators.
 *
 * Halving the step three times and comparing consecutive results estimates the order without
 * needing a solution to compare against, which is what the scenarios with a non zero steering rate
 * require: their trajectories are clothoid-like and have no elementary closed form. Fourth order
 * cuts the step error sixteen fold, first order only halves it, so running forwardEuler() through
 * the same estimator as rungeKutta() both pins down the claim of rungeKutta() and proves the
 * estimator itself measures what it says it does. The coarse step is several times that of the
 * scenario: 300 steps over kDuration put the ratio at 16.0 for rungeKutta() and at 1.97 to 1.98
 * for forwardEuler(), while at the step of the scenario itself the error of rungeKutta() over the
 * finer steps is down at the roundoff of double, which no longer shrinks with the step, and its
 * ratio falls to 8.8 and below.
 *
 * The control schedules are addressed by the fraction of the run rather than by the elapsed time
 * so that a piecewise constant schedule switches on exactly the same step boundary at every
 * resolution. A discontinuity landing inside a step at one resolution and on its boundary at
 * another would degrade the order for reasons that have nothing to do with the integrator.
 */
template <std::floating_point T>
auto checkConvergenceOrder(
    const KinematicsModel<T>& model, const State<T>& initial_state, T duration,
    std::size_t base_steps, const auto& accel_at, const auto& omega_at
) -> void {
    const auto final_state = [&](IntegrateMode mode, std::size_t steps) -> State<T> {
        Schedule<T> schedule{constantSchedule(
            steps, duration / static_cast<T>(steps), static_cast<T>(0.0), static_cast<T>(0.0)
        )};
        for (std::size_t i{0}; i < steps; ++i) {
            const T fraction{static_cast<T>(i) / static_cast<T>(steps)};
            schedule.controls[i] = Control<T>{accel_at(fraction), omega_at(fraction)};
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
 * @brief Plots the path of every integrator in the plane, each field of its state and its yaw rate
 * against time, over the closed form solution where the scenario has one, with the decimal
 * logarithm of the position and yaw errors against that solution; the error subplots of a scenario
 * without one stay empty. Every curve keeps its color and line style across the nine subplots, so
 * that one legend across the top of the figure covers them all.
 *
 * Both integrators take u and delta exactly, so their curves of u, delta and the yaw rate lie on
 * top of each other and on the closed form, and the speed error, which is roundoff, goes unplotted.
 * The errors go through log10() rather than onto a log axis because matplot hands its data to
 * gnuplot in fixed point with ten decimals, which rounds the errors of Runge-Kutta, down to 1e-13,
 * to zero, and a log axis drops zeros.
 */
auto plotTrajectory(const std::string& title, const Trajectory<double>& trajectory) -> void {
    // The subplot of every field of the state and of the yaw rate, as createAxesHandles() lays them
    // out.
    constexpr std::array<std::size_t, State<double>::size()> kFieldAxes{3, 4, 5, 6, 7};
    constexpr std::size_t kYawRateAxis{8};
    // The subplot of the position and the yaw errors, the first two of kErrorNames.
    constexpr std::array<std::size_t, 2> kErrorAxes{1, 2};
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
        std::vector<double> yaw_rates;
        for (const State<double>& state : states) {
            for (std::size_t f{0}; f < State<double>::size(); ++f) {
                fields[f].push_back(state[f]);
            }
            yaw_rates.push_back(yawRateOf(state));
        }
        axes[0]->plot(fields[0], fields[1], line_spec)->line_width(2.0).color(color);
        for (std::size_t f{0}; f < State<double>::size(); ++f) {
            axes[kFieldAxes[f]]
                ->plot(trajectory.ts, fields[f], line_spec)
                ->line_width(2.0)
                .color(color);
        }
        axes[kYawRateAxis]->plot(trajectory.ts, yaw_rates, line_spec)->line_width(2.0).color(color);
    };

    if (has_reference) {
        std::vector<State<double>> reference_states;
        reference_states.reserve(trajectory.reference.size());
        for (const Sample<double>& sample : trajectory.reference) {
            reference_states.push_back(stateOf(sample));
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
        std::array<std::vector<double>, kErrorAxes.size()> errors;
        for (std::size_t i{0}; i < trace.size(); ++i) {
            errors[0].push_back(positionErrorOf(trace[i], trajectory.reference[i]));
            errors[1].push_back(yawErrorOf(trace[i], trajectory.reference[i]));
        }
        for (std::size_t e{0}; e < errors.size(); ++e) {
            // An error of exactly zero has no logarithm and is left out as a NaN, which gnuplot
            // skips; an error that is zero all along, such as that of a yaw an integrator keeps
            // exactly, leaves nothing to plot at all.
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
    axes[0]->axis(matplot::equal);

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

// Scenarios with a zero steering rate keep a constant steering angle, which puts the center of
// gravity on an arc of constant curvature and leaves a closed form solution to compare against.

TEST_CASE_TEMPLATE("StraightLineAccelerateThenDecelerate", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(5.0)};
    constexpr T kAccel{static_cast<T>(1.2)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};
    const T half{kDuration<T> * static_cast<T>(0.5)};

    // Accelerating over the first half of the run and decelerating as hard over the second takes
    // the speed from 5 up to 17 m / s and back.
    const State<T> initial_state{startAt(kDelta, kInitialU)};
    Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};
    std::fill(
        schedule.controls.begin() + kNumSteps / 2, schedule.controls.end(),
        Control<T>{-kAccel, static_cast<T>(0.0)}
    );
    const Sample<T> start{sampleOf(initial_state)};
    const Sample<T> midpoint{analyticAt(start, kAccel, half)};
    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            return t <= half ? analyticAt(start, kAccel, t)
                             : analyticAt(midpoint, -kAccel, t - half);
        })
    };

    // A zero steering angle leaves no yaw rate and no lateral velocity, so every integrator has to
    // keep the heading and the vehicle on its axis exactly.
    for (const IntegrateMode mode : kIntegrateModes) {
        CAPTURE(mode);
        for (const State<T>& state : trajectory.trace(mode)) {
            CHECK_EQ(state.yaw(), kInitialYaw<T>);
            CHECK_EQ(state.delta(), static_cast<T>(0.0));
            CHECK_EQ(lateralVelocityOf(state), static_cast<T>(0.0));
        }
    }
    // The speed comes back to where it started after the symmetric acceleration profile.
    const T top_speed{kInitialU + kAccel * half};
    CHECK_LE(
        std::abs(speedOf(trajectory.trace(IntegrateMode::RUNGE_KUTTA).back()) - kInitialU),
        accuracyFloor(top_speed)
    );

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("straight line accelerate then decelerate", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("ConstantSpeedCircle", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kDelta{static_cast<T>(0.15)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    const T kappa{curvatureOf(kDelta)};
    const T radius{static_cast<T>(1.0) / kappa};
    // A constant steering angle puts the center of gravity on a circle whose radius follows from
    // the geometry alone: R = hypot(l_r, wheelbase / tan(delta)).
    CHECK_EQ(
        radius, doctest::Approx(std::hypot(kRearWheelbase<T>, kWheelbase<T> / std::tan(kDelta)))
                    .epsilon(roundoffTolerance<T>())
    );

    // The speed that takes the vehicle round the circle twice over kDuration, about 10.8 m / s.
    const T speed{static_cast<T>(4.0) * std::numbers::pi_v<T> / (kappa * kDuration<T>)};
    const State<T> initial_state{startAt(kDelta, speed * std::cos(slipAngleOf(kDelta)))};
    const Schedule<T> schedule{
        constantSchedule(kNumSteps, h, static_cast<T>(0.0), static_cast<T>(0.0))
    };
    const Sample<T> start{sampleOf(initial_state)};
    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            return analyticAt(start, static_cast<T>(0.0), t);
        })
    };

    // Every sample has to sit on the circle centered at the instantaneous center of rotation, at
    // the speed and the yaw rate the vehicle starts with.
    const T course{kInitialYaw<T> + slipAngleOf(kDelta)};
    const Vec2<T> rotation_center{
        start.center.x() - radius * std::sin(course), start.center.y() + radius * std::cos(course)
    };
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    for (const State<T>& state : runge_kutta) {
        CHECK_LE(
            std::abs(centerOf(state).euclideanTo(rotation_center) - radius), accuracyFloor(radius)
        );
        CHECK_LE(std::abs(worldVelocityOf(state).euclidean() - speed), accuracyFloor(speed));
        CHECK_LE(std::abs(yawRateOf(state) - kappa * speed), accuracyFloor(kappa * speed));
    }
    // Two full laps bring the vehicle back to where it started, heading two turns further on.
    const T final_yaw{kInitialYaw<T> + static_cast<T>(4.0) * std::numbers::pi_v<T>};
    CHECK_LE(centerOf(runge_kutta.back()).euclideanTo(start.center), accuracyFloor(radius));
    CHECK_LE(std::abs(runge_kutta.back().yaw() - final_yaw), accuracyFloor(final_yaw));

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("constant speed circle", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("AcceleratingTurn", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kDelta{static_cast<T>(0.15)};
    constexpr T kInitialU{static_cast<T>(8.0)};
    constexpr T kAccel{static_cast<T>(0.9)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    // The speed grows from 8 to 26 m / s along the circle the constant steering angle fixes, so
    // the vehicle laps it faster and faster.
    const State<T> initial_state{startAt(kDelta, kInitialU)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};
    const Sample<T> start{sampleOf(initial_state)};
    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            return analyticAt(start, kAccel, t);
        })
    };

    // Accelerating through a turn is where the acceleration has to stay along the body axis rather
    // than along the course angle: resolving it along the course instead leaves the velocity
    // drifting off the slip angle the steering geometry dictates, an error that neither the
    // straight line nor the constant speed circle scenario can expose.
    const T beta{slipAngleOf(kDelta)};
    const T kappa{curvatureOf(kDelta)};
    const T top_speed{start.speed + kAccel / std::cos(beta) * kDuration<T>};
    const T top_yaw{trajectory.reference.back().yaw};
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const State<T>& state{runge_kutta[i]};
        const T speed{start.speed + kAccel / std::cos(beta) * trajectory.ts[i]};
        CHECK_LE(std::abs(worldVelocityOf(state).euclidean() - speed), accuracyFloor(top_speed));
        CHECK_LE(
            std::abs(std::atan2(lateralVelocityOf(state), state.u()) - beta), accuracyFloor(beta)
        );
        // state.yaw accumulates without wrapping (see ConstantSpeedCircle, which drives it past a
        // full turn on purpose), while atan2() always answers in (-pi, pi], so the two only agree
        // modulo 2 * pi once the course angle winds past that range over the run.
        const T course_angle{state.yaw() + beta};
        const Vec2<T> velocity{worldVelocityOf(state)};
        const T velocity_angle{std::atan2(velocity.y(), velocity.x())};
        CHECK_LE(
            std::abs(
                std::remainder(
                    velocity_angle - course_angle, static_cast<T>(2.0) * std::numbers::pi_v<T>
                )
            ),
            accuracyFloor(top_yaw)
        );
        CHECK_LE(std::abs(yawRateOf(state) - kappa * speed), accuracyFloor(kappa * top_speed));
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("accelerating turn", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("DecelerateThroughZeroIntoReverse", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kDelta{static_cast<T>(0.15)};
    constexpr T kInitialU{static_cast<T>(12.0)};
    constexpr T kAccel{static_cast<T>(-1.2)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    const State<T> initial_state{startAt(kDelta, kInitialU)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};
    const Sample<T> start{sampleOf(initial_state)};
    const Trajectory<T> trajectory{
        checkClosedForm(model, initial_state, schedule, [&](T t) -> Sample<T> {
            return analyticAt(start, kAccel, t);
        })
    };

    // The speed runs from +12 down through zero, halfway through the run, into reverse at -12, so
    // the run crosses the one point where the velocity vector carries no direction at all. Taking
    // the sign from the longitudinal component is what lets the trajectory continue into reverse
    // instead of bouncing back: the magnitude alone leaves the vehicle turning the wrong way once
    // it reverses.
    const T beta{slipAngleOf(kDelta)};
    const T kappa{curvatureOf(kDelta)};
    const T top_speed{std::abs(start.speed)};
    T top_yaw{static_cast<T>(0.0)};
    for (const Sample<T>& sample : trajectory.reference) {
        raiseTo(top_yaw, std::abs(sample.yaw));
    }
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    bool saw_forward{false};
    bool saw_reverse{false};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const State<T>& state{runge_kutta[i]};
        const T speed{start.speed + kAccel / std::cos(beta) * trajectory.ts[i]};
        CHECK_LE(std::abs(speedOf(state) - speed), accuracyFloor(top_speed));
        CHECK_LE(std::abs(yawRateOf(state) - kappa * speed), accuracyFloor(kappa * top_speed));
        // The magnitude stays non-negative throughout and therefore cannot tell the two halves
        // of the run apart on its own.
        CHECK_LE(
            std::abs(worldVelocityOf(state).euclidean() - std::abs(speed)), accuracyFloor(top_speed)
        );
        // state.yaw accumulates without wrapping, while atan2() always answers in (-pi, pi], so
        // the two only agree modulo 2 * pi once the course angle winds past that range over the
        // run (see the matching comment in AcceleratingTurn).
        const T course_angle{state.yaw() + beta};
        const Vec2<T> velocity{worldVelocityOf(state)};
        if (speed > static_cast<T>(1.0)) {
            saw_forward = true;
            const T velocity_angle{std::atan2(velocity.y(), velocity.x())};
            CHECK_LE(
                std::abs(
                    std::remainder(
                        velocity_angle - course_angle, static_cast<T>(2.0) * std::numbers::pi_v<T>
                    )
                ),
                accuracyFloor(top_yaw)
            );
        } else if (speed < static_cast<T>(-1.0)) {
            saw_reverse = true;
            // Reversing points the velocity the other way round while the heading is unchanged.
            const T reverse_velocity_angle{std::atan2(-velocity.y(), -velocity.x())};
            CHECK_LE(
                std::abs(
                    std::remainder(
                        reverse_velocity_angle - course_angle,
                        static_cast<T>(2.0) * std::numbers::pi_v<T>
                    )
                ),
                accuracyFloor(top_yaw)
            );
            // A left steering angle turns the vehicle clockwise once it drives backwards.
            CHECK_LT(yawRateOf(state), static_cast<T>(0.0));
        } else {
            // Around the reversal the velocity vector shrinks to zero and carries no direction
            // worth asserting on, so only the signed speed above is checked in that band.
        }
    }
    CHECK(saw_forward);
    CHECK(saw_reverse);
    CHECK_LT(speedOf(runge_kutta.back()), static_cast<T>(0.0));

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("decelerate through zero into reverse", trajectory);
        }
    }
}

// Scenarios with a non zero steering rate sweep the steering angle, which turns the path into a
// clothoid-like curve with no elementary closed form. They lean on the algebraic invariants and on
// the observed order of convergence instead. With no solution to compare against, their plots
// leave the errors out.

TEST_CASE_TEMPLATE("ConstantSteeringRateClothoid", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kInitialDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(8.0)};
    constexpr T kAccel{static_cast<T>(0.0)};
    constexpr T kOmega{static_cast<T>(0.012)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    const State<T> initial_state{startAt(kInitialDelta, kInitialU)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, kOmega)};
    const Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    checkInvariants(trajectory, initial_state, schedule);

    // The steering angle ramps linearly up to 0.24 rad, so the curvature grows with it and the
    // vehicle spirals inwards: the yaw rate has to keep increasing.
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    for (std::size_t i{1}; i < runge_kutta.size(); ++i) {
        CHECK_GT(yawRateOf(runge_kutta[i]), yawRateOf(runge_kutta[i - 1]));
    }

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder(
            model, initial_state, kDuration<T>, 300, [](T) -> T { return kAccel; },
            [](T) -> T { return kOmega; }
        );
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("constant steering rate clothoid", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("AcceleratingSteeringReversal", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kInitialDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(6.0)};
    constexpr T kAccel{static_cast<T>(0.48)};
    constexpr T kOmega{static_cast<T>(0.048)};
    constexpr std::size_t kNumSteps{2000};
    const T h{kDuration<T> / static_cast<T>(kNumSteps)};

    // Steering out to 0.24 rad, back through the straight ahead position and out the other way,
    // all while accelerating from 6 to 15.6 m / s. This is the only scenario where both terms of
    // the lateral acceleration are alive at once: accel * tan(delta) vanishes wherever the steering
    // angle is zero, and u * omega / cos(delta)^2 vanishes wherever the steering rate is, so a
    // model that dropped either one still passes every scenario above.
    const auto omega_at = [](T fraction) -> T {
        return (fraction < static_cast<T>(0.25) || fraction >= static_cast<T>(0.75)) ? kOmega
                                                                                     : -kOmega;
    };
    Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};
    for (std::size_t i{0}; i < kNumSteps; ++i) {
        schedule.controls[i].omega() = omega_at(static_cast<T>(i) / static_cast<T>(kNumSteps));
    }

    const State<T> initial_state{startAt(kInitialDelta, kInitialU)};
    const Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    checkInvariants(trajectory, initial_state, schedule);

    // The steering angle has to come back to where it started, having visited both sides on the
    // way, and the lateral velocity has to change sign with it.
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    T max_delta{static_cast<T>(0.0)};
    T min_delta{static_cast<T>(0.0)};
    for (const State<T>& state : runge_kutta) {
        max_delta = std::max(max_delta, state.delta());
        min_delta = std::min(min_delta, state.delta());
        // Driving forwards, the lateral velocity leans to whichever side the steering angle does.
        if (std::abs(state.delta()) > static_cast<T>(0.01)) {
            CHECK_EQ(
                lateralVelocityOf(state) > static_cast<T>(0.0), state.delta() > static_cast<T>(0.0)
            );
        }
    }
    CHECK_GT(max_delta, static_cast<T>(0.1));
    CHECK_LT(min_delta, static_cast<T>(-0.1));
    CHECK_LE(std::abs(runge_kutta.back().delta()), accuracyFloor(max_delta));

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder(
            model, initial_state, kDuration<T>, 300, [](T) -> T { return kAccel; }, omega_at
        );
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("accelerating steering reversal", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("EvolveMatchesMarchingSingleSteps", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    const T tolerance{roundoffTolerance<T>()};

    SUBCASE("AlongASchedule") {
        const State<T> initial_state{startAt(static_cast<T>(0.15), static_cast<T>(8.0))};
        // No two entries agree within a span, so an evolve() that read the wrong index cannot
        // pass.
        const Schedule<T> schedule{
            {static_cast<T>(0.010), static_cast<T>(0.012), static_cast<T>(0.008),
             static_cast<T>(0.011), static_cast<T>(0.009)},
            {Control<T>{static_cast<T>(1.2), static_cast<T>(-0.05)},
             Control<T>{static_cast<T>(-1.2), static_cast<T>(0.0)},
             Control<T>{static_cast<T>(0.4), static_cast<T>(0.07)},
             Control<T>{static_cast<T>(0.9), static_cast<T>(0.05)},
             Control<T>{static_cast<T>(0.0), static_cast<T>(0.02)}}
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
        const State<T> initial_state{
            static_cast<T>(1.0), static_cast<T>(2.0), kInitialYaw<T>, static_cast<T>(8.0),
            static_cast<T>(0.15)
        };
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
    const KinematicsModel<T> model{unboundedModel<T>()};
    const State<T> state{
        static_cast<T>(1.0), static_cast<T>(-2.0), kInitialYaw<T>, static_cast<T>(8.0),
        static_cast<T>(0.15)
    };
    const Control<T> control{static_cast<T>(-0.6), static_cast<T>(0.03)};
    const T tolerance{roundoffTolerance<T>()};

    SUBCASE("EveryFieldMatchesTheKinematicEquations") {
        // Written out from the slip angle the literature puts the model in rather than from the
        // body velocity tangent() is made of: the center of gravity moves along the course
        // yaw + beta at the speed u / cos(beta) and turns at speed * sin(beta) / l_r, while the
        // two controls are the rates of u and delta.
        const State<T> k{model.tangent(state, control)};
        const T beta{slipAngleOf(state.delta())};
        const T speed{state.u() / std::cos(beta)};
        const Vec2<T> velocity{
            speed * std::cos(state.yaw() + beta), speed * std::sin(state.yaw() + beta)
        };
        CHECK_LE(Vec2<T>{k.x(), k.y()}.euclideanTo(velocity), tolerance);
        CHECK_LE(std::abs(k.yaw() - speed * std::sin(beta) / kRearWheelbase<T>), tolerance);
        CHECK_EQ(k.u(), control.a());
        CHECK_EQ(k.delta(), control.omega());
    }

    SUBCASE("StandingStillIsAnEquilibrium") {
        // At rest with both controls at zero, whichever way the vehicle faces and the wheels point,
        // nothing moves: the tangent vanishes and every integrator leaves the state where it is.
        const State<T> standing{
            static_cast<T>(1.0), static_cast<T>(-2.0), kInitialYaw<T>, static_cast<T>(0.0),
            static_cast<T>(0.15)
        };
        const Control<T> idle{static_cast<T>(0.0), static_cast<T>(0.0)};
        CHECK_LE(
            distanceOf(model.tangent(standing, idle), State<T>{static_cast<T>(0.0)}), tolerance
        );
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_LE(distanceOf(stepWith(mode, model, standing, idle, kH<T>), standing), tolerance);
        }
    }
}

// The two controls are the one place the model says no: everything else it takes at face value,
// while a and omega are clamped into the bounds it was built with.

TEST_CASE_TEMPLATE("ControlBoundsClampEveryEntryPoint", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const KinematicsModel<T> unbounded{unboundedModel<T>()};
    const State<T> start{startAt(static_cast<T>(0.15), static_cast<T>(8.0))};

    SUBCASE("BoundsComeBackOutAndDefaultToInfinite") {
        CHECK_EQ(model.control_lower_bound(), Control<T>{kALo<T>, kOmegaLo<T>});
        CHECK_EQ(model.control_upper_bound(), Control<T>{kAUp<T>, kOmegaUp<T>});
        CHECK_EQ(unbounded.control_lower_bound(), Control<T>{-std::numeric_limits<T>::infinity()});
        CHECK_EQ(unbounded.control_upper_bound(), Control<T>{std::numeric_limits<T>::infinity()});
    }

    SUBCASE("EveryEntryPointClamps") {
        // Controls beyond the bounds have to act exactly as the bounds they are clamped to.
        constexpr T kLooseAccel{static_cast<T>(50.0)};
        constexpr T kLooseOmega{static_cast<T>(-7.0)};
        const Control<T> loose{kLooseAccel, kLooseOmega};
        const Control<T> clamped{kAUp<T>, kOmegaLo<T>};
        const State<T> k{model.tangent(start, loose)};
        CHECK_EQ(k.u(), kAUp<T>);
        CHECK_EQ(k.delta(), kOmegaLo<T>);
        CHECK_EQ(k, model.tangent(start, clamped));
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
        Schedule<T> schedule{
            constantSchedule(kNumSteps, kH<T>, static_cast<T>(0.0), static_cast<T>(0.0))
        };
        Schedule<T> clamped{schedule};
        for (std::size_t i{0}; i < kNumSteps; ++i) {
            const T t{static_cast<T>(i) * kH<T>};
            schedule.controls[i] =
                Control<T>{static_cast<T>(8.0) * std::sin(t), static_cast<T>(1.2) * std::cos(t)};
            clamped.controls[i] = Control<T>{
                std::clamp(schedule.controls[i].a(), kALo<T>, kAUp<T>),
                std::clamp(schedule.controls[i].omega(), kOmegaLo<T>, kOmegaUp<T>)
            };
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
        const KinematicsModel<T> wide{
            kFrontWheelbase<T>, kRearWheelbase<T>, Control<T>{static_cast<T>(-1.0e3)},
            Control<T>{static_cast<T>(1.0e3)}
        };
        const Schedule<T> schedule{
            constantSchedule(200, kH<T>, static_cast<T>(1.5), static_cast<T>(0.05))
        };
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

TEST_CASE_TEMPLATE("StateArithmetic", T, float, double) {
    // Binary fractions keep every expected result exact for float as well as for double, and no
    // two fields agree, so an operator that mixed two of them up cannot hide behind a coincidence.
    const State<T> lhs{
        static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.75), static_cast<T>(7.5),
        static_cast<T>(0.125)
    };
    const State<T> rhs{
        static_cast<T>(0.5), static_cast<T>(1.5), static_cast<T>(-0.25), static_cast<T>(2.5),
        static_cast<T>(-0.75)
    };

    SUBCASE("LayoutCoversEveryField") {
        static_assert(State<T>::size() == 5);
        CHECK_EQ(sizeof(State<T>), State<T>::size() * sizeof(T));
        CHECK_EQ(lhs.data(), &lhs.x());
        CHECK_EQ(&lhs.delta() - lhs.data(), static_cast<std::ptrdiff_t>(State<T>::size()) - 1);
        CHECK_EQ(State<T>{}, State<T>{T{}, T{}, T{}, T{}, T{}});
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
        CHECK_EQ(
            lhs + rhs, State<T>{
                           static_cast<T>(2.0), static_cast<T>(-1.0), static_cast<T>(0.5),
                           static_cast<T>(10.0), static_cast<T>(-0.625)
                       }
        );
        CHECK_EQ(
            lhs - rhs, State<T>{
                           static_cast<T>(1.0), static_cast<T>(-4.0), static_cast<T>(1.0),
                           static_cast<T>(5.0), static_cast<T>(0.875)
                       }
        );
        const State<T> scaled{
            static_cast<T>(3.0), static_cast<T>(-5.0), static_cast<T>(1.5), static_cast<T>(15.0),
            static_cast<T>(0.25)
        };
        CHECK_EQ(lhs * static_cast<T>(2.0), scaled);
        CHECK_EQ(static_cast<T>(2.0) * lhs, scaled);
        CHECK_EQ(scaled / static_cast<T>(2.0), lhs);
        CHECK_EQ(
            -lhs, State<T>{
                      static_cast<T>(-1.5), static_cast<T>(2.5), static_cast<T>(-0.75),
                      static_cast<T>(-7.5), static_cast<T>(-0.125)
                  }
        );
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
    const Control<T> lhs{static_cast<T>(0.5), static_cast<T>(-0.25)};
    const Control<T> rhs{static_cast<T>(-1.5), static_cast<T>(0.75)};

    SUBCASE("LayoutCoversEveryField") {
        static_assert(Control<T>::size() == 2);
        CHECK_EQ(sizeof(Control<T>), Control<T>::size() * sizeof(T));
        const std::array<const T*, 2> fields{&lhs.a(), &lhs.omega()};
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
          State<T>{
              static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.3), static_cast<T>(7.9),
              static_cast<T>(0.15)
          },
          State<T>{
              static_cast<T>(-3.2), static_cast<T>(4.1), static_cast<T>(-1.1), static_cast<T>(-6.0),
              static_cast<T>(-0.1)
          }}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(state).code == std::errc{});
        State<T> restored{};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, state);
    }
}

TEST_CASE_TEMPLATE("ControlSerializationRoundtrip", T, float, double) {
    for (const Control<T>& control :
         {Control<T>{}, Control<T>{kALo<T>, kOmegaLo<T>}, Control<T>{kAUp<T>, kOmegaUp<T>},
          Control<T>{std::numeric_limits<T>::infinity()}}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(control).code == std::errc{});
        Control<T> restored{static_cast<T>(9.0)};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, control);
    }
}

/**
 * @brief Interpolates a trajectory of states or controls through every piecewise function.
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
    checkInterpolation(
        State<T>{
            static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.3), static_cast<T>(7.5),
            static_cast<T>(0.15)
        },
        State<T>{
            static_cast<T>(7.5), static_cast<T>(1.25), static_cast<T>(-0.4), static_cast<T>(-1.5),
            static_cast<T>(0.05)
        }
    );
}

TEST_CASE_TEMPLATE("ControlTrajectoriesInterpolate", T, float, double) {
    checkInterpolation(
        Control<T>{static_cast<T>(1.5), static_cast<T>(-0.02)},
        Control<T>{static_cast<T>(-0.4), static_cast<T>(0.01)}
    );
}

TEST_CASE_TEMPLATE("ModelSerializationRoundtrip", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(model).code == std::errc{});
    KinematicsModel<T> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.front_wheelbase(), model.front_wheelbase());
    CHECK_EQ(restored.rear_wheelbase(), model.rear_wheelbase());
    CHECK_EQ(restored.wheelbase(), model.wheelbase());
    CHECK_EQ(restored.control_lower_bound(), model.control_lower_bound());
    CHECK_EQ(restored.control_upper_bound(), model.control_upper_bound());
    // The controls are well beyond the bounds, so a restored model that lost the bounds or the
    // geometry would step somewhere else.
    const State<T> state{startAt(static_cast<T>(0.15), static_cast<T>(8.0))};
    const Control<T> control{static_cast<T>(9.0), static_cast<T>(-2.0)};
    CHECK_LE(
        distanceOf(
            restored.rungeKutta(state, control, kH<T>), model.rungeKutta(state, control, kH<T>)
        ),
        roundoffTolerance<T>()
    );
}

TEST_CASE_TEMPLATE("DefaultModel", T, float, double) {
    // Unit wheelbases with no bounds: what a deserialization starts from.
    const KinematicsModel<T> model{};
    CHECK_EQ(model.front_wheelbase(), static_cast<T>(1.0));
    CHECK_EQ(model.rear_wheelbase(), static_cast<T>(1.0));
    CHECK_EQ(model.wheelbase(), static_cast<T>(2.0));
    CHECK_EQ(model.control_lower_bound(), Control<T>{-std::numeric_limits<T>::infinity()});
    CHECK_EQ(model.control_upper_bound(), Control<T>{std::numeric_limits<T>::infinity()});
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE("RejectsInvalidArguments", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    const State<T> state{startAt(static_cast<T>(0.15), static_cast<T>(8.0))};

    SUBCASE("ModelRejectsNonPositiveWheelbase") {
        CHECK_THROWS_AS(
            (KinematicsModel<T>{static_cast<T>(0.0), kRearWheelbase<T>}), std::invalid_argument
        );
        CHECK_THROWS_AS(
            (KinematicsModel<T>{kFrontWheelbase<T>, static_cast<T>(-1.0)}), std::invalid_argument
        );
    }

    SUBCASE("ModelRejectsControlBoundsThatDoNotBracketZero") {
        CHECK_THROWS_AS(
            (KinematicsModel<T>{
                kFrontWheelbase<T>, kRearWheelbase<T>, Control<T>{static_cast<T>(0.5), kOmegaLo<T>},
                Control<T>{kAUp<T>, kOmegaUp<T>}
            }),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            (KinematicsModel<T>{
                kFrontWheelbase<T>, kRearWheelbase<T>, Control<T>{kALo<T>, static_cast<T>(0.1)},
                Control<T>{kAUp<T>, kOmegaUp<T>}
            }),
            std::invalid_argument
        );
    }

    SUBCASE("IntegratorsRejectNonPositiveTimeStep") {
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const T h : {static_cast<T>(0.0), static_cast<T>(-0.01)}) {
                CHECK_THROWS_AS(
                    static_cast<void>(stepWith(
                        mode, model, state, Control<T>{static_cast<T>(1.0), static_cast<T>(0.0)}, h
                    )),
                    std::invalid_argument
                );
            }
        }
    }

    SUBCASE("IntegratorsRejectSingularSteeringAngle") {
        // The steering angle is carried by the state, so this is where an omega that drove it out
        // of range over an earlier step surfaces.
        State<T> singular{state};
        singular.delta() = std::numbers::pi_v<T> * static_cast<T>(0.5);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(stepWith(
                    mode, model, singular, Control<T>{static_cast<T>(1.0), static_cast<T>(0.0)},
                    kH<T>
                )),
                std::invalid_argument
            );
        }
    }

    SUBCASE("EvolveRejectsMismatchedSpanSizes") {
        const Schedule<T> short_accels{
            std::vector<T>(4, kH<T>),
            std::vector<Control<T>>(3, Control<T>{static_cast<T>(1.0), static_cast<T>(0.0)})
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(evolveWith(mode, model, state, short_accels)),
                std::invalid_argument
            );
        }
    }

    SUBCASE("EvolveRejectsTraceOfWrongLength") {
        const Schedule<T> schedule{
            constantSchedule(4, kH<T>, static_cast<T>(1.0), static_cast<T>(0.0))
        };
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

} // namespace basjoo::bicycle

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options("kinematics_model_test", "unit test of KinematicsModel class");
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
