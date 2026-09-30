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
    fig->size(1800, 1000);
    return fig;
}

auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    const std::array<std::pair<const char*, const char*>, 6> labels{
        {{"x", "y"},
         {"t", "speed"},
         {"t", "yaw"},
         {"t", "position error"},
         {"t", "delta"},
         {"t", "u, v, r"}}
    };
    std::vector<matplot::axes_handle> axes;
    axes.reserve(labels.size());
    for (std::size_t i = 0; i < labels.size(); ++i) {
        axes.emplace_back(fig->add_subplot(2, 3, i));
        axes[i]->grid(matplot::on);
        axes[i]->hold(matplot::on);
        axes[i]->font_size(12.0);
        axes[i]->x_axis().label_font_size(12.0);
        axes[i]->y_axis().label_font_size(12.0);
        axes[i]->xlabel(labels[i].first);
        axes[i]->ylabel(labels[i].second);
    }
    return axes;
}

} // namespace

namespace basjoo::bicycle {

namespace {

using ::basjoo::math::IntegrateMode;

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

inline constexpr std::array<IntegrateMode, 3> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER, IntegrateMode::RUNGE_KUTTA
};

template <std::floating_point T>
using State = KinematicsModelState<T>;

template <std::floating_point T>
[[nodiscard]] auto boundedModel() -> KinematicsModel<T> {
    return KinematicsModel<T>{kFrontWheelbase<T>, kRearWheelbase<T>, kALo<T>, kAUp<T>,
                              kOmegaLo<T>,        kOmegaUp<T>};
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
    return State<T>{static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, delta, u};
}

/**
 * @brief Piecewise constant controls: step i holds accels[i] and omegas[i] over hs[i].
 */
template <std::floating_point T>
struct Schedule final {
    std::vector<T> hs;
    std::vector<T> accels;
    std::vector<T> omegas;
};

template <std::floating_point T>
[[nodiscard]] auto constantSchedule(std::size_t num_steps, T h, T accel, T omega) -> Schedule<T> {
    return Schedule<T>{
        std::vector<T>(num_steps, h), std::vector<T>(num_steps, accel),
        std::vector<T>(num_steps, omega)
    };
}

/**
 * @brief One sample of the analytic solution of the model.
 *
 * speed is the total velocity of the center of gravity, i.e. what speedOf() answers, rather than
 * the longitudinal component the acceleration control acts on.
 */
template <std::floating_point T>
struct Sample final {
    ::basjoo::math::Vec2<T> center;
    T yaw;
    T speed;
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
    return state.u * std::tan(state.delta) / kWheelbase<T>;
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
    return std::copysign(std::hypot(state.u, lateralVelocityOf(state)), state.u);
}

/**
 * @brief Gathers the flat position and velocity fields of a state back into a Vec2.
 *
 * State stores its scalars flat and keeps its velocity in the body frame, while the reference
 * solution below is written in terms of planar vectors in the world frame, so these two keep the
 * assertions reading the way the geometry does.
 */
template <std::floating_point T>
[[nodiscard]] auto centerOf(const State<T>& state) noexcept -> ::basjoo::math::Vec2<T> {
    return ::basjoo::math::Vec2<T>{state.x, state.y};
}

template <std::floating_point T>
[[nodiscard]] auto worldVelocityOf(const State<T>& state) noexcept -> ::basjoo::math::Vec2<T> {
    const T v{lateralVelocityOf(state)};
    return ::basjoo::math::Vec2<T>{
        state.u * std::cos(state.yaw) - v * std::sin(state.yaw),
        state.u * std::sin(state.yaw) + v * std::cos(state.yaw)
    };
}

/**
 * @brief Analytic solution under a constant acceleration and a constant steering angle.
 *
 * Only valid while omega is zero: a constant steering angle gives a constant path curvature that
 * does not depend on the speed, so the center of gravity always traces a circular arc of radius
 * 1 / kappa and the acceleration only changes how fast that arc is traversed. Note that accel is
 * the derivative of the longitudinal component u rather than of the total speed, and the two
 * differ by the constant factor cos(beta).
 */
template <std::floating_point T>
[[nodiscard]] auto analyticAt(const Sample<T>& initial, T accel, T delta, T t) noexcept
    -> Sample<T> {
    const T beta{slipAngleOf(delta)};
    const T kappa{curvatureOf(delta)};
    const T speed_rate{accel / std::cos(beta)};
    const T arc_length{initial.speed * t + static_cast<T>(0.5) * speed_rate * t * t};
    const T course{initial.yaw + beta};
    const T speed{initial.speed + speed_rate * t};
    if (kappa == static_cast<T>(0.0)) {
        return Sample<T>{
            {initial.center.x + arc_length * std::cos(course),
             initial.center.y + arc_length * std::sin(course)},
            initial.yaw,
            speed
        };
    }
    return Sample<T>{
        {initial.center.x + (std::sin(course + kappa * arc_length) - std::sin(course)) / kappa,
         initial.center.y - (std::cos(course + kappa * arc_length) - std::cos(course)) / kappa},
        initial.yaw + kappa * arc_length,
        speed
    };
}

/**
 * @brief Error bound the fourth order Runge-Kutta integrator has to stay within.
 *
 * Tight enough that an integrator that silently degraded to forward Euler, whose error is three
 * to five orders of magnitude larger in every scenario below, would fail immediately.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto rungeKuttaTolerance() noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(5.0e-3) : static_cast<T>(1.0e-6);
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

template <std::floating_point T>
[[nodiscard]] auto positionErrorOf(const State<T>& state, const Sample<T>& reference) noexcept
    -> T {
    return std::hypot(state.x - reference.center.x, state.y - reference.center.y);
}

/**
 * @brief Largest per field difference between two states.
 *
 * Written so that a NaN in either state propagates into the result instead of being skipped over
 * the way std::max() would skip it, which would let a diverging integrator pass a check against
 * zero.
 */
template <std::floating_point T>
[[nodiscard]] auto distanceOf(const State<T>& lhs, const State<T>& rhs) noexcept -> T {
    T result{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < State<T>::size(); ++i) {
        const T difference{std::abs(lhs.data()[i] - rhs.data()[i])};
        if (!(difference <= result)) {
            result = difference;
        }
    }
    return result;
}

template <std::floating_point T>
[[nodiscard]] auto stepWith(
    IntegrateMode mode, const KinematicsModel<T>& model, const State<T>& state,
    std::type_identity_t<T> accel, std::type_identity_t<T> omega, std::type_identity_t<T> h
) -> State<T> {
    switch (mode) {
    case IntegrateMode::FORWARD_EULER:
        return model.forwardEuler(state, accel, omega, h);
    case IntegrateMode::SYMPLECTIC_EULER:
        return model.symplecticEuler(state, accel, omega, h);
    case IntegrateMode::RUNGE_KUTTA:
        return model.rungeKutta(state, accel, omega, h);
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
            initial_state, schedule.accels, schedule.omegas, schedule.hs, trace
        );
    case IntegrateMode::SYMPLECTIC_EULER:
        return model.template evolve<IntegrateMode::SYMPLECTIC_EULER>(
            initial_state, schedule.accels, schedule.omegas, schedule.hs, trace
        );
    case IntegrateMode::RUNGE_KUTTA:
        return model.template evolve<IntegrateMode::RUNGE_KUTTA>(
            initial_state, schedule.accels, schedule.omegas, schedule.hs, trace
        );
    }
    std::unreachable();
}

/**
 * @brief One run of every integrator through the same schedule, each trace led by the initial
 * state so that it lines up with ts and with the analytic samples, if any.
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
 */
template <std::floating_point T>
[[nodiscard]] auto simulate(
    const KinematicsModel<T>& model, const State<T>& initial_state, const Schedule<T>& schedule
) -> Trajectory<T> {
    const std::size_t size{schedule.hs.size()};
    Trajectory<T> trajectory;
    trajectory.ts.reserve(size + 1);
    trajectory.ts.push_back(static_cast<T>(0.0));
    for (const T h : schedule.hs) {
        trajectory.ts.push_back(trajectory.ts.back() + h);
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        std::vector<State<T>>& trace{trajectory.traces[static_cast<std::size_t>(mode)]};
        trace.resize(size + 1);
        static_cast<void>(evolveWith(mode, model, initial_state, schedule, trace));
    }
    return trajectory;
}

/**
 * @brief Samples the analytic solution alongside an already marched trajectory.
 *
 * @param reference_at maps a step index in [0, schedule size] onto the analytic solution there.
 */
template <std::floating_point T>
auto fillReference(Trajectory<T>& trajectory, const auto& reference_at) -> void {
    trajectory.reference.reserve(trajectory.ts.size());
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        trajectory.reference.push_back(reference_at(i));
    }
}

/**
 * @brief Asserts the algebraic relations the kinematic model keeps along any trajectory.
 *
 * The steering angle and the longitudinal velocity are driven straight by the two controls, so
 * they have to come back exactly where accumulating the piecewise constant controls by hand puts
 * them. This is what carries the scenarios that have no closed form solution.
 */
template <std::floating_point T>
auto checkInvariants(
    const std::vector<State<T>>& states, const State<T>& initial_state, const Schedule<T>& schedule,
    T tolerance
) -> void {
    const std::size_t size{schedule.hs.size()};
    T delta{initial_state.delta};
    T u{initial_state.u};
    T max_delta_error{static_cast<T>(0.0)};
    T max_u_error{static_cast<T>(0.0)};
    for (std::size_t i{0}; i <= size; ++i) {
        max_delta_error = std::max(max_delta_error, std::abs(states[i].delta - delta));
        max_u_error = std::max(max_u_error, std::abs(states[i].u - u));
        if (i < size) {
            delta += schedule.omegas[i] * schedule.hs[i];
            u += schedule.accels[i] * schedule.hs[i];
        }
    }
    CHECK_LE(max_delta_error, tolerance);
    CHECK_LE(max_u_error, tolerance);
}

/**
 * @brief Asserts that Runge-Kutta tracks the analytic solution.
 */
template <std::floating_point T>
auto checkAgainstAnalytic(const Trajectory<T>& trajectory, T symplectic_euler_bound) -> void {
    const T tolerance{rungeKuttaTolerance<T>()};
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    const std::vector<State<T>>& symplectic_euler{
        trajectory.trace(IntegrateMode::SYMPLECTIC_EULER)
    };
    T max_position_error{static_cast<T>(0.0)};
    T max_yaw_error{static_cast<T>(0.0)};
    T max_speed_error{static_cast<T>(0.0)};
    T max_symplectic_error{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const Sample<T>& reference{trajectory.reference[i]};
        max_position_error =
            std::max(max_position_error, positionErrorOf(runge_kutta[i], reference));
        max_yaw_error = std::max(max_yaw_error, std::abs(runge_kutta[i].yaw - reference.yaw));
        max_speed_error =
            std::max(max_speed_error, std::abs(speedOf(runge_kutta[i]) - reference.speed));
        max_symplectic_error =
            std::max(max_symplectic_error, positionErrorOf(symplectic_euler[i], reference));
    }
    CHECK_LE(max_position_error, tolerance);
    CHECK_LE(max_yaw_error, tolerance);
    CHECK_LE(max_speed_error, tolerance);
    // symplecticEuler() is only first order, so it gets a loose per scenario bound rather than the
    // Runge-Kutta tolerance. It still has to track the analytic solution: without this an
    // implementation that returned the state untouched would sail through the evolve() check below.
    CHECK_LE(max_symplectic_error, symplectic_euler_bound);
}

/**
 * @brief Estimates the observed order of convergence of the integrators.
 *
 * Halving the step three times and comparing consecutive results estimates the order without
 * needing a solution to compare against, which is what the scenarios with a non zero steering rate
 * require: their trajectories are clothoid-like and have no elementary closed form. Fourth order
 * quarters the step error sixteen fold, first order only halves it, so running forward Euler
 * through the same estimator both pins down the Runge-Kutta claim and proves the estimator itself
 * measures what it says it does.
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
            schedule.accels[i] = accel_at(fraction);
            schedule.omegas[i] = omega_at(fraction);
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
    CHECK_GT(runge_kutta_ratio, static_cast<T>(12.0));
    CHECK_LT(runge_kutta_ratio, static_cast<T>(24.0));
    const T euler_ratio{ratio_of(IntegrateMode::FORWARD_EULER)};
    CHECK_GT(euler_ratio, static_cast<T>(1.5));
    CHECK_LT(euler_ratio, static_cast<T>(3.0));
}

auto plotTrajectory(const std::string& title, const Trajectory<double>& trajectory) -> void {
    matplot::figure_handle fig = createFigureHandle();
    fig->title(title);
    std::vector<matplot::axes_handle> axes = createAxesHandles(fig);
    const bool has_reference{!trajectory.reference.empty()};
    std::vector<std::string> legend;
    if (has_reference) {
        std::vector<double> xs, ys, speeds, yaws;
        for (const Sample<double>& sample : trajectory.reference) {
            xs.push_back(sample.center.x);
            ys.push_back(sample.center.y);
            speeds.push_back(sample.speed);
            yaws.push_back(sample.yaw);
        }
        axes[0]->plot(xs, ys, "-")->line_width(2.0);
        axes[1]->plot(trajectory.ts, speeds, "-")->line_width(2.0);
        axes[2]->plot(trajectory.ts, yaws, "-")->line_width(2.0);
        legend.emplace_back("analytic");
    }
    const std::array<const char*, kIntegrateModes.size()> names{
        "forward euler", "symplectic euler", "runge kutta"
    };
    const std::array<const char*, kIntegrateModes.size()> line_specs{":", "-.", "--"};
    for (const IntegrateMode mode : kIntegrateModes) {
        std::vector<double> xs, ys, speeds, yaws, errors;
        for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
            const State<double>& state{trajectory.trace(mode)[i]};
            xs.push_back(state.x);
            ys.push_back(state.y);
            speeds.push_back(speedOf(state));
            yaws.push_back(state.yaw);
            if (has_reference) {
                errors.push_back(positionErrorOf(state, trajectory.reference[i]));
            }
        }
        const char* line_spec{line_specs[static_cast<std::size_t>(mode)]};
        axes[0]->plot(xs, ys, line_spec)->line_width(2.0);
        axes[1]->plot(trajectory.ts, speeds, line_spec)->line_width(2.0);
        axes[2]->plot(trajectory.ts, yaws, line_spec)->line_width(2.0);
        if (has_reference) {
            axes[3]->semilogy(trajectory.ts, errors, line_spec)->line_width(2.0);
        }
        legend.emplace_back(names[static_cast<std::size_t>(mode)]);
    }
    // The steering angle and the body frame velocities along the Runge-Kutta trace.
    std::vector<double> deltas, us, vs, rs;
    for (const State<double>& state : trajectory.trace(IntegrateMode::RUNGE_KUTTA)) {
        deltas.push_back(state.delta);
        us.push_back(state.u);
        vs.push_back(lateralVelocityOf(state));
        rs.push_back(yawRateOf(state));
    }
    axes[4]->plot(trajectory.ts, deltas, "-")->line_width(2.0);
    axes[5]->plot(trajectory.ts, us, "-")->line_width(2.0);
    axes[5]->plot(trajectory.ts, vs, "--")->line_width(2.0);
    axes[5]->plot(trajectory.ts, rs, ":")->line_width(2.0);
    axes[0]->axis(matplot::equal);
    for (std::size_t i{0}; i < 3; ++i) {
        axes[i]->legend(legend);
    }
    if (has_reference) {
        axes[3]->legend(std::vector<std::string>{legend.begin() + 1, legend.end()});
    }
    axes[5]->legend({"u", "v", "r"});
    fig->show();
}

} // namespace

// Scenarios with a zero steering rate keep a constant steering angle, which puts the center of
// gravity on an arc of constant curvature and leaves a closed form solution to compare against.

TEST_CASE_TEMPLATE("StraightLineAccelerateThenDecelerate", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(5.0)};
    constexpr T kAccel{static_cast<T>(2.0)};
    constexpr T kPhaseDuration{static_cast<T>(6.0)};
    constexpr std::size_t kStepsPerPhase{1200};
    const T h{kPhaseDuration / static_cast<T>(kStepsPerPhase)};

    const State<T> initial_state{startAt(kDelta, kInitialU)};
    Schedule<T> schedule{constantSchedule(kStepsPerPhase * 2, h, kAccel, static_cast<T>(0.0))};
    std::fill(schedule.accels.begin() + kStepsPerPhase, schedule.accels.end(), -kAccel);

    Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, speedOf(initial_state)};
    const Sample<T> midpoint{analyticAt(start, kAccel, kDelta, kPhaseDuration)};
    fillReference(trajectory, [&](std::size_t i) -> Sample<T> {
        const T t{h * static_cast<T>(i)};
        return i <= kStepsPerPhase ? analyticAt(start, kAccel, kDelta, t)
                                   : analyticAt(midpoint, -kAccel, kDelta, t - kPhaseDuration);
    });

    // symplecticEuler() is first order and lands about 0.03 m off over this run, so the bound
    // leaves roughly a factor of two of room while still failing outright on an implementation
    // that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(0.06)};
    checkAgainstAnalytic(trajectory, kSymplecticEulerBound);
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    checkInvariants(runge_kutta, initial_state, schedule, rungeKuttaTolerance<T>());

    // A zero steering angle has to leave the heading untouched and the vehicle on its axis
    // throughout.
    for (const State<T>& state : runge_kutta) {
        CHECK_EQ(state.yaw, doctest::Approx(kInitialYaw<T>).epsilon(rungeKuttaTolerance<T>()));
        CHECK_EQ(state.delta, static_cast<T>(0.0));
        CHECK_LE(std::abs(lateralVelocityOf(state)), rungeKuttaTolerance<T>());
        CHECK_LE(std::abs(yawRateOf(state)), rungeKuttaTolerance<T>());
    }
    // The speed comes back to where it started after the symmetric acceleration profile.
    CHECK_EQ(
        speedOf(runge_kutta.back()), doctest::Approx(kInitialU).epsilon(rungeKuttaTolerance<T>())
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
    constexpr T kInitialU{static_cast<T>(8.0)};
    constexpr T kAccel{static_cast<T>(0.0)};
    constexpr std::size_t kNumSteps{800};

    const T kappa{curvatureOf(kDelta)};
    const T radius{static_cast<T>(1.0) / kappa};
    // A constant steering angle puts the center of gravity on a circle whose radius follows from
    // the geometry alone: R = hypot(l_r, wheelbase / tan(delta)).
    CHECK_EQ(
        radius, doctest::Approx(std::hypot(kRearWheelbase<T>, kWheelbase<T> / std::tan(kDelta)))
                    .epsilon(rungeKuttaTolerance<T>())
    );

    const State<T> initial_state{startAt(kDelta, kInitialU)};
    const T initial_speed{speedOf(initial_state)};
    const T period{static_cast<T>(2.0) * std::numbers::pi_v<T> / (kappa * initial_speed)};
    const T h{period / static_cast<T>(kNumSteps)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};

    Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, initial_speed};
    fillReference(trajectory, [&](std::size_t i) -> Sample<T> {
        return analyticAt(start, kAccel, kDelta, h * static_cast<T>(i));
    });

    // symplecticEuler() is first order and lands about 0.48 m off over this run, so the bound
    // leaves roughly a factor of two of room while still failing outright on an implementation
    // that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(1.0)};
    checkAgainstAnalytic(trajectory, kSymplecticEulerBound);
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    checkInvariants(runge_kutta, initial_state, schedule, rungeKuttaTolerance<T>());

    // Every sample has to sit on the circle centered at the instantaneous center of rotation.
    const T course{kInitialYaw<T> + slipAngleOf(kDelta)};
    const ::basjoo::math::Vec2<T> rotation_center{
        start.center.x - radius * std::sin(course), start.center.y + radius * std::cos(course)
    };
    for (const State<T>& state : runge_kutta) {
        CHECK_EQ(
            centerOf(state).euclideanTo(rotation_center),
            doctest::Approx(radius).epsilon(rungeKuttaTolerance<T>())
        );
        CHECK_EQ(
            worldVelocityOf(state).euclidean(),
            doctest::Approx(initial_speed).epsilon(rungeKuttaTolerance<T>())
        );
        CHECK_EQ(
            yawRateOf(state),
            doctest::Approx(kappa * initial_speed).epsilon(rungeKuttaTolerance<T>())
        );
    }
    // One full period brings the vehicle back to where it started.
    CHECK_LE(centerOf(runge_kutta.back()).euclideanTo(start.center), rungeKuttaTolerance<T>());
    CHECK_EQ(
        runge_kutta.back().yaw,
        doctest::Approx(kInitialYaw<T> + static_cast<T>(2.0) * std::numbers::pi_v<T>)
            .epsilon(rungeKuttaTolerance<T>())
    );

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
    constexpr T kAccel{static_cast<T>(1.5)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    const State<T> initial_state{startAt(kDelta, kInitialU)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};

    Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, speedOf(initial_state)};
    fillReference(trajectory, [&](std::size_t i) -> Sample<T> {
        return analyticAt(start, kAccel, kDelta, h * static_cast<T>(i));
    });

    // symplecticEuler() is first order and lands about 0.29 m off over this run, so the bound
    // leaves roughly a factor of two of room while still failing outright on an implementation
    // that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(0.6)};
    checkAgainstAnalytic(trajectory, kSymplecticEulerBound);
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    checkInvariants(runge_kutta, initial_state, schedule, rungeKuttaTolerance<T>());

    // Accelerating through a turn is where the acceleration has to stay along the body axis rather
    // than along the course angle: resolving it along the course instead leaves the velocity
    // drifting off the slip angle the steering geometry dictates, an error that neither the
    // straight line nor the constant speed circle scenario can expose.
    const T beta{slipAngleOf(kDelta)};
    const T kappa{curvatureOf(kDelta)};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const State<T>& state{runge_kutta[i]};
        const T speed{speedOf(initial_state) + kAccel / std::cos(beta) * trajectory.ts[i]};
        CHECK_EQ(
            worldVelocityOf(state).euclidean(),
            doctest::Approx(speed).epsilon(rungeKuttaTolerance<T>())
        );
        CHECK_EQ(
            std::atan2(lateralVelocityOf(state), state.u),
            doctest::Approx(beta).epsilon(rungeKuttaTolerance<T>())
        );
        // state.yaw accumulates without wrapping (see ConstantSpeedCircle, which drives it past a
        // full turn on purpose), while atan2() always answers in (-pi, pi], so the two only agree
        // modulo 2 * pi once the course angle winds past that range over the run.
        const T course_angle{state.yaw + beta};
        const ::basjoo::math::Vec2<T> velocity{worldVelocityOf(state)};
        const T velocity_angle{std::atan2(velocity.y, velocity.x)};
        CHECK_LE(
            std::abs(
                std::remainder(
                    velocity_angle - course_angle, static_cast<T>(2.0) * std::numbers::pi_v<T>
                )
            ),
            rungeKuttaTolerance<T>()
        );
        CHECK_EQ(
            yawRateOf(state), doctest::Approx(kappa * speed).epsilon(rungeKuttaTolerance<T>())
        );
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
    constexpr T kAccel{static_cast<T>(-2.0)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    const State<T> initial_state{startAt(kDelta, kInitialU)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};

    Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, speedOf(initial_state)};
    fillReference(trajectory, [&](std::size_t i) -> Sample<T> {
        return analyticAt(start, kAccel, kDelta, h * static_cast<T>(i));
    });

    // symplecticEuler() is first order and lands about 0.66 m off over this run, so the bound
    // leaves roughly a factor of two of room while still failing outright on an implementation
    // that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(1.4)};
    checkAgainstAnalytic(trajectory, kSymplecticEulerBound);
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    checkInvariants(runge_kutta, initial_state, schedule, rungeKuttaTolerance<T>());

    // The speed runs from +12 down through zero into reverse, so the run crosses the one point
    // where the velocity vector carries no direction at all. Taking the sign from the longitudinal
    // component is what lets the trajectory continue into reverse instead of bouncing back: the
    // magnitude alone leaves the vehicle turning the wrong way once it reverses.
    const T beta{slipAngleOf(kDelta)};
    const T kappa{curvatureOf(kDelta)};
    bool saw_forward{false};
    bool saw_reverse{false};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const State<T>& state{runge_kutta[i]};
        const T speed{speedOf(initial_state) + kAccel / std::cos(beta) * trajectory.ts[i]};
        CHECK_EQ(speedOf(state), doctest::Approx(speed).epsilon(rungeKuttaTolerance<T>()));
        CHECK_EQ(
            yawRateOf(state), doctest::Approx(kappa * speed).epsilon(rungeKuttaTolerance<T>())
        );
        // The magnitude stays non-negative throughout and therefore cannot tell the two halves
        // of the run apart on its own.
        CHECK_EQ(
            worldVelocityOf(state).euclidean(),
            doctest::Approx(std::abs(speed)).epsilon(rungeKuttaTolerance<T>())
        );
        // state.yaw accumulates without wrapping, while atan2() always answers in (-pi, pi], so
        // the two only agree modulo 2 * pi once the course angle winds past that range over the
        // run (see the matching comment in AcceleratingTurn).
        const T course_angle{state.yaw + beta};
        const ::basjoo::math::Vec2<T> velocity{worldVelocityOf(state)};
        if (speed > static_cast<T>(1.0)) {
            saw_forward = true;
            const T velocity_angle{std::atan2(velocity.y, velocity.x)};
            CHECK_LE(
                std::abs(
                    std::remainder(
                        velocity_angle - course_angle, static_cast<T>(2.0) * std::numbers::pi_v<T>
                    )
                ),
                rungeKuttaTolerance<T>()
            );
        } else if (speed < static_cast<T>(-1.0)) {
            saw_reverse = true;
            // Reversing points the velocity the other way round while the heading is unchanged.
            const T reverse_velocity_angle{std::atan2(-velocity.y, -velocity.x)};
            CHECK_LE(
                std::abs(
                    std::remainder(
                        reverse_velocity_angle - course_angle,
                        static_cast<T>(2.0) * std::numbers::pi_v<T>
                    )
                ),
                rungeKuttaTolerance<T>()
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
// the observed order of convergence instead.

TEST_CASE_TEMPLATE("ConstantSteeringRateClothoid", T, float, double) {
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kInitialDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(8.0)};
    constexpr T kAccel{static_cast<T>(0.0)};
    constexpr T kOmega{static_cast<T>(0.02)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    const State<T> initial_state{startAt(kInitialDelta, kInitialU)};
    const Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, kOmega)};
    const Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    checkInvariants(runge_kutta, initial_state, schedule, rungeKuttaTolerance<T>());

    // The steering angle ramps linearly, so the curvature grows with it and the vehicle spirals
    // inwards: both the yaw rate and its rate of change have to keep increasing.
    const State<T>& final_state{runge_kutta.back()};
    CHECK_EQ(
        final_state.delta,
        doctest::Approx(kInitialDelta + kOmega * kDuration).epsilon(rungeKuttaTolerance<T>())
    );
    CHECK_EQ(final_state.u, doctest::Approx(kInitialU).epsilon(rungeKuttaTolerance<T>()));
    CHECK_GT(yawRateOf(final_state), yawRateOf(runge_kutta.front()));
    for (std::size_t i{1}; i < runge_kutta.size(); ++i) {
        CHECK_GT(yawRateOf(runge_kutta[i]), yawRateOf(runge_kutta[i - 1]));
    }

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder(
            model, initial_state, kDuration, 300, [](T) -> T { return kAccel; },
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
    constexpr T kAccel{static_cast<T>(0.8)};
    constexpr T kOmega{static_cast<T>(0.08)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    // Steering out, back through the straight ahead position and out the other way, all while
    // accelerating. This is the only scenario where both terms of the lateral acceleration are
    // alive at once: accel * tan(delta) vanishes wherever the steering angle is zero, and
    // u * omega / cos(delta)^2 vanishes wherever the steering rate is, so a model that dropped
    // either one still passes every scenario above.
    const auto omega_at = [](T fraction) -> T {
        return (fraction < static_cast<T>(0.25) || fraction >= static_cast<T>(0.75)) ? kOmega
                                                                                     : -kOmega;
    };
    Schedule<T> schedule{constantSchedule(kNumSteps, h, kAccel, static_cast<T>(0.0))};
    for (std::size_t i{0}; i < kNumSteps; ++i) {
        schedule.omegas[i] = omega_at(static_cast<T>(i) / static_cast<T>(kNumSteps));
    }

    const State<T> initial_state{startAt(kInitialDelta, kInitialU)};
    const Trajectory<T> trajectory{simulate(model, initial_state, schedule)};
    const std::vector<State<T>>& runge_kutta{trajectory.trace(IntegrateMode::RUNGE_KUTTA)};
    checkInvariants(runge_kutta, initial_state, schedule, rungeKuttaTolerance<T>());

    // The steering angle has to come back to where it started, having visited both sides on the
    // way, and the lateral velocity has to change sign with it.
    T max_delta{static_cast<T>(0.0)};
    T min_delta{static_cast<T>(0.0)};
    for (const State<T>& state : runge_kutta) {
        max_delta = std::max(max_delta, state.delta);
        min_delta = std::min(min_delta, state.delta);
        // Driving forwards, the lateral velocity leans to whichever side the steering angle does.
        if (std::abs(state.delta) > static_cast<T>(0.01)) {
            CHECK_EQ(
                lateralVelocityOf(state) > static_cast<T>(0.0), state.delta > static_cast<T>(0.0)
            );
        }
    }
    CHECK_GT(max_delta, static_cast<T>(0.1));
    CHECK_LT(min_delta, static_cast<T>(-0.1));
    CHECK_LE(std::abs(runge_kutta.back().delta), rungeKuttaTolerance<T>());
    CHECK_EQ(
        runge_kutta.back().u,
        doctest::Approx(kInitialU + kAccel * kDuration).epsilon(rungeKuttaTolerance<T>())
    );

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder(
            model, initial_state, kDuration, 300, [](T) -> T { return kAccel; }, omega_at
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
            {static_cast<T>(1.2), static_cast<T>(-1.2), static_cast<T>(0.4), static_cast<T>(0.9),
             static_cast<T>(0.0)},
            {static_cast<T>(-0.05), static_cast<T>(0.0), static_cast<T>(0.07), static_cast<T>(0.05),
             static_cast<T>(0.02)}
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            // Marching the single step integrator by hand gives the state after every step, which
            // is what the trace is supposed to hold behind the initial state.
            std::vector<State<T>> expected{initial_state};
            for (std::size_t i{0}; i < schedule.hs.size(); ++i) {
                expected.push_back(stepWith(
                    mode, model, expected.back(), schedule.accels[i], schedule.omegas[i],
                    schedule.hs[i]
                ));
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
            static_cast<T>(1.0), static_cast<T>(2.0), kInitialYaw<T>, static_cast<T>(0.15),
            static_cast<T>(8.0)
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

TEST_CASE_TEMPLATE("TangentCarriesTheControlsAsRates", T, float, double) {
    // The delta and u fields of the tangent are the steering rate and the acceleration handed in,
    // whatever the state.
    const KinematicsModel<T> model{unboundedModel<T>()};
    constexpr T kAccel{static_cast<T>(-0.6)};
    constexpr T kOmega{static_cast<T>(0.03)};
    const State<T> state{
        static_cast<T>(1.0), static_cast<T>(-2.0), kInitialYaw<T>, static_cast<T>(0.15),
        static_cast<T>(8.0)
    };
    const State<T> k{model.tangent(state, kAccel, kOmega)};
    CHECK_EQ(k.delta, kOmega);
    CHECK_EQ(k.u, kAccel);
    CHECK_EQ(k.yaw, yawRateOf(state));
}

// The two controls are the one place the model says no: everything else it takes at face value,
// while a and omega are clamped into the bounds it was built with.

TEST_CASE_TEMPLATE("ControlBoundsClampEveryEntryPoint", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const KinematicsModel<T> unbounded{unboundedModel<T>()};
    const State<T> start{startAt(static_cast<T>(0.15), static_cast<T>(8.0))};

    SUBCASE("BoundsComeBackOutAndDefaultToInfinite") {
        CHECK_EQ(model.a_lower_bound(), kALo<T>);
        CHECK_EQ(model.a_upper_bound(), kAUp<T>);
        CHECK_EQ(model.omega_lower_bound(), kOmegaLo<T>);
        CHECK_EQ(model.omega_upper_bound(), kOmegaUp<T>);
        CHECK_EQ(unbounded.a_lower_bound(), -std::numeric_limits<T>::infinity());
        CHECK_EQ(unbounded.a_upper_bound(), std::numeric_limits<T>::infinity());
        CHECK_EQ(unbounded.omega_lower_bound(), -std::numeric_limits<T>::infinity());
        CHECK_EQ(unbounded.omega_upper_bound(), std::numeric_limits<T>::infinity());
    }

    SUBCASE("EveryEntryPointClamps") {
        // Controls beyond the bounds have to act exactly as the bounds they are clamped to.
        constexpr T kLooseAccel{static_cast<T>(50.0)};
        constexpr T kLooseOmega{static_cast<T>(-7.0)};
        const State<T> k{model.tangent(start, kLooseAccel, kLooseOmega)};
        CHECK_EQ(k.u, kAUp<T>);
        CHECK_EQ(k.delta, kOmegaLo<T>);
        CHECK_EQ(k, model.tangent(start, kAUp<T>, kOmegaLo<T>));
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_LE(
                distanceOf(
                    stepWith(mode, model, start, kLooseAccel, kLooseOmega, kH<T>),
                    stepWith(mode, model, start, kAUp<T>, kOmegaLo<T>, kH<T>)
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
            schedule.accels[i] = static_cast<T>(8.0) * std::sin(t);
            schedule.omegas[i] = static_cast<T>(1.2) * std::cos(t);
            clamped.accels[i] = std::clamp(schedule.accels[i], kALo<T>, kAUp<T>);
            clamped.omegas[i] = std::clamp(schedule.omegas[i], kOmegaLo<T>, kOmegaUp<T>);
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
        const KinematicsModel<T> wide{kFrontWheelbase<T>,     kRearWheelbase<T>,
                                      static_cast<T>(-1.0e3), static_cast<T>(1.0e3),
                                      static_cast<T>(-1.0e3), static_cast<T>(1.0e3)};
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
        static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.75), static_cast<T>(0.125),
        static_cast<T>(7.5)
    };
    const State<T> rhs{
        static_cast<T>(0.5), static_cast<T>(1.5), static_cast<T>(-0.25), static_cast<T>(-0.75),
        static_cast<T>(2.5)
    };

    SUBCASE("LayoutCoversEveryField") {
        static_assert(State<T>::size() == 5);
        CHECK_EQ(sizeof(State<T>), State<T>::size() * sizeof(T));
        CHECK_EQ(lhs.data(), &lhs.x);
        CHECK_EQ(&lhs.u - lhs.data(), static_cast<std::ptrdiff_t>(State<T>::size()) - 1);
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
                           static_cast<T>(-0.625), static_cast<T>(10.0)
                       }
        );
        CHECK_EQ(
            lhs - rhs, State<T>{
                           static_cast<T>(1.0), static_cast<T>(-4.0), static_cast<T>(1.0),
                           static_cast<T>(0.875), static_cast<T>(5.0)
                       }
        );
        const State<T> scaled{
            static_cast<T>(3.0), static_cast<T>(-5.0), static_cast<T>(1.5), static_cast<T>(0.25),
            static_cast<T>(15.0)
        };
        CHECK_EQ(lhs * static_cast<T>(2.0), scaled);
        CHECK_EQ(static_cast<T>(2.0) * lhs, scaled);
        CHECK_EQ(scaled / static_cast<T>(2.0), lhs);
        CHECK_EQ(
            -lhs, State<T>{
                      static_cast<T>(-1.5), static_cast<T>(2.5), static_cast<T>(-0.75),
                      static_cast<T>(-0.125), static_cast<T>(-7.5)
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

TEST_CASE_TEMPLATE("StateSerializationRoundtrip", T, float, double) {
    for (const State<T>& state :
         {State<T>{},
          State<T>{
              static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.3), static_cast<T>(0.15),
              static_cast<T>(7.9)
          },
          State<T>{
              static_cast<T>(-3.2), static_cast<T>(4.1), static_cast<T>(-1.1), static_cast<T>(-0.1),
              static_cast<T>(-6.0)
          }}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(state).code == std::errc{});
        State<T> restored{};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, state);
    }
}

TEST_CASE_TEMPLATE("StateTrajectoriesInterpolate", T, float, double) {
    // A state is a vector in the sense of VecArithmetic, which is all the piecewise functions and
    // their proxy ask of a value type. Every field below is affine in t, which linear, natural
    // cubic and quintic splines all reproduce exactly, so what is left is the roundoff of the
    // spline solves.
    using ::basjoo::math::FunctionProxy;
    using ::basjoo::math::makeFunctionProxy;
    using ::basjoo::math::PiecewiseCubicFunction;
    using ::basjoo::math::PiecewiseLinearFunction;
    using ::basjoo::math::PiecewiseQuinticFunction;
    static_assert(::basjoo::math::VecArithmetic<State<T>>);
    static_assert(::basjoo::math::isVecArithmeticV<State<T>>);
    static_assert(!::basjoo::math::MatArithmetic<State<T>>);
    static_assert(!::basjoo::math::isMatArithmeticV<State<T>>);
    constexpr T kTolerance{
        std::same_as<T, float> ? static_cast<T>(1.0e-4) : static_cast<T>(1.0e-10)
    };

    const State<T> origin{
        static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.3), static_cast<T>(0.15),
        static_cast<T>(7.5)
    };
    const State<T> rate{
        static_cast<T>(7.5), static_cast<T>(1.25), static_cast<T>(-0.4), static_cast<T>(0.05),
        static_cast<T>(-1.5)
    };
    const auto exact = [&](T t) -> State<T> { return origin + rate * t; };
    // Unevenly spaced, so that no spline solve gets to lean on equal intervals.
    const std::vector<T> ts{
        static_cast<T>(0.0), static_cast<T>(0.5), static_cast<T>(1.25), static_cast<T>(2.0),
        static_cast<T>(3.0)
    };
    std::vector<State<T>> ys;
    ys.reserve(ts.size());
    for (const T t : ts) {
        ys.push_back(exact(t));
    }
    const T lower{static_cast<T>(0.3)};
    const T upper{static_cast<T>(2.6)};
    const State<T> area{
        origin * (upper - lower) + rate * ((upper * upper - lower * lower) * static_cast<T>(0.5))
    };

    std::vector<FunctionProxy<State<T>>> functions;
    functions.emplace_back(makeFunctionProxy(PiecewiseLinearFunction<State<T>>{ts, ys}));
    functions.emplace_back(makeFunctionProxy(PiecewiseCubicFunction<State<T>>{ts, ys}));
    functions.emplace_back(makeFunctionProxy(PiecewiseQuinticFunction<State<T>>{ts, ys}));
    for (const FunctionProxy<State<T>>& function : functions) {
        for (const T t :
             {static_cast<T>(0.2), static_cast<T>(0.9), static_cast<T>(1.6), static_cast<T>(2.7)}) {
            CHECK_LE(distanceOf(function->eval(t), exact(t)), kTolerance);
            CHECK_LE(distanceOf(function->derivative(t), rate), kTolerance);
        }
        CHECK_LE(distanceOf(function->integral(lower, upper), area), kTolerance);

        // Restored into an empty proxy, so the concrete function has to come through the archive.
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(function).code == std::errc{});
        FunctionProxy<State<T>> restored{};
        REQUIRE(in(restored).code == std::errc{});
        REQUIRE(restored.has_value());
        CHECK(proxy_typeid(*restored) == proxy_typeid(*function));
        CHECK_EQ(restored->eval(lower), function->eval(lower));
    }
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
    CHECK_EQ(restored.a_lower_bound(), model.a_lower_bound());
    CHECK_EQ(restored.a_upper_bound(), model.a_upper_bound());
    CHECK_EQ(restored.omega_lower_bound(), model.omega_lower_bound());
    CHECK_EQ(restored.omega_upper_bound(), model.omega_upper_bound());
    // The controls are well beyond the bounds, so a restored model that lost the bounds or the
    // geometry would step somewhere else.
    const State<T> state{startAt(static_cast<T>(0.15), static_cast<T>(8.0))};
    CHECK_LE(
        distanceOf(
            restored.rungeKutta(state, static_cast<T>(9.0), static_cast<T>(-2.0), kH<T>),
            model.rungeKutta(state, static_cast<T>(9.0), static_cast<T>(-2.0), kH<T>)
        ),
        roundoffTolerance<T>()
    );
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
                kFrontWheelbase<T>, kRearWheelbase<T>, static_cast<T>(0.5), kAUp<T>, kOmegaLo<T>,
                kOmegaUp<T>
            }),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            (KinematicsModel<T>{
                kFrontWheelbase<T>, kRearWheelbase<T>, kALo<T>, kAUp<T>, static_cast<T>(0.1),
                kOmegaUp<T>
            }),
            std::invalid_argument
        );
    }

    SUBCASE("IntegratorsRejectNonPositiveTimeStep") {
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const T h : {static_cast<T>(0.0), static_cast<T>(-0.01)}) {
                CHECK_THROWS_AS(
                    static_cast<void>(
                        stepWith(mode, model, state, static_cast<T>(1.0), static_cast<T>(0.0), h)
                    ),
                    std::invalid_argument
                );
            }
        }
    }

    SUBCASE("IntegratorsRejectSingularSteeringAngle") {
        // The steering angle is carried by the state, so this is where an omega that drove it out
        // of range over an earlier step surfaces.
        State<T> singular{state};
        singular.delta = std::numbers::pi_v<T> * static_cast<T>(0.5);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(
                    stepWith(mode, model, singular, static_cast<T>(1.0), static_cast<T>(0.0), kH<T>)
                ),
                std::invalid_argument
            );
        }
    }

    SUBCASE("EvolveRejectsMismatchedSpanSizes") {
        const Schedule<T> short_accels{
            std::vector<T>(4, kH<T>), std::vector<T>(3, static_cast<T>(1.0)),
            std::vector<T>(4, static_cast<T>(0.0))
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
