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
#include <cmath>
#include <concepts>
#include <cstddef>
#include <iterator>
#include <limits>
#include <numbers>
#include <span>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>
#include <vector>

#include "cxxopts.hpp"
#include "matplot/matplot.h"
#include "zpp_bits.h"

#include "basjoo/math/dense/vec2.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

namespace {

bool plot_graph{false};

auto createFigureHandle() noexcept -> matplot::figure_handle {
    matplot::figure_handle fig = matplot::figure();
    fig->size(1800, 1000);
    return fig;
}

auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    std::vector<matplot::axes_handle> axes;
    axes.reserve(6);
    for (std::size_t i = 0; i < 6; ++i) {
        axes.emplace_back(fig->add_subplot(2, 3, i));
        axes[i]->grid(matplot::on);
        axes[i]->hold(matplot::on);
        axes[i]->font_size(12.0);
        axes[i]->x_axis().label_font_size(12.0);
        axes[i]->y_axis().label_font_size(12.0);
    }
    axes[0]->xlabel("x");
    axes[0]->ylabel("y");
    axes[1]->xlabel("t");
    axes[1]->ylabel("speed");
    axes[2]->xlabel("t");
    axes[2]->ylabel("yaw");
    axes[3]->xlabel("t");
    axes[3]->ylabel("position error");
    axes[4]->xlabel("t");
    axes[4]->ylabel("delta");
    axes[5]->xlabel("t");
    axes[5]->ylabel("u, v, r");
    return axes;
}

} // namespace

namespace basjoo::bicycle {

namespace {

template <std::floating_point T>
inline constexpr T kFrontWheelbase{static_cast<T>(1.2)};
template <std::floating_point T>
inline constexpr T kRearWheelbase{static_cast<T>(1.4)};
template <std::floating_point T>
inline constexpr T kWheelbase{kFrontWheelbase<T> + kRearWheelbase<T>};
template <std::floating_point T>
inline constexpr T kInitialYaw{static_cast<T>(0.3)};

/**
 * @brief One sample of the analytic solution of the model.
 *
 * speed is the total velocity of the center of gravity, i.e. what State::speed() answers, rather
 * than the longitudinal component the acceleration control acts on.
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
 * Deliberately spelled out from the literature rather than taken from State::slipAngle(), so that
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
 * @brief Gathers the flat position and velocity fields of a state back into a Vec2.
 *
 * State stores its seven scalars flat and keeps its velocity in the body frame, while the
 * reference solution below is written in terms of planar vectors in the world frame, so these two
 * keep the assertions reading the way the geometry does.
 */
template <std::floating_point T>
[[nodiscard]] auto centerOf(const State<T>& state) noexcept -> ::basjoo::math::Vec2<T> {
    return ::basjoo::math::Vec2<T>{state.x, state.y};
}

template <std::floating_point T>
[[nodiscard]] auto worldVelocityOf(const State<T>& state) noexcept -> ::basjoo::math::Vec2<T> {
    return ::basjoo::math::Vec2<T>{
        state.u * std::cos(state.yaw) - state.v * std::sin(state.yaw),
        state.u * std::sin(state.yaw) + state.v * std::cos(state.yaw)
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
 */
template <std::floating_point T>
[[nodiscard]] auto distanceOf(const State<T>& lhs, const State<T>& rhs) noexcept -> T {
    T result{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < State<T>::size(); ++i) {
        result = std::max(result, std::abs(lhs.data()[i] - rhs.data()[i]));
    }
    return result;
}

template <std::floating_point T>
struct Trajectory final {
    std::vector<T> ts;
    std::vector<State<T>> runge_kutta;
    std::vector<State<T>> forward_euler;
    std::vector<State<T>> symplectic_euler;
    std::vector<Sample<T>> reference;
};

/**
 * @brief Marches every integrator step by step.
 */
template <std::floating_point T>
[[nodiscard]] auto simulate(
    const KinematicsModel<T>& model, const State<T>& initial_state, std::span<const T> accels,
    std::span<const T> omegas, std::span<const T> hs
) -> Trajectory<T> {
    const std::size_t size{accels.size()};
    Trajectory<T> trajectory;
    trajectory.ts.reserve(size + 1);
    trajectory.runge_kutta.reserve(size + 1);
    trajectory.forward_euler.reserve(size + 1);
    trajectory.symplectic_euler.reserve(size + 1);
    trajectory.ts.push_back(static_cast<T>(0.0));
    trajectory.runge_kutta.push_back(initial_state);
    trajectory.forward_euler.push_back(initial_state);
    trajectory.symplectic_euler.push_back(initial_state);
    for (std::size_t i{0}; i < size; ++i) {
        trajectory.ts.push_back(trajectory.ts.back() + hs[i]);
        trajectory.runge_kutta.push_back(
            model.rungeKutta(trajectory.runge_kutta.back(), accels[i], omegas[i], hs[i])
        );
        trajectory.forward_euler.push_back(
            model.forwardEuler(trajectory.forward_euler.back(), accels[i], omegas[i], hs[i])
        );
        trajectory.symplectic_euler.push_back(
            model.symplecticEuler(trajectory.symplectic_euler.back(), accels[i], omegas[i], hs[i])
        );
    }
    return trajectory;
}

/**
 * @brief Samples the analytic solution alongside an already marched trajectory.
 *
 * @param reference_at maps a step index in [0, accels.size()] onto the analytic solution there.
 */
template <std::floating_point T, typename ReferenceFn>
auto fillReference(Trajectory<T>& trajectory, const ReferenceFn& reference_at) -> void {
    const std::size_t size{trajectory.ts.size()};
    trajectory.reference.reserve(size);
    for (std::size_t i{0}; i < size; ++i) {
        trajectory.reference.push_back(reference_at(i));
    }
}

/**
 * @brief Asserts the algebraic relations the kinematic model keeps along any trajectory.
 *
 * The steering angle and the longitudinal velocity are driven straight by the two controls, so
 * they have to come back exactly where accumulating the piecewise constant controls by hand puts
 * them. This is what carries the scenarios that have no closed form solution. The remaining two
 * velocity components are not integrated at all but rebuilt from u and delta after every step, so
 * they have to sit on the constraint exactly rather than within a tolerance.
 */
template <std::floating_point T>
auto checkInvariants(
    const std::vector<State<T>>& states, const State<T>& initial_state, std::span<const T> accels,
    std::span<const T> omegas, std::span<const T> hs, T tolerance
) -> void {
    const std::size_t size{accels.size()};
    T delta{initial_state.delta};
    T u{initial_state.u};
    T max_delta_error{static_cast<T>(0.0)};
    T max_u_error{static_cast<T>(0.0)};
    bool on_constraint{true};
    for (std::size_t i{0}; i <= size; ++i) {
        const State<T>& state{states[i]};
        max_delta_error = std::max(max_delta_error, std::abs(state.delta - delta));
        max_u_error = std::max(max_u_error, std::abs(state.u - u));
        on_constraint = on_constraint &&
                        state.r == state.u * std::tan(state.delta) / kWheelbase<T> &&
                        state.v == kRearWheelbase<T> * state.r;
        if (i < size) {
            delta += omegas[i] * hs[i];
            u += accels[i] * hs[i];
        }
    }
    CHECK_LE(max_delta_error, tolerance);
    CHECK_LE(max_u_error, tolerance);
    CHECK(on_constraint);
}

/**
 * @brief Asserts that Runge-Kutta tracks the analytic solution.
 */
template <std::floating_point T>
auto checkAgainstAnalytic(const Trajectory<T>& trajectory, T symplectic_euler_bound) -> void {
    const T tolerance{rungeKuttaTolerance<T>()};
    const std::size_t size{trajectory.ts.size()};
    T max_position_error{static_cast<T>(0.0)};
    T max_yaw_error{static_cast<T>(0.0)};
    T max_speed_error{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < size; ++i) {
        const State<T>& state{trajectory.runge_kutta[i]};
        const Sample<T>& reference{trajectory.reference[i]};
        max_position_error = std::max(max_position_error, positionErrorOf(state, reference));
        max_yaw_error = std::max(max_yaw_error, std::abs(state.yaw - reference.yaw));
        max_speed_error = std::max(max_speed_error, std::abs(state.speed() - reference.speed));
    }
    CHECK_LE(max_position_error, tolerance);
    CHECK_LE(max_yaw_error, tolerance);
    CHECK_LE(max_speed_error, tolerance);

    // symplecticEuler() is only first order, so it gets a loose per scenario bound rather than the
    // Runge-Kutta tolerance. It still has to track the analytic solution: without this an
    // implementation that returned the state untouched would sail through the batch check below.
    T max_symplectic_error{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < size; ++i) {
        max_symplectic_error = std::max(
            max_symplectic_error,
            positionErrorOf(trajectory.symplectic_euler[i], trajectory.reference[i])
        );
    }
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
template <std::floating_point T, typename AccelFn, typename OmegaFn>
auto checkConvergenceOrder(
    const KinematicsModel<T>& model, const State<T>& initial_state, T duration,
    std::size_t base_steps, const AccelFn& accel_at, const OmegaFn& omega_at
) -> void {
    const auto run = [&](std::size_t steps) -> std::pair<State<T>, State<T>> {
        const T h{duration / static_cast<T>(steps)};
        const std::vector<T> hs(steps, h);
        std::vector<T> accels, omegas;
        accels.reserve(steps);
        omegas.reserve(steps);
        for (std::size_t i{0}; i < steps; ++i) {
            const T fraction{static_cast<T>(i) / static_cast<T>(steps)};
            accels.push_back(accel_at(fraction));
            omegas.push_back(omega_at(fraction));
        }
        return {
            model.rungeKutta(initial_state, accels, omegas, hs),
            model.forwardEuler(initial_state, accels, omegas, hs)
        };
    };

    const auto [coarse_runge_kutta, coarse_euler] = run(base_steps);
    const auto [medium_runge_kutta, medium_euler] = run(base_steps * 2);
    const auto [fine_runge_kutta, fine_euler] = run(base_steps * 4);

    const T runge_kutta_ratio{
        distanceOf(coarse_runge_kutta, medium_runge_kutta) /
        distanceOf(medium_runge_kutta, fine_runge_kutta)
    };
    CHECK_GT(runge_kutta_ratio, static_cast<T>(12.0));
    CHECK_LT(runge_kutta_ratio, static_cast<T>(24.0));

    const T euler_ratio{
        distanceOf(coarse_euler, medium_euler) / distanceOf(medium_euler, fine_euler)
    };
    CHECK_GT(euler_ratio, static_cast<T>(1.5));
    CHECK_LT(euler_ratio, static_cast<T>(3.0));
}

auto plotTrajectory(const std::string& title, const Trajectory<double>& trajectory) -> void {
    const std::size_t size{trajectory.ts.size()};
    const bool has_reference{!trajectory.reference.empty()};
    std::vector<double> reference_xs, reference_ys, runge_kutta_xs, runge_kutta_ys, euler_xs,
        euler_ys, symplectic_xs, symplectic_ys, runge_kutta_speeds, euler_speeds, symplectic_speeds,
        reference_speeds, runge_kutta_yaws, euler_yaws, symplectic_yaws, reference_yaws,
        runge_kutta_errors, euler_errors, symplectic_errors, deltas, us, vs, rs;
    for (std::vector<double>* vec :
         {&reference_xs,
          &reference_ys,
          &runge_kutta_xs,
          &runge_kutta_ys,
          &euler_xs,
          &euler_ys,
          &symplectic_xs,
          &symplectic_ys,
          &runge_kutta_speeds,
          &euler_speeds,
          &symplectic_speeds,
          &reference_speeds,
          &runge_kutta_yaws,
          &euler_yaws,
          &symplectic_yaws,
          &reference_yaws,
          &runge_kutta_errors,
          &euler_errors,
          &symplectic_errors,
          &deltas,
          &us,
          &vs,
          &rs}) {
        vec->reserve(size);
    }
    for (std::size_t i{0}; i < size; ++i) {
        const State<double>& runge_kutta{trajectory.runge_kutta[i]};
        const State<double>& euler{trajectory.forward_euler[i]};
        const State<double>& symplectic{trajectory.symplectic_euler[i]};
        if (has_reference) {
            const Sample<double>& reference{trajectory.reference[i]};
            reference_xs.push_back(reference.center.x);
            reference_ys.push_back(reference.center.y);
            reference_speeds.push_back(reference.speed);
            reference_yaws.push_back(reference.yaw);
            runge_kutta_errors.push_back(positionErrorOf(runge_kutta, reference));
            euler_errors.push_back(positionErrorOf(euler, reference));
            symplectic_errors.push_back(positionErrorOf(symplectic, reference));
        }
        runge_kutta_xs.push_back(runge_kutta.x);
        runge_kutta_ys.push_back(runge_kutta.y);
        runge_kutta_speeds.push_back(runge_kutta.speed());
        runge_kutta_yaws.push_back(runge_kutta.yaw);
        deltas.push_back(runge_kutta.delta);
        us.push_back(runge_kutta.u);
        vs.push_back(runge_kutta.v);
        rs.push_back(runge_kutta.r);
        euler_xs.push_back(euler.x);
        euler_ys.push_back(euler.y);
        euler_speeds.push_back(euler.speed());
        euler_yaws.push_back(euler.yaw);
        symplectic_xs.push_back(symplectic.x);
        symplectic_ys.push_back(symplectic.y);
        symplectic_speeds.push_back(symplectic.speed());
        symplectic_yaws.push_back(symplectic.yaw);
    }

    matplot::figure_handle fig = createFigureHandle();
    fig->title(title);
    std::vector<matplot::axes_handle> axes = createAxesHandles(fig);
    const std::vector<std::string> legend{
        "analytic", "runge kutta", "forward euler", "symplectic euler"
    };
    const std::vector<std::string> legend_without_reference{
        "runge kutta", "forward euler", "symplectic euler"
    };
    if (has_reference) {
        axes[0]->plot(reference_xs, reference_ys, "-")->line_width(2.0);
        axes[1]->plot(trajectory.ts, reference_speeds, "-")->line_width(2.0);
        axes[2]->plot(trajectory.ts, reference_yaws, "-")->line_width(2.0);
        axes[3]->semilogy(trajectory.ts, runge_kutta_errors, "--")->line_width(2.0);
        axes[3]->semilogy(trajectory.ts, euler_errors, ":")->line_width(2.0);
        axes[3]->semilogy(trajectory.ts, symplectic_errors, "-.")->line_width(2.0);
        axes[3]->legend(legend_without_reference);
    }
    axes[0]->plot(runge_kutta_xs, runge_kutta_ys, "--")->line_width(2.0);
    axes[0]->plot(euler_xs, euler_ys, ":")->line_width(2.0);
    axes[0]->plot(symplectic_xs, symplectic_ys, "-.")->line_width(2.0);
    axes[0]->axis(matplot::equal);
    axes[0]->legend(has_reference ? legend : legend_without_reference);
    axes[1]->plot(trajectory.ts, runge_kutta_speeds, "--")->line_width(2.0);
    axes[1]->plot(trajectory.ts, euler_speeds, ":")->line_width(2.0);
    axes[1]->plot(trajectory.ts, symplectic_speeds, "-.")->line_width(2.0);
    axes[1]->legend(has_reference ? legend : legend_without_reference);
    axes[2]->plot(trajectory.ts, runge_kutta_yaws, "--")->line_width(2.0);
    axes[2]->plot(trajectory.ts, euler_yaws, ":")->line_width(2.0);
    axes[2]->plot(trajectory.ts, symplectic_yaws, "-.")->line_width(2.0);
    axes[2]->legend(has_reference ? legend : legend_without_reference);
    axes[4]->plot(trajectory.ts, deltas, "-")->line_width(2.0);
    axes[5]->plot(trajectory.ts, us, "-")->line_width(2.0);
    axes[5]->plot(trajectory.ts, vs, "--")->line_width(2.0);
    axes[5]->plot(trajectory.ts, rs, ":")->line_width(2.0);
    axes[5]->legend({"u", "v", "r"});
    fig->show();
}

} // namespace

// Scenarios with a zero steering rate keep a constant steering angle, which puts the center of
// gravity on an arc of constant curvature and leaves a closed form solution to compare against.

TEST_CASE_TEMPLATE("StraightLineAccelerateThenDecelerate", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(5.0)};
    constexpr T kAccel{static_cast<T>(2.0)};
    constexpr T kPhaseDuration{static_cast<T>(6.0)};
    constexpr std::size_t kStepsPerPhase{1200};
    const T h{kPhaseDuration / static_cast<T>(kStepsPerPhase)};

    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, kDelta, kInitialU
    )};
    const std::vector<T> hs(kStepsPerPhase * 2, h);
    std::vector<T> accels(kStepsPerPhase * 2, kAccel);
    std::fill(accels.begin() + kStepsPerPhase, accels.end(), -kAccel);
    const std::vector<T> omegas(kStepsPerPhase * 2, static_cast<T>(0.0));

    Trajectory<T> trajectory{simulate<T>(model, initial_state, accels, omegas, hs)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, initial_state.speed()};
    const Sample<T> midpoint{analyticAt(start, kAccel, kDelta, kPhaseDuration)};
    fillReference<T>(trajectory, [&](std::size_t i) -> Sample<T> {
        const T t{h * static_cast<T>(i)};
        return i <= kStepsPerPhase ? analyticAt(start, kAccel, kDelta, t)
                                   : analyticAt(midpoint, -kAccel, kDelta, t - kPhaseDuration);
    });

    // symplecticEuler() is first order and lands about 0.03 m off over this run, so the
    // bound leaves roughly a factor of two of room while still failing outright on an
    // implementation that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(0.06)};
    checkAgainstAnalytic<T>(trajectory, kSymplecticEulerBound);
    checkInvariants<T>(
        trajectory.runge_kutta, initial_state, accels, omegas, hs, rungeKuttaTolerance<T>()
    );

    // A zero steering angle has to leave the heading untouched and the vehicle on its axis
    // throughout.
    for (const State<T>& state : trajectory.runge_kutta) {
        CHECK_EQ(state.yaw, doctest::Approx(kInitialYaw<T>).epsilon(rungeKuttaTolerance<T>()));
        CHECK_EQ(state.delta, static_cast<T>(0.0));
        CHECK_LE(std::abs(state.v), rungeKuttaTolerance<T>());
        CHECK_LE(std::abs(state.r), rungeKuttaTolerance<T>());
    }
    // The speed comes back to where it started after the symmetric acceleration profile.
    CHECK_EQ(
        trajectory.runge_kutta.back().speed(),
        doctest::Approx(kInitialU).epsilon(rungeKuttaTolerance<T>())
    );

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("straight line accelerate then decelerate", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("ConstantSpeedCircle", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
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

    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, kDelta, kInitialU
    )};
    const T initial_speed{initial_state.speed()};
    const T period{static_cast<T>(2.0) * std::numbers::pi_v<T> / (kappa * initial_speed)};
    const T h{period / static_cast<T>(kNumSteps)};
    const std::vector<T> hs(kNumSteps, h);
    const std::vector<T> accels(kNumSteps, kAccel);
    const std::vector<T> omegas(kNumSteps, static_cast<T>(0.0));

    Trajectory<T> trajectory{simulate<T>(model, initial_state, accels, omegas, hs)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, initial_speed};
    fillReference<T>(trajectory, [&](std::size_t i) -> Sample<T> {
        return analyticAt(start, kAccel, kDelta, h * static_cast<T>(i));
    });

    // symplecticEuler() is first order and lands about 0.48 m off over this run, so the
    // bound leaves roughly a factor of two of room while still failing outright on an
    // implementation that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(1.0)};
    checkAgainstAnalytic<T>(trajectory, kSymplecticEulerBound);
    checkInvariants<T>(
        trajectory.runge_kutta, initial_state, accels, omegas, hs, rungeKuttaTolerance<T>()
    );

    // Every sample has to sit on the circle centered at the instantaneous center of rotation.
    const T course{kInitialYaw<T> + slipAngleOf(kDelta)};
    const ::basjoo::math::Vec2<T> rotation_center{
        start.center.x - radius * std::sin(course), start.center.y + radius * std::cos(course)
    };
    for (const State<T>& state : trajectory.runge_kutta) {
        CHECK_EQ(
            centerOf(state).euclideanTo(rotation_center),
            doctest::Approx(radius).epsilon(rungeKuttaTolerance<T>())
        );
        CHECK_EQ(
            worldVelocityOf(state).euclidean(),
            doctest::Approx(initial_speed).epsilon(rungeKuttaTolerance<T>())
        );
        CHECK_EQ(state.r, doctest::Approx(kappa * initial_speed).epsilon(rungeKuttaTolerance<T>()));
    }
    // One full period brings the vehicle back to where it started.
    CHECK_LE(
        centerOf(trajectory.runge_kutta.back()).euclideanTo(start.center), rungeKuttaTolerance<T>()
    );
    CHECK_EQ(
        trajectory.runge_kutta.back().yaw,
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
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kDelta{static_cast<T>(0.15)};
    constexpr T kInitialU{static_cast<T>(8.0)};
    constexpr T kAccel{static_cast<T>(1.5)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, kDelta, kInitialU
    )};
    const std::vector<T> hs(kNumSteps, h);
    const std::vector<T> accels(kNumSteps, kAccel);
    const std::vector<T> omegas(kNumSteps, static_cast<T>(0.0));

    Trajectory<T> trajectory{simulate<T>(model, initial_state, accels, omegas, hs)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, initial_state.speed()};
    fillReference<T>(trajectory, [&](std::size_t i) -> Sample<T> {
        return analyticAt(start, kAccel, kDelta, h * static_cast<T>(i));
    });

    // symplecticEuler() is first order and lands about 0.29 m off over this run, so the
    // bound leaves roughly a factor of two of room while still failing outright on an
    // implementation that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(0.6)};
    checkAgainstAnalytic<T>(trajectory, kSymplecticEulerBound);
    checkInvariants<T>(
        trajectory.runge_kutta, initial_state, accels, omegas, hs, rungeKuttaTolerance<T>()
    );

    // Accelerating through a turn is where the acceleration has to stay along the body axis rather
    // than along the course angle: resolving it along the course instead leaves the velocity
    // drifting off the slip angle the steering geometry dictates, an error that neither the
    // straight line nor the constant speed circle scenario can expose.
    const T beta{slipAngleOf(kDelta)};
    const T kappa{curvatureOf(kDelta)};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const State<T>& state{trajectory.runge_kutta[i]};
        const T t{trajectory.ts[i]};
        const T speed{initial_state.speed() + kAccel / std::cos(beta) * t};
        CHECK_EQ(
            worldVelocityOf(state).euclidean(),
            doctest::Approx(speed).epsilon(rungeKuttaTolerance<T>())
        );
        CHECK_EQ(state.slipAngle(), doctest::Approx(beta).epsilon(rungeKuttaTolerance<T>()));
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
        CHECK_EQ(state.r, doctest::Approx(kappa * speed).epsilon(rungeKuttaTolerance<T>()));
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("accelerating turn", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("DecelerateThroughZeroIntoReverse", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kDelta{static_cast<T>(0.15)};
    constexpr T kInitialU{static_cast<T>(12.0)};
    constexpr T kAccel{static_cast<T>(-2.0)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, kDelta, kInitialU
    )};
    const std::vector<T> hs(kNumSteps, h);
    const std::vector<T> accels(kNumSteps, kAccel);
    const std::vector<T> omegas(kNumSteps, static_cast<T>(0.0));

    Trajectory<T> trajectory{simulate<T>(model, initial_state, accels, omegas, hs)};
    const Sample<T> start{centerOf(initial_state), kInitialYaw<T>, initial_state.speed()};
    fillReference<T>(trajectory, [&](std::size_t i) -> Sample<T> {
        return analyticAt(start, kAccel, kDelta, h * static_cast<T>(i));
    });

    // symplecticEuler() is first order and lands about 0.66 m off over this run, so the
    // bound leaves roughly a factor of two of room while still failing outright on an
    // implementation that does not integrate.
    constexpr T kSymplecticEulerBound{static_cast<T>(1.4)};
    checkAgainstAnalytic<T>(trajectory, kSymplecticEulerBound);
    checkInvariants<T>(
        trajectory.runge_kutta, initial_state, accels, omegas, hs, rungeKuttaTolerance<T>()
    );

    // The speed runs from +12 down through zero into reverse, so the run crosses the one point
    // where the velocity vector carries no direction at all. Taking the sign from the longitudinal
    // component is what lets the trajectory continue into reverse instead of bouncing back: the
    // magnitude alone leaves the vehicle turning the wrong way once it reverses.
    const T beta{slipAngleOf(kDelta)};
    const T kappa{curvatureOf(kDelta)};
    bool saw_forward{false};
    bool saw_reverse{false};
    for (std::size_t i{0}; i < trajectory.ts.size(); ++i) {
        const State<T>& state{trajectory.runge_kutta[i]};
        const T speed{initial_state.speed() + kAccel / std::cos(beta) * trajectory.ts[i]};
        CHECK_EQ(state.speed(), doctest::Approx(speed).epsilon(rungeKuttaTolerance<T>()));
        CHECK_EQ(state.r, doctest::Approx(kappa * speed).epsilon(rungeKuttaTolerance<T>()));
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
            CHECK_LT(state.r, static_cast<T>(0.0));
        } else {
            // Around the reversal the velocity vector shrinks to zero and carries no direction
            // worth asserting on, so only the signed speed above is checked in that band.
        }
    }
    CHECK(saw_forward);
    CHECK(saw_reverse);
    CHECK_LT(trajectory.runge_kutta.back().speed(), static_cast<T>(0.0));

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
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kInitialDelta{static_cast<T>(0.0)};
    constexpr T kInitialU{static_cast<T>(8.0)};
    constexpr T kAccel{static_cast<T>(0.0)};
    constexpr T kOmega{static_cast<T>(0.02)};
    constexpr T kDuration{static_cast<T>(12.0)};
    constexpr std::size_t kNumSteps{2400};
    const T h{kDuration / static_cast<T>(kNumSteps)};

    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, kInitialDelta, kInitialU
    )};
    const std::vector<T> hs(kNumSteps, h);
    const std::vector<T> accels(kNumSteps, kAccel);
    const std::vector<T> omegas(kNumSteps, kOmega);

    const Trajectory<T> trajectory{simulate<T>(model, initial_state, accels, omegas, hs)};
    checkInvariants<T>(
        trajectory.runge_kutta, initial_state, accels, omegas, hs, rungeKuttaTolerance<T>()
    );

    // The steering angle ramps linearly, so the curvature grows with it and the vehicle spirals
    // inwards: both the yaw rate and its rate of change have to keep increasing.
    const State<T>& final_state{trajectory.runge_kutta.back()};
    CHECK_EQ(
        final_state.delta,
        doctest::Approx(kInitialDelta + kOmega * kDuration).epsilon(rungeKuttaTolerance<T>())
    );
    CHECK_EQ(final_state.u, doctest::Approx(kInitialU).epsilon(rungeKuttaTolerance<T>()));
    CHECK_GT(final_state.r, trajectory.runge_kutta.front().r);
    for (std::size_t i{1}; i < trajectory.runge_kutta.size(); ++i) {
        CHECK_GT(trajectory.runge_kutta[i].r, trajectory.runge_kutta[i - 1].r);
    }

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder<T>(
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
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
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
    const std::vector<T> hs(kNumSteps, h);
    const std::vector<T> accels(kNumSteps, kAccel);
    std::vector<T> omegas;
    omegas.reserve(kNumSteps);
    for (std::size_t i{0}; i < kNumSteps; ++i) {
        omegas.push_back(omega_at(static_cast<T>(i) / static_cast<T>(kNumSteps)));
    }

    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, kInitialDelta, kInitialU
    )};
    const Trajectory<T> trajectory{simulate<T>(model, initial_state, accels, omegas, hs)};
    checkInvariants<T>(
        trajectory.runge_kutta, initial_state, accels, omegas, hs, rungeKuttaTolerance<T>()
    );

    // The steering angle has to come back to where it started, having visited both sides on the
    // way, and the lateral velocity has to change sign with it.
    T max_delta{static_cast<T>(0.0)};
    T min_delta{static_cast<T>(0.0)};
    for (const State<T>& state : trajectory.runge_kutta) {
        max_delta = std::max(max_delta, state.delta);
        min_delta = std::min(min_delta, state.delta);
        // Driving forwards, the lateral velocity leans to whichever side the steering angle does.
        if (std::abs(state.delta) > static_cast<T>(0.01)) {
            CHECK_EQ(state.v > static_cast<T>(0.0), state.delta > static_cast<T>(0.0));
        }
    }
    CHECK_GT(max_delta, static_cast<T>(0.1));
    CHECK_LT(min_delta, static_cast<T>(-0.1));
    CHECK_LE(std::abs(trajectory.runge_kutta.back().delta), rungeKuttaTolerance<T>());
    CHECK_EQ(
        trajectory.runge_kutta.back().u,
        doctest::Approx(kInitialU + kAccel * kDuration).epsilon(rungeKuttaTolerance<T>())
    );

    if constexpr (std::same_as<T, double>) {
        checkConvergenceOrder<T>(
            model, initial_state, kDuration, 300, [](T) -> T { return kAccel; }, omega_at
        );
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotTrajectory("accelerating steering reversal", trajectory);
        }
    }
}

TEST_CASE_TEMPLATE("StateHeldControlOverloads", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kStateAccel{static_cast<T>(-0.6)};
    constexpr T kStateOmega{static_cast<T>(0.03)};
    constexpr T kAccel{static_cast<T>(1.2)};
    constexpr T kOmega{static_cast<T>(-0.05)};
    constexpr T kH{static_cast<T>(0.01)};
    constexpr std::size_t kNumSteps{200};
    const T tolerance{rungeKuttaTolerance<T>()};

    // The controls held by the state deliberately differ from the ones passed explicitly, so an
    // overload that read the wrong pair cannot pass.
    const State<T> initial_state{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
        static_cast<T>(8.0), kStateAccel, kStateOmega
    )};
    State<T> held_state{initial_state};
    held_state.a = kAccel;
    held_state.omega = kOmega;

    SUBCASE("SingleStepReadsControlsFromState") {
        // u and delta are driven by the two controls alone, so a single step reveals which pair
        // was used, while the pair itself has to come out untouched.
        for (const State<T>& stepped :
             {model.forwardEuler(initial_state, kH), model.symplecticEuler(initial_state, kH),
              model.rungeKutta(initial_state, kH)}) {
            CHECK_EQ(stepped.a, kStateAccel);
            CHECK_EQ(stepped.omega, kStateOmega);
            CHECK_EQ(
                stepped.u, doctest::Approx(initial_state.u + kStateAccel * kH).epsilon(tolerance)
            );
            CHECK_EQ(
                stepped.delta,
                doctest::Approx(initial_state.delta + kStateOmega * kH).epsilon(tolerance)
            );
        }
    }

    SUBCASE("BatchAgreesWithMarchingSingleSteps") {
        const std::vector<T> hs(kNumSteps, kH);
        const std::vector<T> accels(kNumSteps, kAccel);
        const std::vector<T> omegas(kNumSteps, kOmega);
        State<T> forward_euler{held_state};
        State<T> symplectic_euler{held_state};
        State<T> runge_kutta{held_state};
        for (std::size_t i{0}; i < kNumSteps; ++i) {
            forward_euler = model.forwardEuler(forward_euler, kH);
            symplectic_euler = model.symplecticEuler(symplectic_euler, kH);
            runge_kutta = model.rungeKutta(runge_kutta, kH);
        }
        CHECK_LE(distanceOf(model.forwardEuler(held_state, hs), forward_euler), tolerance);
        CHECK_LE(distanceOf(model.symplecticEuler(held_state, hs), symplectic_euler), tolerance);
        CHECK_LE(distanceOf(model.rungeKutta(held_state, hs), runge_kutta), tolerance);
        CHECK_LE(
            distanceOf(model.forwardEuler(initial_state, accels, omegas, hs), forward_euler),
            tolerance
        );
        CHECK_LE(
            distanceOf(model.symplecticEuler(initial_state, accels, omegas, hs), symplectic_euler),
            tolerance
        );
        CHECK_LE(
            distanceOf(model.rungeKutta(initial_state, accels, omegas, hs), runge_kutta), tolerance
        );
    }

    SUBCASE("BatchTraceRecordsEveryStep") {
        // Marching the single step overloads by hand gives the state after every step, which is
        // exactly what the trace parameter is supposed to collect: one entry per span element,
        // none of them the initial state.
        const std::vector<T> hs(5, kH);
        const std::vector<T> accels{
            kAccel, -kAccel, static_cast<T>(0.4), kAccel, static_cast<T>(0.0)
        };
        const std::vector<T> omegas{
            kOmega, static_cast<T>(0.0), static_cast<T>(0.07), -kOmega, static_cast<T>(0.0)
        };
        const auto step = [&](int integrator, const State<T>& state, T h) -> State<T> {
            return integrator == 0   ? model.forwardEuler(state, h)
                   : integrator == 1 ? model.symplecticEuler(state, h)
                                     : model.rungeKutta(state, h);
        };
        // The controls held by held_state stay the same over every step, so the hand rolled loop
        // is handed no schedule at all in that half of the check.
        const auto march = [&](int integrator, bool scheduled) -> std::vector<State<T>> {
            std::vector<State<T>> result;
            State<T> state{scheduled ? initial_state : held_state};
            for (std::size_t i{0}; i < hs.size(); ++i) {
                if (scheduled) {
                    state.a = accels[i];
                    state.omega = omegas[i];
                }
                state = step(integrator, state, hs[i]);
                result.push_back(state);
            }
            return result;
        };
        const auto batch = [&](int integrator, bool scheduled,
                               std::vector<State<T>>& trace) -> State<T> {
            const auto out{std::back_inserter(trace)};
            if (scheduled) {
                return integrator == 0 ? model.forwardEuler(initial_state, accels, omegas, hs, out)
                       : integrator == 1
                           ? model.symplecticEuler(initial_state, accels, omegas, hs, out)
                           : model.rungeKutta(initial_state, accels, omegas, hs, out);
            }
            return integrator == 0   ? model.forwardEuler(held_state, hs, out)
                   : integrator == 1 ? model.symplecticEuler(held_state, hs, out)
                                     : model.rungeKutta(held_state, hs, out);
        };
        for (int integrator{0}; integrator < 3; ++integrator) {
            for (const bool scheduled : {false, true}) {
                std::vector<State<T>> trace;
                const State<T> returned{batch(integrator, scheduled, trace)};
                const std::vector<State<T>> expected{march(integrator, scheduled)};
                REQUIRE_EQ(trace.size(), expected.size());
                for (std::size_t i{0}; i < trace.size(); ++i) {
                    CHECK_LE(distanceOf(trace[i], expected[i]), tolerance);
                }
                CHECK_LE(distanceOf(trace.back(), returned), tolerance);
            }
        }
    }
}

TEST_CASE_TEMPLATE("VelocitiesAreRebuiltFromTheSteeringGeometry", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kH{static_cast<T>(0.01)};
    const State<T> on_manifold{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
        static_cast<T>(8.0), static_cast<T>(-0.6), static_cast<T>(0.03)
    )};
    // v and r are not integrated, so whatever a caller happens to leave in them cannot matter.
    State<T> garbled{on_manifold};
    garbled.v = static_cast<T>(12.3);
    garbled.r = static_cast<T>(-4.5);

    SUBCASE("TheLateralVelocityAndYawRateHandedInAreIgnored") {
        // A model that used the v and r handed in would move a step by about h times them, which is
        // orders of magnitude above the roundoff bound.
        const T tolerance{roundoffTolerance<T>()};
        CHECK_EQ(
            distanceOf(model.forwardEuler(garbled, kH), model.forwardEuler(on_manifold, kH)),
            doctest::Approx(0.0).epsilon(tolerance)
        );
        CHECK_EQ(
            distanceOf(model.symplecticEuler(garbled, kH), model.symplecticEuler(on_manifold, kH)),
            doctest::Approx(0.0).epsilon(tolerance)
        );
        CHECK_EQ(
            distanceOf(model.rungeKutta(garbled, kH), model.rungeKutta(on_manifold, kH)),
            doctest::Approx(0.0).epsilon(tolerance)
        );
        CHECK_EQ(
            distanceOf(model.tangent(garbled), model.tangent(on_manifold)),
            doctest::Approx(0.0).epsilon(tolerance)
        );
    }

    SUBCASE("EveryStateComesBackOnTheConstraint") {
        for (const State<T>& state :
             {on_manifold, model.forwardEuler(garbled, kH), model.symplecticEuler(garbled, kH),
              model.rungeKutta(garbled, kH)}) {
            CHECK_EQ(state.r, state.u * std::tan(state.delta) / kWheelbase<T>);
            CHECK_EQ(state.v, kRearWheelbase<T> * state.r);
        }
    }

    SUBCASE("TangentCarriesOnlyTheFiveIntegratedStates") {
        // v and r are rebuilt rather than integrated, and both controls are held over a step, so
        // four of the nine fields of the tangent are zero.
        const State<T> k{model.tangent(on_manifold)};
        CHECK_EQ(k.v, static_cast<T>(0.0));
        CHECK_EQ(k.r, static_cast<T>(0.0));
        CHECK_EQ(k.a, static_cast<T>(0.0));
        CHECK_EQ(k.omega, static_cast<T>(0.0));
    }
}

// The two controls are the one place the model says no: everything else it takes at face value,
// while a and omega are clamped into the bounds it was built with.

TEST_CASE_TEMPLATE("ControlBoundsClampEveryEntryPoint", T, float, double) {
    constexpr T kALo{static_cast<T>(-5.0)};
    constexpr T kAUp{static_cast<T>(3.0)};
    constexpr T kOmegaLo{static_cast<T>(-0.4)};
    constexpr T kOmegaUp{static_cast<T>(0.5)};
    constexpr T kH{static_cast<T>(0.01)};
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>, kALo, kAUp,
                                   kOmegaLo,           kOmegaUp};
    const KinematicsModel<T> unbounded{kFrontWheelbase<T>, kRearWheelbase<T>};
    const State<T> start{model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
        static_cast<T>(8.0)
    )};

    SUBCASE("BoundsComeBackOutAndDefaultToInfinite") {
        CHECK_EQ(model.a_lower_bound(), kALo);
        CHECK_EQ(model.a_upper_bound(), kAUp);
        CHECK_EQ(model.omega_lower_bound(), kOmegaLo);
        CHECK_EQ(model.omega_upper_bound(), kOmegaUp);
        CHECK_EQ(unbounded.a_lower_bound(), -std::numeric_limits<T>::infinity());
        CHECK_EQ(unbounded.a_upper_bound(), std::numeric_limits<T>::infinity());
        CHECK_EQ(unbounded.omega_lower_bound(), -std::numeric_limits<T>::infinity());
        CHECK_EQ(unbounded.omega_upper_bound(), std::numeric_limits<T>::infinity());
    }

    SUBCASE("EveryEntryPointClamps") {
        const State<T> created{model.createState(
            static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
            static_cast<T>(8.0), static_cast<T>(99.0), static_cast<T>(-99.0)
        )};
        CHECK_EQ(created.a, kAUp);
        CHECK_EQ(created.omega, kOmegaLo);

        // Nothing stops a caller from writing the controls by hand, so the integrators cannot
        // assume the pair they are handed is within the bounds.
        State<T> loose{start};
        loose.a = static_cast<T>(50.0);
        loose.omega = static_cast<T>(-7.0);
        State<T> clamped{loose};
        clamped.a = kAUp;
        clamped.omega = kOmegaLo;
        for (const State<T>& stepped :
             {model.forwardEuler(loose, kH), model.symplecticEuler(loose, kH),
              model.rungeKutta(loose, kH)}) {
            CHECK_EQ(stepped.a, kAUp);
            CHECK_EQ(stepped.omega, kOmegaLo);
        }
        CHECK_EQ(distanceOf(model.tangent(loose), model.tangent(clamped)), static_cast<T>(0.0));
        // The batch overloads inherit the clamp through the single step ones.
        const std::vector<T> hs(3, kH);
        CHECK_EQ(model.rungeKutta(loose, hs).a, kAUp);
        CHECK_EQ(model.rungeKutta(loose, hs).omega, kOmegaLo);
    }

    SUBCASE("ClampingEqualsSteppingTheClampedSchedule") {
        // However the clamp is spelled, it has to come to the same thing as handing an unbounded
        // model the already clamped schedule, step for step. The two walk the same arithmetic but
        // not the same code path, so they are only held to the accuracy the integrator itself is
        // guaranteed: the extra clamps shift which multiply adds the compiler fuses, which moves
        // the last bits of every step and compounds over hundreds of them.
        constexpr std::size_t kNumSteps{200};
        const std::vector<T> hs(kNumSteps, kH);
        std::vector<T> accels, omegas, clamped_accels, clamped_omegas;
        for (std::size_t i{0}; i < kNumSteps; ++i) {
            const T t{static_cast<T>(i) * kH};
            accels.push_back(static_cast<T>(8.0) * std::sin(t));
            omegas.push_back(static_cast<T>(1.2) * std::cos(t));
            clamped_accels.push_back(std::clamp(accels.back(), kALo, kAUp));
            clamped_omegas.push_back(std::clamp(omegas.back(), kOmegaLo, kOmegaUp));
        }
        CHECK_LE(
            distanceOf(
                model.forwardEuler(start, accels, omegas, hs),
                unbounded.forwardEuler(start, clamped_accels, clamped_omegas, hs)
            ),
            rungeKuttaTolerance<T>()
        );
        CHECK_LE(
            distanceOf(
                model.symplecticEuler(start, accels, omegas, hs),
                unbounded.symplecticEuler(start, clamped_accels, clamped_omegas, hs)
            ),
            rungeKuttaTolerance<T>()
        );
        CHECK_LE(
            distanceOf(
                model.rungeKutta(start, accels, omegas, hs),
                unbounded.rungeKutta(start, clamped_accels, clamped_omegas, hs)
            ),
            rungeKuttaTolerance<T>()
        );
    }

    SUBCASE("BoundsNothingEverReachesChangeNothing") {
        const KinematicsModel<T> wide{kFrontWheelbase<T>,     kRearWheelbase<T>,
                                      static_cast<T>(-1.0e3), static_cast<T>(1.0e3),
                                      static_cast<T>(-1.0e3), static_cast<T>(1.0e3)};
        const std::vector<T> hs(200, kH);
        const std::vector<T> accels(200, static_cast<T>(1.5));
        const std::vector<T> omegas(200, static_cast<T>(0.05));
        // Same reasoning as above: bounds nothing reaches must not move the trajectory, but the
        // clamps still stand between the two call sites.
        CHECK_LE(
            distanceOf(
                wide.rungeKutta(start, accels, omegas, hs),
                unbounded.rungeKutta(start, accels, omegas, hs)
            ),
            rungeKuttaTolerance<T>()
        );
    }
}

TEST_CASE_TEMPLATE("ModelSerializationRoundtrip", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>,  kRearWheelbase<T>,    static_cast<T>(-5.0),
                                   static_cast<T>(3.0), static_cast<T>(-0.4), static_cast<T>(0.5)};
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
    // The geometry no longer travels inside a State, so a restored model has to reproduce the
    // states of the original bit for bit.
    CHECK_EQ(
        restored.createState(
            static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
            static_cast<T>(8.0)
        ),
        model.createState(
            static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
            static_cast<T>(8.0)
        )
    );
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE("RejectsInvalidArguments", T, float, double) {
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    constexpr T kDelta{static_cast<T>(0.15)};
    constexpr T kOriginX{static_cast<T>(0.0)};
    constexpr T kOriginY{static_cast<T>(0.0)};

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
                kFrontWheelbase<T>, kRearWheelbase<T>, static_cast<T>(0.5), static_cast<T>(3.0),
                static_cast<T>(-0.4), static_cast<T>(0.5)
            }),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            (KinematicsModel<T>{
                kFrontWheelbase<T>, kRearWheelbase<T>, static_cast<T>(-5.0), static_cast<T>(3.0),
                static_cast<T>(0.1), static_cast<T>(0.5)
            }),
            std::invalid_argument
        );
    }

    SUBCASE("CreateStateRejectsSingularSteeringAngle") {
        CHECK_THROWS_AS(
            static_cast<void>(model.createState(
                kOriginX, kOriginY, kInitialYaw<T>, std::numbers::pi_v<T> * static_cast<T>(0.5),
                static_cast<T>(8.0)
            )),
            std::invalid_argument
        );
    }

    SUBCASE("IntegratorsRejectNonPositiveTimeStep") {
        const State<T> state{
            model.createState(kOriginX, kOriginY, kInitialYaw<T>, kDelta, static_cast<T>(8.0))
        };
        CHECK_THROWS_AS(
            static_cast<void>(
                model.forwardEuler(state, static_cast<T>(1.0), kDelta, static_cast<T>(0.0))
            ),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(
                model.rungeKutta(state, static_cast<T>(1.0), kDelta, static_cast<T>(-0.01))
            ),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(model.symplecticEuler(state, static_cast<T>(0.0))),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(model.rungeKutta(state, static_cast<T>(-0.01))), std::invalid_argument
        );
    }

    SUBCASE("IntegratorsRejectSingularSteeringAngle") {
        // The steering angle is carried by the state now, so this is where an omega that drove it
        // out of range over an earlier step surfaces.
        State<T> state{
            model.createState(kOriginX, kOriginY, kInitialYaw<T>, kDelta, static_cast<T>(8.0))
        };
        state.delta = std::numbers::pi_v<T> * static_cast<T>(0.5);
        CHECK_THROWS_AS(
            static_cast<void>(model.rungeKutta(
                state, static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.01)
            )),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(model.symplecticEuler(
                state, static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.01)
            )),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(model.forwardEuler(state, static_cast<T>(0.01))),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(model.rungeKutta(state, static_cast<T>(0.01))), std::invalid_argument
        );
    }

    SUBCASE("BatchOverloadsRejectMismatchedSpanSizes") {
        const State<T> state{
            model.createState(kOriginX, kOriginY, kInitialYaw<T>, kDelta, static_cast<T>(8.0))
        };
        const std::vector<T> hs(4, static_cast<T>(0.01));
        const std::vector<T> accels(3, static_cast<T>(1.0));
        const std::vector<T> omegas(4, static_cast<T>(0.0));
        CHECK_THROWS_AS(
            static_cast<void>(model.forwardEuler(
                state, std::span<const T>{accels}, std::span<const T>{omegas},
                std::span<const T>{hs}
            )),
            std::invalid_argument
        );
        CHECK_THROWS_AS(
            static_cast<void>(model.rungeKutta(
                state, std::span<const T>{accels}, std::span<const T>{omegas},
                std::span<const T>{hs}
            )),
            std::invalid_argument
        );
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
