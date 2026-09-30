/**
 * @file test_kinematics_flow_controller.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit test of KinematicsFlowController, with a fixed and with an adaptive horizon.
 * @version 0.1
 * @date 2026-08-05
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/bicycle/controllers/kinematics_flow_controller.hpp"

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

#include "basjoo/bicycle/models/kinematics_model.hpp"
#include "basjoo/math/concepts.hpp"
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

auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    const std::array<std::pair<const char*, const char*>, 9> labels{
        {{"x", "y"},
         {"t", "x"},
         {"t", "y"},
         {"t", "yaw"},
         {"t", "delta"},
         {"t", "u"},
         {"t", "a"},
         {"t", "omega"},
         {"t", "tau"}}
    };
    std::vector<matplot::axes_handle> axes;
    axes.reserve(labels.size());
    for (std::size_t i = 0; i < labels.size(); ++i) {
        axes.emplace_back(fig->add_subplot(3, 3, i));
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
// The horizon of a fixed controller and the one an adaptive closed loop run starts with.
template <std::floating_point T>
inline constexpr T kHorizon{static_cast<T>(0.15)};
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};
// The longest horizon marchAlong() lets a step start with, and so how far past the last step its
// reference has to reach.
template <std::floating_point T>
inline constexpr T kMaxPreview{static_cast<T>(3.0)};
// The mean spacing of the waypoints of a reference, ten steps of kH, from which every interval
// strays by up to half either way (see waypointTimesOf()).
inline constexpr double kWaypointSpacing{0.1};
/**
 * @brief Four cost weights that differ from each other and from one.
 *
 * A default constructed controller weighs every term by one, so it takes weights like these to
 * tell a weight that went missing, or landed on the wrong term, apart from one that did not.
 */
template <std::floating_point T>
inline constexpr std::array<T, 4> kWeights{
    static_cast<T>(1.2), static_cast<T>(0.9), static_cast<T>(1.1), static_cast<T>(0.8)
};

inline constexpr std::array<IntegrateMode, 3> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER, IntegrateMode::RUNGE_KUTTA
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
using State = KinematicsFlowControllerState<T, A>;
template <std::floating_point T>
using ModelState = KinematicsModelState<T>;
template <std::floating_point T, bool A>
using Controller = KinematicsFlowController<T, A>;
// A reference trajectory: target states at timestamped waypoints, interpolated linearly between.
template <std::floating_point T, bool A>
using Reference = ::basjoo::math::PiecewiseLinearFunction<State<T, A>>;

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
 * @brief Controller of either mode. tau is the horizon of a fixed one and goes unused by an
 * adaptive one, whose horizon comes with the state.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto controllerOf(
    const KinematicsModel<T>& model,
    const std::type_identity_t<std::array<T, 4>>& weights = kWeights<T>,
    [[maybe_unused]] std::type_identity_t<T> tau = kHorizon<T>
) -> Controller<T, A> {
    if constexpr (A) {
        return Controller<T, A>{model, std::span<const T, 4>{weights}};
    } else {
        return Controller<T, A>{model, std::span<const T, 4>{weights}, tau};
    }
}

/**
 * @brief State of either mode. tau is dropped when the horizon is fixed.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto stateOf(
    std::type_identity_t<T> x, std::type_identity_t<T> y, std::type_identity_t<T> yaw,
    std::type_identity_t<T> delta, std::type_identity_t<T> u, std::type_identity_t<T> a,
    std::type_identity_t<T> omega, [[maybe_unused]] std::type_identity_t<T> tau
) noexcept -> State<T, A> {
    if constexpr (A) {
        return State<T, A>{x, y, yaw, delta, u, a, omega, tau};
    } else {
        return State<T, A>{x, y, yaw, delta, u, a, omega};
    }
}

/**
 * @brief A target state: the controller ignores the controls and the horizon of a target, so they
 * are left NaN, which any use of them would carry into the result.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto targetAt(
    std::type_identity_t<T> x, std::type_identity_t<T> y, std::type_identity_t<T> yaw,
    std::type_identity_t<T> delta, std::type_identity_t<T> u
) noexcept -> State<T, A> {
    constexpr T kNaN{std::numeric_limits<T>::quiet_NaN()};
    return stateOf<T, A>(x, y, yaw, delta, u, kNaN, kNaN, kNaN);
}

template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto plantOf(const State<T, A>& state) noexcept -> ModelState<T> {
    return ModelState<T>{state.x, state.y, state.yaw, state.delta, state.u};
}

/**
 * @brief Horizon the prediction runs over at the given state.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto horizonOf(
    [[maybe_unused]] const Controller<T, A>& controller, [[maybe_unused]] const State<T, A>& state
) noexcept -> T {
    if constexpr (A) {
        return state.tau;
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
        state.tau = tau;
    }
    return state;
}

/**
 * @brief Smallest magnitude among the rates a tangent gives the controller fields.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto smallestRateOf(const State<T, A>& k) noexcept -> T {
    if constexpr (A) {
        return std::min({std::abs(k.a), std::abs(k.omega), std::abs(k.tau)});
    } else {
        return std::min(std::abs(k.a), std::abs(k.omega));
    }
}

/**
 * @brief A vehicle turning left at speed, with both controls inside the bounds.
 *
 * An adaptive horizon starts off its default of one, so that a horizon that went missing somewhere
 * shows, and long enough that every rate towards targetOf() is large enough for the precondition
 * of SingleStepIntegratorsDriveThePlantThroughTheModel.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto startOf() noexcept -> State<T, A> {
    return stateOf<T, A>(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
        static_cast<T>(8.0), static_cast<T>(-0.6), static_cast<T>(0.03), static_cast<T>(0.7)
    );
}

/**
 * @brief A target ahead of startOf(), off by enough in every residual that the rates at the start
 * are far from zero.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto targetOf() noexcept -> State<T, A> {
    return targetAt<T, A>(
        static_cast<T>(1.3), static_cast<T>(0.38), static_cast<T>(0.34), static_cast<T>(0.3),
        static_cast<T>(9.0)
    );
}

/**
 * @brief Turning left, turning right and reversing, each with both controls inside the bounds and,
 * when adaptive, a horizon well clear of zero.
 */
template <std::floating_point T, bool A>
[[nodiscard]] constexpr auto statesOf() noexcept -> std::array<State<T, A>, 3> {
    return {
        startOf<T, A>(),
        stateOf<T, A>(
            static_cast<T>(3.0), static_cast<T>(-2.0), static_cast<T>(-1.1), static_cast<T>(-0.2),
            static_cast<T>(5.0), static_cast<T>(1.2), static_cast<T>(-0.05), static_cast<T>(0.4)
        ),
        stateOf<T, A>(
            static_cast<T>(-1.0), static_cast<T>(4.0), static_cast<T>(2.5), static_cast<T>(0.1),
            static_cast<T>(-3.0), static_cast<T>(-0.4), static_cast<T>(0.2), static_cast<T>(0.8)
        )
    };
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

/**
 * @brief A reference trajectory up to duration, starting at the origin at speed and steered by two
 * smooth open loop controls, handed over as waypoints at the uneven times of waypointTimesOf().
 *
 * Each interval is integrated in equal steps of at most kH that land on the waypoint closing it.
 * Every waypoint is a target state, so that the reference interpolated at any time can be handed
 * to the controller as a preview point (see marchAlong()).
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto referenceOf(const KinematicsModel<T>& model, T duration) -> Reference<T, A> {
    const std::vector<T> ts{waypointTimesOf(duration)};
    std::vector<State<T, A>> waypoints;
    waypoints.reserve(ts.size());
    ModelState<T> plant{
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.0),
        static_cast<T>(8.0)
    };
    waypoints.push_back(targetAt<T, A>(plant.x, plant.y, plant.yaw, plant.delta, plant.u));
    for (std::size_t k{0}; k + 1 < ts.size(); ++k) {
        const T span{ts[k + 1] - ts[k]};
        const auto num_steps{static_cast<std::size_t>(std::ceil(span / kH<T>))};
        const T h{span / static_cast<T>(num_steps)};
        for (std::size_t i{0}; i < num_steps; ++i) {
            const T t{ts[k] + h * static_cast<T>(i)};
            plant = model.rungeKutta(
                plant, static_cast<T>(0.8) * std::sin(static_cast<T>(0.4) * t),
                static_cast<T>(0.12) * std::cos(static_cast<T>(0.5) * t), h
            );
        }
        waypoints.push_back(targetAt<T, A>(plant.x, plant.y, plant.yaw, plant.delta, plant.u));
    }
    return Reference<T, A>{ts, waypoints};
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
    return std::same_as<T, float> ? 1.0e-4 : 1.0e-12;
}

/**
 * @brief Largest per field difference between two states, of the controller or of the model.
 *
 * Written so that a NaN in either state propagates into the result instead of being skipped over
 * the way std::max() would skip it, which would let a diverging integrator pass a check against
 * zero.
 */
template <typename S>
[[nodiscard]] auto distanceOf(const S& lhs, const S& rhs) noexcept -> S::value_type {
    using T = S::value_type;
    T result{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < S::size(); ++i) {
        const T difference{std::abs(lhs.data()[i] - rhs.data()[i])};
        if (!(difference <= result)) {
            result = difference;
        }
    }
    return result;
}

/**
 * @brief Both controls within the bounds and, when adaptive, a positive horizon, which also fails
 * on a NaN.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto isWithinBounds(const State<T, A>& state) noexcept -> bool {
    const bool controls{
        kALo<T> <= state.a && state.a <= kAUp<T> && kOmegaLo<T> <= state.omega &&
        state.omega <= kOmegaUp<T>
    };
    if constexpr (A) {
        return controls && static_cast<T>(0.0) < state.tau;
    } else {
        return controls;
    }
}

template <std::floating_point T, bool A>
[[nodiscard]] auto isSaturated(const State<T, A>& state) noexcept -> bool {
    return state.a == kALo<T> || state.a == kAUp<T> || state.omega == kOmegaLo<T> ||
           state.omega == kOmegaUp<T>;
}

template <std::floating_point T, bool A>
[[nodiscard]] auto stepWith(
    IntegrateMode mode, const Controller<T, A>& controller, const State<T, A>& state,
    const State<T, A>& target_state, std::type_identity_t<T> h
) -> State<T, A> {
    switch (mode) {
    case IntegrateMode::FORWARD_EULER:
        return controller.forwardEuler(state, target_state, h);
    case IntegrateMode::SYMPLECTIC_EULER:
        return controller.symplecticEuler(state, target_state, h);
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
    case IntegrateMode::SYMPLECTIC_EULER:
        return controller.template evolve<IntegrateMode::SYMPLECTIC_EULER>(
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
 * @brief tangent() looking ahead over the horizon at the given state less shortening, through a
 * second controller when the horizon is fixed.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto tangentOverShorterHorizon(
    const Controller<T, A>& controller, State<T, A> state, const State<T, A>& target_state,
    T shortening
) -> State<T, A> {
    if constexpr (A) {
        state.tau -= shortening;
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
 * over the horizon less stage_times times their stage time.
 *
 * With stage_times at one it walks the same arithmetic as rungeKutta(), so the two part only by
 * roundoff, and at zero every stage looks ahead over the full horizon.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto rungeKuttaThroughTangent(
    const Controller<T, A>& controller, const State<T, A>& state, const State<T, A>& target_state,
    T h, T stage_times
) -> State<T, A> {
    constexpr T kHalf{static_cast<T>(0.5)};
    constexpr T kSixth{static_cast<T>(1.0 / 6.0)};
    constexpr T kThird{static_cast<T>(1.0 / 3.0)};
    const KinematicsModel<T>& model{controller.model()};
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
    result.a = std::clamp(result.a, model.a_lower_bound(), model.a_upper_bound());
    result.omega = std::clamp(result.omega, model.omega_lower_bound(), model.omega_upper_bound());
    return result;
}

/**
 * @brief Derivative of f along one field of the state, from the central difference.
 *
 * The central difference is exact for quadratics, and the cost is quadratic in both controls, so
 * it recovers their derivatives up to rounding whatever the step, and a wide step keeps the
 * rounding small.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto derivativeOf(
    const auto& f, const State<T, A>& state, T State<T, A>::* field, std::type_identity_t<T> step
) -> T {
    const auto at = [&](T shift) -> T {
        State<T, A> shifted{state};
        shifted.*field += shift;
        return f(shifted);
    };
    return (at(step) - at(-step)) / (static_cast<T>(2.0) * step);
}

/**
 * @brief Derivative of f along one field of the state, from the seven point stencil.
 *
 * The stencil is exact for polynomials up to degree six, which covers the cost along the horizon:
 * the position residuals are cubic in it. So it recovers that derivative up to rounding whatever
 * the step.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto sevenPointDerivativeOf(
    const auto& f, const State<T, A>& state, T State<T, A>::* field, std::type_identity_t<T> step
) -> T {
    const auto at = [&](T shift) -> T {
        State<T, A> shifted{state};
        shifted.*field += shift;
        return f(shifted);
    };
    return (at(static_cast<T>(3.0) * step) - at(static_cast<T>(-3.0) * step) +
            static_cast<T>(9.0) *
                (at(static_cast<T>(-2.0) * step) - at(static_cast<T>(2.0) * step)) +
            static_cast<T>(45.0) * (at(step) - at(-step))) /
           (static_cast<T>(60.0) * step);
}

/**
 * @brief Marches one integrator num_steps steps of h along the reference from the initial state
 * at time zero, the trace led by that state.
 *
 * Step i starts at t_i = i h and is handed as its target the preview point, the reference
 * interpolated between the two waypoints around t_i plus the horizon the step starts with: the
 * horizon of the controller when that is fixed, the one the state carries when it adapts. Every
 * horizon has to lie within [2 h, kMaxPreview]: twice the step keeps it clear of the instability
 * of the Euler steps, and the reference has to reach kMaxPreview past the last step.
 */
template <std::floating_point T, bool A>
[[nodiscard]] auto marchAlong(
    IntegrateMode mode, const Controller<T, A>& controller, const State<T, A>& initial_state,
    const Reference<T, A>& reference, T h, std::size_t num_steps
) -> std::vector<State<T, A>> {
    REQUIRE_GE(reference.maxT(), h * static_cast<T>(num_steps) + kMaxPreview<T>);
    const T shortest_horizon{static_cast<T>(2.0) * h};
    std::vector<State<T, A>> trace;
    trace.reserve(num_steps + 1);
    trace.push_back(initial_state);
    for (std::size_t i{0}; i < num_steps; ++i) {
        const T horizon{horizonOf(controller, trace.back())};
        if (!(shortest_horizon <= horizon && horizon <= kMaxPreview<T>)) {
            FAIL(
                "step ", i, " starts with a horizon of ", horizon, " s, outside [",
                shortest_horizon, ", ", kMaxPreview<T>, "] s"
            );
        }
        const State<T, A> preview_point{reference(h * static_cast<T>(i) + horizon)};
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
) noexcept -> T {
    const std::vector<State<T, A>>& trace{loop.trace(mode)};
    T result{static_cast<T>(0.0)};
    for (std::size_t i{first}; i <= last; ++i) {
        const State<T, A>& simultaneous_point{loop.simultaneous_points[i]};
        const T distance{
            std::hypot(trace[i].x - simultaneous_point.x, trace[i].y - simultaneous_point.y)
        };
        if (!(distance <= result)) {
            result = distance;
        }
    }
    return result;
}

/**
 * @brief Plots the path, the five plant fields and the three controller fields of every
 * integrator against time, the path and the plant fields over those of the reference. A fixed
 * horizon shows as a flat line. Every curve keeps its color across the nine subplots, so that one
 * legend across the top of the figure covers them all.
 */
template <bool A>
auto plotClosedLoop(const std::string& title, const ClosedLoop<double, A>& loop) -> void {
    std::vector<double> reference_xs, reference_ys, reference_yaws, reference_deltas, reference_us;
    for (const State<double, A>& reference : loop.simultaneous_points) {
        reference_xs.push_back(reference.x);
        reference_ys.push_back(reference.y);
        reference_yaws.push_back(reference.yaw);
        reference_deltas.push_back(reference.delta);
        reference_us.push_back(reference.u);
    }
    const std::array<float, 4> reference_color{matplot::default_color(0)};
    const std::array<float, 4> last_reference_color{matplot::default_color(1)};
    const std::array<std::array<float, 4>, kIntegrateModes.size()> mode_colors{
        matplot::default_color(2), matplot::default_color(3), matplot::default_color(4)
    };

    matplot::figure_handle fig = createFigureHandle();
    fig->title(title);
    std::vector<matplot::axes_handle> axes = createAxesHandles(fig);
    axes[0]->plot(reference_xs, reference_ys, "-")->line_width(2.0).color(reference_color);
    axes[0]
        ->plot(
            std::vector<double>{reference_xs.back()}, std::vector<double>{reference_ys.back()}, "o"
        )
        ->line_width(2.0)
        .color(last_reference_color);
    axes[1]->plot(loop.ts, reference_xs, "-")->line_width(2.0).color(reference_color);
    axes[2]->plot(loop.ts, reference_ys, "-")->line_width(2.0).color(reference_color);
    axes[3]->plot(loop.ts, reference_yaws, "-")->line_width(2.0).color(reference_color);
    axes[4]->plot(loop.ts, reference_deltas, "-")->line_width(2.0).color(reference_color);
    axes[5]->plot(loop.ts, reference_us, "-")->line_width(2.0).color(reference_color);
    const std::array<const char*, kIntegrateModes.size()> line_specs{":", "-.", "--"};
    for (const IntegrateMode mode : kIntegrateModes) {
        std::vector<double> xs, ys, yaws, deltas, us, accels, omegas, taus;
        for (const State<double, A>& state : loop.trace(mode)) {
            xs.push_back(state.x);
            ys.push_back(state.y);
            yaws.push_back(state.yaw);
            deltas.push_back(state.delta);
            us.push_back(state.u);
            accels.push_back(state.a);
            omegas.push_back(state.omega);
            if constexpr (A) {
                taus.push_back(state.tau);
            } else {
                taus.push_back(loop.horizon);
            }
        }
        const char* line_spec{line_specs[static_cast<std::size_t>(mode)]};
        const std::array<float, 4>& color{mode_colors[static_cast<std::size_t>(mode)]};
        axes[0]->plot(xs, ys, line_spec)->line_width(2.0).color(color);
        axes[1]->plot(loop.ts, xs, line_spec)->line_width(2.0).color(color);
        axes[2]->plot(loop.ts, ys, line_spec)->line_width(2.0).color(color);
        axes[3]->plot(loop.ts, yaws, line_spec)->line_width(2.0).color(color);
        axes[4]->plot(loop.ts, deltas, line_spec)->line_width(2.0).color(color);
        axes[5]->plot(loop.ts, us, line_spec)->line_width(2.0).color(color);
        axes[6]->plot(loop.ts, accels, line_spec)->line_width(2.0).color(color);
        axes[7]->plot(loop.ts, omegas, line_spec)->line_width(2.0).color(color);
        axes[8]->plot(loop.ts, taus, line_spec)->line_width(2.0).color(color);
    }
    axes[0]->axis(matplot::equal);

    // A matplot figure has no legend of its own, so the legend gets an axes to itself: a strip
    // across the top whose limits keep its curves out of sight, so that only their keys show.
    const matplot::axes_handle key{fig->add_axes({0.05F, 0.935F, 0.9F, 0.03F})};
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
    key->legend(
        {"reference", "last reference", "forward euler", "symplectic euler", "runge kutta"}
    );
    key->legend()->num_columns(kIntegrateModes.size() + 2);
    key->legend()->box(false);
    key->legend()->location(matplot::legend::general_alignment::center);
    key->legend()->font_size(12.0);
    fig->show();
}

} // namespace

} // namespace basjoo::bicycle

TYPE_TO_STRING_AS("FixedFloat", basjoo::bicycle::FixedFloat);
TYPE_TO_STRING_AS("AdaptiveFloat", basjoo::bicycle::AdaptiveFloat);
TYPE_TO_STRING_AS("FixedDouble", basjoo::bicycle::FixedDouble);
TYPE_TO_STRING_AS("AdaptiveDouble", basjoo::bicycle::AdaptiveDouble);

namespace basjoo::bicycle {

TEST_CASE_TEMPLATE(
    "ConstructorKeepsTheModelTheWeightsAndTheHorizon", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    // The controller has no geometry or bounds of its own, so model() has to hand back the very
    // model it was built with.
    const KinematicsModel<T>& model{controller.model()};
    CHECK_EQ(model.front_wheelbase(), kFrontWheelbase<T>);
    CHECK_EQ(model.rear_wheelbase(), kRearWheelbase<T>);
    CHECK_EQ(model.a_lower_bound(), kALo<T>);
    CHECK_EQ(model.a_upper_bound(), kAUp<T>);
    CHECK_EQ(model.omega_lower_bound(), kOmegaLo<T>);
    CHECK_EQ(model.omega_upper_bound(), kOmegaUp<T>);
    // No two weights agree, so a shuffle cannot pass.
    CHECK(std::ranges::equal(controller.weights(), kWeights<T>));
    if constexpr (!kAdaptive) {
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
    // A default constructed controller has unit weights and a unit, unbounded model, and a fixed
    // one a unit horizon, none of which agrees with the original, so every field checked below has
    // to have come through the archive.
    Controller<T, kAdaptive> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.model().front_wheelbase(), kFrontWheelbase<T>);
    CHECK_EQ(restored.model().rear_wheelbase(), kRearWheelbase<T>);
    CHECK_EQ(restored.model().a_lower_bound(), kALo<T>);
    CHECK_EQ(restored.model().a_upper_bound(), kAUp<T>);
    CHECK_EQ(restored.model().omega_lower_bound(), kOmegaLo<T>);
    CHECK_EQ(restored.model().omega_upper_bound(), kOmegaUp<T>);
    CHECK(std::ranges::equal(restored.weights(), kWeights<T>));
    if constexpr (!kAdaptive) {
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
        State<T, kAdaptive> restored{};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, state);
    }
}

TEST_CASE_TEMPLATE(
    "StateIndexingAndFill", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const State<T, kAdaptive> state{startOf<T, kAdaptive>()};

    SUBCASE("IndexingWalksTheFieldsInOrder") {
        // operator[] reaches every field in the order data() lays them out in, and writes through
        // as well as it reads.
        State<T, kAdaptive> written{};
        for (std::size_t i{0}; i < State<T, kAdaptive>::size(); ++i) {
            CHECK_EQ(&state[i], state.data() + i);
            written[i] = state[i];
        }
        CHECK_EQ(written, state);
    }

    SUBCASE("FillConstructorSetsEveryField") {
        const State<T, kAdaptive> filled{static_cast<T>(0.75)};
        for (std::size_t i{0}; i < State<T, kAdaptive>::size(); ++i) {
            CHECK_EQ(filled[i], static_cast<T>(0.75));
        }
        // Filled with zero, the horizon included, a state is the zero generic code adds, which the
        // default state is not once it carries a horizon, as that defaults to one.
        CHECK_EQ(state + State<T, kAdaptive>{static_cast<T>(0.0)}, state);
        if constexpr (kAdaptive) {
            CHECK_NE(state + State<T, kAdaptive>{}, state);
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

    const S origin{stateOf<T, kAdaptive>(
        static_cast<T>(1.5), static_cast<T>(-2.5), static_cast<T>(0.3), static_cast<T>(0.15),
        static_cast<T>(7.5), static_cast<T>(-0.6), static_cast<T>(0.03), static_cast<T>(0.7)
    )};
    const S rate{stateOf<T, kAdaptive>(
        static_cast<T>(7.5), static_cast<T>(1.25), static_cast<T>(-0.4), static_cast<T>(0.05),
        static_cast<T>(-1.5), static_cast<T>(0.2), static_cast<T>(-0.01), static_cast<T>(0.1)
    )};
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
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const auto loose = [](State<T, kAdaptive> state) -> State<T, kAdaptive> {
        state.a = static_cast<T>(50.0);
        state.omega = static_cast<T>(-7.0);
        return state;
    };

    SUBCASE("PlantFieldsEqualTheModelTangent") {
        // The controller spells the plant flow out itself instead of calling the model, so that the
        // flow and the cost can share their trigonometry. This is what keeps that copy honest: any
        // change to KinematicsModel::tangent() that is not mirrored in the controller fails here
        // bit for bit, and so does a clamp that went missing.
        for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
            for (const State<T, kAdaptive>& handed : {state, loose(state)}) {
                CHECK_EQ(
                    plantOf(controller.tangent(handed, target)),
                    model.tangent(plantOf(handed), handed.a, handed.omega)
                );
            }
        }
    }

    SUBCASE("LooseControlsChangeNothing") {
        // The controls enter the flow and the cost only through their bounds, so controls beyond
        // the bounds may not move a single bit of the tangent.
        for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
            State<T, kAdaptive> clamped{state};
            clamped.a = kAUp<T>;
            clamped.omega = kOmegaLo<T>;
            CHECK_EQ(controller.tangent(loose(state), target), controller.tangent(clamped, target));
        }
    }
}

TEST_CASE_TEMPLATE(
    "CostIsHalfTheWeightedSumOfTheResiduals", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // cost() and the rates share their residuals, so the residuals are spelled out here once more,
    // straight from the formulas documented with KinematicsFlowController::tangent(). The two sides
    // arrange the arithmetic differently and round differently.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const T L{kWheelbase<T>};
    const T lr{kRearWheelbase<T>};
    const T kTwo{static_cast<T>(2.0)};
    const T kSix{static_cast<T>(6.0)};
    for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
        const T tau{horizonOf(controller, state)};
        const T s{std::sin(state.yaw)};
        const T c{std::cos(state.yaw)};
        const T tan_delta{std::tan(state.delta)};
        const T sec2_delta{static_cast<T>(1.0) / (std::cos(state.delta) * std::cos(state.delta))};
        const T r{state.u * tan_delta / L};
        const T v{lr * r};
        const T r_dot{(state.a * tan_delta + state.u * state.omega * sec2_delta) / L};
        const T v_dot{lr * r_dot};
        const T rx{
            state.x - target.x + tau * (state.u * c - v * s) +
            tau * tau / kTwo * (state.a * c - v_dot * s - r * (state.u * s + v * c)) +
            tau * tau * tau / kSix *
                (-r * r * (state.u * c - v * s) - kTwo * r * (state.a * s + v_dot * c) -
                 r_dot * (state.u * s + v * c))
        };
        const T ry{
            state.y - target.y + tau * (state.u * s + v * c) +
            tau * tau / kTwo * (state.a * s + v_dot * c + r * (state.u * c - v * s)) +
            tau * tau * tau / kSix *
                (-r * r * (state.u * s + v * c) + kTwo * r * (state.a * c - v_dot * s) +
                 r_dot * (state.u * c - v * s))
        };
        const T ryaw{state.yaw - target.yaw + tau * r + tau * tau / kTwo * r_dot};
        const T rdelta{state.delta + tau * state.omega - target.delta};
        const T ru{state.u + tau * state.a - target.u};
        const T expected{
            (kWeights<T>[0] * (rx * rx + ry * ry) / (L * L) + kWeights<T>[1] * ryaw * ryaw +
             kWeights<T>[2] * rdelta * rdelta + kWeights<T>[3] * (tau * ru / L) * (tau * ru / L)) /
            kTwo
        };
        CHECK_EQ(
            controller.cost(state, target),
            doctest::Approx(expected).epsilon(derivativeTolerance<T>())
        );
    }
}

TEST_CASE_TEMPLATE(
    "PredictionIsExactUnderAConstantAcceleration", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // Along a straight line under a constant acceleration the vehicle moves by a polynomial of
    // degree two in time, which the prediction takes in exactly, so every residual against where
    // the vehicle gets to vanishes. A target a centimetre off sets the scale the cost is held to.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    constexpr T kTau{static_cast<T>(0.6)};
    const Controller<T, kAdaptive> controller{
        controllerOf<T, kAdaptive>(unboundedModel<T>(), kWeights<T>, kTau)
    };
    const T x{static_cast<T>(1.0)};
    const T y{static_cast<T>(-0.5)};
    const T yaw{static_cast<T>(0.4)};
    const T u{static_cast<T>(6.0)};
    const T a{static_cast<T>(1.5)};
    const State<T, kAdaptive> state{
        stateOf<T, kAdaptive>(x, y, yaw, static_cast<T>(0.0), u, a, static_cast<T>(0.0), kTau)
    };
    const T distance{u * kTau + a * kTau * kTau / static_cast<T>(2.0)};
    const State<T, kAdaptive> target{targetAt<T, kAdaptive>(
        x + distance * std::cos(yaw), y + distance * std::sin(yaw), yaw, static_cast<T>(0.0),
        u + a * kTau
    )};
    State<T, kAdaptive> off{target};
    off.x += static_cast<T>(0.01);
    const T off_cost{controller.cost(state, off)};
    REQUIRE_GT(off_cost, static_cast<T>(0.0));
    CHECK_LE(controller.cost(state, target), roundoffTolerance<T>() * off_cost);
}

TEST_CASE_TEMPLATE(
    "PredictionErrorIsOfFourthOrderOnAConstantTurn", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // With both controls at zero the prediction is exact through tau^3, so along a constant turn
    // the position residual against where the vehicle gets to is of fourth order in the horizon:
    // halving the horizon divides it by sixteen. Only the position is weighed, so the cost is half
    // that residual squared over L^2.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const KinematicsModel<T> model{unboundedModel<T>()};
    const std::array<T, 4> position_only{
        static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0)
    };
    const T yaw{static_cast<T>(0.3)};
    const T delta{static_cast<T>(0.12)};
    const T u{static_cast<T>(8.0)};
    const T r{u * std::tan(delta) / kWheelbase<T>};
    const T v{kRearWheelbase<T> * r};
    const auto residual = [&](T tau) -> T {
        const Controller<T, kAdaptive> controller{
            controllerOf<T, kAdaptive>(model, position_only, tau)
        };
        const State<T, kAdaptive> state{stateOf<T, kAdaptive>(
            static_cast<T>(0.0), static_cast<T>(0.0), yaw, delta, u, static_cast<T>(0.0),
            static_cast<T>(0.0), tau
        )};
        const T end_yaw{yaw + r * tau};
        const State<T, kAdaptive> target{targetAt<T, kAdaptive>(
            (u * (std::sin(end_yaw) - std::sin(yaw)) + v * (std::cos(end_yaw) - std::cos(yaw))) / r,
            (v * (std::sin(end_yaw) - std::sin(yaw)) - u * (std::cos(end_yaw) - std::cos(yaw))) / r,
            end_yaw, delta, u
        )};
        return kWheelbase<T> * std::sqrt(static_cast<T>(2.0) * controller.cost(state, target));
    };
    const T ratio{residual(static_cast<T>(0.8)) / residual(static_cast<T>(0.4))};
    CHECK_GT(ratio, static_cast<T>(15.0));
    CHECK_LT(ratio, static_cast<T>(17.0));
}

TEST_CASE_TEMPLATE(
    "RatesAreThePreconditionedNegativeGradientOfTheCost", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // The rates are da/dt = -(L^2/tau^5) dJ/da, domega/dt = -(1/tau^3) dJ/domega and, when
    // adaptive, dtau/dt = -tau dJ/dtau. They take the derivatives of the residuals in closed
    // form, while the finite differences recover those of cost() up to rounding, so a slip in the
    // closed forms shows here. The acceleration enters the residuals only through powers of tau,
    // so the cost hardly moves with it and its step has to be all the wider. The model is
    // unbounded, so no clamp cuts into the differences.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(unboundedModel<T>())};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const auto cost = [&](const State<T, kAdaptive>& state) -> T {
        return controller.cost(state, target);
    };
    const T wheelbase2{kWheelbase<T> * kWheelbase<T>};
    constexpr T kAStep{static_cast<T>(10.0)};
    constexpr T kOmegaStep{static_cast<T>(1.0)};
    constexpr T kTauStep{static_cast<T>(0.05)};
    const double tolerance{derivativeTolerance<T>()};
    for (const State<T, kAdaptive>& state : statesOf<T, kAdaptive>()) {
        const T tau{horizonOf(controller, state)};
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        CHECK_EQ(
            k.a, doctest::Approx(
                     -wheelbase2 / (tau * tau * tau * tau * tau) *
                     derivativeOf(cost, state, &State<T, kAdaptive>::a, kAStep)
                 )
                     .epsilon(tolerance)
        );
        CHECK_EQ(
            k.omega, doctest::Approx(
                         -derivativeOf(cost, state, &State<T, kAdaptive>::omega, kOmegaStep) /
                         (tau * tau * tau)
                     )
                         .epsilon(tolerance)
        );
        if constexpr (kAdaptive) {
            CHECK_EQ(
                k.tau,
                doctest::Approx(
                    -tau * sevenPointDerivativeOf(cost, state, &State<T, kAdaptive>::tau, kTauStep)
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
    const KinematicsModel<T> model{boundedModel<T>()};
    const State<T, kAdaptive> state{startOf<T, kAdaptive>()};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    constexpr T kWeight{static_cast<T>(0.6)};
    constexpr T kZero{static_cast<T>(0.0)};

    SUBCASE("DeltaTermAlone") {
        // Left with its delta term alone, the cost reads J = w2 R^2/2 with R = delta + tau omega -
        // delta_target, whose gradient dJ/domega = w2 R tau has a closed form, and the steering
        // rate descends it with the preconditioner 1/tau^3. Put differently, an Euler step of
        // length h moves omega by w2 h / tau times the Newton step -R / tau. The acceleration has
        // nothing left to descend.
        const Controller<T, kAdaptive> controller{
            controllerOf<T, kAdaptive>(model, {kZero, kZero, kWeight, kZero})
        };
        const T horizon{horizonOf(controller, state)};
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        const T residual{state.delta + horizon * state.omega - target.delta};
        const T gradient{kWeight * residual * horizon};
        CHECK_EQ(
            k.omega, doctest::Approx(-gradient / (horizon * horizon * horizon))
                         .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(k.a, kZero);
    }

    SUBCASE("UTermAlone") {
        // Left with its u term alone, the cost reads J = w3 (tau R / L)^2/2 with R = u + tau a -
        // u_target, whose gradient dJ/da = w3 tau^3 R / L^2 has a closed form, and the
        // acceleration descends it with the preconditioner L^2/tau^5, which again comes to
        // w3 h / tau times the Newton step -R / tau per Euler step. The steering rate has nothing
        // left to descend.
        const Controller<T, kAdaptive> controller{
            controllerOf<T, kAdaptive>(model, {kZero, kZero, kZero, kWeight})
        };
        const T horizon{horizonOf(controller, state)};
        const State<T, kAdaptive> k{controller.tangent(state, target)};
        const T wheelbase{kWheelbase<T>};
        const T residual{state.u + horizon * state.a - target.u};
        const T gradient{
            kWeight * horizon * horizon * horizon * residual / (wheelbase * wheelbase)
        };
        CHECK_EQ(
            k.a, doctest::Approx(
                     -wheelbase * wheelbase / (horizon * horizon * horizon * horizon * horizon) *
                     gradient
                 )
                     .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(k.omega, kZero);
    }

    SUBCASE("MixedPartialsMatchThePreconditioners") {
        // Both rates are preconditioned partial derivatives of one and the same cost, so their
        // cross derivatives are the one mixed partial of J scaled by the two preconditioners,
        // whose ratio is L^2/tau^2 whatever the weights, the state and the target. The rates are
        // affine in both controls, so central differences recover the derivatives up to rounding
        // whatever the step, and a wide step keeps that rounding small next to the differences.
        // The model is unbounded so that no clamp cuts into them.
        const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(unboundedModel<T>())};
        const T horizon{horizonOf(controller, state)};
        constexpr T kStep{static_cast<T>(10.0)};
        const auto rates_at = [&](T a_shift, T omega_shift) -> State<T, kAdaptive> {
            State<T, kAdaptive> shifted{state};
            shifted.a += a_shift;
            shifted.omega += omega_shift;
            return controller.tangent(shifted, target);
        };
        const T a_rate_by_omega{
            (rates_at(kZero, kStep).a - rates_at(kZero, -kStep).a) / (static_cast<T>(2.0) * kStep)
        };
        const T omega_rate_by_a{
            (rates_at(kStep, kZero).omega - rates_at(-kStep, kZero).omega) /
            (static_cast<T>(2.0) * kStep)
        };
        CHECK_EQ(
            a_rate_by_omega / omega_rate_by_a,
            doctest::Approx(kWheelbase<T> * kWheelbase<T> / (horizon * horizon))
                .epsilon(roundoffTolerance<T>())
        );
    }
}

TEST_CASE_TEMPLATE(
    "CostAndRatesAreInvariantUnderRigidMotions", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    // One weight covers x and y together, so turning the state and the target together about the
    // origin and shifting them together moves neither the cost nor the rates of the controller
    // fields.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(boundedModel<T>())};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const T angle{static_cast<T>(0.7)};
    const T shift_x{static_cast<T>(5.0)};
    const T shift_y{static_cast<T>(-3.0)};
    const auto moved = [&](State<T, kAdaptive> state) -> State<T, kAdaptive> {
        const T x{std::cos(angle) * state.x - std::sin(angle) * state.y + shift_x};
        const T y{std::sin(angle) * state.x + std::cos(angle) * state.y + shift_y};
        state.x = x;
        state.y = y;
        state.yaw += angle;
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
        CHECK_EQ(k_moved.a, doctest::Approx(k.a).epsilon(tolerance));
        CHECK_EQ(k_moved.omega, doctest::Approx(k.omega).epsilon(tolerance));
        if constexpr (kAdaptive) {
            CHECK_EQ(k_moved.tau, doctest::Approx(k.tau).epsilon(tolerance));
        }
    }
}

TEST_CASE_TEMPLATE(
    "SingleStepIntegratorsDriveThePlantThroughTheModel", P, FixedFloat, AdaptiveFloat, FixedDouble,
    AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    const T tolerance{roundoffTolerance<T>()};
    const T h{kH<T>};
    const State<T, kAdaptive> start{startOf<T, kAdaptive>()};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};
    const State<T, kAdaptive> k{controller.tangent(start, target)};
    // The comparisons below hold the integrators to a tolerance, so the rates at this start have to
    // be large enough that holding a controller field still over a step, or stepping the plant
    // under the updated controls instead of the held ones, lands far outside it. The latter moves
    // delta and u by h^2 times a rate.
    REQUIRE_GT(h * h * smallestRateOf(k), static_cast<T>(10.0) * tolerance);

    SUBCASE("EulerPlantFieldsFollowTheModelIntegrators") {
        // The plant half of an Euler step is the model's own step under the controls the step
        // starts with, so the five plant fields have to land where the model puts them.
        CHECK_LE(
            distanceOf(
                plantOf(controller.forwardEuler(start, target, h)),
                model.forwardEuler(plantOf(start), start.a, start.omega, h)
            ),
            tolerance
        );
        CHECK_LE(
            distanceOf(
                plantOf(controller.symplecticEuler(start, target, h)),
                model.symplecticEuler(plantOf(start), start.a, start.omega, h)
            ),
            tolerance
        );
    }

    SUBCASE("EulerControllerFieldsStepAlongTheRates") {
        // The controller fields take one Euler step along the rates tangent() gives at the start
        // of the step, whichever integrator carries the plant: the symplectic staging of the
        // plant does not extend to them.
        for (const State<T, kAdaptive>& stepped :
             {controller.forwardEuler(start, target, h),
              controller.symplecticEuler(start, target, h)}) {
            CHECK_EQ(
                stepped.a,
                doctest::Approx(std::clamp(start.a + k.a * h, kALo<T>, kAUp<T>)).epsilon(tolerance)
            );
            CHECK_EQ(
                stepped.omega,
                doctest::Approx(std::clamp(start.omega + k.omega * h, kOmegaLo<T>, kOmegaUp<T>))
                    .epsilon(tolerance)
            );
            if constexpr (kAdaptive) {
                CHECK_EQ(stepped.tau, doctest::Approx(start.tau + k.tau * h).epsilon(tolerance));
            }
        }
    }

    SUBCASE("RungeKuttaStagesSeeTheHorizonLessTheirStageTime") {
        // The target is held over the step while the vehicle moves on towards it, so the two
        // midpoint stages look ahead over their horizon less h/2 and the last one over its horizon
        // less h. Handing every stage the full horizon instead is required to land well outside
        // the tolerance, or the comparison could not tell the two apart.
        const State<T, kAdaptive> expected{
            rungeKuttaThroughTangent(controller, start, target, h, static_cast<T>(1.0))
        };
        REQUIRE_GT(
            distanceOf(
                expected,
                rungeKuttaThroughTangent(controller, start, target, h, static_cast<T>(0.0))
            ),
            static_cast<T>(10.0) * tolerance
        );
        CHECK_LE(distanceOf(controller.rungeKutta(start, target, h), expected), tolerance);
    }

    SUBCASE("ControlsNeverLeaveTheBounds") {
        // A target far out of reach, looked ahead to over a tenth of a second, drives both control
        // rates hard enough that a single unclamped step overshoots a bound, so the clamp at the
        // end of every integrator is all that keeps the controls in. The overshoot is required
        // first, or the checks below would hold trivially.
        const State<T, kAdaptive> distant{targetAt<T, kAdaptive>(
            static_cast<T>(40.0), static_cast<T>(-30.0), static_cast<T>(-1.0), static_cast<T>(-0.4),
            static_cast<T>(30.0)
        )};
        constexpr T kShortHorizon{static_cast<T>(0.1)};
        const Controller<T, kAdaptive> short_sighted{
            controllerOf<T, kAdaptive>(model, kWeights<T>, kShortHorizon)
        };
        const State<T, kAdaptive> from{withHorizon(start, kShortHorizon)};
        const State<T, kAdaptive> pull{short_sighted.tangent(from, distant)};
        const T a_step{from.a + pull.a * h};
        const T omega_step{from.omega + pull.omega * h};
        REQUIRE((a_step < kALo<T> || kAUp<T> < a_step));
        REQUIRE((omega_step < kOmegaLo<T> || kOmegaUp<T> < omega_step));
        // Controls handed in beyond the bounds are only held to come back inside them. Where they
        // land in there is left open on purpose: the controller steps from the control as handed
        // in and clamps afterwards while the plant flow clamps first, and the two only part ways
        // for a control that was out of bounds to begin with.
        State<T, kAdaptive> loose{from};
        loose.a = static_cast<T>(50.0);
        loose.omega = static_cast<T>(-7.0);
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const State<T, kAdaptive>& handed : {from, loose}) {
                CHECK(isWithinBounds(stepWith(mode, short_sighted, handed, distant, h)));
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
        targets.push_back(
            targetAt<T, kAdaptive>(
                goal.x + shift, goal.y - shift, goal.yaw + static_cast<T>(0.2) * shift,
                goal.delta - static_cast<T>(0.3) * shift, goal.u + shift
            )
        );
    }

    for (const IntegrateMode mode : kIntegrateModes) {
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
    // at t_i. As h shrinks, all three integrators close in on the loop whose target slides along
    // the reference continuously, but only to first order in h: the target comes in once per step
    // and is held while the step is taken, which caps rungeKutta() at first order as well. Halving
    // h therefore halves the gap to the next resolution, and the ratio below sits near 2 for all
    // three. Every resolution interpolates one and the same reference, its waypoints several times
    // sparser than the coarsest step, so the runs differ in the step alone. Interpolated, the
    // preview point slides along with an adaptive horizon as well, which is why that one is
    // followed here too: rounded to a waypoint, it would jump a whole waypoint each time the
    // horizon crossed the middle between two. The model is unbounded, since a clamp kicking in
    // would kink the flow and blur the order.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const KinematicsModel<T> model{unboundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    constexpr T kDuration{static_cast<T>(4.0)};
    constexpr std::size_t kFineSteps{1280};
    const T fine_h{kDuration / static_cast<T>(kFineSteps)};
    const Reference<T, kAdaptive> reference{
        referenceOf<T, kAdaptive>(model, kDuration + kMaxPreview<T>)
    };
    const State<T, kAdaptive> start{stateOf<T, kAdaptive>(
        static_cast<T>(0.5), static_cast<T>(-0.6), static_cast<T>(0.2), static_cast<T>(0.02),
        static_cast<T>(7.0), static_cast<T>(0.0), static_cast<T>(0.0), kHorizon<T>
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
    // around them (see marchAlong()). Every run lasts a minute, for --plot-graph to show where the
    // three integrators take an adaptive horizon, and is checked from five to 20 seconds.
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    constexpr std::size_t kNumSteps{6000};
    constexpr std::size_t kSettlingSteps{500};
    constexpr std::size_t kCheckedSteps{2000};
    const T h{kH<T>};
    const Reference<T, kAdaptive> reference{
        referenceOf<T, kAdaptive>(model, h * static_cast<T>(kNumSteps) + kMaxPreview<T>)
    };

    SUBCASE("FromOffTheReference") {
        // The vehicle starts off the reference by enough to drive its controls into the bounds.
        // The loop then stays within 20 cm of the simultaneous points with a fixed horizon and
        // within 30 cm with an adaptive one, for every integrator. Previewing the horizon the
        // state carries keeps an adaptive horizon from taking up the along track offset (see
        // KinematicsFlowController::tangent()), but leaves nothing to pin the horizon, which
        // settles wherever the along track lag or lead of the integrator leaves it: past 20
        // seconds at about half a second under forward Euler, a quarter under symplectic Euler and
        // a third under Runge-Kutta.
        const State<T, kAdaptive> start{stateOf<T, kAdaptive>(
            static_cast<T>(1.5), static_cast<T>(-2.0), static_cast<T>(0.1), static_cast<T>(0.0),
            static_cast<T>(5.0), static_cast<T>(0.0), static_cast<T>(0.0), kHorizon<T>
        )};
        const T settled_distance{kAdaptive ? static_cast<T>(0.3) : static_cast<T>(0.2)};
        const ClosedLoop<T, kAdaptive> loop{
            runClosedLoop(controller, start, reference, h, kNumSteps)
        };
        for (const IntegrateMode mode : kIntegrateModes) {
            const std::vector<State<T, kAdaptive>>& trace{loop.trace(mode)};
            CHECK(std::ranges::all_of(trace, isWithinBounds<T, kAdaptive>));
            CHECK(std::ranges::any_of(trace, isSaturated<T, kAdaptive>));
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
        // horizon at kHorizon, so there is no offset to close or to take up. The loop then stays
        // within 5 cm of the simultaneous points with either horizon, for every integrator. What
        // it leaves is mostly the reference itself: between two waypoints the interpolation cuts
        // the corner of the path they were taken off by up to some 3 cm, and it kinks at every
        // waypoint, neither of which the vehicle can follow exactly.
        const State<T, kAdaptive>& on{reference.ys().front()};
        const State<T, kAdaptive> start{stateOf<T, kAdaptive>(
            on.x, on.y, on.yaw, on.delta, on.u, static_cast<T>(0.0), static_cast<T>(0.12),
            kHorizon<T>
        )};
        const T settled_distance{static_cast<T>(0.05)};
        const ClosedLoop<T, kAdaptive> loop{
            runClosedLoop(controller, start, reference, h, kNumSteps)
        };
        for (const IntegrateMode mode : kIntegrateModes) {
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

TEST_CASE_TEMPLATE("StabilityBoundsOfTheEulerSteps", T, float, double) {
    // Along a straight line with unit weights on position and u alone, the closed loop stays on the
    // line and is linear in the along track error, its rate and the acceleration, so an Euler step
    // of it is stable only above the bounds documented with KinematicsFlowController::tangent().
    // Every step takes as its preview point the point of the line the horizon past where the
    // reference is as the step starts, and the error is taken against the simultaneous point,
    // where the reference is as the step ends. Three percent to either side of a bound leaves the
    // spectral radius about one percent off one, which shrinks or grows the error by some four
    // orders of magnitude over a thousand steps. The envelope over the last hundred steps keeps a
    // zero crossing of the oscillating error from passing for decay.
    const KinematicsModel<T> model{unboundedModel<T>()};
    const std::array<T, 4> weights{
        static_cast<T>(1.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(1.0)
    };
    const T h{kH<T>};
    constexpr T kSpeed{static_cast<T>(2.0)};
    constexpr T kOffset{static_cast<T>(-0.1)};
    constexpr std::size_t kNumSteps{1000};
    constexpr std::size_t kWindow{100};
    const auto envelope = [&](IntegrateMode mode, T tau) -> T {
        const Controller<T, false> controller{model, std::span<const T, 4>{weights}, tau};
        State<T, false> state{
            kOffset, static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0),
            kSpeed,  static_cast<T>(0.0), static_cast<T>(0.0)
        };
        T result{static_cast<T>(0.0)};
        for (std::size_t i{0}; i < kNumSteps; ++i) {
            const T t{h * static_cast<T>(i)};
            state = stepWith(
                mode, controller, state,
                targetAt<T, false>(
                    kSpeed * (t + tau), static_cast<T>(0.0), static_cast<T>(0.0),
                    static_cast<T>(0.0), kSpeed
                ),
                h
            );
            if (i + kWindow >= kNumSteps) {
                const T error{std::abs(state.x - kSpeed * (t + h))};
                if (!(error <= result)) {
                    result = error;
                }
            }
        }
        return result;
    };
    for (const auto& [mode, bound] :
         {std::pair{IntegrateMode::FORWARD_EULER, static_cast<T>(1.4077050200457586)},
          std::pair{IntegrateMode::SYMPLECTIC_EULER, static_cast<T>(18.0 / 11.0)}}) {
        CHECK_LT(
            envelope(mode, bound * static_cast<T>(1.03) * h),
            static_cast<T>(0.01) * std::abs(kOffset)
        );
        CHECK_GT(
            envelope(mode, bound * static_cast<T>(0.97) * h),
            static_cast<T>(100.0) * std::abs(kOffset)
        );
    }
}

TEST_CASE_TEMPLATE("AdaptiveMatchesTheFixedHorizonControllerAtItsHorizon", T, float, double) {
    // At the horizon a state carries, the adaptive controller walks the arithmetic of the fixed one
    // built with that horizon: the plant fields agree bit for bit, and the rates and the cost to
    // within roundoff.
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T, true> adaptive{controllerOf<T, true>(model)};
    const State<T, true> target{targetOf<T, true>()};
    const State<T, false> fixed_target{target.x, target.y, target.yaw,  target.delta,
                                       target.u, target.a, target.omega};
    const T tolerance{roundoffTolerance<T>()};
    for (const State<T, true>& state : statesOf<T, true>()) {
        const Controller<T, false> fixed{controllerOf<T, false>(model, kWeights<T>, state.tau)};
        const State<T, false> fixed_state{state.x, state.y, state.yaw,  state.delta,
                                          state.u, state.a, state.omega};
        const State<T, false> expected{fixed.tangent(fixed_state, fixed_target)};
        const State<T, true> k{adaptive.tangent(state, target)};
        CHECK_EQ(plantOf(k), plantOf(expected));
        CHECK_EQ(k.a, doctest::Approx(expected.a).epsilon(tolerance));
        CHECK_EQ(k.omega, doctest::Approx(expected.omega).epsilon(tolerance));
        CHECK_EQ(
            adaptive.cost(state, target),
            doctest::Approx(fixed.cost(fixed_state, fixed_target)).epsilon(tolerance)
        );
    }
}

TEST_CASE_TEMPLATE("AdaptiveAlongTrackOffsetsCostNothing", T, float, double) {
    // Driving straight at a target ahead with its speed and both controls at zero, a distance d
    // short of it, tau = d / u zeroes every residual whatever d (see
    // KinematicsFlowController::tangent()). The cost vanishes there, and since it never drops below
    // zero, so does its gradient, which leaves the three rates nothing to descend: the horizon
    // takes up an along track offset instead of the loop closing it. The cost is measured against
    // the one at a horizon a fifth off, which is required to be positive, or the check could not
    // tell that horizon apart from any other.
    const Controller<T, true> controller{controllerOf<T, true>(boundedModel<T>())};
    const T yaw{kInitialYaw<T>};
    const T u{static_cast<T>(8.0)};
    for (const T distance : {static_cast<T>(0.5), static_cast<T>(2.0), static_cast<T>(6.0)}) {
        const State<T, true> target{targetAt<T, true>(
            distance * std::cos(yaw), distance * std::sin(yaw), yaw, static_cast<T>(0.0), u
        )};
        const State<T, true> state{
            static_cast<T>(0.0), static_cast<T>(0.0), yaw,         static_cast<T>(0.0), u,
            static_cast<T>(0.0), static_cast<T>(0.0), distance / u
        };
        State<T, true> off{state};
        off.tau *= static_cast<T>(1.2);
        const T off_cost{controller.cost(off, target)};
        REQUIRE_GT(off_cost, static_cast<T>(0.0));
        CHECK_LE(controller.cost(state, target), roundoffTolerance<T>() * off_cost);
    }
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE(
    "RejectsInvalidArguments", P, FixedFloat, AdaptiveFloat, FixedDouble, AdaptiveDouble
) {
    using T = P::value_type;
    constexpr bool kAdaptive{P::kAdaptive};
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T, kAdaptive> controller{controllerOf<T, kAdaptive>(model)};
    const State<T, kAdaptive> start{startOf<T, kAdaptive>()};
    const State<T, kAdaptive> target{targetOf<T, kAdaptive>()};

    SUBCASE("SingularSteeringAngle") {
        // The steering angle is carried by the state, so this is where an omega that drove it out
        // of range over an earlier step surfaces.
        State<T, kAdaptive> state{start};
        state.delta = std::numbers::pi_v<T> * static_cast<T>(0.5);
        CHECK_THROWS_AS(
            static_cast<void>(controller.tangent(state, target)), std::invalid_argument
        );
        CHECK_THROWS_AS(static_cast<void>(controller.cost(state, target)), std::invalid_argument);
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK_THROWS_AS(
                static_cast<void>(stepWith(mode, controller, state, target, kH<T>)),
                std::invalid_argument
            );
        }
    }

    SUBCASE("NonPositiveTimeStep") {
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
        // An Euler step is fine with a horizon that spans exactly the step, while the last stage
        // of rungeKutta() looks ahead over the horizon less the step, which has to stay positive
        // for the target to lie ahead of that stage, so it needs strictly more.
        const T h{kH<T>};
        const auto step_over = [&](IntegrateMode mode, T tau) -> State<T, kAdaptive> {
            const Controller<T, kAdaptive> horizon_controller{
                controllerOf<T, kAdaptive>(model, kWeights<T>, tau)
            };
            return stepWith(mode, horizon_controller, withHorizon(start, tau), target, h);
        };
        for (const IntegrateMode mode :
             {IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER}) {
            CHECK_NOTHROW(static_cast<void>(step_over(mode, h)));
            CHECK_THROWS_AS(
                static_cast<void>(step_over(mode, h * static_cast<T>(0.5))), std::invalid_argument
            );
        }
        CHECK_THROWS_AS(
            static_cast<void>(step_over(IntegrateMode::RUNGE_KUTTA, h)), std::invalid_argument
        );
        CHECK_NOTHROW(
            static_cast<void>(step_over(IntegrateMode::RUNGE_KUTTA, h * static_cast<T>(1.01)))
        );
    }

    SUBCASE("NonPositiveHorizon") {
        // A fixed horizon is checked once, when the controller is built, an adaptive one wherever
        // a state is handed in.
        for (const T tau : {static_cast<T>(0.0), -kH<T>, std::numeric_limits<T>::quiet_NaN()}) {
            if constexpr (kAdaptive) {
                State<T, kAdaptive> state{start};
                state.tau = tau;
                CHECK_THROWS_AS(
                    static_cast<void>(controller.tangent(state, target)), std::invalid_argument
                );
                CHECK_THROWS_AS(
                    static_cast<void>(controller.cost(state, target)), std::invalid_argument
                );
            } else {
                CHECK_THROWS_AS(
                    static_cast<void>(controllerOf<T, kAdaptive>(model, kWeights<T>, tau)),
                    std::invalid_argument
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

} // namespace basjoo::bicycle

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options(
        "kinematics_flow_controller_test", "unit test of KinematicsFlowController class"
    );
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
