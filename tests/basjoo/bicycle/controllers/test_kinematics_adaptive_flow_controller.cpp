/**
 * @file basjoo/bicycle/controllers/test_kinematics_adaptive_flow_controller.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Unit test of KinematicsAdaptiveFlowController.
 * @version 0.1
 * @date 2026-10-16
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/bicycle/controllers/kinematics_adaptive_flow_controller.hpp"

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

#include "basjoo/bicycle/controllers/kinematics_flow_controller.hpp"
#include "basjoo/bicycle/models/kinematics_model.hpp"
#include "basjoo/math/integrate_mode.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

namespace {

bool plot_graph{false};

auto createFigureHandle() noexcept -> matplot::figure_handle {
    matplot::figure_handle fig = matplot::figure();
    fig->size(2400, 1000);
    return fig;
}

auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    const std::array<std::pair<const char*, const char*>, 7> labels{
        {{"x", "y"},
         {"t", "distance to reference"},
         {"t", "u"},
         {"t", "delta"},
         {"t", "a"},
         {"t", "omega"},
         {"t", "tau"}}
    };
    std::vector<matplot::axes_handle> axes;
    axes.reserve(labels.size());
    for (std::size_t i = 0; i < labels.size(); ++i) {
        axes.emplace_back(fig->add_subplot(2, 4, i));
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
// How far ahead along a reference every target of a closed loop run lies, which is also where a
// closed loop run starts its horizon.
template <std::floating_point T>
inline constexpr T kLead{static_cast<T>(0.15)};
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};

inline constexpr std::array<IntegrateMode, 3> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER, IntegrateMode::RUNGE_KUTTA
};

template <std::floating_point T>
using State = detail::KinematicsAdaptiveFlowControllerState<T>;
template <std::floating_point T>
using ModelState = detail::KinematicsModelState<T>;
template <std::floating_point T>
using Controller = KinematicsAdaptiveFlowController<T>;

/**
 * @brief Five cost weights that differ from each other and from one.
 *
 * A default constructed controller weighs every term by one, so it takes weights like these to
 * tell a weight that went missing, or landed on the wrong term, apart from one that did not.
 */
template <std::floating_point T>
inline constexpr std::array<T, 5> kWeights{
    static_cast<T>(0.7), static_cast<T>(1.3), static_cast<T>(0.9), static_cast<T>(1.1),
    static_cast<T>(0.8)
};

template <std::floating_point T>
[[nodiscard]] auto boundedModel() -> KinematicsModel<T> {
    return KinematicsModel<T>{kFrontWheelbase<T>, kRearWheelbase<T>, kALo<T>, kAUp<T>,
                              kOmegaLo<T>,        kOmegaUp<T>};
}

template <std::floating_point T>
[[nodiscard]] auto unboundedModel() -> KinematicsModel<T> {
    return KinematicsModel<T>{kFrontWheelbase<T>, kRearWheelbase<T>};
}

template <std::floating_point T>
[[nodiscard]] auto controllerOf(
    const KinematicsModel<T>& model,
    const std::type_identity_t<std::array<T, 5>>& weights = kWeights<T>
) -> Controller<T> {
    return Controller<T>{model, std::span<const T, 5>{weights}};
}

/**
 * @brief A target state: tangent() ignores the a, omega and tau of a target, so they are left NaN,
 * which any use of them would carry into the result.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto targetAt(T x, T y, T yaw, T delta, T u) noexcept -> State<T> {
    constexpr T kNaN{std::numeric_limits<T>::quiet_NaN()};
    return State<T>{x, y, yaw, delta, u, kNaN, kNaN, kNaN};
}

template <std::floating_point T>
[[nodiscard]] constexpr auto plantOf(const State<T>& state) noexcept -> ModelState<T> {
    return ModelState<T>{state.x, state.y, state.yaw, state.delta, state.u};
}

/**
 * @brief A vehicle turning left at speed, with both controls inside the bounds and a horizon off
 * its default of one, so that a horizon that went missing somewhere shows.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto startOf() noexcept -> State<T> {
    return State<T>{static_cast<T>(0.0),  static_cast<T>(0.0), kInitialYaw<T>,
                    static_cast<T>(0.15), static_cast<T>(8.0), static_cast<T>(-0.6),
                    static_cast<T>(0.03), static_cast<T>(0.5)};
}

/**
 * @brief A target ahead of startOf(), off by enough in every residual that all three rates at the
 * start are far from zero.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto targetOf() noexcept -> State<T> {
    return targetAt(
        static_cast<T>(1.3), static_cast<T>(0.38), static_cast<T>(0.34), static_cast<T>(0.3),
        static_cast<T>(9.0)
    );
}

/**
 * @brief Turning left, turning right and reversing, each with both controls inside the bounds and
 * a horizon well clear of zero.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto statesOf() noexcept -> std::array<State<T>, 3> {
    return {
        startOf<T>(),
        State<T>{
            static_cast<T>(3.0), static_cast<T>(-2.0), static_cast<T>(-1.1), static_cast<T>(-0.2),
            static_cast<T>(5.0), static_cast<T>(1.2), static_cast<T>(-0.05), static_cast<T>(0.4)
        },
        State<T>{
            static_cast<T>(-1.0), static_cast<T>(4.0), static_cast<T>(2.5), static_cast<T>(0.1),
            static_cast<T>(-3.0), static_cast<T>(-0.4), static_cast<T>(0.2), static_cast<T>(0.8)
        }
    };
}

/**
 * @brief A reference trajectory sampled every h, starting at the origin at speed and steered by
 * two smooth open loop controls. Every sample is a target.
 */
template <std::floating_point T>
[[nodiscard]] auto referenceOf(const KinematicsModel<T>& model, T h, std::size_t num_steps)
    -> std::vector<State<T>> {
    std::vector<State<T>> reference;
    reference.reserve(num_steps + 1);
    ModelState<T> plant{
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.0),
        static_cast<T>(8.0)
    };
    reference.push_back(targetAt(plant.x, plant.y, plant.yaw, plant.delta, plant.u));
    for (std::size_t i{0}; i < num_steps; ++i) {
        const T t{h * static_cast<T>(i)};
        plant = model.rungeKutta(
            plant, static_cast<T>(0.8) * std::sin(static_cast<T>(0.4) * t),
            static_cast<T>(0.12) * std::cos(static_cast<T>(0.5) * t), h
        );
        reference.push_back(targetAt(plant.x, plant.y, plant.yaw, plant.delta, plant.u));
    }
    return reference;
}

/**
 * @brief Bound on the drift between two code paths that walk the same arithmetic.
 *
 * The compiler is free to fuse a multiply and an add into one FMA at one call site and not at
 * another, for instance once a test hands one state to both the controller and the model and the
 * two share a product that ends up computed only once. That moves the last bit or so of a step,
 * and the closed loop does not amplify it over the few steps any comparison below marches. Every
 * defect such a comparison is meant to catch lands at least an order of magnitude above the bound,
 * which the tests require explicitly wherever that is not obvious.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto roundoffTolerance() noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(1.0e-5) : static_cast<T>(1.0e-12);
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
 * @brief Both controls within the bounds and a positive horizon, which also fails on a NaN.
 */
template <std::floating_point T>
[[nodiscard]] auto isWithinBounds(const State<T>& state) noexcept -> bool {
    return kALo<T> <= state.a && state.a <= kAUp<T> && kOmegaLo<T> <= state.omega &&
           state.omega <= kOmegaUp<T> && static_cast<T>(0.0) < state.tau;
}

template <std::floating_point T>
[[nodiscard]] auto isSaturated(const State<T>& state) noexcept -> bool {
    return state.a == kALo<T> || state.a == kAUp<T> || state.omega == kOmegaLo<T> ||
           state.omega == kOmegaUp<T>;
}

template <std::floating_point T>
[[nodiscard]] auto stepWith(
    IntegrateMode mode, const Controller<T>& controller, const State<T>& state,
    const State<T>& target_state, std::type_identity_t<T> h
) -> State<T> {
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

template <std::floating_point T>
auto evolveWith(
    IntegrateMode mode, const Controller<T>& controller, const State<T>& initial_state,
    std::type_identity_t<std::span<const State<T>>> target_states,
    std::type_identity_t<std::span<const T>> hs,
    std::type_identity_t<std::span<State<T>>> trace = {}
) -> State<T> {
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
 * @brief One classic Runge-Kutta step spelled out through tangent(), the later stages looking ahead
 * over the horizon they have reached less stage_times times their stage time.
 *
 * With stage_times at one it walks the same arithmetic as rungeKutta(), so the two part only by
 * roundoff, and at zero it hands every stage the horizon it has reached as it is.
 */
template <std::floating_point T>
[[nodiscard]] auto rungeKuttaThroughTangent(
    const Controller<T>& controller, const State<T>& state, const State<T>& target_state, T h,
    T stage_times
) -> State<T> {
    constexpr T kHalf{static_cast<T>(0.5)};
    constexpr T kSixth{static_cast<T>(1.0 / 6.0)};
    constexpr T kThird{static_cast<T>(1.0 / 3.0)};
    const KinematicsModel<T>& model{controller.model()};
    const State<T> k1{controller.tangent(state, target_state) * h};
    State<T> stage{state + k1 * kHalf};
    stage.tau -= stage_times * h * kHalf;
    const State<T> k2{controller.tangent(stage, target_state) * h};
    stage = state + k2 * kHalf;
    stage.tau -= stage_times * h * kHalf;
    const State<T> k3{controller.tangent(stage, target_state) * h};
    stage = state + k3;
    stage.tau -= stage_times * h;
    const State<T> k4{controller.tangent(stage, target_state) * h};
    State<T> result{state};
    result += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
    result.a = std::clamp(result.a, model.a_lower_bound(), model.a_upper_bound());
    result.omega = std::clamp(result.omega, model.omega_lower_bound(), model.omega_upper_bound());
    return result;
}

/**
 * @brief The tracking cost J of tangent(), written out from the residuals
 * KinematicsFlowController::tangent() lists, over the horizon the state carries and with the
 * controls taken unclamped.
 *
 * It shares no code with the rates of the controller, which are transcribed from Mathematica, so
 * the two only agree where the transcription does.
 */
template <std::floating_point T>
[[nodiscard]] auto costOf(
    const Controller<T>& controller, const State<T>& state, const State<T>& target_state
) -> T {
    const std::span<const T> w{controller.weights()};
    const T L{controller.model().wheelbase()};
    const T lr{controller.model().rear_wheelbase()};
    const T tau{state.tau};
    const T sin_yaw{std::sin(state.yaw)};
    const T cos_yaw{std::cos(state.yaw)};
    const T tan_delta{std::tan(state.delta)};
    const T sec2_delta{static_cast<T>(1.0) + tan_delta * tan_delta};
    const T u1{state.u + state.a * tau};
    const T r{state.u * tan_delta / L};
    const T v{lr * r};
    const T r1{(u1 * tan_delta + tau * state.u * state.omega * sec2_delta) / L};
    const T v1{lr * r1};
    const T rx{
        state.x - target_state.x +
        tau * (u1 * cos_yaw - v1 * sin_yaw - tau * r * (state.u * sin_yaw + v * cos_yaw))
    };
    const T ry{
        state.y - target_state.y +
        tau * (u1 * sin_yaw + v1 * cos_yaw + tau * r * (state.u * cos_yaw - v * sin_yaw))
    };
    const T ryaw{state.yaw - target_state.yaw + tau * r1};
    const T rdelta{state.delta - target_state.delta + tau * state.omega};
    const T ru{u1 - target_state.u};
    return w[0] * (rx / L) * (rx / L) + w[1] * (ry / L) * (ry / L) + w[2] * ryaw * ryaw +
           w[3] * rdelta * rdelta + w[4] * (tau * ru / L) * (tau * ru / L);
}

/**
 * @brief Derivative of f along one field of the state, from the five point stencil.
 *
 * The stencil is exact for polynomials up to degree four, which covers the cost along a, omega and
 * tau: it is quadratic in the two controls and quartic in the horizon. So it recovers those
 * derivatives up to rounding whatever the step, and a wide step keeps the rounding small.
 */
template <std::floating_point T>
[[nodiscard]] auto derivativeOf(const auto& f, const State<T>& state, T State<T>::* field, T step)
    -> T {
    const auto at = [&](T shift) -> T {
        State<T> shifted{state};
        shifted.*field += shift;
        return f(shifted);
    };
    return (at(-static_cast<T>(2.0) * step) - static_cast<T>(8.0) * at(-step) +
            static_cast<T>(8.0) * at(step) - at(static_cast<T>(2.0) * step)) /
           (static_cast<T>(12.0) * step);
}

/**
 * @brief One closed loop run of every integrator, each trace led by the initial state so that it
 * lines up with ts and with the reference samples the run is plotted against.
 */
template <std::floating_point T>
struct ClosedLoop final {
    std::vector<T> ts;
    std::vector<State<T>> reference;
    std::array<std::vector<State<T>>, kIntegrateModes.size()> traces;

    [[nodiscard]] auto trace(IntegrateMode mode) const noexcept -> const std::vector<State<T>>& {
        return traces[static_cast<std::size_t>(mode)];
    }
};

/**
 * @brief Marches every integrator through evolve() along a reference sampled every sample_h.
 *
 * Each of the num_steps steps spans stride samples and targets the sample lead samples ahead of
 * where it starts.
 */
template <std::floating_point T>
[[nodiscard]] auto runClosedLoop(
    const Controller<T>& controller, const State<T>& initial_state,
    const std::vector<State<T>>& reference, T sample_h, std::size_t lead, std::size_t num_steps,
    std::size_t stride = 1
) -> ClosedLoop<T> {
    const T h{sample_h * static_cast<T>(stride)};
    std::vector<State<T>> targets;
    targets.reserve(num_steps);
    for (std::size_t i{0}; i < num_steps; ++i) {
        targets.push_back(reference[stride * i + lead]);
    }
    const std::vector<T> hs(num_steps, h);
    ClosedLoop<T> loop;
    for (std::size_t i{0}; i <= num_steps; ++i) {
        loop.ts.push_back(h * static_cast<T>(i));
        loop.reference.push_back(reference[stride * i]);
    }
    for (const IntegrateMode mode : kIntegrateModes) {
        std::vector<State<T>>& trace{loop.traces[static_cast<std::size_t>(mode)]};
        trace.resize(num_steps + 1);
        static_cast<void>(evolveWith(mode, controller, initial_state, targets, hs, trace));
    }
    return loop;
}

/**
 * @brief Largest distance to the reference from the given step on.
 *
 * Written so that a NaN propagates into the result, as in distanceOf().
 */
template <std::floating_point T>
[[nodiscard]] auto settledDistanceOf(
    const ClosedLoop<T>& loop, IntegrateMode mode, std::size_t from
) noexcept -> T {
    const std::vector<State<T>>& trace{loop.trace(mode)};
    T result{static_cast<T>(0.0)};
    for (std::size_t i{from}; i < trace.size(); ++i) {
        const T distance{
            std::hypot(trace[i].x - loop.reference[i].x, trace[i].y - loop.reference[i].y)
        };
        if (!(distance <= result)) {
            result = distance;
        }
    }
    return result;
}

auto plotClosedLoop(const std::string& title, const ClosedLoop<double>& loop) -> void {
    std::vector<double> reference_xs, reference_ys, reference_us, reference_deltas;
    for (const State<double>& reference : loop.reference) {
        reference_xs.push_back(reference.x);
        reference_ys.push_back(reference.y);
        reference_us.push_back(reference.u);
        reference_deltas.push_back(reference.delta);
    }

    matplot::figure_handle fig = createFigureHandle();
    fig->title(title);
    std::vector<matplot::axes_handle> axes = createAxesHandles(fig);
    axes[0]->plot(reference_xs, reference_ys, "-")->line_width(2.0);
    axes[0]
        ->plot(
            std::vector<double>{reference_xs.back()}, std::vector<double>{reference_ys.back()}, "o"
        )
        ->line_width(2.0);
    axes[2]->plot(loop.ts, reference_us, "-")->line_width(2.0);
    axes[3]->plot(loop.ts, reference_deltas, "-")->line_width(2.0);
    const std::array<const char*, kIntegrateModes.size()> line_specs{":", "-.", "--"};
    for (const IntegrateMode mode : kIntegrateModes) {
        std::vector<double> xs, ys, distances, us, deltas, accels, omegas, taus;
        for (std::size_t i{0}; i < loop.ts.size(); ++i) {
            const State<double>& state{loop.trace(mode)[i]};
            const State<double>& reference{loop.reference[i]};
            xs.push_back(state.x);
            ys.push_back(state.y);
            distances.push_back(std::hypot(state.x - reference.x, state.y - reference.y));
            us.push_back(state.u);
            deltas.push_back(state.delta);
            accels.push_back(state.a);
            omegas.push_back(state.omega);
            taus.push_back(state.tau);
        }
        const char* line_spec{line_specs[static_cast<std::size_t>(mode)]};
        axes[0]->plot(xs, ys, line_spec)->line_width(2.0);
        axes[1]->plot(loop.ts, distances, line_spec)->line_width(2.0);
        axes[2]->plot(loop.ts, us, line_spec)->line_width(2.0);
        axes[3]->plot(loop.ts, deltas, line_spec)->line_width(2.0);
        axes[4]->plot(loop.ts, accels, line_spec)->line_width(2.0);
        axes[5]->plot(loop.ts, omegas, line_spec)->line_width(2.0);
        axes[6]->plot(loop.ts, taus, line_spec)->line_width(2.0);
    }
    axes[0]->axis(matplot::equal);
    const std::vector<std::string> legend{"forward euler", "symplectic euler", "runge kutta"};
    const std::vector<std::string> legend_with_reference{
        "reference", "forward euler", "symplectic euler", "runge kutta"
    };
    axes[0]->legend(
        {"reference", "last reference", "forward euler", "symplectic euler", "runge kutta"}
    );
    axes[1]->legend(legend);
    axes[2]->legend(legend_with_reference);
    axes[3]->legend(legend_with_reference);
    axes[4]->legend(legend);
    axes[5]->legend(legend);
    axes[6]->legend(legend);
    fig->show();
}

} // namespace

// The controller carries a model and five weights and nothing else, and hands both back as it was
// given them.

TEST_CASE_TEMPLATE("ConstructorKeepsTheModelAndTheWeights", T, float, double) {
    const Controller<T> controller{boundedModel<T>(), std::span<const T, 5>{kWeights<T>}};
    // The controller has no geometry or bounds of its own, so model() has to hand back the very
    // model it was built with.
    const KinematicsModel<T>& model{controller.model()};
    CHECK_EQ(model.front_wheelbase(), kFrontWheelbase<T>);
    CHECK_EQ(model.rear_wheelbase(), kRearWheelbase<T>);
    CHECK_EQ(model.a_lower_bound(), kALo<T>);
    CHECK_EQ(model.a_upper_bound(), kAUp<T>);
    CHECK_EQ(model.omega_lower_bound(), kOmegaLo<T>);
    CHECK_EQ(model.omega_upper_bound(), kOmegaUp<T>);
    // The weights come back in the order they went in, and no two of them agree, so a shuffle
    // cannot pass.
    CHECK(std::ranges::equal(controller.weights(), kWeights<T>));
}

TEST_CASE_TEMPLATE("ControllerSerializationRoundtrip", T, float, double) {
    const Controller<T> controller{controllerOf(boundedModel<T>())};
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(controller).code == std::errc{});
    // A default constructed controller has unit weights and a unit, unbounded model, none of which
    // agrees with the original, so every field checked below has to have come through the archive.
    Controller<T> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.model().front_wheelbase(), kFrontWheelbase<T>);
    CHECK_EQ(restored.model().rear_wheelbase(), kRearWheelbase<T>);
    CHECK_EQ(restored.model().a_lower_bound(), kALo<T>);
    CHECK_EQ(restored.model().a_upper_bound(), kAUp<T>);
    CHECK_EQ(restored.model().omega_lower_bound(), kOmegaLo<T>);
    CHECK_EQ(restored.model().omega_upper_bound(), kOmegaUp<T>);
    CHECK(std::ranges::equal(restored.weights(), kWeights<T>));
    // Same arithmetic along the same code path, so a restored controller has to reproduce the flow
    // of the original to within roundoff: the two calls are inlined separately and the compiler
    // may fuse them differently. Roundoff is relative, so the bound scales with the largest field
    // of the flow. A weight or a bound that went missing moves it far more.
    const State<T> original{controller.tangent(startOf<T>(), targetOf<T>())};
    T magnitude{static_cast<T>(1.0)};
    for (std::size_t i{0}; i < State<T>::size(); ++i) {
        magnitude = std::max(magnitude, std::abs(original.data()[i]));
    }
    CHECK_LE(
        distanceOf(restored.tangent(startOf<T>(), targetOf<T>()), original),
        roundoffTolerance<T>() * magnitude
    );
}

TEST_CASE_TEMPLATE("StateSerializationRoundtrip", T, float, double) {
    for (const State<T>& state : {State<T>{}, startOf<T>()}) {
        auto [data, in, out] = zpp::bits::data_in_out();
        REQUIRE(out(state).code == std::errc{});
        State<T> restored{};
        REQUIRE(in(restored).code == std::errc{});
        CHECK_EQ(restored, state);
    }
}

// tangent() is the whole closed loop flow: the plant flow of the model in its first five fields,
// and in a, omega and tau the rates at which the two controls and the horizon descend the tracking
// cost.

TEST_CASE_TEMPLATE("TangentCarriesThePlantFlowOfTheModel", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    const State<T> target{targetOf<T>()};
    // The same state with both controls beyond the bounds.
    const auto loose = [](State<T> state) -> State<T> {
        state.a = static_cast<T>(50.0);
        state.omega = static_cast<T>(-7.0);
        return state;
    };

    SUBCASE("PlantFieldsEqualTheModelTangent") {
        // The controller spells the plant flow out itself instead of calling the model, so that
        // the flow and the cost gradient can share their trigonometry. This is what keeps that copy
        // honest: any change to KinematicsModel::tangent() that is not mirrored in the controller
        // fails here bit for bit, and so does a clamp that went missing.
        for (const State<T>& state : statesOf<T>()) {
            for (const State<T>& handed : {state, loose(state)}) {
                CHECK_EQ(
                    plantOf(controller.tangent(handed, target)),
                    model.tangent(plantOf(handed), handed.a, handed.omega)
                );
            }
        }
    }

    SUBCASE("LooseControlsChangeNothing") {
        // The controls enter the flow only through their bounds, so controls beyond the bounds
        // may not move a single bit of the flow. A cost that saw an unclamped control would show up
        // in the three rates.
        for (const State<T>& state : statesOf<T>()) {
            State<T> clamped{state};
            clamped.a = kAUp<T>;
            clamped.omega = kOmegaLo<T>;
            CHECK_EQ(controller.tangent(loose(state), target), controller.tangent(clamped, target));
        }
    }
}

TEST_CASE_TEMPLATE("RatesAreThePreconditionedNegativeGradientOfTheCost", T, float, double) {
    // The rates are da/dt = -(L^2/tau^5) dJ/da, domega/dt = -(1/tau^3) dJ/domega and dtau/dt =
    // -tau dJ/dtau. costOf() writes J out from the residuals independently of the transcribed
    // rates, and derivativeOf() recovers its derivatives up to rounding, so any slip in the
    // transcription shows here, even one made consistently across all three rates. The model is
    // unbounded, so no clamp cuts into the stencil.
    const Controller<T> controller{controllerOf(unboundedModel<T>())};
    const State<T> target{targetOf<T>()};
    const auto cost = [&](const State<T>& state) -> T { return costOf(controller, state, target); };
    const T wheelbase2{kWheelbase<T> * kWheelbase<T>};
    // The cost is quadratic in the controls, so any step does for them and a wide one keeps the
    // rounding of the stencil down, all the wider for the acceleration, which enters the residuals
    // only through tau or tau^2. The horizon has to stay clear of zero across its stencil. The two
    // sides round differently, the stencil all the more for differencing nearby costs, and the
    // tolerance leaves well over an order of magnitude on top of both.
    constexpr T kAStep{static_cast<T>(10.0)};
    constexpr T kOmegaStep{static_cast<T>(1.0)};
    constexpr T kTauStep{static_cast<T>(0.05)};
    const double tolerance{std::same_as<T, float> ? 1.0e-4 : 1.0e-12};
    for (const State<T>& state : statesOf<T>()) {
        const T tau{state.tau};
        const State<T> k{controller.tangent(state, target)};
        CHECK_EQ(
            k.a, doctest::Approx(
                     -wheelbase2 / (tau * tau * tau * tau * tau) *
                     derivativeOf(cost, state, &State<T>::a, kAStep)
                 )
                     .epsilon(tolerance)
        );
        CHECK_EQ(
            k.omega,
            doctest::Approx(
                -derivativeOf(cost, state, &State<T>::omega, kOmegaStep) / (tau * tau * tau)
            )
                .epsilon(tolerance)
        );
        CHECK_EQ(
            k.tau, doctest::Approx(-tau * derivativeOf(cost, state, &State<T>::tau, kTauStep))
                       .epsilon(tolerance)
        );
    }
}

TEST_CASE_TEMPLATE("ControlRatesMatchTheFixedHorizonController", T, float, double) {
    // At the horizon a state carries, the plant flow and the two control rates are those of
    // KinematicsFlowController built with that horizon. The two controllers transcribe the rates
    // from Mathematica output arranged differently, so the comparison holds each to the other:
    // the plant fields walk the same arithmetic and agree bit for bit, while the rates part by
    // roundoff at most.
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    const State<T> target{targetOf<T>()};
    const detail::KinematicsFlowControllerState<T> fixed_target{target.x,     target.y, target.yaw,
                                                                target.delta, target.u, target.a,
                                                                target.omega};
    const T tolerance{roundoffTolerance<T>()};
    for (const State<T>& state : statesOf<T>()) {
        const KinematicsFlowController<T> fixed{
            model, std::span<const T, 5>{kWeights<T>}, state.tau
        };
        const detail::KinematicsFlowControllerState<T> expected{fixed.tangent(
            {state.x, state.y, state.yaw, state.delta, state.u, state.a, state.omega}, fixed_target
        )};
        const State<T> k{controller.tangent(state, target)};
        CHECK_EQ(
            plantOf(k),
            ModelState<T>{expected.x, expected.y, expected.yaw, expected.delta, expected.u}
        );
        CHECK_EQ(k.a, doctest::Approx(expected.a).epsilon(tolerance));
        CHECK_EQ(k.omega, doctest::Approx(expected.omega).epsilon(tolerance));
    }
}

TEST_CASE_TEMPLATE("AlongTrackOffsetsCostNothing", T, float, double) {
    // Driving straight at a target ahead with its speed and both controls at zero, a distance d
    // short of it, with tau = d / u zeroes every residual whatever d (see tangent()), so none of
    // the three rates may move: the horizon takes up an along track offset instead of the loop
    // closing it.
    //
    // The preconditioners blow the rounding of the residuals up by as much as they blow the rates
    // up, so the acceleration and the horizon are measured against the rates they reach once the
    // horizon is off that value by a fifth, which are required to stand well clear of zero, or the
    // check could not tell. The steering rate has nothing to steer on a straight line either way:
    // what it acts on is lateral, and the offset is along the track.
    const Controller<T> controller{controllerOf(boundedModel<T>())};
    const T yaw{kInitialYaw<T>};
    const T u{static_cast<T>(8.0)};
    const T tolerance{roundoffTolerance<T>()};
    for (const T distance : {static_cast<T>(0.5), static_cast<T>(2.0), static_cast<T>(6.0)}) {
        const State<T> target{targetAt(
            distance * std::cos(yaw), distance * std::sin(yaw), yaw, static_cast<T>(0.0), u
        )};
        const State<T> state{
            static_cast<T>(0.0), static_cast<T>(0.0), yaw,         static_cast<T>(0.0), u,
            static_cast<T>(0.0), static_cast<T>(0.0), distance / u
        };
        State<T> off{state};
        off.tau *= static_cast<T>(1.2);
        const State<T> k{controller.tangent(state, target)};
        const State<T> k_off{controller.tangent(off, target)};
        REQUIRE_GT(std::abs(k_off.a), static_cast<T>(1.0e3) * tolerance);
        REQUIRE_GT(std::abs(k_off.tau), static_cast<T>(1.0e3) * tolerance);
        CHECK_LE(std::abs(k.a), tolerance * std::abs(k_off.a));
        CHECK_LE(std::abs(k.tau), tolerance * std::abs(k_off.tau));
        CHECK_LE(std::abs(k.omega), tolerance);
    }
}

// The single step integrators step the plant the way the model does and move the controls and the
// horizon along the rates. evolve() only marches the single step ones.

TEST_CASE_TEMPLATE("SingleStepIntegratorsDriveThePlantThroughTheModel", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    const T tolerance{roundoffTolerance<T>()};
    const T h{kH<T>};
    const State<T> start{startOf<T>()};
    const State<T> target{targetOf<T>()};
    const State<T> k{controller.tangent(start, target)};
    // The comparisons below hold the integrators to a tolerance, so the rates at this start have
    // to be large enough that holding a control or the horizon still over a step, or stepping the
    // plant under the updated controls instead of the held ones, lands far outside it. The latter
    // moves delta and u by h^2 times a rate.
    REQUIRE_GT(
        h * h * std::min({std::abs(k.a), std::abs(k.omega), std::abs(k.tau)}),
        static_cast<T>(10.0) * tolerance
    );

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
        // The controls and the horizon take one Euler step along the rates tangent() gives at the
        // start of the step, whichever integrator carries the plant: the symplectic staging of the
        // plant does not extend to them.
        for (const State<T>& stepped :
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
            CHECK_EQ(stepped.tau, doctest::Approx(start.tau + k.tau * h).epsilon(tolerance));
        }
    }

    SUBCASE("RungeKuttaStagesSeeTheHorizonLessTheirStageTime") {
        // The target is held over the step while the vehicle moves on towards it, so the two
        // midpoint stages look ahead over the horizon they have reached less h/2 and the last one
        // over the horizon it has reached less h. Handing every stage the horizon it has reached
        // as it is is required to land well outside the tolerance, or the comparison could not
        // tell the two apart.
        const State<T> expected{
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
        // A target far out of reach, looked ahead to over a tenth of a second, drives both
        // control rates hard enough that a single unclamped step overshoots a bound, so the clamp
        // at the end of every integrator is all that keeps the controls in. The overshoot is
        // required first, or the checks below would hold trivially.
        const State<T> distant{targetAt(
            static_cast<T>(40.0), static_cast<T>(-30.0), static_cast<T>(-1.0), static_cast<T>(-0.4),
            static_cast<T>(30.0)
        )};
        State<T> short_sighted{start};
        short_sighted.tau = static_cast<T>(0.1);
        const State<T> pull{controller.tangent(short_sighted, distant)};
        const T a_step{short_sighted.a + pull.a * h};
        const T omega_step{short_sighted.omega + pull.omega * h};
        REQUIRE((a_step < kALo<T> || kAUp<T> < a_step));
        REQUIRE((omega_step < kOmegaLo<T> || kOmegaUp<T> < omega_step));
        // Controls handed in beyond the bounds are only held to come back inside them. Where they
        // land in there is left open on purpose: the controller steps from the control as handed
        // in and clamps afterwards while the plant flow clamps first, and the two only part ways
        // for a control that was out of bounds to begin with.
        State<T> loose{short_sighted};
        loose.a = static_cast<T>(50.0);
        loose.omega = static_cast<T>(-7.0);
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const State<T>& from : {short_sighted, loose}) {
                CHECK(isWithinBounds(stepWith(mode, controller, from, distant, h)));
            }
        }
    }
}

TEST_CASE_TEMPLATE("FirstOrderConvergenceUnderAFixedPreview", T, float, double) {
    // Every step looks the same lead ahead along a moving reference, so its target is the reference
    // point at t_i + lead, never the one at t_i. As h shrinks, all three integrators close in on
    // the loop whose target slides along the reference continuously, but only to first order in h:
    // the target comes in once per step and is held while the step is taken, which caps
    // rungeKutta() at first order as well, since its later stages would need the reference at
    // their own stage times for more. Halving h therefore halves the gap to the next resolution,
    // and the ratio below sits near 2 for all three.
    //
    // Every resolution takes its targets off one and the same finely sampled reference, so the
    // runs differ in the step alone, and the lead spans a whole number of those samples. The model
    // is unbounded, since a clamp kicking in would kink the flow and blur the order for reasons
    // that have nothing to do with the integrators.
    const KinematicsModel<T> model{unboundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    constexpr T kDuration{static_cast<T>(4.0)};
    constexpr std::size_t kFineSteps{1280};
    const T fine_h{kDuration / static_cast<T>(kFineSteps)};
    const std::size_t lead{static_cast<std::size_t>(std::lround(kLead<T> / fine_h))};
    const std::vector<State<T>> reference{referenceOf(model, fine_h, kFineSteps + lead)};
    const State<T> start{static_cast<T>(0.5), static_cast<T>(-0.6),
                         static_cast<T>(0.2), static_cast<T>(0.02),
                         static_cast<T>(7.0), static_cast<T>(0.0),
                         static_cast<T>(0.0), kLead<T>};
    const auto run = [&](std::size_t stride) -> ClosedLoop<T> {
        return runClosedLoop(
            controller, start, reference, fine_h, lead, kFineSteps / stride, stride
        );
    };

    const ClosedLoop<T> coarse{run(4)};
    const ClosedLoop<T> medium{run(2)};
    const ClosedLoop<T> fine{run(1)};
    for (const IntegrateMode mode : kIntegrateModes) {
        const T ratio{
            distanceOf(coarse.trace(mode).back(), medium.trace(mode).back()) /
            distanceOf(medium.trace(mode).back(), fine.trace(mode).back())
        };
        CHECK_GT(ratio, static_cast<T>(1.5));
        CHECK_LT(ratio, static_cast<T>(3.0));
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotClosedLoop("fixed preview of a moving reference", fine);
        }
    }
}

TEST_CASE_TEMPLATE("EvolveMatchesMarchingSingleSteps", T, float, double) {
    const Controller<T> controller{controllerOf(boundedModel<T>())};
    const State<T> start{startOf<T>()};
    const State<T> goal{targetOf<T>()};
    // No two entries agree, within a span or across the spans, so an evolve() that read the wrong
    // index cannot pass. Every step stays well shorter than the horizon, as rungeKutta() requires.
    const std::vector<T> hs{
        static_cast<T>(0.010), static_cast<T>(0.012), static_cast<T>(0.008), static_cast<T>(0.011),
        static_cast<T>(0.009)
    };
    std::vector<State<T>> targets;
    for (std::size_t i{0}; i < hs.size(); ++i) {
        const T shift{static_cast<T>(0.1) * static_cast<T>(i)};
        targets.push_back(targetAt(
            goal.x + shift, goal.y - shift, goal.yaw + static_cast<T>(0.2) * shift,
            goal.delta - static_cast<T>(0.3) * shift, goal.u + shift
        ));
    }

    for (const IntegrateMode mode : kIntegrateModes) {
        // Marching the single step integrator by hand gives the state after every step, which is
        // what the trace is supposed to hold behind the initial state.
        std::vector<State<T>> expected{start};
        for (std::size_t i{0}; i < hs.size(); ++i) {
            expected.push_back(stepWith(mode, controller, expected.back(), targets[i], hs[i]));
        }
        std::vector<State<T>> trace(hs.size() + 1);
        const State<T> returned{evolveWith(mode, controller, start, targets, hs, trace)};
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

// Closed loop runs along a reference whose targets lie the lead ahead. Nothing in the cost ties the
// horizon to that lead, so how far the loop settles off the reference depends on where it starts
// (see tangent()).

TEST_CASE_TEMPLATE("ClosedLoopAlongAReferenceTrajectory", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    constexpr std::size_t kNumSteps{2000};
    constexpr std::size_t kSettlingSteps{500};
    const T h{kH<T>};
    // The lead spans a whole number of steps, lead of them.
    const std::size_t lead{static_cast<std::size_t>(std::lround(kLead<T> / h))};
    const std::vector<State<T>> reference{referenceOf(model, h, kNumSteps + lead)};

    SUBCASE("FromOffTheReference") {
        // The vehicle starts off the reference by enough to drive its controls into the bounds.
        // The horizon takes up part of the offset rather than the loop closing it, so the run is
        // only held to stay within the bounds with a positive horizon; --plot-graph shows it.
        const State<T> start{static_cast<T>(1.5), static_cast<T>(-2.0),
                             static_cast<T>(0.1), static_cast<T>(0.0),
                             static_cast<T>(5.0), static_cast<T>(0.0),
                             static_cast<T>(0.0), kLead<T>};
        const ClosedLoop<T> loop{runClosedLoop(controller, start, reference, h, lead, kNumSteps)};
        for (const IntegrateMode mode : kIntegrateModes) {
            const std::vector<State<T>>& trace{loop.trace(mode)};
            CHECK(std::ranges::all_of(trace, isWithinBounds<T>));
            // The controls do reach the bounds on the way, so the check above is not vacuous.
            CHECK(std::ranges::any_of(trace, isSaturated<T>));
        }
        if (plot_graph) {
            if constexpr (std::same_as<T, double>) {
                plotClosedLoop("adaptive horizon, starting off the reference", loop);
            }
        }
    }

    SUBCASE("FromOnTheReference") {
        // The vehicle starts on the reference under the controls of the reference, with the
        // horizon at the lead, so there is no offset for the horizon to take up and the loop stays
        // within a metre of the reference from five seconds on, for every integrator.
        constexpr T kSettledDistance{static_cast<T>(1.0)};
        const State<T> on{reference.front()};
        const State<T> start{
            on.x,    on.y, on.yaw, on.delta, on.u, static_cast<T>(0.0), static_cast<T>(0.12),
            kLead<T>
        };
        const ClosedLoop<T> loop{runClosedLoop(controller, start, reference, h, lead, kNumSteps)};
        for (const IntegrateMode mode : kIntegrateModes) {
            CHECK(std::ranges::all_of(loop.trace(mode), isWithinBounds<T>));
            CHECK_LT(settledDistanceOf(loop, mode, kSettlingSteps), kSettledDistance);
        }
        if (plot_graph) {
            if constexpr (std::same_as<T, double>) {
                plotClosedLoop("adaptive horizon, starting on the reference", loop);
            }
        }
    }
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE("RejectsInvalidArguments", T, float, double) {
    const Controller<T> controller{controllerOf(boundedModel<T>())};
    const State<T> start{startOf<T>()};
    const State<T> target{targetOf<T>()};

    SUBCASE("SingularSteeringAngle") {
        // The steering angle is carried by the state, so this is where an omega that drove it out
        // of range over an earlier step surfaces.
        State<T> state{start};
        state.delta = std::numbers::pi_v<T> * static_cast<T>(0.5);
        CHECK_THROWS_AS(
            static_cast<void>(controller.tangent(state, target)), std::invalid_argument
        );
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
        // The horizon is carried by the state, so this is also where a horizon that shrank below
        // the step over earlier steps surfaces. An Euler step is fine with a horizon that spans
        // exactly the step, while the last stage of rungeKutta() looks ahead over the horizon less
        // the step, so it needs strictly more.
        const T h{kH<T>};
        const auto with_horizon = [&](T tau) -> State<T> {
            State<T> state{start};
            state.tau = tau;
            return state;
        };
        for (const IntegrateMode mode :
             {IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER}) {
            CHECK_NOTHROW(
                static_cast<void>(stepWith(mode, controller, with_horizon(h), target, h))
            );
            CHECK_THROWS_AS(
                static_cast<void>(
                    stepWith(mode, controller, with_horizon(h * static_cast<T>(0.5)), target, h)
                ),
                std::invalid_argument
            );
        }
        CHECK_THROWS_AS(
            static_cast<void>(controller.rungeKutta(with_horizon(h), target, h)),
            std::invalid_argument
        );
        CHECK_NOTHROW(
            static_cast<void>(
                controller.rungeKutta(with_horizon(h * static_cast<T>(1.01)), target, h)
            )
        );
    }

    SUBCASE("NonPositiveHorizon") {
        // tangent() takes any positive horizon, and nothing else.
        for (const T tau : {static_cast<T>(0.0), -kH<T>, std::numeric_limits<T>::quiet_NaN()}) {
            State<T> state{start};
            state.tau = tau;
            CHECK_THROWS_AS(
                static_cast<void>(controller.tangent(state, target)), std::invalid_argument
            );
        }
    }

    SUBCASE("EvolveRejectsMismatchedSpans") {
        const std::vector<T> hs(4, kH<T>);
        const std::vector<State<T>> targets(4, target);
        const std::vector<State<T>> short_targets(3, target);
        std::vector<State<T>> short_trace(hs.size());
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
        "kinematics_adaptive_flow_controller_test",
        "unit test of KinematicsAdaptiveFlowController class"
    );
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
