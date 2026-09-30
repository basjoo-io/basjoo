/**
 * @file test_kinematics_flow_controller.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
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
#include "basjoo/math/integrate_mode.hpp"

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
    const std::array<std::pair<const char*, const char*>, 6> labels{
        {{"x", "y"},
         {"t", "distance to reference"},
         {"t", "u"},
         {"t", "delta"},
         {"t", "a"},
         {"t", "omega"}}
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
// The horizon of the controller, and how far ahead along a reference every target of a closed loop
// run lies.
template <std::floating_point T>
inline constexpr T kHorizon{static_cast<T>(0.15)};
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};

inline constexpr std::array<IntegrateMode, 3> kIntegrateModes{
    IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER, IntegrateMode::RUNGE_KUTTA
};

template <std::floating_point T>
using State = detail::KinematicsFlowControllerState<T>;
template <std::floating_point T>
using ModelState = detail::KinematicsModelState<T>;
template <std::floating_point T>
using Controller = KinematicsFlowController<T>;

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
    const std::type_identity_t<std::array<T, 5>>& weights = kWeights<T>,
    std::type_identity_t<T> tau = kHorizon<T>
) -> Controller<T> {
    return Controller<T>{model, std::span<const T, 5>{weights}, tau};
}

/**
 * @brief A target state: tangent() ignores the controls of the target, so they are left NaN,
 * which any use of them would carry into the result.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto targetAt(T x, T y, T yaw, T delta, T u) noexcept -> State<T> {
    constexpr T kNaN{std::numeric_limits<T>::quiet_NaN()};
    return State<T>{x, y, yaw, delta, u, kNaN, kNaN};
}

template <std::floating_point T>
[[nodiscard]] constexpr auto plantOf(const State<T>& state) noexcept -> ModelState<T> {
    return ModelState<T>{state.x, state.y, state.yaw, state.delta, state.u};
}

/**
 * @brief A vehicle turning left at speed, with both controls inside the bounds.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto startOf() noexcept -> State<T> {
    return State<T>{static_cast<T>(0.0),  static_cast<T>(0.0), kInitialYaw<T>,
                    static_cast<T>(0.15), static_cast<T>(8.0), static_cast<T>(-0.6),
                    static_cast<T>(0.03)};
}

/**
 * @brief Roughly where startOf() gets to one horizon later, but off by enough in every residual
 * that the two rates at the start are far from zero.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto targetOf() noexcept -> State<T> {
    return targetAt(
        static_cast<T>(1.3), static_cast<T>(0.38), static_cast<T>(0.34), static_cast<T>(0.3),
        static_cast<T>(9.0)
    );
}

/**
 * @brief Turning left, turning right and reversing, each with both controls inside the bounds.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto statesOf() noexcept -> std::array<State<T>, 3> {
    return {
        startOf<T>(),
        State<T>{
            static_cast<T>(3.0), static_cast<T>(-2.0), static_cast<T>(-1.1), static_cast<T>(-0.2),
            static_cast<T>(5.0), static_cast<T>(1.2), static_cast<T>(-0.05)
        },
        State<T>{
            static_cast<T>(-1.0), static_cast<T>(4.0), static_cast<T>(2.5), static_cast<T>(0.1),
            static_cast<T>(-3.0), static_cast<T>(-0.4), static_cast<T>(0.2)
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

template <std::floating_point T>
[[nodiscard]] auto isWithinBounds(const State<T>& state) noexcept -> bool {
    return kALo<T> <= state.a && state.a <= kAUp<T> && kOmegaLo<T> <= state.omega &&
           state.omega <= kOmegaUp<T>;
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
 * @brief One classic Runge-Kutta step spelled out through tangent(), with the horizons that the
 * two midpoint stages and the last stage look ahead over handed in explicitly.
 *
 * The first stage looks ahead over the horizon of the controller, the later ones through
 * controllers that differ from it in the horizon alone. It walks the same arithmetic as
 * rungeKutta(), so the two part only by roundoff when they look ahead over the same horizons.
 */
template <std::floating_point T>
[[nodiscard]] auto rungeKuttaThroughTangent(
    const Controller<T>& controller, const State<T>& state, const State<T>& target_state, T mid_tau,
    T end_tau, T h
) -> State<T> {
    constexpr T kHalf{static_cast<T>(0.5)};
    constexpr T kSixth{static_cast<T>(1.0 / 6.0)};
    constexpr T kThird{static_cast<T>(1.0 / 3.0)};
    const KinematicsModel<T>& model{controller.model()};
    std::array<T, 5> weights{};
    std::ranges::copy(controller.weights(), weights.begin());
    const Controller<T> mid{controllerOf(model, weights, mid_tau)};
    const Controller<T> end{controllerOf(model, weights, end_tau)};
    const State<T> k1{controller.tangent(state, target_state) * h};
    const State<T> k2{mid.tangent(state + k1 * kHalf, target_state) * h};
    const State<T> k3{mid.tangent(state + k2 * kHalf, target_state) * h};
    const State<T> k4{end.tangent(state + k3, target_state) * h};
    State<T> result{state};
    result += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
    result.a = std::clamp(result.a, model.a_lower_bound(), model.a_upper_bound());
    result.omega = std::clamp(result.omega, model.omega_lower_bound(), model.omega_upper_bound());
    return result;
}

/**
 * @brief The tracking cost J of tangent(), written out from the residuals its documentation lists,
 * with the controls taken unclamped.
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
    const T tau{controller.tau()};
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
 * @brief Derivative of f along one field of the state, from the central difference.
 *
 * The central difference is exact for quadratics, and the cost is quadratic in both controls, so
 * it recovers their derivatives up to rounding whatever the step, and a wide step keeps the
 * rounding small.
 */
template <std::floating_point T>
[[nodiscard]] auto derivativeOf(const auto& f, const State<T>& state, T State<T>::* field, T step)
    -> T {
    const auto at = [&](T shift) -> T {
        State<T> shifted{state};
        shifted.*field += shift;
        return f(shifted);
    };
    return (at(step) - at(-step)) / (static_cast<T>(2.0) * step);
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
        std::vector<double> xs, ys, distances, us, deltas, accels, omegas;
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
        }
        const char* line_spec{line_specs[static_cast<std::size_t>(mode)]};
        axes[0]->plot(xs, ys, line_spec)->line_width(2.0);
        axes[1]->plot(loop.ts, distances, line_spec)->line_width(2.0);
        axes[2]->plot(loop.ts, us, line_spec)->line_width(2.0);
        axes[3]->plot(loop.ts, deltas, line_spec)->line_width(2.0);
        axes[4]->plot(loop.ts, accels, line_spec)->line_width(2.0);
        axes[5]->plot(loop.ts, omegas, line_spec)->line_width(2.0);
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
    fig->show();
}

} // namespace

// The controller carries a model, five weights and a horizon and nothing else, and hands all three
// back as it was given them.

TEST_CASE_TEMPLATE("ConstructorKeepsTheModelTheWeightsAndTheHorizon", T, float, double) {
    const Controller<T> controller{
        boundedModel<T>(), std::span<const T, 5>{kWeights<T>}, kHorizon<T>
    };
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
    CHECK_EQ(controller.tau(), kHorizon<T>);
}

TEST_CASE_TEMPLATE("ControllerSerializationRoundtrip", T, float, double) {
    const Controller<T> controller{controllerOf(boundedModel<T>())};
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(controller).code == std::errc{});
    // A default constructed controller has unit weights, a unit horizon and a unit, unbounded
    // model, none of which agrees with the original, so every field checked below has to have come
    // through the archive.
    Controller<T> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.model().front_wheelbase(), kFrontWheelbase<T>);
    CHECK_EQ(restored.model().rear_wheelbase(), kRearWheelbase<T>);
    CHECK_EQ(restored.model().a_lower_bound(), kALo<T>);
    CHECK_EQ(restored.model().a_upper_bound(), kAUp<T>);
    CHECK_EQ(restored.model().omega_lower_bound(), kOmegaLo<T>);
    CHECK_EQ(restored.model().omega_upper_bound(), kOmegaUp<T>);
    CHECK(std::ranges::equal(restored.weights(), kWeights<T>));
    CHECK_EQ(restored.tau(), kHorizon<T>);
    // Same arithmetic along the same code path, so a restored controller has to reproduce the flow
    // of the original to within roundoff: the two calls are inlined separately and the compiler
    // may fuse them differently. Roundoff is relative, so the bound scales with the largest field
    // of the flow. A weight, a bound or a horizon that went missing moves it far more.
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
// and in a and omega the rates at which the two controls descend the tracking cost.

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
        // in the two rates.
        for (const State<T>& state : statesOf<T>()) {
            State<T> clamped{state};
            clamped.a = kAUp<T>;
            clamped.omega = kOmegaLo<T>;
            CHECK_EQ(controller.tangent(loose(state), target), controller.tangent(clamped, target));
        }
    }
}

TEST_CASE_TEMPLATE("RatesAreThePreconditionedNegativeGradientOfTheCost", T, float, double) {
    // The rates are da/dt = -(L^2/tau^5) dJ/da and domega/dt = -(1/tau^3) dJ/domega. costOf()
    // writes J out from the residuals independently of the transcribed rates, and derivativeOf()
    // recovers its derivatives up to rounding, so any slip in the transcription shows here, even
    // one made consistently across both rates. The model is unbounded, so no clamp cuts into the
    // differences.
    const Controller<T> controller{controllerOf(unboundedModel<T>())};
    const State<T> target{targetOf<T>()};
    const auto cost = [&](const State<T>& state) -> T { return costOf(controller, state, target); };
    const T tau{controller.tau()};
    const T a_preconditioner{kWheelbase<T> * kWheelbase<T> / (tau * tau * tau * tau * tau)};
    const T omega_preconditioner{static_cast<T>(1.0) / (tau * tau * tau)};
    // The cost is quadratic in the controls, so any step does, and a wide one keeps the rounding
    // of the difference down. The acceleration enters the residuals only through tau or tau^2, so
    // the cost hardly moves with it and its step has to be all the wider. The two sides round
    // differently, the difference all the more for subtracting nearby costs, and the tolerance
    // leaves well over an order of magnitude on top of both.
    constexpr T kAStep{static_cast<T>(10.0)};
    constexpr T kOmegaStep{static_cast<T>(1.0)};
    const double tolerance{std::same_as<T, float> ? 1.0e-4 : 1.0e-12};
    for (const State<T>& state : statesOf<T>()) {
        const State<T> k{controller.tangent(state, target)};
        CHECK_EQ(
            k.a,
            doctest::Approx(-a_preconditioner * derivativeOf(cost, state, &State<T>::a, kAStep))
                .epsilon(tolerance)
        );
        CHECK_EQ(
            k.omega,
            doctest::Approx(
                -omega_preconditioner * derivativeOf(cost, state, &State<T>::omega, kOmegaStep)
            )
                .epsilon(tolerance)
        );
    }
}

TEST_CASE_TEMPLATE("ControlRatesDescendThePreconditionedGradient", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const State<T> state{startOf<T>()};
    const State<T> target{targetOf<T>()};
    const T horizon{kHorizon<T>};
    constexpr T kWeight{static_cast<T>(0.6)};

    SUBCASE("DeltaTermAlone") {
        // Left with its delta term alone, the cost reads J = w3 R^2 with R = delta + tau omega -
        // delta_target, whose gradient dJ/domega = 2 w3 R tau has a closed form, and the steering
        // rate descends it with the preconditioner 1/tau^3. Put differently, an Euler step of
        // length h moves omega by 2 w3 h / tau times the Newton step -R / tau. Any factor off in
        // either the gradient or the preconditioner shows up here, and the acceleration has nothing
        // left to descend.
        const Controller<T> controller{controllerOf(model, {0, 0, 0, kWeight, 0})};
        const State<T> k{controller.tangent(state, target)};
        const T residual{state.delta + horizon * state.omega - target.delta};
        const T gradient{static_cast<T>(2.0) * kWeight * residual * horizon};
        CHECK_EQ(
            k.omega, doctest::Approx(-gradient / (horizon * horizon * horizon))
                         .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(k.a, static_cast<T>(0.0));
    }

    SUBCASE("UTermAlone") {
        // Left with its u term alone, the cost reads J = w4 (tau R / L)^2 with R = u + tau a -
        // u_target, whose gradient dJ/da = 2 w4 tau^3 R / L^2 has a closed form, and the
        // acceleration descends it with the preconditioner L^2/tau^5, which again comes to
        // 2 w4 h / tau times the Newton step -R / tau per Euler step. The steering rate has nothing
        // left to descend.
        const Controller<T> controller{controllerOf(model, {0, 0, 0, 0, kWeight})};
        const State<T> k{controller.tangent(state, target)};
        const T wheelbase{kWheelbase<T>};
        const T residual{state.u + horizon * state.a - target.u};
        const T gradient{
            static_cast<T>(2.0) * kWeight * horizon * horizon * horizon * residual /
            (wheelbase * wheelbase)
        };
        CHECK_EQ(
            k.a, doctest::Approx(
                     -wheelbase * wheelbase / (horizon * horizon * horizon * horizon * horizon) *
                     gradient
                 )
                     .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(k.omega, static_cast<T>(0.0));
    }

    SUBCASE("MixedPartialsMatchThePreconditioners") {
        // Both rates are preconditioned partial derivatives of one and the same cost, so their
        // cross derivatives are the one mixed partial of J scaled by the two preconditioners:
        // (d a_rate / d omega) / (d omega_rate / d a) = (L^2/tau^5) / (1/tau^3) = L^2/tau^2,
        // whatever the weights, the state and the target. The two rates are separate hand derived
        // expressions, so an algebra slip in either one, such as a dropped term or a power of tau
        // off by one, breaks the ratio. Every residual is affine in a and omega, which leaves both
        // rates affine in them as well, so central differences recover the derivatives up to
        // rounding whatever the step, and a wide step keeps that rounding small next to the
        // differences. The model is unbounded so that no clamp cuts into the differences.
        const Controller<T> controller{controllerOf(unboundedModel<T>())};
        constexpr T kStep{static_cast<T>(1.0)};
        const auto rates_at = [&](T a_shift, T omega_shift) -> State<T> {
            State<T> shifted{state};
            shifted.a += a_shift;
            shifted.omega += omega_shift;
            return controller.tangent(shifted, target);
        };
        const T a_rate_by_omega{
            (rates_at(static_cast<T>(0.0), kStep).a - rates_at(static_cast<T>(0.0), -kStep).a) /
            (static_cast<T>(2.0) * kStep)
        };
        const T omega_rate_by_a{
            (rates_at(kStep, static_cast<T>(0.0)).omega -
             rates_at(-kStep, static_cast<T>(0.0)).omega) /
            (static_cast<T>(2.0) * kStep)
        };
        CHECK_EQ(
            a_rate_by_omega / omega_rate_by_a,
            doctest::Approx(kWheelbase<T> * kWheelbase<T> / (horizon * horizon))
                .epsilon(roundoffTolerance<T>())
        );
    }
}

// The single step integrators step the plant the way the model does and move the controls along the
// rates. evolve() only marches the single step ones.

TEST_CASE_TEMPLATE("SingleStepIntegratorsDriveThePlantThroughTheModel", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    const T tolerance{roundoffTolerance<T>()};
    const T h{kH<T>};
    const State<T> start{startOf<T>()};
    const State<T> target{targetOf<T>()};
    const State<T> k{controller.tangent(start, target)};
    // The comparisons below hold the integrators to a tolerance, so the rates at this start have
    // to be large enough that holding the controls still over a step, or stepping the plant under
    // the updated controls instead of the held ones, lands far outside it. The latter moves delta
    // and u by h^2 times a rate.
    REQUIRE_GT(
        h * h * std::min(std::abs(k.a), std::abs(k.omega)), static_cast<T>(10.0) * tolerance
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

    SUBCASE("EulerControlsStepAlongTheRates") {
        // The controls take one Euler step along the rates tangent() gives at the start of the
        // step, whichever integrator carries the plant: the symplectic staging of the plant does
        // not extend to the controls.
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
        }
    }

    SUBCASE("RungeKuttaStagesSeeTheHorizonLessTheirStageTime") {
        // The target is held over the step while the vehicle moves on towards it, so the two
        // midpoint stages look ahead over the horizon less h/2 and the last one over the horizon
        // less h. Handing every stage the full horizon instead is required to land well outside
        // the tolerance, or the comparison could not tell the two apart.
        const T horizon{controller.tau()};
        const State<T> expected{rungeKuttaThroughTangent(
            controller, start, target, horizon - h * static_cast<T>(0.5), horizon - h, h
        )};
        REQUIRE_GT(
            distanceOf(
                expected, rungeKuttaThroughTangent(controller, start, target, horizon, horizon, h)
            ),
            static_cast<T>(10.0) * tolerance
        );
        CHECK_LE(distanceOf(controller.rungeKutta(start, target, h), expected), tolerance);
    }

    SUBCASE("ControlsNeverLeaveTheBounds") {
        // A target far out of reach, looked ahead to over a tenth of a second, drives both
        // rates hard enough that a single unclamped step overshoots a bound, so the clamp at the
        // end of every integrator is all that keeps the controls in. The overshoot is required
        // first, or the checks below would hold trivially.
        const State<T> distant{targetAt(
            static_cast<T>(40.0), static_cast<T>(-30.0), static_cast<T>(-1.0), static_cast<T>(-0.4),
            static_cast<T>(30.0)
        )};
        const Controller<T> short_sighted{controllerOf(model, kWeights<T>, static_cast<T>(0.1))};
        const State<T> pull{short_sighted.tangent(start, distant)};
        const T a_step{start.a + pull.a * h};
        const T omega_step{start.omega + pull.omega * h};
        REQUIRE((a_step < kALo<T> || kAUp<T> < a_step));
        REQUIRE((omega_step < kOmegaLo<T> || kOmegaUp<T> < omega_step));
        // Controls handed in beyond the bounds are only held to come back inside them. Where they
        // land in there is left open on purpose: the controller steps from the control as handed
        // in and clamps afterwards while the plant flow clamps first, and the two only part ways
        // for a control that was out of bounds to begin with.
        State<T> loose{start};
        loose.a = static_cast<T>(50.0);
        loose.omega = static_cast<T>(-7.0);
        for (const IntegrateMode mode : kIntegrateModes) {
            for (const State<T>& from : {start, loose}) {
                CHECK(isWithinBounds(stepWith(mode, short_sighted, from, distant, h)));
            }
        }
    }
}

TEST_CASE_TEMPLATE("FirstOrderConvergenceUnderAFixedPreview", T, float, double) {
    // Every step looks the same horizon ahead along a moving reference, so its target is the
    // reference point at t_i + tau, never the one at t_i. As h shrinks, all three integrators
    // close in on the loop whose target slides along the reference continuously, but only to first
    // order in h: the target comes in once per step and is held while the step is taken, which caps
    // rungeKutta() at first order as well, since its later stages would need the reference points
    // ahead of their own stage times for more. Halving h therefore halves the gap to the next
    // resolution, and the ratio below sits near 2 for all three.
    //
    // Every resolution takes its targets off one and the same finely sampled reference, so the
    // runs differ in the step alone, and the horizon spans a whole number of those samples. The
    // model is unbounded, since a clamp kicking in would kink the flow and blur the order for
    // reasons that have nothing to do with the integrators.
    const KinematicsModel<T> model{unboundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    constexpr T kDuration{static_cast<T>(4.0)};
    constexpr std::size_t kFineSteps{1280};
    const T fine_h{kDuration / static_cast<T>(kFineSteps)};
    const std::size_t lead{static_cast<std::size_t>(std::lround(kHorizon<T> / fine_h))};
    const std::vector<State<T>> reference{referenceOf(model, fine_h, kFineSteps + lead)};
    const State<T> start{static_cast<T>(0.5),  static_cast<T>(-0.6), static_cast<T>(0.2),
                         static_cast<T>(0.02), static_cast<T>(7.0),  static_cast<T>(0.0),
                         static_cast<T>(0.0)};
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
    // index cannot pass. Every step stays shorter than the horizon, as rungeKutta() requires.
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

// A closed loop run along a reference: besides what has to hold at every step, it bounds how far
// the loop settles off the reference.

TEST_CASE_TEMPLATE("PerStepTargetsAlongAReferenceTrajectory", T, float, double) {
    // The targets are where a reference vehicle is one horizon ahead, and the controlled vehicle
    // starts off that reference by enough to drive its controls into the bounds. Once that offset
    // has decayed, the loop still settles off the reference, since the one step prediction of the
    // cost does not land where the reference goes over the horizon (see tangent()). At this
    // horizon and along this reference that stays within 20 cm from five seconds on, for every
    // integrator; --plot-graph shows it.
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
    constexpr std::size_t kNumSteps{2000};
    constexpr std::size_t kSettlingSteps{500};
    constexpr T kSettledDistance{static_cast<T>(0.2)};
    const T h{kH<T>};
    // The horizon spans a whole number of steps, lead of them.
    const std::size_t lead{static_cast<std::size_t>(std::lround(kHorizon<T> / h))};
    const std::vector<State<T>> reference{referenceOf(model, h, kNumSteps + lead)};
    const State<T> start{static_cast<T>(1.5), static_cast<T>(-2.0), static_cast<T>(0.1),
                         static_cast<T>(0.0), static_cast<T>(5.0),  static_cast<T>(0.0),
                         static_cast<T>(0.0)};

    const ClosedLoop<T> loop{runClosedLoop(controller, start, reference, h, lead, kNumSteps)};
    for (const IntegrateMode mode : kIntegrateModes) {
        const std::vector<State<T>>& trace{loop.trace(mode)};
        CHECK(std::ranges::all_of(trace, isWithinBounds<T>));
        // The controls do reach the bounds on the way, so the check above is not vacuous.
        CHECK(std::ranges::any_of(trace, isSaturated<T>));
        // Written so that a NaN propagates into the result, as in distanceOf().
        T settled_distance{static_cast<T>(0.0)};
        for (std::size_t i{kSettlingSteps}; i < trace.size(); ++i) {
            const T distance{
                std::hypot(trace[i].x - loop.reference[i].x, trace[i].y - loop.reference[i].y)
            };
            if (!(distance <= settled_distance)) {
                settled_distance = distance;
            }
        }
        CHECK_LT(settled_distance, kSettledDistance);
    }

    if (plot_graph) {
        if constexpr (std::same_as<T, double>) {
            plotClosedLoop("per step targets along a reference trajectory", loop);
        }
    }
}

#if BASJOO_CHECK_PARAMS == 1

TEST_CASE_TEMPLATE("RejectsInvalidArguments", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const Controller<T> controller{controllerOf(model)};
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
        // An Euler step is fine with a horizon that spans exactly the step, while the last stage
        // of rungeKutta() looks ahead over the horizon less the step, which has to stay positive
        // for the target to lie ahead of that stage, so it needs strictly more. The controller
        // itself takes any positive horizon.
        const T h{kH<T>};
        const Controller<T> at_step{controllerOf(model, kWeights<T>, h)};
        const Controller<T> within_step{controllerOf(model, kWeights<T>, h * static_cast<T>(0.5))};
        for (const IntegrateMode mode :
             {IntegrateMode::FORWARD_EULER, IntegrateMode::SYMPLECTIC_EULER}) {
            CHECK_NOTHROW(static_cast<void>(stepWith(mode, at_step, start, target, h)));
            CHECK_THROWS_AS(
                static_cast<void>(stepWith(mode, within_step, start, target, h)),
                std::invalid_argument
            );
        }
        CHECK_THROWS_AS(
            static_cast<void>(at_step.rungeKutta(start, target, h)), std::invalid_argument
        );
        CHECK_NOTHROW(
            static_cast<void>(controllerOf(model, kWeights<T>, h * static_cast<T>(1.01))
                                  .rungeKutta(start, target, h))
        );
        for (const T tau : {static_cast<T>(0.0), -h, std::numeric_limits<T>::quiet_NaN()}) {
            CHECK_THROWS_AS(
                static_cast<void>(controllerOf(model, kWeights<T>, tau)), std::invalid_argument
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
