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
#include <cstdint>
#include <iterator>
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

#include "basjoo/bicycle/models/kinematics_model.hpp"
#include "basjoo/bicycle/models/state.hpp"

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
    axes[1]->ylabel("distance to reference");
    axes[2]->xlabel("t");
    axes[2]->ylabel("u");
    axes[3]->xlabel("t");
    axes[3]->ylabel("delta");
    axes[4]->xlabel("t");
    axes[4]->ylabel("a");
    axes[5]->xlabel("t");
    axes[5]->ylabel("omega");
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
inline constexpr T kHorizon{static_cast<T>(0.15)};
template <std::floating_point T>
inline constexpr T kH{static_cast<T>(0.01)};

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
[[nodiscard]] auto controllerOf(
    const KinematicsModel<T>& model, const std::array<T, 5>& weights = kWeights<T>
) -> KinematicsFlowController<T> {
    return KinematicsFlowController<T>{model, std::span<const T, 5>{weights}};
}

/**
 * @brief A vehicle turning left at speed, with both controls inside the bounds.
 */
template <std::floating_point T>
[[nodiscard]] auto startOf(const KinematicsModel<T>& model) -> State<T> {
    return model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.15),
        static_cast<T>(8.0), static_cast<T>(-0.6), static_cast<T>(0.03)
    );
}

/**
 * @brief Roughly where startOf() gets to one horizon later, but off by enough in every residual
 * that the two rates at the start are far from zero.
 */
template <std::floating_point T>
[[nodiscard]] auto targetOf(const KinematicsModel<T>& model) -> State<T> {
    return model.createState(
        static_cast<T>(1.3), static_cast<T>(0.38), static_cast<T>(0.34), static_cast<T>(0.3),
        static_cast<T>(9.0)
    );
}

/**
 * @brief A reference trajectory sampled every h, starting at the origin at speed and steered by
 * two smooth open loop controls.
 */
template <std::floating_point T>
[[nodiscard]] auto referenceOf(const KinematicsModel<T>& model, T h, std::size_t num_steps)
    -> std::vector<State<T>> {
    std::vector<State<T>> reference;
    reference.reserve(num_steps + 1);
    reference.push_back(model.createState(
        static_cast<T>(0.0), static_cast<T>(0.0), kInitialYaw<T>, static_cast<T>(0.0),
        static_cast<T>(8.0)
    ));
    for (std::size_t i{0}; i < num_steps; ++i) {
        const T t{h * static_cast<T>(i)};
        reference.push_back(model.rungeKutta(
            reference.back(), static_cast<T>(0.8) * std::sin(static_cast<T>(0.4) * t),
            static_cast<T>(0.12) * std::cos(static_cast<T>(0.5) * t), h
        ));
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
 * @brief Largest difference over the first fields of two states.
 *
 * Written so that a NaN in either state propagates into the result instead of being skipped over
 * the way std::max() would skip it, which would let a diverging integrator pass a check against
 * zero.
 */
template <std::floating_point T>
[[nodiscard]] auto distanceOverFieldsOf(
    const State<T>& lhs, const State<T>& rhs, std::size_t num_fields
) noexcept -> T {
    T result{static_cast<T>(0.0)};
    for (std::size_t i{0}; i < num_fields; ++i) {
        const T difference{std::abs(lhs.data()[i] - rhs.data()[i])};
        if (!(difference <= result)) {
            result = difference;
        }
    }
    return result;
}

/**
 * @brief Largest per field difference between two states.
 */
template <std::floating_point T>
[[nodiscard]] auto distanceOf(const State<T>& lhs, const State<T>& rhs) noexcept -> T {
    return distanceOverFieldsOf(lhs, rhs, State<T>::size());
}

/**
 * @brief Largest difference over the seven plant fields, i.e. over all but a and omega.
 */
template <std::floating_point T>
[[nodiscard]] auto plantDistanceOf(const State<T>& lhs, const State<T>& rhs) noexcept -> T {
    return distanceOverFieldsOf(lhs, rhs, State<T>::size() - 2);
}

/**
 * @brief Whether v and r sit exactly where the kinematic constraint puts them.
 *
 * Neither is integrated: both are rebuilt from u and delta at the end of every step, so they have
 * to satisfy the constraint bit for bit rather than within a tolerance.
 */
template <std::floating_point T>
[[nodiscard]] auto isOnConstraint(const State<T>& state) noexcept -> bool {
    return state.r == state.u * std::tan(state.delta) / kWheelbase<T> &&
           state.v == kRearWheelbase<T> * state.r;
}

template <std::floating_point T>
[[nodiscard]] auto isWithinBounds(const State<T>& state) noexcept -> bool {
    return kALo<T> <= state.a && state.a <= kAUp<T> && kOmegaLo<T> <= state.omega &&
           state.omega <= kOmegaUp<T>;
}

/**
 * @brief One classic Runge-Kutta step spelled out through tangent(), with the horizons that the
 * two midpoint stages and the last stage look ahead over handed in explicitly.
 *
 * It walks the same arithmetic as rungeKutta(), so the two part only by roundoff when they hand
 * tangent() the same horizons.
 */
template <std::floating_point T>
[[nodiscard]] auto rungeKuttaThroughTangent(
    const KinematicsFlowController<T>& controller, const State<T>& state,
    const State<T>& target_state, T horizon, T mid_horizon, T end_horizon, T h
) -> State<T> {
    constexpr T kHalf{static_cast<T>(0.5)};
    constexpr T kSixth{static_cast<T>(1.0 / 6.0)};
    constexpr T kThird{static_cast<T>(1.0 / 3.0)};
    const KinematicsModel<T>& model{controller.model()};
    const State<T> k1{controller.tangent(state, target_state, horizon) * h};
    const State<T> k2{controller.tangent(state + k1 * kHalf, target_state, mid_horizon) * h};
    const State<T> k3{controller.tangent(state + k2 * kHalf, target_state, mid_horizon) * h};
    const State<T> k4{controller.tangent(state + k3, target_state, end_horizon) * h};
    State<T> result{state};
    result += k1 * kSixth + k2 * kThird + k3 * kThird + k4 * kSixth;
    result.a = std::clamp(result.a, model.a_lower_bound(), model.a_upper_bound());
    result.omega = std::clamp(result.omega, model.omega_lower_bound(), model.omega_upper_bound());
    result.r = result.u * std::tan(result.delta) / model.wheelbase();
    result.v = model.rear_wheelbase() * result.r;
    return result;
}

enum class Integrator : std::uint8_t {
    FORWARD_EULER,
    SYMPLECTIC_EULER,
    RUNGE_KUTTA
};

inline constexpr std::array<Integrator, 3> kIntegrators{
    Integrator::FORWARD_EULER, Integrator::SYMPLECTIC_EULER, Integrator::RUNGE_KUTTA
};

template <std::floating_point T>
[[nodiscard]] auto stepWith(
    Integrator integrator, const KinematicsFlowController<T>& controller, const State<T>& state,
    const State<T>& target_state, T horizon, T h
) -> State<T> {
    switch (integrator) {
    case Integrator::FORWARD_EULER:
        return controller.forwardEuler(state, target_state, horizon, h);
    case Integrator::SYMPLECTIC_EULER:
        return controller.symplecticEuler(state, target_state, horizon, h);
    case Integrator::RUNGE_KUTTA:
        return controller.rungeKutta(state, target_state, horizon, h);
    }
    std::unreachable();
}

/**
 * @brief Runs one of the batch overloads and appends every state it traces.
 *
 * target_states is either one state per step or a single state held over the whole run, which
 * picks the matching overload.
 */
template <std::floating_point T>
auto batchWith(
    Integrator integrator, const KinematicsFlowController<T>& controller,
    const State<T>& initial_state, const auto& target_states, const std::vector<T>& horizons,
    const std::vector<T>& hs, std::vector<State<T>>& trace
) -> State<T> {
    const auto out{std::back_inserter(trace)};
    switch (integrator) {
    case Integrator::FORWARD_EULER:
        return controller.forwardEuler(initial_state, target_states, horizons, hs, out);
    case Integrator::SYMPLECTIC_EULER:
        return controller.symplecticEuler(initial_state, target_states, horizons, hs, out);
    case Integrator::RUNGE_KUTTA:
        return controller.rungeKutta(initial_state, target_states, horizons, hs, out);
    }
    std::unreachable();
}

/**
 * @brief One closed loop run of every integrator, each trace led by the initial state so that it
 * lines up with ts and with the reference the run is plotted against.
 */
template <std::floating_point T>
struct ClosedLoop final {
    std::vector<T> ts;
    std::vector<State<T>> reference;
    std::vector<State<T>> forward_euler;
    std::vector<State<T>> symplectic_euler;
    std::vector<State<T>> runge_kutta;
};

template <std::floating_point T>
[[nodiscard]] auto traceOf(const ClosedLoop<T>& loop, Integrator integrator) noexcept
    -> const std::vector<State<T>>& {
    switch (integrator) {
    case Integrator::FORWARD_EULER:
        return loop.forward_euler;
    case Integrator::SYMPLECTIC_EULER:
        return loop.symplectic_euler;
    case Integrator::RUNGE_KUTTA:
        return loop.runge_kutta;
    }
    std::unreachable();
}

/**
 * @brief Marches every integrator through the batch overloads, leaving ts and the reference to the
 * caller.
 */
template <std::floating_point T>
[[nodiscard]] auto runClosedLoop(
    const KinematicsFlowController<T>& controller, const State<T>& initial_state,
    const auto& target_states, const std::vector<T>& horizons, const std::vector<T>& hs
) -> ClosedLoop<T> {
    ClosedLoop<T> loop;
    const auto march = [&](Integrator integrator, std::vector<State<T>>& trace) -> void {
        trace.reserve(hs.size() + 1);
        trace.push_back(initial_state);
        static_cast<void>(
            batchWith(integrator, controller, initial_state, target_states, horizons, hs, trace)
        );
    };
    march(Integrator::FORWARD_EULER, loop.forward_euler);
    march(Integrator::SYMPLECTIC_EULER, loop.symplectic_euler);
    march(Integrator::RUNGE_KUTTA, loop.runge_kutta);
    return loop;
}

auto plotClosedLoop(const std::string& title, const ClosedLoop<double>& loop) -> void {
    const std::size_t size{loop.ts.size()};
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
    const auto plot_trace = [&](const std::vector<State<double>>& trace,
                                const char* line_spec) -> void {
        std::vector<double> xs, ys, distances, us, deltas, accels, omegas;
        for (std::size_t i{0}; i < size; ++i) {
            const State<double>& state{trace[i]};
            const State<double>& reference{loop.reference[i]};
            xs.push_back(state.x);
            ys.push_back(state.y);
            distances.push_back(std::hypot(state.x - reference.x, state.y - reference.y));
            us.push_back(state.u);
            deltas.push_back(state.delta);
            accels.push_back(state.a);
            omegas.push_back(state.omega);
        }
        axes[0]->plot(xs, ys, line_spec)->line_width(2.0);
        axes[1]->plot(loop.ts, distances, line_spec)->line_width(2.0);
        axes[2]->plot(loop.ts, us, line_spec)->line_width(2.0);
        axes[3]->plot(loop.ts, deltas, line_spec)->line_width(2.0);
        axes[4]->plot(loop.ts, accels, line_spec)->line_width(2.0);
        axes[5]->plot(loop.ts, omegas, line_spec)->line_width(2.0);
    };
    plot_trace(loop.forward_euler, ":");
    plot_trace(loop.symplectic_euler, "-.");
    plot_trace(loop.runge_kutta, "--");
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

// The controller carries a model and five weights and nothing else, and hands both back as it was
// given them.

TEST_CASE_TEMPLATE("ConstructorKeepsTheModelAndTheWeights", T, float, double) {
    const KinematicsFlowController<T> controller{
        boundedModel<T>(), std::span<const T, 5>{kWeights<T>}
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
    const std::span<const T> weights{controller.weights()};
    const std::span<const T> expected{kWeights<T>};
    REQUIRE_EQ(weights.size(), expected.size());
    for (std::size_t i{0}; i < weights.size(); ++i) {
        CHECK_EQ(weights[i], expected[i]);
    }
}

TEST_CASE_TEMPLATE("ControllerSerializationRoundtrip", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(controller).code == std::errc{});
    // A default constructed controller has unit weights and a unit, unbounded model, none of which
    // agrees with the original, so every field checked below has to have come through the archive.
    KinematicsFlowController<T> restored{};
    REQUIRE(in(restored).code == std::errc{});
    CHECK_EQ(restored.model().front_wheelbase(), kFrontWheelbase<T>);
    CHECK_EQ(restored.model().rear_wheelbase(), kRearWheelbase<T>);
    CHECK_EQ(restored.model().a_lower_bound(), kALo<T>);
    CHECK_EQ(restored.model().a_upper_bound(), kAUp<T>);
    CHECK_EQ(restored.model().omega_lower_bound(), kOmegaLo<T>);
    CHECK_EQ(restored.model().omega_upper_bound(), kOmegaUp<T>);
    const std::span<const T> weights{restored.weights()};
    const std::span<const T> expected{kWeights<T>};
    REQUIRE_EQ(weights.size(), expected.size());
    for (std::size_t i{0}; i < weights.size(); ++i) {
        CHECK_EQ(weights[i], expected[i]);
    }
    // Same arithmetic along the same code path, so a restored controller has to reproduce the flow
    // of the original to within roundoff: the two calls are inlined separately and the compiler
    // may fuse them differently. A weight or a bound that went missing moves it far more.
    CHECK_EQ(
        distanceOf(
            restored.tangent(startOf(model), targetOf(model), kHorizon<T>),
            controller.tangent(startOf(model), targetOf(model), kHorizon<T>)
        ),
        doctest::Approx(0.0).epsilon(roundoffTolerance<T>())
    );
}

// tangent() is the whole closed loop flow: the plant flow of the model in its first seven fields,
// and in a and omega the rates at which the two controls descend the tracking cost.

TEST_CASE_TEMPLATE("TangentCarriesThePlantFlowOfTheModel", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    const State<T> target{targetOf(model)};
    // Turning left, turning right and reversing, each with both controls inside the bounds.
    const std::array<State<T>, 3> states{
        startOf(model),
        model.createState(
            static_cast<T>(3.0), static_cast<T>(-2.0), static_cast<T>(-1.1), static_cast<T>(-0.2),
            static_cast<T>(5.0), static_cast<T>(1.2), static_cast<T>(-0.05)
        ),
        model.createState(
            static_cast<T>(-1.0), static_cast<T>(4.0), static_cast<T>(2.5), static_cast<T>(0.1),
            static_cast<T>(-3.0), static_cast<T>(-0.4), static_cast<T>(0.2)
        )
    };

    SUBCASE("PlantFieldsEqualTheModelTangent") {
        // The controller spells the plant flow out itself instead of calling the model, so that the
        // flow and the cost gradient can share their trigonometry. This is what keeps that copy
        // honest: any change to KinematicsModel::tangent() that is not mirrored in the controller
        // fails here bit for bit, and so does a clamp that went missing, since every state is also
        // handed in with both controls beyond the bounds and with v and r off the constraint.
        for (const State<T>& state : states) {
            State<T> loose{state};
            loose.a = static_cast<T>(50.0);
            loose.omega = static_cast<T>(-7.0);
            loose.v = static_cast<T>(12.3);
            loose.r = static_cast<T>(-4.5);
            for (const State<T>& handed : {state, loose}) {
                CHECK_EQ(
                    plantDistanceOf(
                        controller.tangent(handed, target, kHorizon<T>), model.tangent(handed)
                    ),
                    static_cast<T>(0.0)
                );
            }
        }
    }

    SUBCASE("StaleVelocitiesAndLooseControlsChangeNothing") {
        // v and r are rebuilt from u and delta rather than read, and the controls enter the flow
        // only through their bounds, so neither a stale v and r nor controls beyond the bounds may
        // move a single bit of the flow. A cost that read v or r, or saw an unclamped control,
        // would show up in the two rates.
        for (const State<T>& state : states) {
            State<T> garbled{state};
            garbled.v = static_cast<T>(12.3);
            garbled.r = static_cast<T>(-4.5);
            State<T> loose{state};
            loose.a = static_cast<T>(50.0);
            loose.omega = static_cast<T>(-7.0);
            State<T> clamped{state};
            clamped.a = kAUp<T>;
            clamped.omega = kOmegaLo<T>;
            CHECK_EQ(
                distanceOf(
                    controller.tangent(garbled, target, kHorizon<T>),
                    controller.tangent(state, target, kHorizon<T>)
                ),
                static_cast<T>(0.0)
            );
            CHECK_EQ(
                distanceOf(
                    controller.tangent(loose, target, kHorizon<T>),
                    controller.tangent(clamped, target, kHorizon<T>)
                ),
                static_cast<T>(0.0)
            );
        }
    }
}

TEST_CASE_TEMPLATE("ControlRatesDescendThePreconditionedGradient", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const State<T> state{startOf(model)};
    const State<T> target{targetOf(model)};
    const T horizon{kHorizon<T>};
    constexpr T kWeight{static_cast<T>(0.6)};

    SUBCASE("DeltaTermAlone") {
        // Left with its delta term alone, the cost reads J = w3 R^2 with R = delta + H omega -
        // delta_target, whose gradient dJ/domega = 2 w3 R H has a closed form, and the steering
        // rate descends it with the preconditioner 1/H^3. Put differently, an Euler step of length
        // h moves omega by 2 w3 h / H times the Newton step -R / H. Any factor off in either the
        // gradient or the preconditioner shows up here, and the acceleration has nothing left to
        // descend.
        const std::array<T, 5> weights{
            static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), kWeight,
            static_cast<T>(0.0)
        };
        const KinematicsFlowController<T> controller{controllerOf<T>(model, weights)};
        const State<T> k{controller.tangent(state, target, horizon)};
        const T residual{state.delta + horizon * state.omega - target.delta};
        const T gradient{static_cast<T>(2.0) * kWeight * residual * horizon};
        CHECK_EQ(
            k.omega, doctest::Approx(-gradient / (horizon * horizon * horizon))
                         .epsilon(roundoffTolerance<T>())
        );
        CHECK_EQ(k.a, static_cast<T>(0.0));
    }

    SUBCASE("UTermAlone") {
        // Left with its u term alone, the cost reads J = w4 (H R / L)^2 with R = u + H a -
        // u_target, whose gradient dJ/da = 2 w4 H^3 R / L^2 has a closed form, and the
        // acceleration descends it with the preconditioner L^2/H^5, which again comes to 2 w4 h / H
        // times the Newton step -R / H per Euler step. The steering rate has nothing left to
        // descend.
        const std::array<T, 5> weights{
            static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0),
            kWeight
        };
        const KinematicsFlowController<T> controller{controllerOf<T>(model, weights)};
        const State<T> k{controller.tangent(state, target, horizon)};
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
        // (d a_rate / d omega) / (d omega_rate / d a) = (L^2/H^5) / (1/H^3) = L^2/H^2, whatever the
        // weights, the state and the target. The two rates are separate hand derived expressions,
        // so an algebra slip in either one, such as a dropped term or a power of H off by one,
        // breaks the ratio. Every residual is affine in a and omega, which leaves both rates affine
        // in them as well, so central differences recover the derivatives up to rounding whatever
        // the step, and a wide step keeps that rounding small next to the differences. The model
        // is unbounded so that no clamp cuts into the differences.
        const KinematicsModel<T> unbounded{kFrontWheelbase<T>, kRearWheelbase<T>};
        const KinematicsFlowController<T> controller{controllerOf<T>(unbounded)};
        constexpr T kStep{static_cast<T>(1.0)};
        const auto rates_at = [&](T a_shift, T omega_shift) -> State<T> {
            State<T> shifted{state};
            shifted.a += a_shift;
            shifted.omega += omega_shift;
            return controller.tangent(shifted, target, horizon);
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

// The single step integrators hand the plant to the model, or for rungeKutta() to the flow above,
// and move the controls along the rates. The batch overloads only march the single step ones.

TEST_CASE_TEMPLATE("SingleStepIntegratorsDriveThePlantThroughTheModel", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    const T tolerance{roundoffTolerance<T>()};
    const State<T> start{startOf(model)};
    const State<T> target{targetOf(model)};
    const State<T> k{controller.tangent(start, target, kHorizon<T>)};
    // The comparisons below hold the integrators to a tolerance, so the rates at this start have
    // to be large enough that holding the controls still over a step, or stepping the plant under
    // the updated controls instead of the held ones, lands far outside it. The latter moves delta
    // and u by h^2 times a rate.
    REQUIRE_GT(
        kH<T> * kH<T> * std::min(std::abs(k.a), std::abs(k.omega)), static_cast<T>(10.0) * tolerance
    );

    SUBCASE("EulerPlantFieldsFollowTheModelIntegrators") {
        // The plant half of an Euler step is the model's own step under the controls the step
        // starts with, so the seven plant fields have to land where the model puts them.
        CHECK_LE(
            plantDistanceOf(
                controller.forwardEuler(start, target, kHorizon<T>, kH<T>),
                model.forwardEuler(start, kH<T>)
            ),
            tolerance
        );
        CHECK_LE(
            plantDistanceOf(
                controller.symplecticEuler(start, target, kHorizon<T>, kH<T>),
                model.symplecticEuler(start, kH<T>)
            ),
            tolerance
        );
    }

    SUBCASE("EulerControlsStepAlongTheRates") {
        // The controls take one Euler step along the rates tangent() gives at the start of the
        // step, whichever integrator carries the plant: the symplectic staging of the plant does
        // not extend to the controls.
        for (const State<T>& stepped :
             {controller.forwardEuler(start, target, kHorizon<T>, kH<T>),
              controller.symplecticEuler(start, target, kHorizon<T>, kH<T>)}) {
            CHECK_EQ(
                stepped.a, doctest::Approx(std::clamp(start.a + k.a * kH<T>, kALo<T>, kAUp<T>))
                               .epsilon(tolerance)
            );
            CHECK_EQ(
                stepped.omega,
                doctest::Approx(std::clamp(start.omega + k.omega * kH<T>, kOmegaLo<T>, kOmegaUp<T>))
                    .epsilon(tolerance)
            );
        }
    }

    SUBCASE("RungeKuttaStagesSeeTheHorizonLessTheirStageTime") {
        // The target is held over the step while the vehicle moves on towards it, so the two
        // midpoint stages look ahead over the horizon less h/2 and the last one over the horizon
        // less h. Handing every stage the full horizon instead is required to land well outside
        // the tolerance, or the comparison could not tell the two apart.
        const T horizon{kHorizon<T>};
        const T h{kH<T>};
        const State<T> expected{rungeKuttaThroughTangent(
            controller, start, target, horizon, horizon - h * static_cast<T>(0.5), horizon - h, h
        )};
        REQUIRE_GT(
            distanceOf(
                expected,
                rungeKuttaThroughTangent(controller, start, target, horizon, horizon, horizon, h)
            ),
            static_cast<T>(10.0) * tolerance
        );
        CHECK_LE(distanceOf(controller.rungeKutta(start, target, horizon, h), expected), tolerance);
    }

    SUBCASE("ControlsNeverLeaveTheBounds") {
        // A target far out of reach, looked ahead to over a tenth of a second, drives both
        // rates hard enough that a single unclamped step overshoots a bound, so the clamp at the
        // end of every integrator is all that keeps the controls in. The overshoot is required
        // first, or the checks below would hold trivially.
        const State<T> distant{model.createState(
            static_cast<T>(40.0), static_cast<T>(-30.0), static_cast<T>(-1.0), static_cast<T>(-0.4),
            static_cast<T>(30.0)
        )};
        constexpr T kShortHorizon{static_cast<T>(0.1)};
        const State<T> pull{controller.tangent(start, distant, kShortHorizon)};
        const T a_step{start.a + pull.a * kH<T>};
        const T omega_step{start.omega + pull.omega * kH<T>};
        REQUIRE((a_step < kALo<T> || kAUp<T> < a_step));
        REQUIRE((omega_step < kOmegaLo<T> || kOmegaUp<T> < omega_step));
        // Controls handed in beyond the bounds are only held to come back inside them. Where they
        // land in there is left open on purpose: the controller steps from the control as handed
        // in and clamps afterwards while KinematicsModel clamps first, and the two only part ways
        // for a control that was out of bounds to begin with.
        State<T> loose{start};
        loose.a = static_cast<T>(50.0);
        loose.omega = static_cast<T>(-7.0);
        for (const Integrator integrator : kIntegrators) {
            for (const State<T>& from : {start, loose}) {
                CHECK(isWithinBounds(
                    stepWith(integrator, controller, from, distant, kShortHorizon, kH<T>)
                ));
            }
        }
    }

    SUBCASE("VelocitiesComeBackOnTheConstraint") {
        // v and r are rebuilt from u and delta at the end of every step, by the model for the two
        // Euler overloads and by the controller itself for rungeKutta(), so whatever v and r a step
        // starts from, all three have to land on the constraint bit for bit.
        State<T> garbled{start};
        garbled.v = static_cast<T>(12.3);
        garbled.r = static_cast<T>(-4.5);
        for (const Integrator integrator : kIntegrators) {
            CHECK(isOnConstraint(
                stepWith(integrator, controller, garbled, target, kHorizon<T>, kH<T>)
            ));
        }
    }
}

TEST_CASE_TEMPLATE("FirstOrderConvergenceUnderAFixedPreview", T, float, double) {
    // Every step looks the same horizon H ahead along a moving reference, so its target is the
    // reference point at t_i + H, never the one at t_i. As h shrinks, all three integrators close
    // in on the loop whose target slides along the reference continuously, but only to first order
    // in h: the target comes in once per step and is held while the step is taken, which caps
    // rungeKutta() at first order as well, since its later stages would need the reference points
    // ahead of their own stage times for more. Halving h therefore halves the gap to the next
    // resolution, and the ratio below sits near 2 for all three.
    //
    // Every resolution takes its targets off one and the same finely sampled reference, so the
    // runs differ in the step alone, and the horizon spans a whole number of those samples. The
    // model is unbounded, since a clamp kicking in would kink the flow and blur the order for
    // reasons that have nothing to do with the integrators.
    const KinematicsModel<T> model{kFrontWheelbase<T>, kRearWheelbase<T>};
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    constexpr T kDuration{static_cast<T>(4.0)};
    constexpr std::size_t kFineSteps{1280};
    const T fine_h{kDuration / static_cast<T>(kFineSteps)};
    const std::size_t lead{static_cast<std::size_t>(std::lround(kHorizon<T> / fine_h))};
    const std::vector<State<T>> reference{referenceOf(model, fine_h, kFineSteps + lead)};
    const State<T> start{model.createState(
        static_cast<T>(0.5), static_cast<T>(-0.6), static_cast<T>(0.2), static_cast<T>(0.02),
        static_cast<T>(7.0)
    )};
    const auto run = [&](std::size_t stride) -> ClosedLoop<T> {
        const std::size_t steps{kFineSteps / stride};
        const T h{fine_h * static_cast<T>(stride)};
        std::vector<State<T>> targets;
        targets.reserve(steps);
        for (std::size_t i{0}; i < steps; ++i) {
            targets.push_back(reference[stride * i + lead]);
        }
        const std::vector<T> hs(steps, h);
        const std::vector<T> horizons(steps, kHorizon<T>);
        ClosedLoop<T> loop{runClosedLoop(controller, start, targets, horizons, hs)};
        for (std::size_t i{0}; i <= steps; ++i) {
            loop.ts.push_back(h * static_cast<T>(i));
            loop.reference.push_back(reference[stride * i]);
        }
        return loop;
    };

    const ClosedLoop<T> coarse{run(4)};
    const ClosedLoop<T> medium{run(2)};
    const ClosedLoop<T> fine{run(1)};
    for (const Integrator integrator : kIntegrators) {
        const T ratio{
            distanceOf(traceOf(coarse, integrator).back(), traceOf(medium, integrator).back()) /
            distanceOf(traceOf(medium, integrator).back(), traceOf(fine, integrator).back())
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

TEST_CASE_TEMPLATE("BatchOverloadsMatchMarchingSingleSteps", T, float, double) {
    const KinematicsModel<T> model{boundedModel<T>()};
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    const State<T> start{startOf(model)};
    const State<T> goal{targetOf(model)};
    // No two entries agree, within a span or across the spans, so a batch overload that read the
    // wrong index, or mixed the horizons up with the steps, cannot pass. Every horizon outlasts
    // its step, as rungeKutta() requires.
    const std::vector<T> hs{
        static_cast<T>(0.010), static_cast<T>(0.012), static_cast<T>(0.008), static_cast<T>(0.011),
        static_cast<T>(0.009)
    };
    const std::vector<T> horizons{
        static_cast<T>(0.50), static_cast<T>(0.46), static_cast<T>(0.55), static_cast<T>(0.42),
        static_cast<T>(0.48)
    };
    std::vector<State<T>> targets;
    for (std::size_t i{0}; i < hs.size(); ++i) {
        const T shift{static_cast<T>(0.1) * static_cast<T>(i)};
        targets.push_back(model.createState(
            goal.x + shift, goal.y - shift, goal.yaw + static_cast<T>(0.2) * shift,
            goal.delta - static_cast<T>(0.3) * shift, goal.u + shift
        ));
    }

    for (const Integrator integrator : kIntegrators) {
        for (const bool per_step : {true, false}) {
            // Marching the single step overload by hand gives the state after every step, which
            // is exactly what the trace is supposed to collect: one entry per step, none of them
            // the initial state.
            std::vector<State<T>> expected;
            State<T> state{start};
            for (std::size_t i{0}; i < hs.size(); ++i) {
                state = stepWith(
                    integrator, controller, state, per_step ? targets[i] : goal, horizons[i], hs[i]
                );
                expected.push_back(state);
            }
            std::vector<State<T>> trace;
            const State<T> returned{
                per_step ? batchWith(integrator, controller, start, targets, horizons, hs, trace)
                         : batchWith(integrator, controller, start, goal, horizons, hs, trace)
            };
            REQUIRE_EQ(trace.size(), expected.size());
            for (std::size_t i{0}; i < trace.size(); ++i) {
                CHECK_LE(distanceOf(trace[i], expected[i]), roundoffTolerance<T>());
            }
            CHECK_EQ(returned, trace.back());
        }
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
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    constexpr std::size_t kNumSteps{2000};
    constexpr std::size_t kSettlingSteps{500};
    constexpr T kSettledDistance{static_cast<T>(0.2)};
    const T h{kH<T>};
    // The horizon spans a whole number of steps, lead of them.
    const std::size_t lead{static_cast<std::size_t>(std::lround(kHorizon<T> / h))};
    const std::vector<State<T>> reference{referenceOf(model, h, kNumSteps + lead)};
    std::vector<State<T>> targets;
    targets.reserve(kNumSteps);
    for (std::size_t i{0}; i < kNumSteps; ++i) {
        targets.push_back(reference[i + lead]);
    }
    const std::vector<T> hs(kNumSteps, h);
    const std::vector<T> horizons(kNumSteps, kHorizon<T>);
    const State<T> start{model.createState(
        static_cast<T>(1.5), static_cast<T>(-2.0), static_cast<T>(0.1), static_cast<T>(0.0),
        static_cast<T>(5.0)
    )};

    ClosedLoop<T> loop{runClosedLoop(controller, start, targets, horizons, hs)};
    for (std::size_t i{0}; i <= kNumSteps; ++i) {
        loop.ts.push_back(h * static_cast<T>(i));
        loop.reference.push_back(reference[i]);
    }
    for (const Integrator integrator : kIntegrators) {
        const std::vector<State<T>>& trace{traceOf(loop, integrator)};
        REQUIRE_EQ(trace.size(), kNumSteps + 1);
        bool within_bounds{true};
        bool on_constraint{true};
        bool saturated{false};
        for (const State<T>& state : trace) {
            within_bounds = within_bounds && isWithinBounds(state);
            on_constraint = on_constraint && isOnConstraint(state);
            saturated = saturated || state.a == kALo<T> || state.a == kAUp<T> ||
                        state.omega == kOmegaLo<T> || state.omega == kOmegaUp<T>;
        }
        CHECK(within_bounds);
        CHECK(on_constraint);
        // The controls do reach the bounds on the way, so the first check is not vacuous.
        CHECK(saturated);
        // Written so that a NaN propagates into the result, as in distanceOverFieldsOf().
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
    const KinematicsFlowController<T> controller{controllerOf<T>(model)};
    const State<T> start{startOf(model)};
    const State<T> target{targetOf(model)};

    SUBCASE("SingularSteeringAngle") {
        // The steering angle is carried by the state, so this is where an omega that drove it out
        // of range over an earlier step surfaces.
        State<T> state{start};
        state.delta = std::numbers::pi_v<T> * static_cast<T>(0.5);
        CHECK_THROWS_AS(
            static_cast<void>(controller.tangent(state, target, kHorizon<T>)), std::invalid_argument
        );
        for (const Integrator integrator : kIntegrators) {
            CHECK_THROWS_AS(
                static_cast<void>(
                    stepWith(integrator, controller, state, target, kHorizon<T>, kH<T>)
                ),
                std::invalid_argument
            );
        }
    }

    SUBCASE("NonPositiveTimeStep") {
        for (const Integrator integrator : kIntegrators) {
            for (const T h : {static_cast<T>(0.0), static_cast<T>(-0.01)}) {
                CHECK_THROWS_AS(
                    static_cast<void>(
                        stepWith(integrator, controller, start, target, kHorizon<T>, h)
                    ),
                    std::invalid_argument
                );
            }
        }
    }

    SUBCASE("HorizonHasToOutlastTheStep") {
        // An Euler step is fine with a horizon that spans exactly the step, while the last stage
        // of rungeKutta() looks ahead over the horizon less the step, which has to stay positive
        // for the target to lie ahead of that stage, so it needs strictly more. tangent() itself
        // takes any positive horizon.
        const T h{kH<T>};
        for (const Integrator integrator :
             {Integrator::FORWARD_EULER, Integrator::SYMPLECTIC_EULER}) {
            CHECK_NOTHROW(static_cast<void>(stepWith(integrator, controller, start, target, h, h)));
            CHECK_THROWS_AS(
                static_cast<void>(
                    stepWith(integrator, controller, start, target, h * static_cast<T>(0.5), h)
                ),
                std::invalid_argument
            );
        }
        CHECK_THROWS_AS(
            static_cast<void>(controller.rungeKutta(start, target, h, h)), std::invalid_argument
        );
        CHECK_NOTHROW(
            static_cast<void>(controller.rungeKutta(start, target, h * static_cast<T>(1.01), h))
        );
        CHECK_THROWS_AS(
            static_cast<void>(controller.tangent(start, target, static_cast<T>(0.0))),
            std::invalid_argument
        );
    }

    SUBCASE("BatchOverloadsRejectMismatchedSpanSizes") {
        const std::vector<T> hs(4, kH<T>);
        const std::vector<T> horizons(4, kHorizon<T>);
        const std::vector<T> short_horizons(3, kHorizon<T>);
        const std::vector<State<T>> targets(4, target);
        const std::vector<State<T>> short_targets(3, target);
        for (const Integrator integrator : kIntegrators) {
            std::vector<State<T>> trace;
            CHECK_THROWS_AS(
                static_cast<void>(
                    batchWith(integrator, controller, start, short_targets, horizons, hs, trace)
                ),
                std::invalid_argument
            );
            CHECK_THROWS_AS(
                static_cast<void>(
                    batchWith(integrator, controller, start, targets, short_horizons, hs, trace)
                ),
                std::invalid_argument
            );
            CHECK_THROWS_AS(
                static_cast<void>(
                    batchWith(integrator, controller, start, target, short_horizons, hs, trace)
                ),
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
