/**
 * @file test_state.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-08-08
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/bicycle/models/state.hpp"

#include <cmath>
#include <concepts>
#include <cstddef>
#include <numbers>
#include <system_error>

#include "zpp_bits.h"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::bicycle {

namespace {

template <std::floating_point T>
[[nodiscard]] constexpr auto tolerance() noexcept -> T {
    return std::same_as<T, float> ? static_cast<T>(1.0e-5) : static_cast<T>(1.0e-12);
}

template <std::floating_point T>
[[nodiscard]] auto roundtrip(const State<T>& state) -> State<T> {
    auto [data, in, out] = zpp::bits::data_in_out();
    REQUIRE(out(state).code == std::errc{});
    State<T> restored{};
    REQUIRE(in(restored).code == std::errc{});
    return restored;
}

/**
 * @brief Two operands whose fields are all distinct and exactly representable.
 *
 * Every field carries a different value so that an operator that mixed two of them up, or that
 * left one untouched, cannot hide behind a coincidence. Sticking to binary fractions keeps the
 * expected results exact for float as well as for double, including under the reciprocal
 * multiplication State::operator/=() performs.
 */
template <std::floating_point T>
[[nodiscard]] constexpr auto lhsOperand() noexcept -> State<T> {
    return State<T>{
        .x{static_cast<T>(1.5)},
        .y{static_cast<T>(-2.5)},
        .yaw{static_cast<T>(0.75)},
        .delta{static_cast<T>(0.125)},
        .u{static_cast<T>(7.5)},
        .v{static_cast<T>(-0.25)},
        .r{static_cast<T>(0.5)},
        .a{static_cast<T>(3.0)},
        .omega{static_cast<T>(-0.375)}
    };
}

template <std::floating_point T>
[[nodiscard]] constexpr auto rhsOperand() noexcept -> State<T> {
    return State<T>{
        .x{static_cast<T>(0.5)},
        .y{static_cast<T>(1.5)},
        .yaw{static_cast<T>(-0.25)},
        .delta{static_cast<T>(-0.75)},
        .u{static_cast<T>(2.5)},
        .v{static_cast<T>(0.75)},
        .r{static_cast<T>(-1.5)},
        .a{static_cast<T>(-1.25)},
        .omega{static_cast<T>(0.625)}
    };
}

template <std::floating_point T>
auto checkFields(const State<T>& state, T x, T y, T yaw, T delta, T u, T v, T r, T a, T omega)
    -> void {
    CHECK_EQ(state.x, x);
    CHECK_EQ(state.y, y);
    CHECK_EQ(state.yaw, yaw);
    CHECK_EQ(state.delta, delta);
    CHECK_EQ(state.u, u);
    CHECK_EQ(state.v, v);
    CHECK_EQ(state.r, r);
    CHECK_EQ(state.a, a);
    CHECK_EQ(state.omega, omega);
}

/**
 * @brief Runs a whole chain of the arithmetic operators through constant evaluation.
 */
template <std::floating_point T>
[[nodiscard]] consteval auto foldedAtCompileTime() noexcept -> State<T> {
    State<T> result{lhsOperand<T>()};
    result += rhsOperand<T>();
    result -= rhsOperand<T>();
    result *= static_cast<T>(2.0);
    result /= static_cast<T>(4.0);
    return -result;
}

} // namespace

TEST_CASE_TEMPLATE("Layout", T, float, double) {
    SUBCASE("SizeCoversEveryField") {
        CHECK_EQ(State<T>::size(), 9);
        static_assert(State<T>::size() == 9);
    }

    SUBCASE("DataSpansEveryFieldInDeclarationOrder") {
        // data() and size() are the contiguous view the rest of the code gets onto the nine
        // fields, so every one of them has to sit inside that range, in declaration order.
        State<T> state{lhsOperand<T>()};
        CHECK_EQ(state.data(), &state.x);
        CHECK_EQ(&state.y - state.data(), 1);
        CHECK_EQ(&state.yaw - state.data(), 2);
        CHECK_EQ(&state.delta - state.data(), 3);
        CHECK_EQ(&state.u - state.data(), 4);
        CHECK_EQ(&state.v - state.data(), 5);
        CHECK_EQ(&state.r - state.data(), 6);
        CHECK_EQ(&state.a - state.data(), 7);
        CHECK_EQ(&state.omega - state.data(), static_cast<std::ptrdiff_t>(State<T>::size()) - 1);
        CHECK_EQ(sizeof(State<T>), State<T>::size() * sizeof(T));

        const State<T>& const_state{state};
        CHECK_EQ(const_state.data(), &const_state.x);
    }

    SUBCASE("DefaultConstructedIsAllZeros") {
        const State<T> state{};
        checkFields<T>(
            state, static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0),
            static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0), static_cast<T>(0.0),
            static_cast<T>(0.0), static_cast<T>(0.0)
        );
    }
}

TEST_CASE_TEMPLATE("Arithmetic", T, float, double) {
    const State<T> lhs{lhsOperand<T>()};
    const State<T> rhs{rhsOperand<T>()};

    SUBCASE("Equality") {
        CHECK_EQ(lhs, lhsOperand<T>());
        CHECK_NE(lhs, rhs);
        // Every field has to take part in the comparison, so perturbing any one of them alone
        // already makes the two differ.
        for (std::size_t i{0}; i < State<T>::size(); ++i) {
            State<T> perturbed{lhs};
            perturbed.data()[i] += static_cast<T>(1.0);
            CHECK_NE(perturbed, lhs);
        }
    }

    SUBCASE("AdditionTouchesEveryField") {
        State<T> sum{lhs};
        sum += rhs;
        checkFields<T>(
            sum, static_cast<T>(2.0), static_cast<T>(-1.0), static_cast<T>(0.5),
            static_cast<T>(-0.625), static_cast<T>(10.0), static_cast<T>(0.5), static_cast<T>(-1.0),
            static_cast<T>(1.75), static_cast<T>(0.25)
        );
        CHECK_EQ(lhs + rhs, sum);
    }

    SUBCASE("SubtractionTouchesEveryField") {
        State<T> difference{lhs};
        difference -= rhs;
        checkFields<T>(
            difference, static_cast<T>(1.0), static_cast<T>(-4.0), static_cast<T>(1.0),
            static_cast<T>(0.875), static_cast<T>(5.0), static_cast<T>(-1.0), static_cast<T>(2.0),
            static_cast<T>(4.25), static_cast<T>(-1.0)
        );
        CHECK_EQ(lhs - rhs, difference);
    }

    SUBCASE("ScalingTouchesEveryField") {
        State<T> scaled{lhs};
        scaled *= static_cast<T>(2.0);
        checkFields<T>(
            scaled, static_cast<T>(3.0), static_cast<T>(-5.0), static_cast<T>(1.5),
            static_cast<T>(0.25), static_cast<T>(15.0), static_cast<T>(-0.5), static_cast<T>(1.0),
            static_cast<T>(6.0), static_cast<T>(-0.75)
        );
        CHECK_EQ(lhs * static_cast<T>(2.0), scaled);
        CHECK_EQ(static_cast<T>(2.0) * lhs, scaled);
    }

    SUBCASE("DivisionTouchesEveryField") {
        State<T> divided{lhs};
        divided /= static_cast<T>(4.0);
        checkFields<T>(
            divided, static_cast<T>(0.375), static_cast<T>(-0.625), static_cast<T>(0.1875),
            static_cast<T>(0.03125), static_cast<T>(1.875), static_cast<T>(-0.0625),
            static_cast<T>(0.125), static_cast<T>(0.75), static_cast<T>(-0.09375)
        );
        CHECK_EQ(lhs / static_cast<T>(4.0), divided);
    }

    SUBCASE("NegationTouchesEveryField") {
        checkFields<T>(
            -lhs, static_cast<T>(-1.5), static_cast<T>(2.5), static_cast<T>(-0.75),
            static_cast<T>(-0.125), static_cast<T>(-7.5), static_cast<T>(0.25),
            static_cast<T>(-0.5), static_cast<T>(-3.0), static_cast<T>(0.375)
        );
        CHECK_EQ(-(-lhs), lhs);
        CHECK_EQ(-State<T>{lhs}, -lhs);
    }

    SUBCASE("RvalueOverloadsAgreeWithLvalueOnes") {
        CHECK_EQ(State<T>{lhs} + rhs, lhs + rhs);
        CHECK_EQ(lhs + State<T>{rhs}, lhs + rhs);
        CHECK_EQ(State<T>{lhs} + State<T>{rhs}, lhs + rhs);
        // The rvalue right hand side of a subtraction is negated in place and added to, rather
        // than subtracted from, so it is the one overload whose operand order can flip.
        CHECK_EQ(lhs - State<T>{rhs}, lhs - rhs);
        CHECK_EQ(State<T>{lhs} - rhs, lhs - rhs);
        CHECK_EQ(State<T>{lhs} - State<T>{rhs}, lhs - rhs);
        CHECK_EQ(State<T>{lhs} * static_cast<T>(2.0), lhs * static_cast<T>(2.0));
        CHECK_EQ(static_cast<T>(2.0) * State<T>{lhs}, lhs * static_cast<T>(2.0));
        CHECK_EQ(State<T>{lhs} / static_cast<T>(4.0), lhs / static_cast<T>(4.0));
    }

    SUBCASE("UsableInConstantEvaluation") {
        constexpr State<T> folded{foldedAtCompileTime<T>()};
        State<T> expected{lhs};
        expected += rhs;
        expected -= rhs;
        expected *= static_cast<T>(2.0);
        expected /= static_cast<T>(4.0);
        CHECK_EQ(folded, -expected);
    }
}

TEST_CASE_TEMPLATE("SerializationRoundtrip", T, float, double) {
    SUBCASE("DefaultConstructed") {
        const State<T> state{};
        CHECK_EQ(roundtrip(state), state);
    }

    SUBCASE("DrivingForward") {
        const State<T> state{
            .x{static_cast<T>(1.5)},
            .y{static_cast<T>(-2.5)},
            .yaw{static_cast<T>(0.3)},
            .delta{static_cast<T>(0.15)},
            .u{static_cast<T>(7.9)},
            .v{static_cast<T>(0.64)},
            .r{static_cast<T>(0.46)},
            .a{static_cast<T>(1.2)},
            .omega{static_cast<T>(-0.08)}
        };
        const State<T> restored{roundtrip(state)};
        CHECK_EQ(restored, state);
        checkFields<T>(
            restored, state.x, state.y, state.yaw, state.delta, state.u, state.v, state.r, state.a,
            state.omega
        );
    }

    SUBCASE("Reversing") {
        const State<T> state{
            .x{static_cast<T>(-3.2)},
            .y{static_cast<T>(4.1)},
            .yaw{static_cast<T>(-1.1)},
            .delta{static_cast<T>(-0.1)},
            .u{static_cast<T>(-6.0)},
            .v{static_cast<T>(-0.33)},
            .r{static_cast<T>(-0.24)},
            .a{static_cast<T>(-0.7)},
            .omega{static_cast<T>(0.05)}
        };
        const State<T> restored{roundtrip(state)};
        CHECK_EQ(restored, state);
        // The sign of the speed is derived rather than stored, so the roundtrip has to preserve
        // it through the velocity components alone.
        CHECK_EQ(restored.speed(), state.speed());
        CHECK_LT(restored.speed(), static_cast<T>(0.0));
    }
}

TEST_CASE_TEMPLATE("SpeedAndSlipAngle", T, float, double) {
    constexpr T kLongitudinal{static_cast<T>(8.0)};
    constexpr T kLateral{static_cast<T>(0.65)};

    SUBCASE("ForwardKeepsPositiveSign") {
        const State<T> state{.u{kLongitudinal}, .v{kLateral}};
        CHECK_EQ(
            state.speed(),
            doctest::Approx(std::hypot(kLongitudinal, kLateral)).epsilon(tolerance<T>())
        );
        CHECK_GT(state.speed(), static_cast<T>(0.0));
        CHECK_EQ(
            state.slipAngle(),
            doctest::Approx(std::atan2(kLateral, kLongitudinal)).epsilon(tolerance<T>())
        );
        CHECK_GT(state.slipAngle(), static_cast<T>(0.0));
    }

    SUBCASE("ReverseFlipsSignOfSpeed") {
        const State<T> state{.u{-kLongitudinal}, .v{-kLateral}};
        CHECK_EQ(
            state.speed(),
            doctest::Approx(-std::hypot(kLongitudinal, kLateral)).epsilon(tolerance<T>())
        );
        // The magnitude alone cannot tell the two apart, which is why speed() takes its sign from
        // the longitudinal component.
        CHECK_EQ(
            std::hypot(state.u, state.v),
            doctest::Approx(std::hypot(kLongitudinal, kLateral)).epsilon(tolerance<T>())
        );
        // Driving backwards points the velocity the other way round, so the slip angle sits near
        // the far end of the range rather than near zero.
        CHECK_GT(std::abs(state.slipAngle()), std::numbers::pi_v<T> * static_cast<T>(0.5));
    }

    SUBCASE("StandstillIsSignless") {
        const State<T> state{};
        CHECK_EQ(state.speed(), static_cast<T>(0.0));
        CHECK_EQ(state.slipAngle(), static_cast<T>(0.0));
    }

    SUBCASE("BothAreHeadingIndependent") {
        // u, v and r live in the body frame, so neither derived quantity may look at the heading.
        const State<T> state{.u{kLongitudinal}, .v{kLateral}};
        for (const T yaw :
             {static_cast<T>(0.0), static_cast<T>(1.9), static_cast<T>(3.0),
              static_cast<T>(-2.7)}) {
            State<T> rotated{state};
            rotated.yaw = yaw;
            CHECK_EQ(rotated.speed(), state.speed());
            CHECK_EQ(rotated.slipAngle(), state.slipAngle());
        }
    }
}

} // namespace basjoo::bicycle
