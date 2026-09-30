/**
 * @file test_piecewise_cubic_curve2.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-09-01
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/curves/piecewise_cubic_curve.hpp"

#include <cmath>
#include <numbers>
#include <tuple>

#include "cxxopts.hpp"
#include "matplot/matplot.h"
#include "zpp_bits.h"

#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/functions/boundary_mode.hpp"
#include "basjoo/math/utils.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

namespace {

bool plot_graph{false};

auto createFigureHandle() noexcept -> matplot::figure_handle {
    matplot::figure_handle fig = matplot::figure();
    fig->size(1600, 1000);
    return fig;
}

} // namespace

namespace basjoo::math {

TEST_CASE("LeftSemiCircleTest") {
    const auto exact_semi_circle = [](double theta) noexcept -> Vec2d {
        return Vec2d{2.0 * std::cos(theta), 2.0 * std::sin(theta)};
    };

    constexpr std::size_t kNumAnchors{21};
    constexpr double kStart{0.0};
    constexpr double kEnd{M_PI};
    constexpr double kStep{(kEnd - kStart) / (kNumAnchors - 1)};

    std::vector<Vec2d> anchor_points(kNumAnchors);
    std::ranges::generate(
        anchor_points, [t = kStart - kStep, h = kStep]() mutable noexcept -> Vec2d {
            t += h;
            return Vec2d{2.0 * std::cos(t), 2.0 * std::sin(t)};
        }
    );

    BoundaryMode<Vec2d> b0, bf;
    SUBCASE("Clamped") {
        b0 = BoundaryMode<Vec2d>{.order = 1, .derivative = Vec2d{0.0, 1.0}};
        bf = BoundaryMode<Vec2d>{.order = 1, .derivative = Vec2d{0.0, -1.0}};
    }
    SUBCASE("Natural") {
        b0 = BoundaryMode<Vec2d>{.order = 2, .derivative = Vec2d{-0.5, 0.0}};
        bf = BoundaryMode<Vec2d>{.order = 2, .derivative = Vec2d{0.5, 0.0}};
    }

    const PiecewiseCubicCurve<Vec2d> semi_circle{anchor_points, b0, bf};

    CHECK_EQ(semi_circle.minS(), 0.0);
    CHECK_EQ(semi_circle.maxS(), doctest::Approx(2.0 * M_PI).epsilon(1E-6));

    for (std::size_t i{1}; i < kNumAnchors; ++i) {
        const double theta{kStart + (i - 0.5) * kStep};
        const double s_exact{theta * 2.0};
        const Vec2d point = exact_semi_circle(theta);
        const auto [s, l] = semi_circle.inverse(point);
        const Vec2d tangent{-std::sin(theta), std::cos(theta)};
        const Vec2d normal{-std::cos(theta), -std::sin(theta)};
        CHECK_EQ(s, doctest::Approx(s_exact).epsilon(1E-6));
        CHECK_EQ(l, doctest::Approx(0.0).epsilon(1E-5));
        CHECK(semi_circle.tangent(s_exact).identicalTo(tangent, 1E-4));
        CHECK(semi_circle.normal(s_exact).identicalTo(normal, 1E-4));
        CHECK_EQ(semi_circle.curvature(s_exact), doctest::Approx(0.5).epsilon(1E-3));
    }

    const std::array<Vec2d, 2> milestone_points{
        Vec2d{std::sqrt(3.0) * 0.5, 0.5}, Vec2d{-1.5, std::sqrt(3.0) * 1.5}
    };

    auto [s, l] = semi_circle.inverse(milestone_points[0]);
    CHECK_EQ(s, doctest::Approx(M_PI / 3.0).epsilon(1E-4));
    CHECK_EQ(l, doctest::Approx(1.0).epsilon(1E-5));
    CHECK(semi_circle(s, l).identicalTo(milestone_points[0], 1E-4));

    std::tie(s, l) = semi_circle.inverse(milestone_points[1]);
    CHECK_EQ(s, doctest::Approx(4.0 * M_PI / 3.0).epsilon(1E-5));
    CHECK_EQ(l, doctest::Approx(-1.0).epsilon(1E-5));
    CHECK(semi_circle(s, l).identicalTo(milestone_points[1], 1E-4));

    if (plot_graph) {
        using namespace matplot;

        std::array<double, kNumAnchors> anchor_xs, anchor_ys;
        std::ranges::generate(anchor_xs, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].x;
        });
        std::ranges::generate(anchor_ys, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].y;
        });

        constexpr std::size_t kNumExact{101};
        std::array<double, kNumExact> exact_xs, exact_ys;
        std::ranges::generate(
            exact_xs,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 2.0 * std::cos(t);
            }
        );
        std::ranges::generate(
            exact_ys,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 2.0 * std::sin(t);
            }
        );

        constexpr std::size_t kNumTest{20};
        std::array<double, kNumTest> test_xs, test_ys;
        {
            const double start_s{semi_circle.minS()}, end_s{semi_circle.maxS()};
            const double step_s{(end_s - start_s) / kNumTest};
            for (std::size_t i{0}; i < kNumTest; ++i) {
                const Vec2d point = semi_circle(start_s + step_s * (i + 0.5));
                test_xs[i] = point.x;
                test_ys[i] = point.y;
            }
        }

        constexpr std::size_t kNumIntpl{101};
        std::array<double, kNumIntpl> intpl_xs, intpl_ys;
        {
            const double start_s{semi_circle.minS()}, end_s{semi_circle.maxS()};
            const double step_s{(end_s - start_s) / (kNumIntpl - 1)};
            for (std::size_t i{0}; i < kNumIntpl; ++i) {
                const Vec2d point = semi_circle(start_s + step_s * i);
                intpl_xs[i] = point.x;
                intpl_ys[i] = point.y;
            }
        }

        auto fig = createFigureHandle();
        fig->title("Semi-Circle");
        hold(on);
        grid(on);
        axis(equal);
        plot(anchor_xs, anchor_ys, "b.");
        plot(exact_xs, exact_ys, "b:");
        plot(test_xs, test_ys, "r*");
        plot(intpl_xs, intpl_ys, "r:");
        fig->show();
    }
}

TEST_CASE("RightSemiCircleTest") {
    const auto exact_semi_circle = [](double theta) noexcept -> Vec2d {
        return Vec2d{2.0 * std::cos(theta), 2.0 * std::sin(theta)};
    };

    constexpr std::size_t kNumAnchors{21};
    constexpr double kStart{M_PI};
    constexpr double kEnd{0.0};
    constexpr double kStep{(kEnd - kStart) / (kNumAnchors - 1)};

    std::vector<Vec2d> anchor_points(kNumAnchors);
    std::ranges::generate(
        anchor_points, [t = kStart - kStep, h = kStep]() mutable noexcept -> Vec2d {
            t += h;
            return Vec2d{2.0 * std::cos(t), 2.0 * std::sin(t)};
        }
    );

    BoundaryMode<Vec2d> b0, bf;
    SUBCASE("Clamped") {
        b0 = BoundaryMode<Vec2d>{.order = 1, .derivative = Vec2d{0.0, 1.0}};
        bf = BoundaryMode<Vec2d>{.order = 1, .derivative = Vec2d{0.0, -1.0}};
    }
    SUBCASE("Natural") {
        b0 = BoundaryMode<Vec2d>{.order = 2, .derivative = Vec2d{0.5, 0.0}};
        bf = BoundaryMode<Vec2d>{.order = 2, .derivative = Vec2d{-0.5, 0.0}};
    }

    const PiecewiseCubicCurve<Vec2d> semi_circle{anchor_points, b0, bf};

    CHECK_EQ(semi_circle.minS(), 0.0);
    CHECK_EQ(semi_circle.maxS(), doctest::Approx(2.0 * M_PI).epsilon(1E-6));

    for (std::size_t i{1}; i < kNumAnchors; ++i) {
        const double theta{kStart + (i - 0.5) * kStep};
        const double s_exact{(M_PI - theta) * 2.0};
        const Vec2d point = exact_semi_circle(theta);
        const auto [s, l] = semi_circle.inverse(point);
        const Vec2d tangent{std::sin(theta), -std::cos(theta)};
        const Vec2d normal{-std::cos(theta), -std::sin(theta)};
        CHECK_EQ(s, doctest::Approx(s_exact).epsilon(1E-6));
        CHECK_EQ(l, doctest::Approx(0.0).epsilon(1E-5));
        CHECK(semi_circle.tangent(s_exact).identicalTo(tangent, 1E-4));
        CHECK(semi_circle.normal(s_exact).identicalTo(normal, 1E-4));
        CHECK_EQ(semi_circle.curvature(s_exact), doctest::Approx(0.5).epsilon(1E-3));
    }

    const std::array<Vec2d, 2> milestone_points{
        Vec2d{std::numbers::sqrt3 * 0.5, 0.5}, Vec2d{-1.5, std::numbers::sqrt3 * 1.5}
    };

    auto [s, l] = semi_circle.inverse(milestone_points[0]);
    CHECK_EQ(s, doctest::Approx(M_PI * 5.0 / 3.0).epsilon(1E-4));
    CHECK_EQ(l, doctest::Approx(1.0).epsilon(1E-5));
    CHECK(semi_circle(s, l).identicalTo(milestone_points[0], 1E-3));

    std::tie(s, l) = semi_circle.inverse(milestone_points[1]);
    CHECK_EQ(s, doctest::Approx(M_PI * 2.0 / 3.0).epsilon(1E-5));
    CHECK_EQ(l, doctest::Approx(-1.0).epsilon(1E-5));
    CHECK(semi_circle(s, l).identicalTo(milestone_points[1], 1E-4));

    if (plot_graph) {
        using namespace matplot;

        std::array<double, kNumAnchors> anchor_xs, anchor_ys;
        std::ranges::generate(anchor_xs, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].x;
        });
        std::ranges::generate(anchor_ys, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].y;
        });

        constexpr std::size_t kNumExact{101};
        std::array<double, kNumExact> exact_xs, exact_ys;
        std::ranges::generate(
            exact_xs,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 2.0 * std::cos(t);
            }
        );
        std::ranges::generate(
            exact_ys,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 2.0 * std::sin(t);
            }
        );

        constexpr std::size_t kNumTest{20};
        std::array<double, kNumTest> test_xs, test_ys;
        {
            const double start_s{semi_circle.minS()}, end_s{semi_circle.maxS()};
            const double step_s{(end_s - start_s) / kNumTest};
            for (std::size_t i{0}; i < kNumTest; ++i) {
                const Vec2d point = semi_circle(start_s + step_s * (i + 0.5));
                test_xs[i] = point.x;
                test_ys[i] = point.y;
            }
        }

        constexpr std::size_t kNumIntpl{101};
        std::array<double, kNumIntpl> intpl_xs, intpl_ys;
        {
            const double start_s{semi_circle.minS()}, end_s{semi_circle.maxS()};
            const double step_s{(end_s - start_s) / (kNumIntpl - 1)};
            for (std::size_t i{0}; i < kNumIntpl; ++i) {
                const Vec2d point = semi_circle(start_s + step_s * i);
                intpl_xs[i] = point.x;
                intpl_ys[i] = point.y;
            }
        }

        auto fig = createFigureHandle();
        fig->title("Semi-Circle");
        hold(on);
        grid(on);
        axis(equal);
        plot(anchor_xs, anchor_ys, "b.");
        plot(exact_xs, exact_ys, "b:");
        plot(test_xs, test_ys, "r*");
        plot(intpl_xs, intpl_ys, "r:");
        fig->show();
    }
}

TEST_CASE("LogarithmicSpiralTest") {
    const auto exact_log_spiral = [](double theta) noexcept -> Vec2d {
        return Vec2d{
            0.5 * std::exp(0.2 * theta) * std::cos(theta),
            0.5 * std::exp(0.2 * theta) * std::sin(theta)
        };
    };

    constexpr std::size_t kNumAnchors{101};
    constexpr double kStart{0.0};
    constexpr double kEnd{8.0 * M_PI};
    constexpr double kStep{(kEnd - kStart) / (kNumAnchors - 1)};

    std::vector<Vec2d> anchor_points(kNumAnchors);
    std::ranges::generate(
        anchor_points, [t = kStart - kStep, h = kStep]() mutable noexcept -> Vec2d {
            t += h;
            return Vec2d{
                0.5 * std::exp(0.2 * t) * std::cos(t), 0.5 * std::exp(0.2 * t) * std::sin(t)
            };
        }
    );

    const PiecewiseCubicCurve<Vec2d> log_spiral{anchor_points};

    for (std::size_t i{10}; i < kNumAnchors - 9; ++i) {
        const Vec2d point = exact_log_spiral(kStart + (i - 0.5) * kStep);
        const auto [s, l] = log_spiral.inverse(point);
        CHECK_EQ(l, doctest::Approx(0.0).epsilon(2E-3));
    }

    if (plot_graph) {
        using namespace matplot;

        std::array<double, kNumAnchors> anchor_xs, anchor_ys;
        std::ranges::generate(anchor_xs, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].x;
        });
        std::ranges::generate(anchor_ys, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].y;
        });

        constexpr std::size_t kNumExact{1001};
        std::array<double, kNumExact> exact_xs, exact_ys;
        std::ranges::generate(
            exact_xs,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 0.5 * std::exp(0.2 * t) * std::cos(t);
            }
        );
        std::ranges::generate(
            exact_ys,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 0.5 * std::exp(0.2 * t) * std::sin(t);
            }
        );

        constexpr std::size_t kNumTest{100};
        std::array<double, kNumTest> test_xs, test_ys;
        {
            const double start_s{log_spiral.minS()}, end_s{log_spiral.maxS()};
            const double step_s{(end_s - start_s) / kNumTest};
            for (std::size_t i{0}; i < kNumTest; ++i) {
                const Vec2d point = log_spiral(start_s + step_s * (i + 0.5));
                test_xs[i] = point.x;
                test_ys[i] = point.y;
            }
        }

        constexpr std::size_t kNumIntpl{1001};
        std::array<double, kNumIntpl> intpl_xs, intpl_ys;
        {
            const double start_s{log_spiral.minS()}, end_s{log_spiral.maxS()};
            const double step_s{(end_s - start_s) / (kNumIntpl - 1)};
            for (std::size_t i{0}; i < kNumIntpl; ++i) {
                const Vec2d point = log_spiral(start_s + step_s * i);
                intpl_xs[i] = point.x;
                intpl_ys[i] = point.y;
            }
        }

        auto fig = createFigureHandle();
        fig->title("Logarithmic Spiral");
        hold(on);
        grid(on);
        axis(equal);
        plot(anchor_xs, anchor_ys, "b.");
        plot(exact_xs, exact_ys, "b:");
        plot(test_xs, test_ys, "r*");
        plot(intpl_xs, intpl_ys, "r:");
        fig->show();
    }
}

TEST_CASE("LissajousCurveTest") {
    const auto exact_lissajous_curve = [](double theta) noexcept -> Vec2d {
        return Vec2d{2.0 * std::sin(2.0 * theta), 2.0 * std::sin(3.0 * theta)};
    };

    constexpr std::size_t kNumAnchors{101};
    constexpr double kStart{0.0};
    constexpr double kEnd{2.0 * M_PI};
    constexpr double kStep{(kEnd - kStart) / (kNumAnchors - 1)};

    std::vector<Vec2d> anchor_points(kNumAnchors);
    std::ranges::generate(
        anchor_points, [t = kStart - kStep, h = kStep]() mutable noexcept -> Vec2d {
            t += h;
            return Vec2d{2.0 * std::sin(2.0 * t), 2.0 * std::sin(3.0 * t)};
        }
    );

    const PiecewiseCubicCurve<Vec2d> lissajous_curve{periodic_tag{}, anchor_points};

    CHECK(lissajous_curve.periodic());

    const double perimeter{lissajous_curve.maxS() - lissajous_curve.minS()};
    for (double dt : {-2.0 * perimeter, -perimeter, perimeter, 2.0 * perimeter}) {
        for (double s : linspace(lissajous_curve.minS() + 1E-3, lissajous_curve.maxS() - 1E-3, 9)) {
            CHECK(lissajous_curve.eval(s + dt).identicalTo(lissajous_curve.eval(s), 1E-6));
            CHECK(lissajous_curve.tangent(s + dt).identicalTo(lissajous_curve.tangent(s), 1E-6));
            CHECK(lissajous_curve.normal(s + dt).identicalTo(lissajous_curve.normal(s), 1E-6));
            CHECK_EQ(
                lissajous_curve.curvature(s + dt),
                doctest::Approx(lissajous_curve.curvature(s)).epsilon(1E-6)
            );
        }
    }

    const auto& arc_lengths{lissajous_curve.arcLengths()};

    for (std::size_t i{1}; i < kNumAnchors; ++i) {
        const Vec2d point = exact_lissajous_curve(kStart + (i - 0.5) * kStep);
        const auto [s, l] = lissajous_curve.inverse(point, arc_lengths[i - 1], arc_lengths[i]);
        CHECK_EQ(l, doctest::Approx(0.0).epsilon(7E-4));
    }

    if (plot_graph) {
        using namespace matplot;

        std::array<double, kNumAnchors> anchor_xs, anchor_ys;
        std::ranges::generate(anchor_xs, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].x;
        });
        std::ranges::generate(anchor_ys, [&anchor_points, i = -1]() mutable noexcept -> double {
            ++i;
            return anchor_points[i].y;
        });

        constexpr std::size_t kNumExact{1001};
        std::array<double, kNumExact> exact_xs, exact_ys;
        std::ranges::generate(
            exact_xs,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 2.0 * std::sin(2.0 * t);
            }
        );
        std::ranges::generate(
            exact_ys,
            [t = kStart - (kEnd - kStart) / (kNumExact - 1),
             h = (kEnd - kStart) / (kNumExact - 1)]() mutable noexcept -> double {
                t += h;
                return 2.0 * std::sin(3.0 * t);
            }
        );

        constexpr std::size_t kNumTest{100};
        std::array<double, kNumTest> test_xs, test_ys;
        {
            const double start_s{lissajous_curve.minS()}, end_s{lissajous_curve.maxS()};
            const double step_s{(end_s - start_s) / kNumTest};
            for (std::size_t i{0}; i < kNumTest; ++i) {
                const Vec2d point = lissajous_curve(start_s + step_s * (i + 0.5));
                test_xs[i] = point.x;
                test_ys[i] = point.y;
            }
        }

        constexpr std::size_t kNumIntpl{1001};
        std::array<double, kNumIntpl> intpl_xs, intpl_ys;
        {
            const double start_s{lissajous_curve.minS()}, end_s{lissajous_curve.maxS()};
            const double step_s{(end_s - start_s) / (kNumIntpl - 1)};
            for (std::size_t i{0}; i < kNumIntpl; ++i) {
                const Vec2d point = lissajous_curve(start_s + step_s * i);
                intpl_xs[i] = point.x;
                intpl_ys[i] = point.y;
            }
        }

        auto fig = createFigureHandle();
        fig->title("LissajousCurve");
        hold(on);
        grid(on);
        axis(equal);
        plot(anchor_xs, anchor_ys, "b.");
        plot(exact_xs, exact_ys, "b:");
        plot(test_xs, test_ys, "r*");
        plot(intpl_xs, intpl_ys, "r:");
        fig->show();
    }
}

TEST_CASE("Serialization") {
    [[maybe_unused]] const auto exact_lissajous_curve = [](double theta) noexcept -> Vec2d {
        return Vec2d{2.0 * std::sin(2.0 * theta), 2.0 * std::sin(3.0 * theta)};
    };

    constexpr std::size_t kNumAnchors{101};
    constexpr double kStart{0.0};
    constexpr double kEnd{2.0 * M_PI};
    constexpr double kStep{(kEnd - kStart) / (kNumAnchors - 1)};

    std::vector<Vec2d> anchor_points(kNumAnchors);
    std::ranges::generate(
        anchor_points, [t = kStart - kStep, h = kStep]() mutable noexcept -> Vec2d {
            t += h;
            return Vec2d{2.0 * std::sin(2.0 * t), 2.0 * std::sin(3.0 * t)};
        }
    );

    const PiecewiseCubicCurve<Vec2d> lissajous_curve{periodic_tag{}, anchor_points};

    auto [data, out] = zpp::bits::data_out();
    out(lissajous_curve).or_throw();

    PiecewiseCubicCurve<Vec2d> other_lissajous_curve;

    auto in = zpp::bits::in(data);
    in(other_lissajous_curve).or_throw();

    const auto& arc_length{lissajous_curve.arcLengths()};
    const auto& other_arc_length{other_lissajous_curve.arcLengths()};
    const auto& other_anchor_points{other_lissajous_curve.anchorPoints()};

    for (std::size_t i = 0; i < kNumAnchors; ++i) {
        CHECK_EQ(other_arc_length[i], doctest::Approx(arc_length[i]).epsilon(kEpsilon));
        CHECK_EQ(other_anchor_points[i].x, doctest::Approx(anchor_points[i].x).epsilon(kEpsilon));
        CHECK_EQ(other_anchor_points[i].y, doctest::Approx(anchor_points[i].y).epsilon(kEpsilon));
    }
}

} // namespace basjoo::math

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options(
        "piecewise_linear_function_test", "unit test of PiecewiseLinearFunction class"
    );
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
