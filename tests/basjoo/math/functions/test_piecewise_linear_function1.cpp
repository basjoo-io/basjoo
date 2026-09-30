/**
 * @file test_piecewise_linear_function1.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-11
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/functions/piecewise_linear_function.hpp"

#include "cxxopts.hpp"
#include "matplot/matplot.h"
#include "zpp_bits.h"

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

auto createAxesHandles(matplot::figure_handle& fig) noexcept -> std::vector<matplot::axes_handle> {
    std::vector<matplot::axes_handle> axes;
    axes.reserve(2);
    for (std::size_t i = 0; i < 2; ++i) {
        axes.emplace_back(fig->add_subplot(1, 2, i));
        axes[i]->grid(matplot::on);
        axes[i]->hold(matplot::on);
        axes[i]->font_size(12.0);
        axes[i]->x_axis().label_font_size(12.0);
        axes[i]->y_axis().label_font_size(12.0);
    }
    axes[0]->axis(matplot::equal);
    axes[0]->title("func");
    axes[1]->title("dfunc");
    return axes;
}

} // namespace

namespace basjoo::math {

TEST_CASE("QuinticPolynomial") {
    constexpr auto exact_func = [](double t) noexcept -> double {
        return 0.45 + 5.3 * t - 1.3 * t * t + 0.65 * t * t * t + 0.075 * t * t * t * t -
               0.0014 * t * t * t * t * t;
    };
    constexpr auto exact_dfunc = [](double t) noexcept -> double {
        return 5.3 - 2.6 * t + 1.95 * t * t + 0.3 * t * t * t - 0.007 * t * t * t * t;
    };
    constexpr auto exact_integral = [](double lo, double up) noexcept -> double {
        return 0.45 * (up - lo) + 2.65 * (up * up - lo * lo) -
               1.3 / 3.0 * (up * up * up - lo * lo * lo) +
               0.1625 * (up * up * up * up - lo * lo * lo * lo) +
               0.015 * (up * up * up * up * up - lo * lo * lo * lo * lo) -
               0.0014 / 6.0 * (up * up * up * up * up * up - lo * lo * lo * lo * lo * lo);
    };

    constexpr double kStart{-2.0};
    constexpr double kEnd{2.0};
    constexpr std::size_t kSize{41};

    const std::vector<double> ts = linspace(kStart, kEnd, kSize);
    std::vector<double> ys;
    ys.reserve(kSize);
    for (double t : ts) {
        ys.emplace_back(exact_func(t));
    }

    const PiecewiseLinearFunction<double> func{ts, ys};

    CHECK_EQ(func.minT(), doctest::Approx(kStart).epsilon(kEpsilon));
    CHECK_EQ(func.maxT(), doctest::Approx(kEnd).epsilon(kEpsilon));
    CHECK_EQ(func.minY(), doctest::Approx(exact_func(kStart)).epsilon(kEpsilon));
    CHECK_EQ(func.maxY(), doctest::Approx(exact_func(kEnd)).epsilon(kEpsilon));

    for (double t : ts) {
        CHECK_EQ(func(t), exact_func(t));
    }

    for (double t : linspace(kStart, kEnd, 31)) {
        CHECK_EQ(func(t), doctest::Approx(exact_func(t)).epsilon(1E-2));
        CHECK_EQ(func.derivative(t), doctest::Approx(exact_dfunc(t)).epsilon(1E-1));
    }

    CHECK_EQ(func.integral(-2.0, 2.0), doctest::Approx(exact_integral(-2.0, 2.0)).epsilon(1E-2));
    CHECK_EQ(
        func.integral(-1.46, -0.25), doctest::Approx(exact_integral(-1.46, -0.25)).epsilon(1E-3)
    );
    CHECK_EQ(
        func.integral(0.153, 1.97), doctest::Approx(exact_integral(0.153, 1.97)).epsilon(1E-3)
    );

    if (plot_graph) {
        using namespace matplot;

        const std::vector<double> plot_ts = ::basjoo::math::linspace(-4.0, 4.0, 801);
        const std::vector<double> exact_ts{func.ts().begin(), func.ts().end()};

        figure_handle fig = createFigureHandle();
        std::vector<axes_handle> axes = createAxesHandles(fig);
        axes[0]->plot(exact_ts, transform(exact_ts, exact_func), ".");
        axes[0]->plot(plot_ts, transform(plot_ts, func), "-");
        axes[1]->plot(exact_ts, transform(exact_ts, exact_dfunc), ".");
        auto dfunc = [&func](double t) -> double { return func.derivative(t); };
        axes[1]->plot(plot_ts, transform(plot_ts, dfunc), "-");

        fig->show();
    }
}

TEST_CASE("SineTest") {
    const auto exact_func = [](double t) noexcept -> double {
        return 16.43 * std::sin(5.7 * t) - 3.2;
    };
    const auto exact_dfunc = [](double t) noexcept -> double {
        return 16.43 * 5.7 * std::cos(5.7 * t);
    };
    const auto exact_integral = [](double lo, double up) noexcept -> double {
        return 16.43 / 5.7 * (std::cos(5.7 * lo) - std::cos(5.7 * up)) + 3.2 * (lo - up);
    };

    constexpr double kStart{-0.12};
    constexpr double kEnd{M_PI * 2.0 / 5.7 - 0.12};
    constexpr std::size_t kSize{41};

    const std::vector<double> ts = linspace(kStart, kEnd, kSize);
    std::vector<double> ys;
    ys.reserve(kSize);
    for (double t : ts) {
        ys.emplace_back(exact_func(t));
    }

    const PiecewiseLinearFunction<double> func{ts, ys};

    CHECK_EQ(func.minT(), doctest::Approx(kStart).epsilon(kEpsilon));
    CHECK_EQ(func.maxT(), doctest::Approx(kEnd).epsilon(kEpsilon));
    CHECK_EQ(func.minY(), doctest::Approx(-16.43 - 3.2).epsilon(1E-2));
    CHECK_EQ(func.maxY(), doctest::Approx(16.43 - 3.2).epsilon(1E-2));

    CHECK_EQ(func(kStart + kEpsilon), doctest::Approx(func(kEnd - kEpsilon)).epsilon(1E-6));
    CHECK_EQ(
        func.derivative(kStart + kEpsilon),
        doctest::Approx(func.derivative(kEnd - kEpsilon)).epsilon(1)
    );

    for (double t : ts) {
        CHECK_EQ(func(t), exact_func(t));
        CHECK_EQ(func.derivative(t), doctest::Approx(exact_dfunc(t)).epsilon(1E1));
    }

    for (double t : linspace(kStart, kEnd, 31)) {
        CHECK_EQ(func(t), doctest::Approx(exact_func(t)).epsilon(1E-2));
        CHECK_EQ(func.derivative(t), doctest::Approx(exact_dfunc(t)).epsilon(1E1));
    }

    CHECK_EQ(
        func.integral(-0.09, 0.23), doctest::Approx(exact_integral(-0.09, 0.23)).epsilon(1E-2)
    );
    CHECK_EQ(func.integral(0.18, 0.45), doctest::Approx(exact_integral(0.18, 0.45)).epsilon(1E-2));
    CHECK_EQ(func.integral(-0.11, 0.9), doctest::Approx(exact_integral(-0.11, 0.9)).epsilon(1E-2));

    if (plot_graph) {
        using namespace matplot;

        const std::vector<double> plot_ts = ::basjoo::math::linspace(kStart - 0.1, kEnd + 0.1, 801);
        const std::vector<double> exact_ts{func.ts().begin(), func.ts().end()};

        figure_handle fig = createFigureHandle();
        std::vector<axes_handle> axes = createAxesHandles(fig);
        axes[0]->plot(exact_ts, transform(exact_ts, exact_func), ".");
        axes[0]->plot(plot_ts, transform(plot_ts, func), "-");
        axes[1]->plot(exact_ts, transform(exact_ts, exact_dfunc), ".");
        auto dfunc = [&func](double t) -> double { return func.derivative(t); };
        axes[1]->plot(plot_ts, transform(plot_ts, dfunc), "-");

        fig->show();
    }
}

TEST_CASE("PeriodicSineTest") {
    const auto exact_func = [](double t) noexcept -> double {
        return 16.43 * std::sin(5.7 * t) - 3.2;
    };
    const auto exact_integral = [](double lo, double up) noexcept -> double {
        return 16.43 / 5.7 * (std::cos(5.7 * lo) - std::cos(5.7 * up)) + 3.2 * (lo - up);
    };

    constexpr double kStart{-0.12};
    constexpr double kPeriod{M_PI * 2.0 / 5.7};
    constexpr double kEnd{kStart + kPeriod};
    constexpr std::size_t kSize{41};

    const std::vector<double> ts = linspace(kStart, kEnd, kSize);
    std::vector<double> ys;
    ys.reserve(kSize);
    for (double t : ts) {
        ys.emplace_back(exact_func(t));
    }
    ys.back() = ys.front();

    const PiecewiseLinearFunction<double> func{periodic_tag{}, ts, ys};
    const PiecewiseLinearFunction<double> nonperiodic_func{ts, ys};

    CHECK(func.periodic());
    CHECK_FALSE(nonperiodic_func.periodic());

    for (double t : linspace(kStart, kEnd, 21)) {
        CHECK_EQ(func(t), doctest::Approx(exact_func(t)).epsilon(5E-2));
    }

    for (double dt : {-3.0 * kPeriod, -kPeriod, kPeriod, 2.0 * kPeriod, 5.0 * kPeriod}) {
        for (double t : linspace(kStart + 1E-3, kEnd - 1E-3, 9)) {
            CHECK_EQ(func(t + dt), doctest::Approx(func(t)).epsilon(1E-9));
            CHECK_EQ(func.derivative(t + dt), doctest::Approx(func.derivative(t)).epsilon(1E-6));
        }
    }

    const double one_period{func.integral(kStart, kEnd)};
    CHECK_EQ(one_period, doctest::Approx(exact_integral(kStart, kEnd)).epsilon(1E-2));
    CHECK_EQ(
        func.integral(kStart, kEnd + 2.0 * kPeriod + 0.37),
        doctest::Approx(3.0 * one_period + func.integral(kStart, kStart + 0.37)).epsilon(1E-6)
    );
    CHECK_EQ(
        func.integral(kStart - 1.2 * kPeriod, kEnd + 0.4),
        doctest::Approx(exact_integral(kStart - 1.2 * kPeriod, kEnd + 0.4)).epsilon(1E-2)
    );
}

TEST_CASE("Serialization") {
    constexpr auto exact_func = [](double t) noexcept -> double {
        return 0.45 + 5.3 * t - 1.3 * t * t + 0.65 * t * t * t;
    };

    const std::vector<double> ts = linspace(-2.0, 2.0, 41);
    std::vector<double> ys;
    for (double t : ts) {
        ys.emplace_back(exact_func(t));
    }

    const PiecewiseLinearFunction<double> func{ts, ys};

    auto [data, out] = zpp::bits::data_out();
    out(func).or_throw();

    PiecewiseLinearFunction<double> other_func;

    auto in = zpp::bits::in(data);
    in(other_func).or_throw();

    for (double t : ts) {
        CHECK_EQ(other_func(t), exact_func(t));
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
