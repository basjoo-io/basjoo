/**
 * @file test_chebyshev.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-07
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/chebyshev.hpp"

#include "cxxopts.hpp"
#include "matplot/matplot.h"
#include "zpp_bits.h"

#include "basjoo/math/utils.hpp"

#define DOCTEST_CONFIG_IMPLEMENT
#include "doctest/doctest.h"

namespace {

bool plot_graph{false};

auto createFigureHandle() -> matplot::figure_handle {
    matplot::figure_handle fig = matplot::figure();
    fig->size(1600, 1000);
    return fig;
}

auto createAxesHandles(matplot::figure_handle& fig) -> std::vector<matplot::axes_handle> {
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
    axes[0]->title("primitive func");
    axes[1]->title("primitive dfunc");
    axes[2]->title("primitive integral func");
    axes[3]->title("Chebyshev func");
    axes[4]->title("Chebyshev dfunc");
    axes[5]->title("Chebyshev integral func");
    return axes;
}

} // namespace

namespace basjoo::math {

TEST_CASE("SinFunc") {
    const auto exact_sin = [](double t) -> double { return std::sin(t); };
    const double lower_bound{0.0}, upper_bound{M_PI * 2.0};
    constexpr std::size_t kSize{10};

    const Chebyshev<double, kSize> chebyshev_sin{exact_sin, lower_bound, upper_bound};
    const Chebyshev<double, kSize> chebyshev_dsin = chebyshev_sin.derivative();
    const Chebyshev<double, kSize> chebyshev_isin = chebyshev_sin.integral();

    CHECK_EQ(chebyshev_sin.size(), kSize);
    CHECK_EQ(chebyshev_sin.num_trunc(), kSize);
    CHECK_EQ(chebyshev_sin.lower_bound(), lower_bound);
    CHECK_EQ(chebyshev_sin.upper_bound(), upper_bound);

    CHECK_EQ(chebyshev_dsin.size(), kSize);
    CHECK_EQ(chebyshev_dsin.num_trunc(), kSize);
    CHECK_EQ(chebyshev_dsin.lower_bound(), lower_bound);
    CHECK_EQ(chebyshev_dsin.upper_bound(), upper_bound);

    CHECK_EQ(chebyshev_isin.size(), kSize);
    CHECK_EQ(chebyshev_isin.num_trunc(), kSize);
    CHECK_EQ(chebyshev_isin.lower_bound(), lower_bound);
    CHECK_EQ(chebyshev_isin.upper_bound(), upper_bound);

    const std::size_t num_samples{1001};
    const std::vector<double> sample_ts = linspace(lower_bound, upper_bound, num_samples);
    for (double t : sample_ts) {
        CHECK_EQ(chebyshev_sin(t), doctest::Approx(exact_sin(t)).epsilon(1E-3));
        CHECK_EQ(chebyshev_dsin(t), doctest::Approx(std::cos(t)).epsilon(1E-3));
        CHECK_EQ(chebyshev_isin(t), doctest::Approx(1.0 - std::cos(t)).epsilon(1E-3));
    }

    if (plot_graph) {
        using namespace matplot;
        const std::size_t num_plot{1001};
        const std::vector<double> plot_ts =
            linspace(chebyshev_sin.lower_bound(), chebyshev_sin.upper_bound(), num_plot);
        std::vector<double> plot_ys_0;
        plot_ys_0.reserve(num_plot);
        std::vector<double> plot_ys_1;
        plot_ys_1.reserve(num_plot);
        std::vector<double> plot_ys_2;
        plot_ys_2.reserve(num_plot);
        std::vector<double> plot_ys_3;
        plot_ys_3.reserve(num_plot);
        std::vector<double> plot_ys_4;
        plot_ys_4.reserve(num_plot);
        std::vector<double> plot_ys_5;
        plot_ys_5.reserve(num_plot);
        for (double t : plot_ts) {
            plot_ys_0.push_back(exact_sin(t));
            plot_ys_1.push_back(std::cos(t));
            plot_ys_2.push_back(1.0 - std::cos(t));
            plot_ys_3.push_back(chebyshev_sin(t));
            plot_ys_4.push_back(chebyshev_dsin(t));
            plot_ys_5.push_back(chebyshev_isin(t));
        }

        figure_handle fig = createFigureHandle();
        std::vector<axes_handle> axes = createAxesHandles(fig);
        axes[0]->plot(plot_ts, plot_ys_0, "-")->line_width(2.0);
        axes[1]->plot(plot_ts, plot_ys_1, "-")->line_width(2.0);
        axes[2]->plot(plot_ts, plot_ys_2, "-")->line_width(2.0);
        axes[3]->plot(plot_ts, plot_ys_3, "-")->line_width(2.0);
        axes[4]->plot(plot_ts, plot_ys_4, "-")->line_width(2.0);
        axes[5]->plot(plot_ts, plot_ys_5, "-")->line_width(2.0);
        show();
    }
}

TEST_CASE("Serialization") {
    const auto exact_sin = [](double t) noexcept -> double { return std::sin(t); };
    const double lower_bound{0.0}, upper_bound{M_PI * 2.0};
    constexpr std::size_t kSize{10};

    const Chebyshev<double, kSize> chebyshev_sin{exact_sin, lower_bound, upper_bound};

    auto [data, out] = zpp::bits::data_out();
    out(chebyshev_sin).or_throw();

    Chebyshev<double, kSize> other_chebyshev_sin;

    auto in = zpp::bits::in(data);
    in(other_chebyshev_sin).or_throw();

    CHECK_EQ(other_chebyshev_sin.size(), chebyshev_sin.size());
    CHECK_EQ(other_chebyshev_sin.num_trunc(), chebyshev_sin.num_trunc());
    CHECK_EQ(other_chebyshev_sin.lower_bound(), chebyshev_sin.lower_bound());
    CHECK_EQ(other_chebyshev_sin.upper_bound(), chebyshev_sin.upper_bound());

    for (std::size_t i{0}; i < other_chebyshev_sin.size(); ++i) {
        CHECK_EQ(other_chebyshev_sin.coeff(i), chebyshev_sin.coeff(i));
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
