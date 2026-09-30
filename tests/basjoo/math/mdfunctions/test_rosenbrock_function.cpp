/**
 * @file test_rosenbrock_function.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-11-03
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"

#include "cxxopts.hpp"
#include "matplot/matplot.h"
#include "zpp_bits.h"

#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vectorx.hpp"

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

TEST_CASE_TEMPLATE("Basic", T, Vec2<float>, Vec2<double>, VectorX<float>, VectorX<double>) {
    using value_type = typename T::value_type;

    auto exact_rosenbrock_function = [](const T& x) noexcept -> value_type {
        return (1.0 - x[0]) * (1.0 - x[0]) + 100.0 * (x[1] - x[0] * x[0]) * (x[1] - x[0] * x[0]);
    };
    auto exact_rosenbrock_gradient = [](const T& x) noexcept -> T {
        T gradient(2);
        gradient[0] = 2.0 * (x[0] - 1.0) + 400.0 * x[0] * (x[0] * x[0] - x[1]);
        gradient[1] = 200.0 * (x[1] - x[0] * x[0]);
        return gradient;
    };

    RosenbrockFunction<T> rosenbrock_function{1.0, 100.0};

    CHECK_EQ(rosenbrock_function.num_dimensions(), 2);

    T x(2);
    x[0] = -6.234;
    x[1] = 1.563;

    CHECK_EQ(rosenbrock_function(x), doctest::Approx(exact_rosenbrock_function(x)).epsilon(1E-6));

    const T gradient{rosenbrock_function.gradient(x)};
    CHECK_EQ(gradient[0], doctest::Approx(exact_rosenbrock_gradient(x)[0]).epsilon(1E-6));
    CHECK_EQ(gradient[1], doctest::Approx(exact_rosenbrock_gradient(x)[1]).epsilon(1E-6));

    CHECK_EQ(
        rosenbrock_function.gradient(x, 0),
        doctest::Approx(exact_rosenbrock_gradient(x)[0]).epsilon(1E-6)
    );
    CHECK_EQ(
        rosenbrock_function.gradient(x, 1),
        doctest::Approx(exact_rosenbrock_gradient(x)[1]).epsilon(1E-6)
    );

    if (plot_graph) {
        using namespace matplot;
        figure_handle fig = createFigureHandle();
        axes_handle ax = fig->add_subplot(1, 1, 0);
        ax->grid(matplot::on);
        ax->hold(matplot::on);
        ax->fcontour(
              [&rosenbrock_function](value_type x0, value_type x1) noexcept -> value_type {
                  return rosenbrock_function.eval({x0, x1});
              },
              {-2.0, 2.0, -1.0, 3.0}, iota(0.01, 1.0, 10.01)
        )
            ->line_width(1.0);
        fig->show();
    }
}

TEST_CASE_TEMPLATE("Serialization", T, Vec2<float>, Vec2<double>, VectorX<float>, VectorX<double>) {
    RosenbrockFunction<T> rosenbrock_function{1.0, 100.0};

    auto [data, out] = zpp::bits::data_out();
    out(rosenbrock_function).or_throw();

    RosenbrockFunction<T> other_rosenbrock_function;
    auto in = zpp::bits::in(data);
    in(other_rosenbrock_function).or_throw();

    CHECK_EQ(rosenbrock_function.a, other_rosenbrock_function.a);
    CHECK_EQ(rosenbrock_function.b, other_rosenbrock_function.b);
}

} // namespace basjoo::math

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options("rosenbrock_function_test", "unit test of rosenbrock function");
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
