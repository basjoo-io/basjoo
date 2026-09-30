/**
 * @file test_route_line_cubic_offset_planner.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-26
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/bicycle/planners/route_line_cubic_offset_planner.hpp"

#include "cxxopts.hpp"
#include "matplot/matplot.h"

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
        axes.emplace_back(fig->add_subplot(2, 1, i));
        axes[i]->grid(matplot::on);
        axes[i]->hold(matplot::on);
        axes[i]->font_size(12.0);
        axes[i]->x_axis().label_font_size(12.0);
        axes[i]->y_axis().label_font_size(12.0);
    }
    axes[0]->axis(matplot::equal);
    axes[0]->xlabel("x");
    axes[0]->ylabel("y");
    axes[1]->xlabel("s");
    axes[1]->ylabel("curvature");
    return axes;
}

} // namespace

namespace basjoo::bicycle {

TEST_CASE("LaneFollow") {
    constexpr std::size_t num_samples = 21;
    std::vector<double> sample_ss = ::basjoo::math::linspace(0.0, 20.0, num_samples);
    Path2d path;

    std::vector<::basjoo::math::Vec2d> sketch_points;
    std::vector<double> sketch_points_x, sketch_points_y;

    SUBCASE("BentLane") {
        sketch_points = {{0.0, 0.0}, {8.0, 0.0}, {12.0, 2.0}, {20.0, 2.0}};
        sketch_points_x = {0.0, 8.0, 12.0, 20.0};
        sketch_points_y = {0.0, 0.0, 2.0, 2.0};

        RouteLineCubicOffsetPlanner<::basjoo::math::Vec2d> route_line_offset_planner{
            sketch_points, sample_ss
        };
        route_line_offset_planner.setOffsetCost(1.0);
        route_line_offset_planner.setCurvatureCost(100.0);
        route_line_offset_planner.setDCurvatureCost(10000.0);
        route_line_offset_planner.setDdxRange(-1.0, 1.0);
        route_line_offset_planner.setDdyRange(-1.0, 1.0);
        route_line_offset_planner.setInitialState({0.0, 0.0}, {1.0, 0.0});
        route_line_offset_planner.setFinalState({20.0, 2.0}, {1.0, 0.0});
        path = route_line_offset_planner.solve().first;

        CHECK_EQ(path.anchorPoints().front().x, doctest::Approx(0.0).epsilon(1e-6));
        CHECK_EQ(path.anchorPoints().front().y, doctest::Approx(0.0).epsilon(1e-6));
        CHECK_EQ(path.anchorPoints().back().x, doctest::Approx(20.0).epsilon(1e-6));
        CHECK_EQ(path.anchorPoints().back().y, doctest::Approx(2.0).epsilon(1e-6));
    }

    SUBCASE("CurvedLane") {
        const auto semi_circle = [](double theta) -> ::basjoo::math::Vec2d {
            return ::basjoo::math::Vec2d{std::cos(theta) * 7.0 + 7.0, std::sin(theta) * 7.0};
        };
        for (const double theta : ::basjoo::math::linspace(M_PI, 0.0, 15)) {
            const ::basjoo::math::Vec2d sketch_point{semi_circle(theta)};
            sketch_points.push_back(sketch_point);
            sketch_points_x.push_back(sketch_point.x);
            sketch_points_y.push_back(sketch_point.y);
        };

        RouteLineCubicOffsetPlanner<::basjoo::math::Vec2d> route_line_offset_planner{
            sketch_points, sample_ss
        };
        route_line_offset_planner.setOffsetCost(1.0);
        route_line_offset_planner.setCurvatureCost(100.0);
        route_line_offset_planner.setDCurvatureCost(10000.0);
        route_line_offset_planner.setDdxRange(-1.0, 1.0);
        route_line_offset_planner.setDdyRange(-1.0, 1.0);
        route_line_offset_planner.setInitialState({0.0, 0.0}, {0.0, 1.0});
        route_line_offset_planner.setFinalState({14.0, 0.0}, {0.0, -1.0});
        path = route_line_offset_planner.solve().first;

        CHECK_EQ(path.anchorPoints().front().x, doctest::Approx(0.0).epsilon(1e-6));
        CHECK_EQ(path.anchorPoints().front().y, doctest::Approx(0.0).epsilon(1e-6));
        CHECK_EQ(path.anchorPoints().back().x, doctest::Approx(14.0).epsilon(1e-6));
        CHECK_EQ(path.anchorPoints().back().y, doctest::Approx(0.0).epsilon(1e-6));
    }

    if (plot_graph) {
        using namespace matplot;
        const std::vector<double> plot_ss =
            ::basjoo::math::linspace(path.minS(), path.maxS(), num_samples);
        std::vector<double> plot_xs;
        plot_xs.reserve(num_samples);
        std::vector<double> plot_ys;
        plot_ys.reserve(num_samples);
        std::vector<double> plot_ks;
        plot_ks.reserve(num_samples);
        for (const ::basjoo::math::Vec2d& point : path.anchorPoints()) {
            plot_xs.push_back(point.x);
            plot_ys.push_back(point.y);
        }
        for (const double s : plot_ss) {
            plot_ks.push_back(path.curvature(s));
        }

        figure_handle fig = createFigureHandle();
        std::vector<axes_handle> axes = createAxesHandles(fig);
        axes[0]->plot(plot_xs, plot_ys, ".");
        axes[0]->plot(sketch_points_x, sketch_points_y, "c")->line_width(2.0);
        axes[1]->plot(plot_ss, plot_ks, "-")->line_width(2.0);
        fig->show();
    }
}

TEST_CASE("TrivialScene") {
    constexpr std::size_t num_samples = 21;
    std::vector<double> sample_ss = ::basjoo::math::linspace(0.0, 20.0, num_samples);
    const std::vector<::basjoo::math::Vec2d> sketch_points{{0.0, 0.0}, {20.0, 0.0}};
    RouteLineCubicOffsetPlanner<::basjoo::math::Vec2d> route_line_offset_planner{
        sketch_points, sample_ss
    };
    route_line_offset_planner.setInitialState({0.0, 2.0}, {1.0, 0.0});
    route_line_offset_planner.setFinalState({20.0, 0.0}, {1.0, 0.0});
    route_line_offset_planner.setOffsetCost(1.0);
    route_line_offset_planner.setCurvatureCost(100.0);
    route_line_offset_planner.setDCurvatureCost(10000.0);
    route_line_offset_planner.setDdxRange(-1.0, 1.0);
    route_line_offset_planner.setDdyRange(-1.0, 1.0);
    const Path2d path = route_line_offset_planner.solve().first;

    CHECK_EQ(path.anchorPoints().back().x, doctest::Approx(20.0).epsilon(1e-6));
    CHECK_EQ(path.anchorPoints().back().y, doctest::Approx(0.0).epsilon(1e-6));

    if (plot_graph) {
        using namespace matplot;
        const std::vector<double> plot_ss =
            ::basjoo::math::linspace(path.minS(), path.maxS(), num_samples);
        std::vector<double> plot_xs;
        plot_xs.reserve(num_samples);
        std::vector<double> plot_ys;
        plot_ys.reserve(num_samples);
        std::vector<double> plot_ks;
        plot_ks.reserve(num_samples);
        for (const ::basjoo::math::Vec2d& point : path.anchorPoints()) {
            plot_xs.push_back(point.x);
            plot_ys.push_back(point.y);
        }
        for (const double s : plot_ss) {
            plot_ks.push_back(path.curvature(s));
        }

        figure_handle fig = createFigureHandle();
        std::vector<axes_handle> axes = createAxesHandles(fig);
        axes[0]->plot(plot_xs, plot_ys, ".");
        axes[0]
            ->plot(
                {sketch_points.front().x, sketch_points.back().x},
                {sketch_points.front().y, sketch_points.back().y}, "c"
            )
            ->line_width(2.0);
        axes[1]->plot(plot_ss, plot_ks, "-")->line_width(2.0);
        fig->show();
    }
}

TEST_CASE("BorderScene") {
    constexpr std::size_t num_samples = 21;
    std::vector<double> sample_ss = ::basjoo::math::linspace(0.0, 20.0, num_samples);
    const std::vector<::basjoo::math::Vec2d> sketch_points{{0.0, 0.0}, {20.0, 0.0}};
    RouteLineCubicOffsetPlanner<::basjoo::math::Vec2d> route_line_offset_planner{
        sketch_points, sample_ss
    };
    route_line_offset_planner.setInitialState({0.0, 0.0}, {1.0, 0.0});
    route_line_offset_planner.setFinalState({20.0, 0.0}, {1.0, 0.0});
    route_line_offset_planner.setOffsetCost(1.0);
    route_line_offset_planner.setCurvatureCost(100.0);
    route_line_offset_planner.setDCurvatureCost(10000.0);
    route_line_offset_planner.setDdxRange(-1.0, 1.0);
    route_line_offset_planner.setDdyRange(-1.0, 1.0);

    std::pmr::vector<::basjoo::math::Vec2d> bound_points_1{
        ::basjoo::math::Vec2d{4.2, 1.0}, ::basjoo::math::Vec2d{7.7, 1.0}
    };
    std::pmr::vector<::basjoo::math::Vec2d> bound_points_2{
        ::basjoo::math::Vec2d{11.5, -1.0}, ::basjoo::math::Vec2d{16.4, -1.0}
    };

    Path2d path;

    SUBCASE("SoftBorders") {
        std::vector<SoftBorder2d> soft_borders{
            SoftBorder2d{0, ::basjoo::bicycle::Chirality::RIGHT, bound_points_1, 100.0, 100.0},
            SoftBorder2d{1, ::basjoo::bicycle::Chirality::LEFT, bound_points_2, 100.0, 100.0}
        };

        route_line_offset_planner.setSoftBorders(soft_borders);
        path = route_line_offset_planner.solve().first;
    }

    SUBCASE("HardBorders") {
        std::vector<HardBorder2d> hard_borders{
            HardBorder2d{0, ::basjoo::bicycle::Chirality::RIGHT, bound_points_1},
            HardBorder2d{1, ::basjoo::bicycle::Chirality::LEFT, bound_points_2}
        };

        route_line_offset_planner.setHardBorders(hard_borders);
        path = route_line_offset_planner.solve().first;
    }

    if (plot_graph) {
        using namespace matplot;
        const std::vector<double> plot_ss =
            ::basjoo::math::linspace(path.minS(), path.maxS(), num_samples);
        std::vector<double> plot_xs;
        plot_xs.reserve(num_samples);
        std::vector<double> plot_ys;
        plot_ys.reserve(num_samples);
        std::vector<double> plot_ks;
        plot_ks.reserve(num_samples);
        for (const ::basjoo::math::Vec2d& point : path.anchorPoints()) {
            plot_xs.push_back(point.x);
            plot_ys.push_back(point.y);
        }
        for (const double s : plot_ss) {
            plot_ks.push_back(path.curvature(s));
        }

        figure_handle fig = createFigureHandle();
        std::vector<axes_handle> axes = createAxesHandles(fig);
        axes[0]->plot(plot_xs, plot_ys, ".")->line_width(2.0);
        axes[0]
            ->plot(
                {sketch_points.front().x, sketch_points.back().x},
                {sketch_points.front().y, sketch_points.back().y}, "c"
            )
            ->line_width(2.0);
        axes[0]->plot(
            {bound_points_1.front().x, bound_points_1.back().x},
            {bound_points_1.back().y, bound_points_1.back().y}, "r"
        );
        axes[0]->plot(
            {bound_points_1.front().x, bound_points_1.front().x},
            {bound_points_1.front().y, bound_points_1.front().y - 2.0}, "r"
        );
        axes[0]->plot(
            {bound_points_1.back().x, bound_points_1.back().x},
            {bound_points_1.back().y, bound_points_1.back().y - 2.0}, "r"
        );
        axes[0]->plot(
            {bound_points_2.front().x, bound_points_2.back().x},
            {bound_points_2.back().y, bound_points_2.back().y}, "r"
        );
        axes[0]->plot(
            {bound_points_2.front().x, bound_points_2.front().x},
            {bound_points_2.front().y, bound_points_2.front().y + 2.0}, "r"
        );
        axes[0]->plot(
            {bound_points_2.back().x, bound_points_2.back().x},
            {bound_points_2.back().y, bound_points_2.back().y + 2.0}, "r"
        );
        axes[1]->plot(plot_ss, plot_ks, "-")->line_width(2.0);
        fig->show();
    }
}

} // namespace basjoo::bicycle

auto main(int argc, const char* argv[]) -> int {
    cxxopts::Options options(
        "route_line_cubic_offset_planner_test", "unit test of RouteLineCubicOffsetPlanner class"
    );
    options.add_options()(
        "plot-graph", "plot test graph", cxxopts::value<bool>()->default_value("false")
    );
    cxxopts::ParseResult result = options.parse(argc, argv);
    plot_graph = result["plot-graph"].as<bool>();
    doctest::Context context(argc, argv);
    return context.run();
}
