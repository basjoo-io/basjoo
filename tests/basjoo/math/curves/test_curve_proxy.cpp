/**
 * @file test_curve_proxy.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-06-27
 *
 * @copyright Copyright (c) 2024 basjoo development team
 *            All rights reserved.
 *
 */

#include "basjoo/math/curves/curve_proxy.hpp"

#include <tuple>
#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/curves/piecewise_cubic_curve.hpp"
#include "basjoo/math/curves/piecewise_linear_curve.hpp"
#include "basjoo/math/curves/piecewise_quintic_curve.hpp"
#include "basjoo/math/dense/vec2.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("Curve2PolymorphismTest") {
    const auto exact_semi_circle = [](double theta) noexcept -> Vec2d {
        return Vec2d{2.0 * std::cos(theta), 2.0 * std::sin(theta)};
    };

    constexpr std::size_t kNumAnchors{101};
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

    PiecewiseLinearCurve<Vec2d> linear_curve(anchor_points);
    PiecewiseCubicCurve<Vec2d> cubic_curve(anchor_points);
    PiecewiseQuinticCurve<Vec2d> quintic_curve(anchor_points);

    std::vector<CurveProxy<Vec2d>> curves;
    curves.emplace_back(makeCurveProxy(linear_curve));
    curves.emplace_back(makeCurveProxy(std::move(cubic_curve)));
    curves.emplace_back(makeCurveProxy(std::move(quintic_curve)));

    for (const auto& curve : curves) {
        const auto& arc_lengths{curve->arcLengths()};

        CHECK_EQ(curve->minS(), 0.0);
        CHECK_EQ(curve->maxS(), doctest::Approx(M_PI * 2.0).epsilon(1E-3));

        for (std::size_t i = 0; i < kNumAnchors; ++i) {
            const double theta{kStart + kStep * i};
            const Vec2d p1 = curve->eval(arc_lengths[i]);
            const Vec2d p2 = exact_semi_circle(theta);
            CHECK_EQ(p1.x, doctest::Approx(p2.x).epsilon(kEpsilon));
            CHECK_EQ(p1.y, doctest::Approx(p2.y).epsilon(kEpsilon));
        }

        for (std::size_t i = 1; i < kNumAnchors; ++i) {
            const double theta{kStart + kStep * (i - 0.5)};
            const Vec2d point = exact_semi_circle(theta);
            const auto [s, l] = curve->inverse(point);
            CHECK_EQ(s, doctest::Approx(2.0 * theta).epsilon(1E-3));
            CHECK_EQ(l, doctest::Approx(0.0).epsilon(1E-3));
        }
    }
}

TEST_CASE("Curve3PolymorphismTest") {
    const auto exact_helix = [](double theta) noexcept -> Vec3d {
        return Vec3d{2.0 * std::cos(theta), 2.0 * std::sin(theta), theta};
    };

    constexpr std::size_t kNumAnchors{401};
    constexpr double kStart{0.0};
    constexpr double kEnd{M_PI * 4.0};
    constexpr double kStep{(kEnd - kStart) / (kNumAnchors - 1)};

    std::vector<Vec3d> anchor_points(kNumAnchors);
    std::ranges::generate(
        anchor_points, [t = kStart - kStep, h = kStep]() mutable noexcept -> Vec3d {
            t += h;
            return Vec3d{2.0 * std::cos(t), 2.0 * std::sin(t), t};
        }
    );

    PiecewiseLinearCurve<Vec3d> linear_curve(anchor_points);
    PiecewiseCubicCurve<Vec3d> cubic_curve{anchor_points};
    PiecewiseQuinticCurve<Vec3d> quintic_curve{anchor_points};

    std::vector<CurveProxy<Vec3d>> curves;
    curves.emplace_back(makeCurveProxy(linear_curve));
    curves.emplace_back(makeCurveProxy(std::move(cubic_curve)));
    curves.emplace_back(makeCurveProxy(std::move(quintic_curve)));

    for (const auto& curve : curves) {
        const auto& arc_lengths{curve->arcLengths()};

        CHECK_EQ(curve->minS(), 0.0);
        CHECK_EQ(curve->maxS(), doctest::Approx(M_PI * 4.0 * std::sqrt(5.0)).epsilon(1E-3));

        for (std::size_t i = 0; i < kNumAnchors; ++i) {
            const double theta{kStart + kStep * i};
            const Vec3d p1 = curve->eval(arc_lengths[i]);
            const Vec3d p2 = exact_helix(theta);
            CHECK_EQ(p1.x, doctest::Approx(p2.x).epsilon(kEpsilon));
            CHECK_EQ(p1.y, doctest::Approx(p2.y).epsilon(kEpsilon));
        }

        for (std::size_t i = 1; i < kNumAnchors; ++i) {
            const double theta{kStart + kStep * (i - 0.5)};
            const Vec3d point = exact_helix(theta);
            const auto [s, l, v] = curve->inverse(point);
            CHECK_EQ(s, doctest::Approx(std::sqrt(5.0) * theta).epsilon(1E-3));
            CHECK_EQ(l, doctest::Approx(0.0).epsilon(1E-3));
            CHECK_EQ(v, doctest::Approx(0.0).epsilon(1E-3));
        }
    }
}

TEST_CASE("Curve2SerializationTest") {
    constexpr std::size_t kNumAnchors{11};
    std::vector<Vec2d> anchor_points(kNumAnchors);
    std::ranges::generate(anchor_points, [t = 0.0]() mutable noexcept -> Vec2d {
        t += 1.0;
        return Vec2d{t, t * t};
    });

    std::vector<CurveProxy<Vec2d>> curves;
    curves.emplace_back(makeCurveProxy(PiecewiseLinearCurve<Vec2d>(anchor_points)));
    curves.emplace_back(makeCurveProxy(PiecewiseCubicCurve<Vec2d>(anchor_points)));
    curves.emplace_back(makeCurveProxy(PiecewiseQuinticCurve<Vec2d>(anchor_points)));

    for (const auto& curve : curves) {
        auto [data, out] = zpp::bits::data_out();
        out(curve).or_throw();

        CurveProxy<Vec2d> other_curve{curve};
        auto in = zpp::bits::in(data);
        in(other_curve).or_throw();

        CHECK(proxy_typeid(*other_curve) == proxy_typeid(*curve));

        const auto& arc_lengths{curve->arcLengths()};
        const auto& other_arc_lengths{other_curve->arcLengths()};
        REQUIRE_EQ(other_arc_lengths.size(), arc_lengths.size());
        for (std::size_t i{0}; i < arc_lengths.size(); ++i) {
            CHECK_EQ(other_arc_lengths[i], arc_lengths[i]);
        }

        const auto& anchors{curve->anchorPoints()};
        const auto& other_anchors{other_curve->anchorPoints()};
        REQUIRE_EQ(other_anchors.size(), anchors.size());
        for (std::size_t i{0}; i < anchors.size(); ++i) {
            CHECK_EQ(other_anchors[i].x, anchors[i].x);
            CHECK_EQ(other_anchors[i].y, anchors[i].y);
        }
    }
}

TEST_CASE("Curve3SerializationTest") {
    constexpr std::size_t kNumAnchors{11};
    std::vector<Vec3d> anchor_points(kNumAnchors);
    std::ranges::generate(anchor_points, [t = 0.0]() mutable noexcept -> Vec3d {
        t += 1.0;
        return Vec3d{t, t * t, t * t * t};
    });

    std::vector<CurveProxy<Vec3d>> curves;
    curves.emplace_back(makeCurveProxy(PiecewiseLinearCurve<Vec3d>(anchor_points)));
    curves.emplace_back(makeCurveProxy(PiecewiseCubicCurve<Vec3d>(anchor_points)));
    curves.emplace_back(makeCurveProxy(PiecewiseQuinticCurve<Vec3d>(anchor_points)));

    for (const auto& curve : curves) {
        auto [data, out] = zpp::bits::data_out();
        out(curve).or_throw();

        CurveProxy<Vec3d> other_curve{curve};
        auto in = zpp::bits::in(data);
        in(other_curve).or_throw();

        CHECK(proxy_typeid(*other_curve) == proxy_typeid(*curve));

        const auto& arc_lengths{curve->arcLengths()};
        const auto& other_arc_lengths{other_curve->arcLengths()};
        REQUIRE_EQ(other_arc_lengths.size(), arc_lengths.size());
        for (std::size_t i{0}; i < arc_lengths.size(); ++i) {
            CHECK_EQ(other_arc_lengths[i], arc_lengths[i]);
        }

        const auto& anchors{curve->anchorPoints()};
        const auto& other_anchors{other_curve->anchorPoints()};
        REQUIRE_EQ(other_anchors.size(), anchors.size());
        for (std::size_t i{0}; i < anchors.size(); ++i) {
            CHECK_EQ(other_anchors[i].x, anchors[i].x);
            CHECK_EQ(other_anchors[i].y, anchors[i].y);
            CHECK_EQ(other_anchors[i].z, anchors[i].z);
        }
    }
}

} // namespace basjoo::math
