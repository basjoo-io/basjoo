/**
 * @file test_polygon.cpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-09-02
 *
 * @copyright Copyright (c) 2026 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/math/polygon.hpp"

#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/dense/vec2.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE("CcwSquareTest") {
    const std::vector<Vec2d> vertices{
        Vec2d{0.0, 0.0}, Vec2d{2.0, 0.0}, Vec2d{2.0, 2.0}, Vec2d{0.0, 2.0}, Vec2d{0.0, 0.0}
    };
    const Polygon2d square{vertices};

    CHECK_EQ(square.signedArea(), doctest::Approx(4.0).epsilon(1E-9));
    CHECK(isConvex(square));
    CHECK_FALSE(isConcave(square));

    CHECK(square.eval(0.0).identicalTo(Vec2d{0.0, 0.0}, 1E-9));

    CHECK(square.contains(Vec2d{1.0, 1.0}));
    CHECK(square.contains(Vec2d{0.1, 0.1}));
    CHECK_FALSE(square.contains(Vec2d{3.0, 3.0}));
    CHECK_FALSE(square.contains(Vec2d{-1.0, 1.0}));

    // boundary points (inclusive)
    CHECK(square.contains(Vec2d{1.0, 0.0}));
    CHECK(square.contains(Vec2d{2.0, 1.0}));
    CHECK(square.contains(Vec2d{0.0, 0.0}));
}

TEST_CASE("CwTriangleTest") {
    // (0,0) -> (0,2) -> (2,0) -> close, traced clockwise (negative signed area).
    const std::vector<Vec2d> vertices{
        Vec2d{0.0, 0.0}, Vec2d{0.0, 2.0}, Vec2d{2.0, 0.0}, Vec2d{0.0, 0.0}
    };
    const Polygon2d triangle{vertices};

    CHECK_EQ(triangle.signedArea(), doctest::Approx(-2.0).epsilon(1E-9));
    CHECK(isConvex(triangle));
    CHECK_FALSE(isConcave(triangle));

    CHECK(triangle.contains(Vec2d{0.5, 0.5}));
    CHECK_FALSE(triangle.contains(Vec2d{1.5, 1.5}));
    CHECK_FALSE(triangle.contains(Vec2d{-0.5, 0.5}));

    // boundary: hypotenuse x + y = 2
    CHECK(triangle.contains(Vec2d{1.0, 1.0}));
}

TEST_CASE("ConcaveLShapeTest") {
    // A 4x4 square with the top-right 2x2 quadrant notched out, traced CCW.
    const std::vector<Vec2d> vertices{Vec2d{0.0, 0.0}, Vec2d{4.0, 0.0}, Vec2d{4.0, 2.0},
                                      Vec2d{2.0, 2.0}, Vec2d{2.0, 4.0}, Vec2d{0.0, 4.0},
                                      Vec2d{0.0, 0.0}};
    const Polygon2d l_shape{vertices};

    CHECK_EQ(l_shape.signedArea(), doctest::Approx(12.0).epsilon(1E-9));
    CHECK_FALSE(isConvex(l_shape));
    CHECK(isConcave(l_shape));

    // inside the two "arms" of the L
    CHECK(l_shape.contains(Vec2d{3.0, 1.0}));
    CHECK(l_shape.contains(Vec2d{1.0, 3.0}));
    CHECK(l_shape.contains(Vec2d{1.0, 1.0}));

    // inside the notch: outside the polygon even though it is within the bounding box
    CHECK_FALSE(l_shape.contains(Vec2d{3.0, 3.0}));

    CHECK_FALSE(l_shape.contains(Vec2d{5.0, 5.0}));

    // the reflex vertex itself is a boundary point
    CHECK(l_shape.contains(Vec2d{2.0, 2.0}));
}

TEST_CASE("CollinearVertexStillConvexTest") {
    // Same square as CcwSquareTest but with an extra vertex collinear on the bottom edge.
    const std::vector<Vec2d> vertices{Vec2d{0.0, 0.0}, Vec2d{1.0, 0.0}, Vec2d{2.0, 0.0},
                                      Vec2d{2.0, 2.0}, Vec2d{0.0, 2.0}, Vec2d{0.0, 0.0}};
    const Polygon2d square{vertices};

    CHECK(isConvex(square));
    CHECK_FALSE(isConcave(square));
}

TEST_CASE("Serialization") {
    const std::vector<Vec2d> vertices{Vec2d{0.0, 0.0}, Vec2d{4.0, 0.0}, Vec2d{4.0, 2.0},
                                      Vec2d{2.0, 2.0}, Vec2d{2.0, 4.0}, Vec2d{0.0, 4.0},
                                      Vec2d{0.0, 0.0}};
    const Polygon2d l_shape{vertices};

    auto [data, out] = zpp::bits::data_out();
    out(l_shape).or_throw();

    Polygon2d other_l_shape;

    auto in = zpp::bits::in(data);
    in(other_l_shape).or_throw();

    const auto vs{l_shape.vertices()};
    const auto other_vs{other_l_shape.vertices()};
    REQUIRE_EQ(other_vs.size(), vs.size());
    for (std::size_t i{0}; i < vs.size(); ++i) {
        CHECK(other_vs[i].identicalTo(vs[i], 1E-9));
    }

    CHECK_EQ(other_l_shape.signedArea(), doctest::Approx(l_shape.signedArea()).epsilon(1E-9));
    CHECK_EQ(isConvex(other_l_shape), isConvex(l_shape));
    CHECK_EQ(isConcave(other_l_shape), isConcave(l_shape));

    CHECK(other_l_shape.contains(Vec2d{3.0, 1.0}));
    CHECK(other_l_shape.contains(Vec2d{1.0, 3.0}));
    CHECK_FALSE(other_l_shape.contains(Vec2d{3.0, 3.0}));
}

} // namespace basjoo::math
