/**
 * @file mdfunction_proxy_test.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2024-12-14
 *
 * @copyright Copyright (c) 2024 basjoo development team.
 *            All rights reserved.
 *
 */

#include "basjoo/math/mdfunctions/mdfunction_proxy.hpp"

#include <vector>

#include "zpp_bits.h"

#include "basjoo/math/dense/detail/dense_generate_trait.hpp"
#include "basjoo/math/dense/matrix.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/dense/vector.hpp"
#include "basjoo/math/mdfunctions/linear_mdfunction.hpp"
#include "basjoo/math/mdfunctions/quadratic_mdfunction.hpp"
#include "basjoo/math/mdfunctions/rosenbrock_function.hpp"

#define DOCTEST_CONFIG_IMPLEMENT_WITH_MAIN
#include "doctest/doctest.h"

namespace basjoo::math {

TEST_CASE_TEMPLATE(
    "Polymorphism", T, Vector<float, 5>, Vector<double, 5>, Vector<float, std::dynamic_extent>, Vector<double, std::dynamic_extent>
) {
    T linear_coeffs(5);
    linear_coeffs[0] = 674.0;
    linear_coeffs[1] = -194.45;
    linear_coeffs[2] = 22.55;
    linear_coeffs[3] = 1.056;
    linear_coeffs[4] = -0.47656;

    ::basjoo::math::detail::DenseGenerateTraitT<T> quadratic_coeffs(5, 5);
    quadratic_coeffs[0, 0] = 498.1031;
    quadratic_coeffs[0, 1] = -463.8805;
    quadratic_coeffs[0, 2] = 576.4003;
    quadratic_coeffs[0, 3] = -433.1227;
    quadratic_coeffs[0, 4] = -241.3050;
    quadratic_coeffs[1, 0] = -463.8805;
    quadratic_coeffs[1, 1] = -390.2700;
    quadratic_coeffs[1, 2] = -192.6189;
    quadratic_coeffs[1, 3] = 356.3896;
    quadratic_coeffs[1, 4] = -951.7053;
    quadratic_coeffs[2, 0] = 576.4003;
    quadratic_coeffs[2, 1] = -192.6189;
    quadratic_coeffs[2, 2] = -95.0645;
    quadratic_coeffs[2, 3] = -305.2551;
    quadratic_coeffs[2, 4] = -186.8198;
    quadratic_coeffs[3, 0] = -433.1227;
    quadratic_coeffs[3, 1] = 356.3896;
    quadratic_coeffs[3, 2] = -305.2551;
    quadratic_coeffs[3, 3] = -948.3788;
    quadratic_coeffs[3, 4] = 69.7871;
    quadratic_coeffs[4, 0] = -241.3050;
    quadratic_coeffs[4, 1] = -951.7053;
    quadratic_coeffs[4, 2] = -186.8198;
    quadratic_coeffs[4, 3] = 69.7871;
    quadratic_coeffs[4, 4] = -919.1390;

    LinearMdFunction<T> linear_mdfunction{1.264, linear_coeffs};
    QuadraticMdFunction<T> quadratic_mdfunction(1.264, linear_coeffs, quadratic_coeffs);

    std::vector<MdFunctionProxy<T>> mdfunctions{};
    mdfunctions.emplace_back(makeMdFunctionProxy(linear_mdfunction));
    mdfunctions.emplace_back(makeMdFunctionProxy(std::move(quadratic_mdfunction)));

    T x(5);
    x[0] = 365.0;
    x[1] = -586.44;
    x[2] = 0.0;
    x[3] = 35.46;
    x[4] = -75.468;

    CHECK_EQ(mdfunctions[0]->num_dimensions(), 5);
    CHECK_EQ(mdfunctions[1]->num_dimensions(), 5);

    CHECK_EQ(mdfunctions[0]->eval(x), doctest::Approx(x.dot(linear_coeffs) + 1.264).epsilon(1E-6));
    CHECK_EQ(
        mdfunctions[1]->eval(x),
        doctest::Approx(x.dot(quadratic_coeffs).dot(x) * 0.5 + x.dot(linear_coeffs) + 1.264)
            .epsilon(1E-6)
    );

    CHECK(mdfunctions[0]->gradient(x).identicalTo(linear_coeffs, 1E-6));
    CHECK(mdfunctions[1]->gradient(x).identicalTo(linear_coeffs + x.dot(quadratic_coeffs), 1E-6));

    CHECK_FALSE(mdfunctions[0]->has_extrema(x));
    CHECK_FALSE(mdfunctions[1]->has_extrema(x));
}

TEST_CASE("SerializationTest") {
    using T = Vector<double, 2>;

    T linear_coeffs(2);
    linear_coeffs[0] = 674.0;
    linear_coeffs[1] = -194.45;

    ::basjoo::math::detail::DenseGenerateTraitT<T> quadratic_coeffs(2, 2);
    quadratic_coeffs[0, 0] = 498.1031;
    quadratic_coeffs[0, 1] = -463.8805;
    quadratic_coeffs[1, 0] = -463.8805;
    quadratic_coeffs[1, 1] = -390.2700;

    std::vector<MdFunctionProxy<T>> mdfunctions{};
    mdfunctions.emplace_back(makeMdFunctionProxy(LinearMdFunction<T>{1.264, linear_coeffs}));
    mdfunctions.emplace_back(
        makeMdFunctionProxy(QuadraticMdFunction<T>(1.264, linear_coeffs, quadratic_coeffs))
    );
    mdfunctions.emplace_back(makeMdFunctionProxy(RosenbrockFunction<T>{}));

    T x(2);
    x[0] = 365.0;
    x[1] = -586.44;

    for (const auto& mdfunction : mdfunctions) {
        auto [data, out] = zpp::bits::data_out();
        out(mdfunction).or_throw();

        MdFunctionProxy<T> other_mdfunction{mdfunction};
        auto in = zpp::bits::in(data);
        in(other_mdfunction).or_throw();

        CHECK(proxy_typeid(*other_mdfunction) == proxy_typeid(*mdfunction));

        CHECK_EQ(other_mdfunction->num_dimensions(), mdfunction->num_dimensions());
        CHECK_EQ(other_mdfunction->eval(x), mdfunction->eval(x));
        CHECK(other_mdfunction->gradient(x).identicalTo(mdfunction->gradient(x), 1E-12));
    }
}

} // namespace basjoo::math
