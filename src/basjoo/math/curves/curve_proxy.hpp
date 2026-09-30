/**
 * @file curve_proxy.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-24
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <cstdint>
#include <span>
#include <system_error>
#include <tuple>
#include <utility>

#include "proxy/proxy.h"
#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/curves/piecewise_cubic_curve.hpp"
#include "basjoo/math/curves/piecewise_linear_curve.hpp"
#include "basjoo/math/curves/piecewise_quintic_curve.hpp"
#include "basjoo/math/dense/vec2.hpp"
#include "basjoo/math/dense/vec3.hpp"

namespace basjoo::math {

namespace detail {

// NOLINTBEGIN(modernize-use-trailing-return-type)

PRO_DEF_MEM_DISPATCH(c_eval, eval);
PRO_DEF_MEM_DISPATCH(c_tangent, tangent);
PRO_DEF_MEM_DISPATCH(c_normal, normal);
PRO_DEF_MEM_DISPATCH(c_binormal, binormal);
PRO_DEF_MEM_DISPATCH(c_curvature, curvature);
PRO_DEF_MEM_DISPATCH(c_torsion, torsion);
PRO_DEF_MEM_DISPATCH(c_inverse, inverse);
PRO_DEF_MEM_DISPATCH(c_minS, minS);
PRO_DEF_MEM_DISPATCH(c_maxS, maxS);
PRO_DEF_MEM_DISPATCH(c_front, front);
PRO_DEF_MEM_DISPATCH(c_back, back);
PRO_DEF_MEM_DISPATCH(c_arcLengths, arcLengths);
PRO_DEF_MEM_DISPATCH(c_anchorPoints, anchorPoints);

// NOLINTEND(modernize-use-trailing-return-type)

template <typename T>
class CurveFacade final {
    static_assert(false, "::basjoo::math::CurveFacade is not implemented for this type argument!");
};

// clang-format off

template <Vec2Arithmetic T>
    requires std::floating_point<typename T::value_type>
class CurveFacade<T> final : public pro::skills::rtti<typename pro::facade_builder
    ::support_copy<pro::constraint_level::nontrivial>
    ::template add_convention<
        pro::operator_dispatch<"()">,
        auto(typename T::value_type) const noexcept -> T,
        auto(typename T::value_type, typename T::value_type) const noexcept -> T
    >
    ::template add_convention<
        c_eval,
        auto(typename T::value_type) const noexcept -> T,
        auto(typename T::value_type, typename T::value_type) const noexcept -> T
    >
    ::template add_convention<c_tangent, auto(typename T::value_type) const noexcept -> T>
    ::template add_convention<c_normal, auto(typename T::value_type) const noexcept -> T>
    ::template add_convention<c_curvature, auto(typename T::value_type) const noexcept -> typename T::value_type>
    ::template add_convention<
        c_inverse,
        auto(T) const noexcept -> std::pair<typename T::value_type, typename T::value_type>,
        auto(T, typename T::value_type, typename T::value_type) const noexcept -> std::pair<typename T::value_type, typename T::value_type>
    >
    ::template add_convention<c_minS, auto() const noexcept -> typename T::value_type>
    ::template add_convention<c_maxS, auto() const noexcept -> typename T::value_type>
    ::template add_convention<c_front, auto() const noexcept -> T>
    ::template add_convention<c_back, auto() const noexcept -> T>
    ::template add_convention<c_arcLengths, auto() const noexcept -> std::span<const typename T::value_type>>
    ::template add_convention<c_anchorPoints, auto() const noexcept -> std::span<const T>>
    >::build {};

template <Vec3Arithmetic T>
    requires std::floating_point<typename T::value_type>
class CurveFacade<T> final : public pro::skills::rtti<typename pro::facade_builder
    ::support_copy<pro::constraint_level::nontrivial>
    ::template add_convention<
        pro::operator_dispatch<"()">,
        auto(typename T::value_type) const noexcept -> T,
        auto(typename T::value_type, typename T::value_type, typename T::value_type) const noexcept -> T
    >
    ::template add_convention<
        c_eval,
        auto(typename T::value_type) const noexcept -> T,
        auto(typename T::value_type, typename T::value_type, typename T::value_type) const noexcept -> T
    >
    ::template add_convention<c_tangent, auto(typename T::value_type) const noexcept -> T>
    ::template add_convention<c_normal, auto(typename T::value_type) const noexcept -> T>
    ::template add_convention<c_binormal, auto(typename T::value_type) const noexcept -> T>
    ::template add_convention<c_curvature, auto(typename T::value_type) const noexcept -> typename T::value_type>
    ::template add_convention<c_torsion, auto(typename T::value_type) const noexcept -> typename T::value_type>
    ::template add_convention<
        c_inverse,
        auto(T) const noexcept -> std::tuple<typename T::value_type, typename T::value_type, typename T::value_type>,
        auto(T, typename T::value_type, typename T::value_type) const noexcept -> std::tuple<typename T::value_type, typename T::value_type, typename T::value_type>
    >
    ::template add_convention<c_minS, auto() const noexcept -> typename T::value_type>
    ::template add_convention<c_maxS, auto() const noexcept -> typename T::value_type>
    ::template add_convention<c_front, auto() const noexcept -> T>
    ::template add_convention<c_back, auto() const noexcept -> T>
    ::template add_convention<c_arcLengths, auto() const noexcept -> std::span<const typename T::value_type>>
    ::template add_convention<c_anchorPoints, auto() const noexcept -> std::span<const T>>
    >::build {};

// clang-format on

enum class CurveProxyTag : std::uint8_t {
    LINEAR,
    CUBIC,
    QUINTIC
};

template <VecArithmetic T>
    requires std::floating_point<typename T::value_type>
constexpr auto serialize(auto& archive, const pro::proxy<CurveFacade<T>>& self) -> zpp::bits::errc {
    if (const auto* curve = proxy_cast<PiecewiseLinearCurve<T>>(&*self)) {
        if (auto result = archive(CurveProxyTag::LINEAR); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        return archive(*curve);
    }
    if (const auto* curve = proxy_cast<PiecewiseCubicCurve<T>>(&*self)) {
        if (auto result = archive(CurveProxyTag::CUBIC); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        return archive(*curve);
    }
    if (const auto* curve = proxy_cast<PiecewiseQuinticCurve<T>>(&*self)) {
        if (auto result = archive(CurveProxyTag::QUINTIC); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*curve);
    }
    return std::errc::invalid_argument;
}

template <VecArithmetic T>
    requires std::floating_point<typename T::value_type>
constexpr auto serialize(auto& archive, pro::proxy<CurveFacade<T>>& self) -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        if (auto* curve = proxy_cast<PiecewiseLinearCurve<T>>(&*self)) {
            if (auto result = archive(CurveProxyTag::LINEAR); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*curve);
        }
        if (auto* curve = proxy_cast<PiecewiseCubicCurve<T>>(&*self)) {
            if (auto result = archive(CurveProxyTag::CUBIC); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*curve);
        }
        if (auto* curve = proxy_cast<PiecewiseQuinticCurve<T>>(&*self)) {
            if (auto result = archive(CurveProxyTag::QUINTIC); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*curve);
        }
        return std::errc::invalid_argument;
    } else {
        CurveProxyTag tag{};
        if (auto result = archive(tag); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        switch (tag) {
        case CurveProxyTag::LINEAR: {
            PiecewiseLinearCurve<T> curve{};
            if (auto result = archive(curve); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<CurveFacade<T>>(std::move(curve));
            return {};
        }
        case CurveProxyTag::CUBIC: {
            PiecewiseCubicCurve<T> curve{};
            if (auto result = archive(curve); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<CurveFacade<T>>(std::move(curve));
            return {};
        }
        case CurveProxyTag::QUINTIC: {
            PiecewiseQuinticCurve<T> curve{};
            if (auto result = archive(curve); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<CurveFacade<T>>(std::move(curve));
            return {};
        }
        }
        return std::errc::invalid_argument;
    }
}

} // namespace detail

template <VecArithmetic T>
using CurveProxy = pro::proxy<detail::CurveFacade<T>>;

template <typename T>
[[using gnu: always_inline]]
inline auto makeCurveProxy(T&& curve) -> CurveProxy<typename std::remove_cvref_t<T>::value_type> {
    return pro::make_proxy<detail::CurveFacade<typename std::remove_cvref_t<T>::value_type>>(
        std::forward<T>(curve)
    );
}

} // namespace basjoo::math
