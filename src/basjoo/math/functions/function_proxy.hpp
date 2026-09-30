/**
 * @file function_proxy.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-08-09
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cstdint>
#include <system_error>
#include <utility>

#include "proxy/proxy.h"
#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/detail/dense_degenerate_trait.hpp"
#include "basjoo/math/functions/piecewise_cubic_function.hpp"
#include "basjoo/math/functions/piecewise_linear_function.hpp"
#include "basjoo/math/functions/piecewise_quintic_function.hpp"

namespace basjoo::math {

namespace detail {

// NOLINTBEGIN(modernize-use-trailing-return-type)

PRO_DEF_MEM_DISPATCH(c_eval, eval);
PRO_DEF_MEM_DISPATCH(c_derivative, derivative);
PRO_DEF_MEM_DISPATCH(c_integral, integral);
PRO_DEF_MEM_DISPATCH(c_minT, minT);
PRO_DEF_MEM_DISPATCH(c_maxT, maxT);

// NOLINTEND(modernize-use-trailing-return-type)

// clang-format off

template <::basjoo::math::GeneralArithmetic T>
class FunctionFacade final : public pro::skills::rtti<typename pro::facade_builder
    ::support_copy<pro::constraint_level::nontrivial>
    ::template add_convention<pro::operator_dispatch<"()">, auto(::basjoo::math::detail::DenseDegenerateTraitT<T>) const noexcept -> T>
    ::template add_convention<c_eval, auto(::basjoo::math::detail::DenseDegenerateTraitT<T>) const noexcept -> T>
    ::template add_convention<c_derivative, auto(::basjoo::math::detail::DenseDegenerateTraitT<T>) const noexcept -> T>
    ::template add_convention<c_integral, auto(::basjoo::math::detail::DenseDegenerateTraitT<T>, ::basjoo::math::detail::DenseDegenerateTraitT<T>) const noexcept -> T>
    ::template add_convention<c_minT, auto() const noexcept -> ::basjoo::math::detail::DenseDegenerateTraitT<T>>
    ::template add_convention<c_maxT, auto() const noexcept -> ::basjoo::math::detail::DenseDegenerateTraitT<T>>
    >::build {};

// clang-format on

enum class FunctionProxyTag : std::uint8_t {
    LINEAR,
    CUBIC,
    QUINTIC
};

template <GeneralArithmetic T>
constexpr auto serialize(auto& archive, const pro::proxy<FunctionFacade<T>>& self)
    -> zpp::bits::errc {
    if (const auto* function1 = proxy_cast<PiecewiseLinearFunction<T>>(&*self)) {
        if (auto result = archive(FunctionProxyTag::LINEAR); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*function1);
    }
    if (const auto* function1 = proxy_cast<PiecewiseCubicFunction<T>>(&*self)) {
        if (auto result = archive(FunctionProxyTag::CUBIC); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*function1);
    }
    if (const auto* function1 = proxy_cast<PiecewiseQuinticFunction<T>>(&*self)) {
        if (auto result = archive(FunctionProxyTag::QUINTIC); zpp::bits::failure(result))
            [[unlikely]] {
            return result;
        }
        return archive(*function1);
    }
    return std::errc::invalid_argument;
}

template <GeneralArithmetic T>
constexpr auto serialize(auto& archive, pro::proxy<FunctionFacade<T>>& self) -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        if (auto* function1 = proxy_cast<PiecewiseLinearFunction<T>>(&*self)) {
            if (auto result = archive(FunctionProxyTag::LINEAR); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*function1);
        }
        if (auto* function1 = proxy_cast<PiecewiseCubicFunction<T>>(&*self)) {
            if (auto result = archive(FunctionProxyTag::CUBIC); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*function1);
        }
        if (auto* function1 = proxy_cast<PiecewiseQuinticFunction<T>>(&*self)) {
            if (auto result = archive(FunctionProxyTag::QUINTIC); zpp::bits::failure(result))
                [[unlikely]] {
                return result;
            }
            return archive(*function1);
        }
        return std::errc::invalid_argument;
    } else {
        FunctionProxyTag tag{};
        if (auto result = archive(tag); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        switch (tag) {
        case FunctionProxyTag::LINEAR: {
            PiecewiseLinearFunction<T> function1{};
            if (auto result = archive(function1); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<FunctionFacade<T>>(std::move(function1));
            return {};
        }
        case FunctionProxyTag::CUBIC: {
            PiecewiseCubicFunction<T> function1{};
            if (auto result = archive(function1); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<FunctionFacade<T>>(std::move(function1));
            return {};
        }
        case FunctionProxyTag::QUINTIC: {
            PiecewiseQuinticFunction<T> function1{};
            if (auto result = archive(function1); zpp::bits::failure(result)) [[unlikely]] {
                return result;
            }
            self = pro::make_proxy<FunctionFacade<T>>(std::move(function1));
            return {};
        }
        }
        return std::errc::invalid_argument;
    }
}

} // namespace detail

template <GeneralArithmetic T>
using FunctionProxy = pro::proxy<detail::FunctionFacade<T>>;

template <typename T>
[[using gnu: always_inline]]
inline auto makeFunctionProxy(T&& function1)
    -> FunctionProxy<typename std::remove_cvref_t<T>::value_type> {
    return pro::make_proxy<detail::FunctionFacade<typename std::remove_cvref_t<T>::value_type>>(
        std::forward<T>(function1)
    );
}

} // namespace basjoo::math
