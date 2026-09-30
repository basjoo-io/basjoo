/**
 * @file dense_arithmetic_base.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2026-10-08
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <concepts>
#include <cstddef>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_lanes.hpp"
#include "basjoo/math/dense/dense_trait.hpp"

namespace basjoo::math {

// Binary/unary operators on rvalue operands mutate and return the operand itself as `T&&`
// (zero copy); lvalue-only operands yield a new `T` by value. Binding the result of an
// expression whose reused operand is a temporary to a reference outlives that temporary. The
// element-wise work runs on the size() storage slots at data() through DenseLanes. A derived type
// declares extents_type: without a dynamic extent, the sum or difference of two lvalues is written
// straight into a default constructed object.
class DenseArithmeticBase {
  public:
    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto
    operator+=(this Self&& self, const std::same_as<std::remove_cvref_t<Self>> auto& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

#if BASJOO_CHECK_PARAMS == 1
        if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
            checkExtents(self, obj, "DenseArithmeticBase::+=: dimensions do not match!");
        }
#endif
        DenseLanes<value_type>::add(self.size(), self.data(), obj.data(), self.data());
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto
    operator-=(this Self&& self, const std::same_as<std::remove_cvref_t<Self>> auto& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;

#if BASJOO_CHECK_PARAMS == 1
        if constexpr (!std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>) {
            checkExtents(self, obj, "DenseArithmeticBase::-=: dimensions do not match!");
        }
#endif
        DenseLanes<value_type>::subtract(self.size(), self.data(), obj.data(), self.data());
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    // Of two lvalues of a fixed size type, the sum is written in one pass into a fresh object.
    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto operator+(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> decltype(auto) {
        using type = std::remove_cvref_t<Self>;
        using value_type = typename type::value_type;

        if constexpr (Reusable<Self>) {
            self += obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj>) {
            obj += self;
            return static_cast<Obj&&>(obj);
        } else if constexpr (type::extents_type::rank_dynamic() == 0) {
            type result;
            DenseLanes<value_type>::add(self.size(), self.data(), obj.data(), result.data());
            return result;
        } else {
            type result{self};
            result += obj;
            return result;
        }
    }

    template <typename Self, typename Obj>
        requires std::same_as<std::remove_cvref_t<Self>, std::remove_cvref_t<Obj>>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self, Obj&& obj) noexcept(
        !BASJOO_CHECK_PARAMS || std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>>
    ) -> decltype(auto) {
        using type = std::remove_cvref_t<Self>;
        using value_type = typename type::value_type;

        if constexpr (Reusable<Self>) {
            self -= obj;
            return static_cast<Self&&>(self);
        } else if constexpr (Reusable<Obj>) {
#if BASJOO_CHECK_PARAMS == 1
            if constexpr (!std::is_nothrow_copy_constructible_v<type>) {
                checkExtents(self, obj, "DenseArithmeticBase::-: dimensions do not match!");
            }
#endif
            // self - obj written over obj.
            DenseLanes<value_type>::subtract(self.size(), self.data(), obj.data(), obj.data());
            return static_cast<Obj&&>(obj);
        } else if constexpr (type::extents_type::rank_dynamic() == 0) {
            type result;
            DenseLanes<value_type>::subtract(self.size(), self.data(), obj.data(), result.data());
            return result;
        } else {
            type result{self};
            result -= obj;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator==(this const Self& self, const std::same_as<Self> auto& obj) noexcept
        -> bool {
        using rank_type = typename Self::rank_type;
        using value_type = typename Self::value_type;

        for (rank_type r{0}; r < Self::rank(); ++r) {
            if (self.extent(r) != obj.extent(r)) [[unlikely]] {
                return false;
            }
        }
        return DenseLanes<value_type>::equal(self.size(), self.data(), obj.data());
    }

    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(this Self&& self, const ScalarArithmetic auto& fac) noexcept
        -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        static_assert(
            ComplexArithmetic<value_type> || Arithmetic<decltype(fac)>,
            "DenseArithmeticBase::*=: a real valued object only scales by a real scalar!"
        );

        // A real factor scales every lane, real and imaginary alike.
        if constexpr (ComplexArithmetic<decltype(fac)>) {
            DenseLanes<value_type>::scale(self.size(), static_cast<value_type>(fac), self.data());
        } else {
            DenseLanes<value_type>::scale(
                self.size(), static_cast<typename DenseLanes<value_type>::real_type>(fac),
                self.data()
            );
        }
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    // Integers divide element by element; anything else is scaled by the reciprocal, taken in the
    // precision of the real or complex factor.
    template <typename Self>
        requires(!std::is_const_v<std::remove_reference_t<Self>>)
    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(this Self&& self, const ScalarArithmetic auto& den) noexcept
        -> std::remove_cvref_t<Self>& {
        using value_type = typename std::remove_cvref_t<Self>::value_type;
        using real_type = typename DenseLanes<value_type>::real_type;
        using size_type = typename std::remove_cvref_t<Self>::size_type;
        static_assert(
            ComplexArithmetic<value_type> || Arithmetic<decltype(den)>,
            "DenseArithmeticBase::/=: a real valued object only scales by a real scalar!"
        );

        if constexpr (std::is_integral_v<value_type>) {
            const auto alpha{static_cast<value_type>(den)};
            value_type* const p{self.data()};
            const size_type n{self.size()};
            for (size_type k{0}; k < n; ++k) {
                p[k] = static_cast<value_type>(p[k] / alpha);
            }
        } else if constexpr (ComplexArithmetic<decltype(den)>) {
            self *= value_type(1.0) / static_cast<value_type>(den);
        } else {
            self *= real_type(1.0) / static_cast<real_type>(den);
        }
        return static_cast<std::remove_cvref_t<Self>&>(self);
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator*(this Self&& self, const ScalarArithmetic auto& fac) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self *= fac;
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result *= fac;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator/(this Self&& self, const ScalarArithmetic auto& den) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self /= den;
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result /= den;
            return result;
        }
    }

    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator-(this Self&& self) noexcept(
        std::is_nothrow_copy_constructible_v<std::remove_cvref_t<Self>> || Reusable<Self>
    ) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfNegate();
            return static_cast<Self&&>(self);
        } else {
            std::remove_cvref_t<Self> result{self};
            result.selfNegate();
            return result;
        }
    }

  protected:
    constexpr DenseArithmeticBase() noexcept = default;
    constexpr DenseArithmeticBase(const DenseArithmeticBase&) noexcept = default;
    constexpr DenseArithmeticBase(DenseArithmeticBase&&) noexcept = default;
    constexpr auto operator=(const DenseArithmeticBase&) noexcept -> DenseArithmeticBase& = default;
    constexpr auto operator=(DenseArithmeticBase&&) noexcept -> DenseArithmeticBase& = default;
    constexpr ~DenseArithmeticBase() noexcept = default;

    // In-place negation behind operator-, protected so that derived types reuse it.
    template <typename Self>
        requires(!std::is_const_v<Self>)
    [[using gnu: always_inline, hot]]
    constexpr auto selfNegate(this Self& self) noexcept -> void {
        DenseLanes<typename Self::value_type>::negate(self.size(), self.data());
        return;
    }

  private:
#if BASJOO_CHECK_PARAMS == 1
    template <typename T>
    static constexpr auto checkExtents(const T& lhs, const T& rhs, const char* message) -> void {
        using rank_type = typename T::rank_type;

        for (rank_type r{0}; r < T::rank(); ++r) {
            if (lhs.extent(r) != rhs.extent(r)) [[unlikely]] {
                throw std::invalid_argument(message);
            }
        }
        return;
    }
#endif
};

template <typename T>
    requires std::derived_from<std::remove_cvref_t<T>, DenseArithmeticBase>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(const ScalarArithmetic auto& fac, T&& obj) noexcept(
    std::is_nothrow_copy_constructible_v<std::remove_cvref_t<T>> || Reusable<T>
) -> decltype(auto) {
    if constexpr (Reusable<T>) {
        obj *= fac;
        return static_cast<T&&>(obj);
    } else {
        return obj * fac;
    }
}

} // namespace basjoo::math
