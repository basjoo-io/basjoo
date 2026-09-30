/**
 * @file type_traits.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-12-28
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

// SFINAE (C++17 style) counterparts of the concepts in basjoo/math/concepts.hpp. Every trait
// mirrors the concept of the same name clause for clause (operands are lvalues, as the parameters
// of a requires-expression are); keep both in sync.

#include <complex>
#include <cstddef>
#include <type_traits>
#include <utility>

namespace basjoo::math {

namespace detail {

template <template <typename...> class Base, typename... Ts>
auto deduceBaseOfTemplate(const volatile Base<Ts...>*) -> void;

// An inaccessible (private/protected) or ambiguous base fails the substitution, i.e. false.
template <template <typename...> class Base, class Derived, typename = void>
struct isBaseOfTemplateImpl final : public std::false_type {};

template <template <typename...> class Base, class Derived>
struct isBaseOfTemplateImpl<
    Base, Derived, std::void_t<decltype(deduceBaseOfTemplate<Base>(std::declval<Derived*>()))>>
    final : public std::true_type {};

} // namespace detail

template <template <typename...> class Base, class Derived>
using isBaseOfTemplate = typename detail::isBaseOfTemplateImpl<Base, Derived>::type;

template <template <typename...> class Base, class Derived>
inline constexpr bool isBaseOfTemplateV = isBaseOfTemplate<Base, Derived>::value;

// A deduced forwarding-reference type whose argument may be recycled: a non-const rvalue.
template <typename T>
struct isReusable final
    : public std::bool_constant<
          !std::is_lvalue_reference_v<T> && !std::is_const_v<std::remove_reference_t<T>>> {};

template <typename T>
inline constexpr bool isReusableV = isReusable<T>::value;

template <typename T, typename = void>
struct isAllocatory final : public std::false_type {};

template <typename T>
struct isAllocatory<
    T,
    std::enable_if_t<
        std::is_same_v<
            decltype(std::declval<T&>().allocate(std::declval<std::size_t&>())),
            typename T::value_type*> &&
            std::is_convertible_v<decltype(std::declval<T&>() == std::declval<const T&>()), bool> &&
            std::is_convertible_v<decltype(std::declval<T&>() != std::declval<const T&>()), bool>,
        std::void_t<decltype(std::declval<T&>().deallocate(
            std::declval<typename T::value_type*&>(), std::declval<std::size_t&>()
        ))>>>
    final : public std::true_type {};

template <typename T>
inline constexpr bool isAllocatoryV = isAllocatory<T>::value;

namespace detail {

template <typename T>
struct isComplexArithmeticImpl : public std::false_type {};

template <typename F>
struct isComplexArithmeticImpl<std::complex<F>> : public std::is_floating_point<F> {};

} // namespace detail

// std::complex<F> with a floating-point F, matched by partial specialization rather than by
// probing operators (see ComplexArithmetic).
template <typename T>
struct isComplexArithmetic final : public detail::isComplexArithmeticImpl<std::remove_cvref_t<T>> {
};

template <typename T>
inline constexpr bool isComplexArithmeticV = isComplexArithmetic<T>::value;

template <typename T>
struct isScalarArithmetic final
    : public std::bool_constant<
          std::is_arithmetic_v<std::remove_cvref_t<T>> || isComplexArithmeticV<T>> {};

template <typename T>
inline constexpr bool isScalarArithmeticV = isScalarArithmetic<T>::value;

template <typename T, typename = void>
struct isVecArithmetic final : public std::false_type {};

template <typename T>
struct isVecArithmetic<
    T,
    std::enable_if_t<
        isScalarArithmeticV<typename std::remove_cvref_t<T>::value_type> &&

        std::is_same_v<
            decltype(std::remove_cvref_t<T>::rank()), typename std::remove_cvref_t<T>::rank_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>()
                         .extent(std::declval<typename std::remove_cvref_t<T>::rank_type&>())),
            typename std::remove_cvref_t<T>::index_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>().size()),
            typename std::remove_cvref_t<T>::size_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>().data()),
            typename std::remove_cvref_t<T>::value_type*> &&

        (std::remove_cvref_t<T>::rank() == 1) &&

        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>()
                         [std::declval<typename std::remove_cvref_t<T>::index_type&>()]),
            typename std::remove_cvref_t<T>::value_type&> &&

        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() + std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() - std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(-std::declval<std::remove_cvref_t<T>&>()), std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() += std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>&> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() -= std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>&> &&
        std::is_convertible_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() == std::declval<std::remove_cvref_t<T>&>()),
            bool> &&

        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() * std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<typename std::remove_cvref_t<T>::value_type&>() * std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() / std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() *= std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>&> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() /= std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>&>>>
    final : public std::true_type {};

template <typename T>
inline constexpr bool isVecArithmeticV = isVecArithmetic<T>::value;

template <typename T, typename = void>
struct isMatArithmetic final : public std::false_type {};

template <typename T>
struct isMatArithmetic<
    T,
    std::enable_if_t<
        isScalarArithmeticV<typename std::remove_cvref_t<T>::value_type> &&

        std::is_same_v<
            decltype(std::remove_cvref_t<T>::rank()), typename std::remove_cvref_t<T>::rank_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>()
                         .extent(std::declval<typename std::remove_cvref_t<T>::rank_type&>())),
            typename std::remove_cvref_t<T>::index_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>()
                         .stride(std::declval<typename std::remove_cvref_t<T>::rank_type&>())),
            typename std::remove_cvref_t<T>::index_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>().nrows()),
            typename std::remove_cvref_t<T>::index_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>().ncols()),
            typename std::remove_cvref_t<T>::index_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>().size()),
            typename std::remove_cvref_t<T>::size_type> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>().data()),
            typename std::remove_cvref_t<T>::value_type*> &&

        (std::remove_cvref_t<T>::rank() == 2) &&

        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>()
                         [std::declval<typename std::remove_cvref_t<T>::index_type&>(),
                          std::declval<typename std::remove_cvref_t<T>::index_type&>()]),
            typename std::remove_cvref_t<T>::value_type&> &&

        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() + std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() - std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(-std::declval<std::remove_cvref_t<T>&>()), std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() += std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>&> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() -= std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>&> &&
        std::is_convertible_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() == std::declval<std::remove_cvref_t<T>&>()),
            bool> &&

        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() * std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<typename std::remove_cvref_t<T>::value_type&>() * std::declval<std::remove_cvref_t<T>&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() / std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() *= std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>&> &&
        std::is_same_v<
            decltype(std::declval<std::remove_cvref_t<T>&>() /= std::declval<typename std::remove_cvref_t<T>::value_type&>()),
            std::remove_cvref_t<T>&>>>
    final : public std::true_type {};

template <typename T>
inline constexpr bool isMatArithmeticV = isMatArithmetic<T>::value;

template <typename T, typename = void>
struct isSparseMatArithmetic final : public std::false_type {};

template <typename T>
struct isSparseMatArithmetic<
    T, std::enable_if_t<
           std::is_same_v<decltype(std::declval<std::remove_cvref_t<T>&>().compress()), void> &&
           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>().nrows()),
               typename std::remove_cvref_t<T>::size_type> &&
           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>().ncols()),
               typename std::remove_cvref_t<T>::size_type> &&
           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>().nnzs()),
               typename std::remove_cvref_t<T>::size_type> &&

           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>()
                            [std::declval<typename std::remove_cvref_t<T>::size_type&>(),
                             std::declval<typename std::remove_cvref_t<T>::size_type&>()]),
               typename std::remove_cvref_t<T>::value_type> &&
           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>().coeff(
                   std::declval<typename std::remove_cvref_t<T>::size_type&>(),
                   std::declval<typename std::remove_cvref_t<T>::size_type&>()
               )),
               typename std::remove_cvref_t<T>::value_type> &&
           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>().eraseCoeff(
                   std::declval<typename std::remove_cvref_t<T>::size_type&>(),
                   std::declval<typename std::remove_cvref_t<T>::size_type&>()
               )),
               void> &&

           std::is_same_v<
               decltype(std::declval<std::remove_cvref_t<T>&>().updateCoeff(
                   std::declval<typename std::remove_cvref_t<T>::size_type&>(),
                   std::declval<typename std::remove_cvref_t<T>::size_type&>(),
                   std::declval<typename std::remove_cvref_t<T>::const_reference&>()
               )),
               void>>>
    final : public std::true_type {};

template <typename T>
inline constexpr bool isSparseMatArithmeticV = isSparseMatArithmetic<T>::value;

template <typename T>
struct isGeneralArithmetic final
    : public std::bool_constant<
          isScalarArithmeticV<T> || isVecArithmeticV<T> || isMatArithmeticV<T>> {};

template <typename T>
inline constexpr bool isGeneralArithmeticV = isGeneralArithmetic<T>::value;

} // namespace basjoo::math
