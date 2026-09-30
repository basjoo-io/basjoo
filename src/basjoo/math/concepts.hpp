/**
 * @file concepts.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-11-15
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <complex>
#include <concepts>
#include <cstddef>
#include <type_traits>

namespace basjoo::math {

// Derived is, or publicly and unambiguously derives from, a specialization of Base.
template <class Derived, template <typename...> class Base>
concept derivedFromTemplate =
    requires(Derived* derived) { []<typename... Ts>(const volatile Base<Ts...>*) {}(derived); };

// A deduced forwarding-reference type whose argument may be recycled: a non-const rvalue.
template <typename T>
concept Reusable = !std::is_lvalue_reference_v<T> && !std::is_const_v<std::remove_reference_t<T>>;

template <typename T>
concept Allocatory = requires(T a, typename T::value_type* p, std::size_t n, const T& ca) {
    { a.allocate(n) } -> std::same_as<typename T::value_type*>;
    { a.deallocate(p, n) };

    { a == ca } -> std::convertible_to<bool>;
    { a != ca } -> std::convertible_to<bool>;
};

template <typename T>
concept Arithmetic = std::is_arithmetic_v<std::remove_cvref_t<T>>;

// std::complex<F> with a floating-point F, the only instantiations the standard specifies. Matched
// structurally rather than by probing operators: an operator probe resolves overloads, which may
// pick up operator templates constrained on ScalarArithmetic and recurse into this concept.
template <typename T>
concept ComplexArithmetic =
    std::floating_point<typename std::remove_cvref_t<T>::value_type> &&
    std::same_as<std::remove_cvref_t<T>, std::complex<typename std::remove_cvref_t<T>::value_type>>;

template <typename T>
concept ScalarArithmetic = Arithmetic<T> || ComplexArithmetic<T>;

template <typename T>
concept VecArithmetic = (
    ScalarArithmetic<typename std::remove_cvref_t<T>::value_type> &&

    requires(std::remove_cvref_t<T> a) {
        { a.data() };
        { a.size() } -> std::same_as<typename std::remove_cvref_t<T>::size_type>;
        { a.stride() } -> std::integral;
    } &&

    requires(std::remove_cvref_t<T> a, typename std::remove_cvref_t<T>::size_type i) {
        { a[i] } -> std::same_as<typename std::remove_cvref_t<T>::value_type&>;
        { a.coeff(i) } -> std::same_as<typename std::remove_cvref_t<T>::value_type>;
    } &&

    requires(std::remove_cvref_t<T> a, std::remove_cvref_t<T> b) {
        { a + b } -> std::same_as<std::remove_cvref_t<T>>;
        { a - b } -> std::same_as<std::remove_cvref_t<T>>;
        { a.euclideanTo(b) };
    } &&

    requires(std::remove_cvref_t<T> a, typename std::remove_cvref_t<T>::value_type b) {
        { a * b } -> std::same_as<std::remove_cvref_t<T>>;
        { b * a } -> std::same_as<std::remove_cvref_t<T>>;
        { a / b } -> std::same_as<std::remove_cvref_t<T>>;
    }
);

template <typename T>
concept MatArithmetic = (
    ScalarArithmetic<typename std::remove_cvref_t<T>::value_type> &&

    requires(std::remove_cvref_t<T> a) {
        { a.data() };
        { a.nrows() } -> std::same_as<typename std::remove_cvref_t<T>::index_type>;
        { a.ncols() } -> std::same_as<typename std::remove_cvref_t<T>::index_type>;
        { a.stride(0) } -> std::same_as<typename std::remove_cvref_t<T>::index_type>;
        { a.stride(1) } -> std::same_as<typename std::remove_cvref_t<T>::index_type>;
    } &&

    requires(std::remove_cvref_t<T> a, typename std::remove_cvref_t<T>::index_type i, typename std::remove_cvref_t<T>::index_type j) {
        { a[i, j] } -> std::same_as<typename std::remove_cvref_t<T>::value_type&>;
        { a.coeff(i, j) } -> std::same_as<typename std::remove_cvref_t<T>::value_type>;
    } &&

    requires(std::remove_cvref_t<T> a, std::remove_cvref_t<T> b) {
        { a + b } -> std::same_as<std::remove_cvref_t<T>>;
        { a - b } -> std::same_as<std::remove_cvref_t<T>>;
    } &&

    requires(std::remove_cvref_t<T> a, typename std::remove_cvref_t<T>::value_type b) {
        { a * b } -> std::same_as<std::remove_cvref_t<T>>;
        { b * a } -> std::same_as<std::remove_cvref_t<T>>;
        { a / b } -> std::same_as<std::remove_cvref_t<T>>;
    }
);

template <typename T>
concept SparseMatArithmetic = (
    requires(std::remove_cvref_t<T> a) {
        { a.compress() } -> std::same_as<void>;
        { a.nrows() } -> std::same_as<typename std::remove_cvref_t<T>::size_type>;
        { a.ncols() } -> std::same_as<typename std::remove_cvref_t<T>::size_type>;
        { a.nnzs() } -> std::same_as<typename std::remove_cvref_t<T>::size_type>;
    } &&

    requires(std::remove_cvref_t<T> a, typename std::remove_cvref_t<T>::size_type i, typename std::remove_cvref_t<T>::size_type j) {
        { a[i, j] } -> std::same_as<typename std::remove_cvref_t<T>::value_type>;
        { a.coeff(i, j) } -> std::same_as<typename std::remove_cvref_t<T>::value_type>;
        { a.eraseCoeff(i, j) } -> std::same_as<void>;
    } &&

    requires(std::remove_cvref_t<T> a, typename std::remove_cvref_t<T>::size_type i, typename std::remove_cvref_t<T>::size_type j, typename std::remove_cvref_t<T>::const_reference val) {
        { a.updateCoeff(i, j, val) } -> std::same_as<void>;
    }
);

template <typename T>
concept GeneralArithmetic = ScalarArithmetic<T> || VecArithmetic<T> || MatArithmetic<T>;

} // namespace basjoo::math
