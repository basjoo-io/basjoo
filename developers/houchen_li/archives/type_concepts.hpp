/**
 * @file type_concepts.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-09-07
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <type_traits>

#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

namespace detail {

template <typename T, typename Enable = void>
using TypeConcept = T;

} // namespace detail

template <typename T, bool C>
using TypeConcept = detail::TypeConcept<T, typename std::enable_if_t<C>>;

template <class Derived, class Base>
using DerivedFrom = TypeConcept<Derived, std::is_base_of_v<Base, Derived>>;

template <class Derived, template <typename...> class Base>
using DerivedFromTemplate = TypeConcept<Derived, isBaseOfTemplateV<Base, Derived>>;

template <typename T>
using Integral = TypeConcept<T, std::is_integral_v<typename std::remove_reference_t<T>>>;

template <typename T>
using FloatingPoint = TypeConcept<T, std::is_floating_point_v<typename std::remove_reference_t<T>>>;

template <typename T>
using Arithmetic = TypeConcept<T, std::is_arithmetic_v<typename std::remove_reference_t<T>>>;

template <typename T>
using Iterable = TypeConcept<T, isIterableV<T>>;

template <typename T>
using Allocatory = TypeConcept<T, isAllocatoryV<T>>;

template <typename T>
using ComplexArithmetic = TypeConcept<T, isComplexArithmeticV<T>>;

template <typename T>
using VecArithmetic = TypeConcept<T, isVecArithmeticV<T>>;

template <typename T>
using MatArithmetic = TypeConcept<T, isMatArithmeticV<T>>;

template <typename T>
using ScalarArithmetic = TypeConcept<T, isScalarArithmeticV<T>>;

template <typename T>
using GeneralArithmetic = TypeConcept<T, isGeneralArithmeticV<T>>;

} // namespace basjoo::math
