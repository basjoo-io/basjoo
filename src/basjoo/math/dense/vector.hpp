/**
 * @file vector.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-04-02
 *
 * @copyright Copyright (c) 2025 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <complex>
#include <iomanip>
#include <numeric>
#include <ostream>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>

#include "zpp_bits.h"

#ifdef BASJOO_USE_BLAS_LAPACK
#include "cblas.h"
#include "lapacke.h"
#endif

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_traits.hpp"
#include "basjoo/math/dense/detail/dense_norm_trait.hpp"
#include "basjoo/math/dense/vector_view.hpp"
#include "basjoo/math/type_traits.hpp"

namespace basjoo::math {

template <ScalarArithmetic Scalar, std::size_t Size>
class alignas(32) Vector final {
  public:
    using value_type = typename DenseTraits<Vector>::value_type;
    using reference = typename DenseTraits<Vector>::reference;
    using const_reference = typename DenseTraits<Vector>::const_reference;
    using pointer = typename DenseTraits<Vector>::pointer;
    using const_pointer = typename DenseTraits<Vector>::const_pointer;
    using size_type = typename DenseTraits<Vector>::size_type;
    using difference_type = typename DenseTraits<Vector>::difference_type;
    using allocator_type = typename DenseTraits<Vector>::allocator_type;

    static constexpr size_type kSize = Size;

    constexpr Vector(const Vector& other) noexcept = default;
    constexpr auto operator=(const Vector& other) noexcept -> Vector& = default;
    constexpr Vector(Vector&& other) noexcept = default;
    constexpr auto operator=(Vector&& other) noexcept -> Vector& = default;
    constexpr ~Vector() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return allocator_type{};
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector([[maybe_unused]] const allocator_type& alloc = {}) noexcept {}

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] size_type size, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (size != kSize) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        [[maybe_unused]] size_type size, const_reference value,
        [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS) {
#if BASJOO_CHECK_PARAMS == 1
        if (size != kSize) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        m_data.fill(value);
    }

    [[using gnu: always_inline]]
    constexpr Vector(
        std::initializer_list<value_type> list, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{list} {
#if BASJOO_CHECK_PARAMS == 1
        if (list.size() != kSize) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    [[using gnu: always_inline]]
    constexpr explicit Vector(
        const std::array<value_type, kSize>& data, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        : m_data{data} {
#if BASJOO_CHECK_PARAMS == 1
        if (data.size() != kSize) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
    }

    template <typename U>
    [[using gnu: always_inline]] constexpr explicit Vector(
        std::span<U, kSize> data, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept
        requires std::is_same_v<std::remove_const_t<U>, value_type>
    {
        std::ranges::copy(data, m_data.begin());
    }

    template <typename U>
    [[using gnu: always_inline]] constexpr explicit Vector(
        VectorView<U> vector_view, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires std::is_same_v<std::remove_const_t<U>, value_type>
    {
#if BASJOO_CHECK_PARAMS == 1
        if (vector_view.size() != kSize) [[unlikely]] {
            throw std::invalid_argument("Vector constructor: size mismatches!");
        }
#endif
        for (size_type i{0}; i < kSize; ++i) {
            operator[](i) = vector_view[i];
        }
    }

    template <typename U>
    [[using gnu: always_inline]] constexpr explicit Vector(
        MatrixView<U> matrix_view, [[maybe_unused]] const allocator_type& alloc = {}
    ) noexcept(!BASJOO_CHECK_PARAMS)
        requires std::is_same_v<std::remove_const_t<U>, value_type>
        : m_data(matrix_view.size()) {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_view.nrows() != 1 && matrix_view.ncols() != 1) [[unlikely]] {
            throw std::out_of_range("this matrix does not covert to a vector.");
        }
        if (matrix_view.size() != kSize) [[unlikely]] {
            throw std::invalid_argument("Matrix size must be equal to Vector size");
        }
#endif
        for (size_type i = 0; i < kSize; ++i) {
            operator[](i) = matrix_view[i];
        }
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::array<value_type, kSize>() const noexcept {
        return std::array<value_type, kSize>{m_data};
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<value_type, kSize>() noexcept {
        return std::span<value_type, kSize>(m_data);
    }

    [[using gnu: pure, always_inline]]
    constexpr operator std::span<const value_type, kSize>() const noexcept {
        return std::span<const value_type, kSize>(m_data);
    }

    [[using gnu: pure, always_inline]]
    constexpr operator VectorView<value_type>() noexcept {
        return VectorView<value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline]]
    constexpr operator VectorView<const value_type>() const noexcept {
        return VectorView<const value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return kSize;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto stride() noexcept -> size_type {
        return 1;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> pointer {
        return m_data.data();
    }
    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: always_inline]]
    static constexpr auto resize([[maybe_unused]] size_type size) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (size != kSize) [[unlikely]] {
            throw std::invalid_argument{"Vector::resize: size mismatches"};
        }
#endif
        return;
    }

    [[using gnu: always_inline]]
    constexpr auto assign([[maybe_unused]] size_type size, const_reference value) -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (size != kSize) [[unlikely]] {
            throw std::invalid_argument{"Vector::assign: size mismatch"};
        }
#endif
        m_data.fill(value);
    }

    [[using gnu: always_inline]]
    constexpr auto fill(const_reference value) noexcept -> void {
        std::ranges::fill(m_data, value);
        return;
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto empty() noexcept -> bool {
        return kSize == 0;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_type i) noexcept -> reference {
        return m_data[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator[](size_type i) const noexcept -> const_reference {
        return m_data[i];
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= kSize) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        return m_data[i];
    }

    [[using gnu: always_inline, hot]]
    constexpr auto updateCoeff(size_type i, const_reference value) noexcept(!BASJOO_CHECK_PARAMS)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= kSize) [[unlikely]] {
            throw std::out_of_range("Vector index out of range");
        }
#endif
        m_data[i] = value;
        return;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(std::pair<size_type, size_type> range) noexcept -> VectorView<value_type> {
        range.second = std::min(range.second, kSize);
        return VectorView<value_type>{
            data() + range.first * stride(), range.second - range.first, stride()
        };
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view(std::pair<size_type, size_type> range) const noexcept
        -> VectorView<const value_type> {
        range.second = std::min(range.second, kSize);
        return VectorView<value_type>{
            data() + range.first * stride(), range.second - range.first, stride()
        };
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view() noexcept -> VectorView<value_type> {
        return view({0, std::numeric_limits<value_type>::max()});
    }

    [[using gnu: pure, always_inline]]
    constexpr auto view() const noexcept -> VectorView<const value_type> {
        return view({0, std::numeric_limits<value_type>::max()});
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+=(const Vector& obj) noexcept -> Vector& {
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(1.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_saxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_daxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_caxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zaxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else {
            std::transform(data(), data() + size(), obj.data(), data(), std::plus<value_type>());
        }
#else
        std::transform(data(), data() + size(), obj.data(), data(), std::plus<value_type>());
#endif
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator+(const Vector& obj) const& noexcept -> Vector {
        Vector result{*this};
        result += obj;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+(Vector&& obj) const& noexcept -> Vector&& {
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+(const Vector& obj) && noexcept -> Vector&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator+(Vector&& obj) && noexcept -> Vector&& {
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-=(const Vector& obj) noexcept -> Vector& {
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr value_type alpha(-1.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_saxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_daxpy(size(), alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_caxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zaxpy(size(), &alpha, obj.data(), 1, data(), 1);
        } else {
            std::transform(data(), data() + size(), obj.data(), data(), std::minus<value_type>());
        }
#else
        std::transform(data(), data() + size(), obj.data(), data(), std::minus<value_type>());
#endif
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-(const Vector& obj) const& noexcept -> Vector {
        Vector result{*this};
        result -= obj;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(Vector&& obj) const& noexcept -> Vector&& {
        obj *= -1.0;
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(const Vector& obj) && noexcept -> Vector&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(Vector&& obj) && noexcept -> Vector&& {
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator==(const Vector& other) const noexcept -> bool {
        for (size_type i{0}; i < kSize; ++i) {
            if (m_data[i] != other.m_data[i]) {
                return false;
            }
        }
        return true;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(const ScalarArithmetic auto& fac) noexcept -> Vector& {
        const auto alpha{static_cast<value_type>(fac)};
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sscal(size(), alpha, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dscal(size(), alpha, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cscal(size(), &alpha, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zscal(size(), &alpha, data(), 1);
        } else {
            std::transform(
                data(), data() + size(), data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x * alpha; }
            );
        }
#else
        std::transform(
            data(), data() + size(), data(),
            [alpha](const value_type& x) constexpr noexcept -> value_type { return x * alpha; }
        );
#endif
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator*(const ScalarArithmetic auto& fac) const& noexcept -> Vector {
        Vector result{*this};
        result *= fac;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator*(const ScalarArithmetic auto& fac) && noexcept -> Vector&& {
        operator*=(fac);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator/=(const ScalarArithmetic auto& den) noexcept -> Vector& {
        if constexpr (std::is_integral_v<value_type>) {
            const auto alpha{static_cast<value_type>(den)};
            std::transform(
                data(), data() + size(), data(),
                [alpha](const value_type& x) constexpr noexcept -> value_type { return x / alpha; }
            );
        } else {
            const auto alpha{static_cast<value_type>(1.0 / den)};
            operator*=(alpha);
        }
        return *this;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator/(const ScalarArithmetic auto& den) const& noexcept -> Vector {
        Vector result{*this};
        result /= den;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator/(const ScalarArithmetic auto& den) && noexcept -> Vector&& {
        operator/=(den);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto operator-() const& noexcept -> Vector {
        Vector result{*this};
        result *= -1.0;
        return result;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-() && noexcept -> Vector&& {
        operator*=(-1.0);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto normalized() const noexcept -> Vector {
        return *this / euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean() const noexcept -> detail::DenseNormTraitT<value_type> {
        detail::DenseNormTraitT<value_type> result(0.0);
        if constexpr (Size == 1) {
            result = std::abs(m_data[0]);
        }
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            result = cblas_snrm2(size(), data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            result = cblas_dnrm2(size(), data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            result = cblas_scnrm2(size(), data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            result = cblas_dznrm2(size(), data(), 1);
        } else {
            result = std::transform_reduce(
                data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
            );
            result = std::sqrt(result);
        }
#else
        result = std::transform_reduce(
            data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
            [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                return std::norm(a);
            }
        );
        result = std::sqrt(result);
#endif
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanSqr() const noexcept -> detail::DenseNormTraitT<value_type> {
        detail::DenseNormTraitT<value_type> result(0.0);
        if constexpr (Size == 1) {
            result = std::norm(m_data[0]);
        }
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            result = cblas_sdot(size(), data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            result = cblas_ddot(size(), data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            result = cblas_cdotc(size(), data(), 1, data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            result = cblas_zdotc(size(), data(), 1, data(), 1);
        } else {
            result = std::transform_reduce(
                data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
                [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                    return std::norm(a);
                }
            );
        }
#else
        result = std::transform_reduce(
            data(), data() + size(), 0.0, std::plus<detail::DenseNormTraitT<value_type>>{},
            [](const value_type& a) constexpr noexcept -> detail::DenseNormTraitT<value_type> {
                return std::norm(a);
            }
        );
#endif
        return result;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclideanTo(const Vector& obj) const noexcept
        -> detail::DenseNormTraitT<value_type> {
        return operator-(obj).euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(VectorView<const value_type> obj, double tol = 1E-8) const noexcept
        -> bool {
        if (kSize != obj.size()) {
            return false;
        }
        detail::DenseNormTraitT<value_type> test(0.0);
        for (size_type i{0}; i < kSize; ++i) {
            test = std::max(
                test, std::abs(operator[](i) - obj[i]) /
                          std::max(std::abs(operator[](i)), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto orthogonalTo(const Vector& obj, value_type tol = 1E-8) const noexcept -> bool {
        return std::abs(dot(obj)) < tol * tol;
    }

    [[using gnu: always_inline]]
    constexpr auto selfConjugated() noexcept -> Vector& {
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            if constexpr (std::is_same_v<value_type, std::complex<float>>) {
                LAPACKE_clacgv_work(size(), reinterpret_cast<lapack_complex_float*>(data()), 1);
            } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
                LAPACKE_zlacgv_work(size(), reinterpret_cast<lapack_complex_double*>(data()), 1);
            } else {
                std::transform(
                    data(), data() + size(), data(),
                    [](const value_type& a) constexpr noexcept -> value_type {
                        return std::conj(a);
                    }
                );
            }
        }
#else
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            std::transform(
                data(), data() + size(), data(),
                [](const value_type& a) constexpr noexcept -> value_type { return std::conj(a); }
            );
        }
#endif
        return *this;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto conjugated() const& noexcept -> Vector {
        Vector result{*this};
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            result.selfConjugated();
        }
        return result;
    }

    [[using gnu: always_inline]]
    constexpr auto conjugated() && noexcept -> Vector&& {
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            selfConjugated();
        }
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Vector& obj) const noexcept -> value_type {
        value_type result(0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        if constexpr (std::is_same_v<value_type, float>) {
            result = cblas_sdot(size(), data(), 1, obj.data(), 1);
        } else if constexpr (std::is_same_v<value_type, double>) {
            result = cblas_ddot(size(), data(), 1, obj.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            result = cblas_cdotu(size(), data(), 1, obj.data(), 1);
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            result = cblas_zdotu(size(), data(), 1, obj.data(), 1);
        } else {
            result = std::transform_reduce(data(), data() + size(), obj.data(), result);
        }
#else
        result = std::transform_reduce(data(), data() + size(), obj.data(), result);
#endif
        return result;
    }

    template <size_type OuterSize, MatrixOrder ObjOrder>
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Matrix<value_type, kSize, OuterSize, ObjOrder>& obj) const noexcept
        -> Vector {
        Vector<value_type, OuterSize> result(OuterSize, 0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr CBLAS_ORDER order{
            ObjOrder == ::basjoo::math::MatrixOrder::COL_MAJOR ? CblasColMajor : CblasRowMajor
        };
        [[maybe_unused]] constexpr blasint lda =
            (ObjOrder == ::basjoo::math::MatrixOrder::COL_MAJOR ? OuterSize : kSize);
        [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sgemv(
                order, CblasNoTrans, OuterSize, kSize, alpha, obj.data(), lda, data(), 1, beta,
                result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dgemv(
                order, CblasNoTrans, OuterSize, kSize, alpha, obj.data(), lda, data(), 1, beta,
                result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cgemv(
                order, CblasNoTrans, OuterSize, kSize, &alpha, obj.data(), lda, data(), 1, &beta,
                result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zgemv(
                order, CblasNoTrans, OuterSize, kSize, &alpha, obj.data(), lda, data(), 1, &beta,
                result.data(), 1
            );
        } else {
            for (size_type j{0}; j < OuterSize; ++j) {
                for (size_type i{0}; i < kSize; ++i) {
                    result[j] += operator[](i) * obj[i, j];
                }
            }
        }
#else
        for (size_type j{0}; j < OuterSize; ++j) {
            for (size_type i{0}; i < kSize; ++i) {
                result[j] += operator[](i) * obj[i, j];
            }
        }
#endif
        return result;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    std::array<value_type, Size> m_data;
};

template <ScalarArithmetic Scalar, std::size_t Size>
[[using gnu: pure, always_inline, hot]]
inline constexpr auto operator*(
    const ScalarArithmetic auto& fac, const Vector<Scalar, Size>& obj
) noexcept -> Vector<Scalar, Size> {
    return obj * fac;
}

template <ScalarArithmetic Scalar, std::size_t Size>
[[using gnu: always_inline, hot]]
inline constexpr auto operator*(
    const ScalarArithmetic auto& fac, Vector<Scalar, Size>&& obj
) noexcept -> Vector<Scalar, Size>&& {
    obj *= fac;
    return std::move(obj);
}

template <typename Char, ScalarArithmetic Scalar, std::size_t Size>
inline auto operator<<(std::basic_ostream<Char>& os, const Vector<Scalar, Size>& vector) noexcept
    -> std::basic_ostream<Char>& {
    using value_type = std::remove_const_t<Scalar>;
    using size_type = typename Vector<Scalar, Size>::size_type;
    constexpr size_type kWidth{isComplexArithmeticV<value_type> ? 32 : 16};
    const size_type size{vector.size()};
    os << std::fixed;
    for (size_type i{0}; i < size; ++i) {
        os << std::setw(kWidth) << vector[i] << '\n';
    }
    return os;
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)

namespace std {

template <basjoo::math::ScalarArithmetic Scalar, std::size_t Size>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const basjoo::math::Vector<Scalar, Size>& vector) noexcept
    -> basjoo::math::detail::DenseNormTrait<basjoo::math::Vector<Scalar, Size>> {
    basjoo::math::detail::DenseNormTraitT<basjoo::math::Vector<Scalar, Size>> result(vector);
    if constexpr (basjoo::math::isComplexArithmeticV<Scalar>) {
        std::transform(
            result.data(), result.data() + result.size(), result.data(),
            [](const Scalar& x) constexpr noexcept
                -> basjoo::math::detail::DenseNormTraitT<Scalar> { return std::abs(x); }
        );
    }
    return result;
}

template <basjoo::math::ScalarArithmetic Scalar, std::size_t Size>
[[using gnu: pure, always_inline]]
inline constexpr auto conj(const basjoo::math::Vector<Scalar, Size>& vector) noexcept
    -> basjoo::math::Vector<Scalar, Size> {
    return vector.conjugated();
}

template <basjoo::math::ScalarArithmetic Scalar, std::size_t Size>
[[using gnu: always_inline]]
inline constexpr auto conj(basjoo::math::Vector<Scalar, Size>&& vector) noexcept
    -> basjoo::math::Vector<Scalar, Size>&& {
    vector.selfConjugated();
    return std::move(vector);
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
