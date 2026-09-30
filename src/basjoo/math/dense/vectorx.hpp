/**
 * @file vectorx.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-04-08
 *
 * @copyright Copyright (c) basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cmath>
#include <complex>
#include <iomanip>
#include <numeric>
#include <ostream>
#include <ranges>
#include <span>
#include <stdexcept>
#include <type_traits>
#include <utility>
#include <vector>

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

template <ScalarArithmetic Scalar, Allocatory Alloc>
class VectorX final {
  public:
    using value_type = typename DenseTraits<VectorX>::value_type;
    using reference = typename DenseTraits<VectorX>::reference;
    using const_reference = typename DenseTraits<VectorX>::const_reference;
    using pointer = typename DenseTraits<VectorX>::pointer;
    using const_pointer = typename DenseTraits<VectorX>::const_pointer;
    using size_type = typename DenseTraits<VectorX>::size_type;
    using difference_type = typename DenseTraits<VectorX>::difference_type;
    using allocator_type = typename DenseTraits<VectorX>::allocator_type;

    VectorX() noexcept = default;
    VectorX(const VectorX& other) = default;
    auto operator=(const VectorX& other) -> VectorX& = default;
    VectorX(VectorX&& other) noexcept = default;
    auto operator=(VectorX&& other) noexcept -> VectorX& = default;
    ~VectorX() noexcept = default;

    [[using gnu: pure, always_inline]]
    auto get_allocator() const noexcept -> allocator_type {
        return m_data.get_allocator();
    }

    [[using gnu: always_inline]]
    explicit VectorX(const allocator_type& alloc) noexcept
        : m_data(alloc) {}

    [[using gnu: always_inline]]
    explicit VectorX(size_type size, const allocator_type& alloc = {})
        : m_data(size, alloc) {
        m_data.shrink_to_fit();
    }

    [[using gnu: always_inline]]
    explicit VectorX(size_type size, const_reference value, const allocator_type& alloc = {})
        : m_data(size, value, alloc) {
        m_data.shrink_to_fit();
    }

    [[using gnu: always_inline]]
    VectorX(std::initializer_list<value_type> data, const allocator_type& alloc = {})
        : m_data(std::move(data), alloc) {}

    [[using gnu: always_inline]]
    explicit VectorX(std::vector<value_type, allocator_type> data, const allocator_type& alloc = {})
        : m_data(std::move(data), alloc) {
        m_data.shrink_to_fit();
    }

    template <typename U>
    [[using gnu: always_inline]]
    explicit VectorX(std::span<U> data, const allocator_type& alloc = {})
        requires std::is_same_v<std::remove_const_t<U>, value_type>
        : m_data(data.begin(), data.end(), alloc) {
        m_data.shrink_to_fit();
    }

    template <typename U>
    [[using gnu: always_inline]]
    explicit VectorX(VectorView<U> vector_view, const allocator_type& alloc = {})
        requires std::is_same_v<std::remove_const_t<U>, value_type>
        : m_data(vector_view.size(), alloc) {
        m_data.shrink_to_fit();
        for (size_type i{0}; i < vector_view.size(); ++i) {
            operator[](i) = vector_view[i];
        }
    }

    template <typename U>
    [[using gnu: always_inline]]
    explicit VectorX(MatrixView<U> matrix_view, const allocator_type& alloc = {})
        requires std::is_same_v<std::remove_const_t<U>, value_type>
        : m_data(matrix_view.size(), alloc) {
#if BASJOO_CHECK_PARAMS == 1
        if (matrix_view.nrows() != 1 && matrix_view.ncols() != 1) [[unlikely]] {
            throw std::out_of_range("this matrix does not covert to a vector.");
        }
#endif
        m_data.shrink_to_fit();
        for (size_type i = 0; i < matrix_view.size(); ++i) {
            operator[](i) = matrix_view[i];
        }
    }

    [[using gnu: pure, always_inline]]
    operator std::vector<value_type, allocator_type>() const& {
        return std::vector<value_type, allocator_type>(m_data.cbegin(), m_data.cend());
    }

    [[using gnu: pure, always_inline]]
    operator std::vector<value_type, allocator_type>() && noexcept {
        return m_data;
    }

    [[using gnu: pure, always_inline]]
    operator std::span<value_type>() noexcept {
        return std::span<value_type>(m_data);
    }

    [[using gnu: pure, always_inline]]
    operator std::span<const value_type>() const noexcept {
        return std::span<const value_type>(m_data);
    }

    [[using gnu: pure, always_inline]]
    operator VectorView<value_type>() noexcept {
        return VectorView<value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline]]
    operator VectorView<const value_type>() const noexcept {
        return VectorView<const value_type>(m_data.data(), m_data.size());
    }

    [[using gnu: pure, always_inline, leaf]]
    auto size() const noexcept -> size_type {
        return m_data.size();
    }

    [[using gnu: pure, always_inline, leaf]]
    static constexpr auto stride() noexcept -> size_type {
        return 1;
    }

    [[using gnu: pure, always_inline]]
    auto data() noexcept -> pointer {
        return m_data.data();
    }
    [[using gnu: pure, always_inline]]
    auto data() const noexcept -> const_pointer {
        return m_data.data();
    }

    [[using gnu: always_inline]]
    auto resize(size_type size) -> void {
        m_data.resize(size);
        m_data.shrink_to_fit();
        return;
    }

    [[using gnu: always_inline]]
    auto assign(size_type size, const_reference value) -> void {
        m_data.assign(size, value);
        m_data.shrink_to_fit();
        return;
    }

    [[using gnu: always_inline, hot]]
    auto fill(const_reference value) noexcept -> void {
        std::ranges::fill_n(data(), size(), value);
        return;
    }

    auto empty() const noexcept -> bool { return m_data.empty(); }

    [[using gnu: pure, always_inline, hot]]
    auto operator[](size_type i) noexcept -> reference {
        return m_data[i];
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator[](size_type i) const noexcept -> const_reference {
        return m_data[i];
    }

    [[using gnu: pure, always_inline, hot]]
    auto coeff(size_type i) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range.");
        }
#endif
        return m_data[i];
    }

    [[using gnu: always_inline, hot]]
    auto updateCoeff(size_type i, const_reference value) const noexcept(!BASJOO_CHECK_PARAMS)
        -> void {
#if BASJOO_CHECK_PARAMS == 1
        if (i >= size()) [[unlikely]] {
            throw std::out_of_range("Vector index out of range.");
        }
#endif
        m_data[i] = value;
        return;
    }

    [[using gnu: pure, always_inline]]
    auto view(std::pair<size_type, size_type> range) noexcept -> VectorView<value_type> {
        range.second = std::min(range.second, size());
        return VectorView<value_type>{
            data() + range.first * stride(), range.second - range.first, stride()
        };
    }

    [[using gnu: pure, always_inline]]
    auto view(std::pair<size_type, size_type> range) const noexcept
        -> VectorView<const value_type> {
        range.second = std::min(range.second, size());
        return VectorView<value_type>{
            data() + range.first * stride(), range.second - range.first, stride()
        };
    }

    [[using gnu: pure, always_inline]]
    auto view() noexcept -> VectorView<value_type> {
        return view({0, std::numeric_limits<value_type>::max()});
    }

    [[using gnu: pure, always_inline]]
    auto view() const noexcept -> VectorView<const value_type> {
        return view({0, std::numeric_limits<value_type>::max()});
    }

    [[using gnu: always_inline, hot]]
    auto operator+=(const VectorX& obj) noexcept(!BASJOO_CHECK_PARAMS) -> VectorX& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::+=: sizes do not match.");
        }
#endif
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
    auto operator+(const VectorX& obj) const& -> VectorX {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::+: sizes do not match.");
        }
#endif
        VectorX result{*this};
        result += obj;
        return result;
    }

    [[using gnu: always_inline, hot]]
    auto operator+(VectorX&& obj) const& noexcept(!BASJOO_CHECK_PARAMS) -> VectorX&& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::+: sizes do not match.");
        }
#endif
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline, hot]]
    auto operator+(const VectorX& obj) && noexcept(!BASJOO_CHECK_PARAMS) -> VectorX&& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::+: sizes do not match.");
        }
#endif
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    auto operator+(VectorX&& obj) && noexcept(!BASJOO_CHECK_PARAMS) -> VectorX&& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::+: sizes do not match.");
        }
#endif
        operator+=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    auto operator-=(const VectorX& obj) noexcept(!BASJOO_CHECK_PARAMS) -> VectorX& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::-=: sizes do not match.");
        }
#endif
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
    auto operator-(const VectorX& obj) const& -> VectorX {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::-: sizes do not match.");
        }
#endif
        VectorX result{*this};
        result -= obj;
        return result;
    }

    [[using gnu: always_inline, hot]]
    auto operator-(VectorX&& obj) const& noexcept(!BASJOO_CHECK_PARAMS) -> VectorX&& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::-: sizes do not match.");
        }
#endif
        obj *= -1.0;
        obj += *this;
        return std::move(obj);
    }

    [[using gnu: always_inline, hot]]
    auto operator-(const VectorX& obj) && noexcept(!BASJOO_CHECK_PARAMS) -> VectorX&& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::-: sizes do not match.");
        }
#endif
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto operator-(VectorX&& obj) && noexcept(!BASJOO_CHECK_PARAMS) -> VectorX&& {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::-: sizes do not match.");
        }
#endif
        operator-=(obj);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    auto operator*=(const ScalarArithmetic auto& fac) noexcept -> VectorX& {
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
    auto operator*(const ScalarArithmetic auto& fac) const& -> VectorX {
        VectorX result{*this};
        result *= fac;
        return result;
    }

    [[using gnu: always_inline, hot]]
    auto operator*(const ScalarArithmetic auto& fac) && noexcept -> VectorX&& {
        operator*=(fac);
        return std::move(*this);
    }

    [[using gnu: always_inline, hot]]
    auto operator/=(const ScalarArithmetic auto& den) noexcept -> VectorX& {
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
    auto operator/(const ScalarArithmetic auto& den) const& -> VectorX {
        VectorX result{*this};
        result /= den;
        return result;
    }

    [[using gnu: always_inline, hot]]
    auto operator/(const ScalarArithmetic auto& den) && noexcept -> VectorX&& {
        operator/=(den);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator-() const& -> VectorX {
        VectorX result{*this};
        result *= -1.0;
        return result;
    }

    [[using gnu: always_inline, hot]]
    auto operator-() && noexcept -> VectorX&& {
        operator*=(-1.0);
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    auto operator==(const VectorX& other) const noexcept -> bool {
        if (size() != other.size()) [[unlikely]] {
            return false;
        }
        const size_type n{size()};
        for (size_type i{0}; i < n; ++i) {
            if (m_data[i] != other.m_data[i]) {
                return false;
            }
        }
        return true;
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto normalized() const noexcept -> VectorX {
        return *this / euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    auto euclidean() const noexcept -> detail::DenseNormTraitT<value_type> {
        detail::DenseNormTraitT<value_type> result(0.0);
        if (size() == 1) {
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
    auto euclideanSqr() const noexcept -> detail::DenseNormTraitT<value_type> {
        detail::DenseNormTraitT<value_type> result(0.0);
        if (size() == 1) {
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
    auto euclideanTo(const VectorX& obj) const noexcept(!BASJOO_CHECK_PARAMS)
        -> detail::DenseNormTraitT<value_type> {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("Vectors must have the same size.");
        }
#endif
        return operator-(obj).euclidean();
    }

    [[using gnu: pure, always_inline, hot]]
    auto identicalTo(VectorView<const value_type> obj, double tol = 1E-8) const noexcept -> bool {
        if (size() != obj.size()) {
            return false;
        }
        size_type n{size()};
        detail::DenseNormTraitT<value_type> test(0.0);
        for (size_type i{0}; i < n; ++i) {
            test = std::max(
                test, std::abs(operator[](i) - obj[i]) /
                          std::max(std::abs(operator[](i)), static_cast<decltype(test)>(1.0))
            );
        }
        return test < tol;
    }

    [[using gnu: pure, always_inline, hot]]
    auto orthogonalTo(const VectorX& obj, value_type tol = 1E-8) const noexcept -> bool {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("Vectors must have the same size.");
        }
#endif
        return std::abs(dot(obj)) < tol * tol;
    }

    [[using gnu: always_inline]]
    auto selfConjugated() noexcept -> VectorX& {
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
    auto conjugated() const& -> VectorX {
        VectorX result{*this};
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            result.selfConjugated();
        }
        return result;
    }

    [[using gnu: always_inline]]
    auto conjugated() && noexcept -> VectorX&& {
        if constexpr (::basjoo::math::isComplexArithmeticV<value_type>) {
            selfConjugated();
        }
        return std::move(*this);
    }

    [[using gnu: pure, always_inline, hot]]
    auto dot(const VectorX& obj) const noexcept(!BASJOO_CHECK_PARAMS) -> value_type {
        value_type result(0.0);
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.size()) [[unlikely]] {
            throw std::invalid_argument("VectorX::dot: vector and vector dimensions do not match");
        }
#endif
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

    template <MatrixOrder ObjOrder>
    [[using gnu: pure, always_inline, hot]]
    auto dot(const MatrixX<value_type, ObjOrder, allocator_type>& obj) const -> VectorX {
#if BASJOO_CHECK_PARAMS == 1
        if (size() != obj.nrows()) [[unlikely]] {
            throw std::invalid_argument("VectorX::dot: Matrix and vector dimensions do not match");
        }
#endif
        const size_type outer_size = obj.ncols();
        VectorX<value_type, allocator_type> result(outer_size, 0.0);
#ifdef BASJOO_USE_BLAS_LAPACK
        [[maybe_unused]] constexpr CBLAS_ORDER cblas_order{
            ObjOrder == ::basjoo::math::MatrixOrder::COL_MAJOR ? CblasColMajor : CblasRowMajor
        };
        [[maybe_unused]] const blasint lda =
            (ObjOrder == ::basjoo::math::MatrixOrder::COL_MAJOR ? outer_size : size());
        [[maybe_unused]] constexpr value_type alpha(1.0), beta(0.0);
        if constexpr (std::is_same_v<value_type, float>) {
            cblas_sgemv(
                cblas_order, CblasNoTrans, outer_size, size(), alpha, obj.data(), lda, data(), 1,
                beta, result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, double>) {
            cblas_dgemv(
                cblas_order, CblasNoTrans, outer_size, size(), alpha, obj.data(), lda, data(), 1,
                beta, result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<float>>) {
            cblas_cgemv(
                cblas_order, CblasNoTrans, outer_size, size(), &alpha, obj.data(), lda, data(), 1,
                &beta, result.data(), 1
            );
        } else if constexpr (std::is_same_v<value_type, std::complex<double>>) {
            cblas_zgemv(
                cblas_order, CblasNoTrans, outer_size, size(), &alpha, obj.data(), lda, data(), 1,
                &beta, result.data(), 1
            );
        } else {
            const size_type n = size();
            for (size_type j{0}; j < outer_size; ++j) {
                for (size_type i{0}; i < n; ++i) {
                    result[j] += operator[](i) * obj[i, j];
                }
            }
        }
#else
        const size_type n = size();
        for (size_type j{0}; j < outer_size; ++j) {
            for (size_type i{0}; i < n; ++i) {
                result[j] += operator[](i) * obj[i, j];
            }
        }
#endif
        return result;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    std::vector<value_type, allocator_type> m_data;
};

template <ScalarArithmetic Scalar, Allocatory Alloc>
[[using gnu: pure, always_inline, hot]]
inline auto operator*(const ScalarArithmetic auto& fac, const VectorX<Scalar, Alloc>& obj)
    -> VectorX<Scalar, Alloc> {
    return obj * fac;
}

template <ScalarArithmetic Scalar, Allocatory Alloc>
[[using gnu: always_inline, hot]]
inline auto operator*(const ScalarArithmetic auto& fac, VectorX<Scalar, Alloc>&& obj) noexcept
    -> VectorX<Scalar, Alloc>&& {
    obj *= fac;
    return std::move(obj);
}

template <typename Char, ScalarArithmetic Scalar, Allocatory Alloc>
inline auto operator<<(std::basic_ostream<Char>& os, const VectorX<Scalar, Alloc>& vector) noexcept
    -> std::basic_ostream<Char>& {
    using value_type = std::remove_const_t<Scalar>;
    using size_type = typename VectorX<Scalar, Alloc>::size_type;
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

template <basjoo::math::ScalarArithmetic Scalar, basjoo::math::Allocatory Alloc>
[[using gnu: pure, always_inline]]
inline auto abs(const basjoo::math::VectorX<Scalar, Alloc>& vector) noexcept
    -> basjoo::math::detail::DenseNormTrait<basjoo::math::VectorX<Scalar, Alloc>> {
    basjoo::math::detail::DenseNormTraitT<basjoo::math::VectorX<Scalar, Alloc>> result(vector);
    if constexpr (basjoo::math::isComplexArithmeticV<Scalar>) {
        std::transform(
            result.data(), result.data() + result.size(), result.data(),
            [](const Scalar& x) constexpr noexcept
                -> basjoo::math::detail::DenseNormTraitT<Scalar> { return std::abs(x); }
        );
    }
    return result;
}

template <basjoo::math::ScalarArithmetic Scalar, basjoo::math::Allocatory Alloc>
[[using gnu: pure, always_inline]]
inline auto conj(const basjoo::math::VectorX<Scalar, Alloc>& vector)
    -> basjoo::math::VectorX<Scalar, Alloc> {
    return vector.conjugated();
}

template <basjoo::math::ScalarArithmetic Scalar, basjoo::math::Allocatory Alloc>
[[using gnu: always_inline]]
inline auto conj(basjoo::math::VectorX<Scalar, Alloc>&& vector) noexcept
    -> basjoo::math::VectorX<Scalar, Alloc>&& {
    vector.selfConjugated();
    return std::move(vector);
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
