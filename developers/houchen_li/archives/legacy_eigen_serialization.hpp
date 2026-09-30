/**
 * @file eigen_serialization.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2023-07-27
 *
 * @copyright Copyright (c) 2023 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <cstddef>
#include <span>
#include <type_traits>
#include <vector>

#include "Eigen/Core"
#include "Eigen/SparseCore"

#include "zpp_bits.h"

namespace Eigen {

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
[[using gnu: always_inline]]
constexpr auto serialize(
    auto& archive, const Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m
) -> zpp::bits::errc {
    const Eigen::Index rows{m.rows()};
    const Eigen::Index cols{m.cols()};
    return archive(
        rows, cols, zpp::bits::bytes(std::span(m.data(), static_cast<std::size_t>(rows * cols)))
    );
}

template <typename Scalar, int Rows, int Cols, int Options, int MaxRows, int MaxCols>
[[using gnu: always_inline]]
constexpr auto serialize(auto& archive, Matrix<Scalar, Rows, Cols, Options, MaxRows, MaxCols>& m)
    -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        const Eigen::Index rows{m.rows()};
        const Eigen::Index cols{m.cols()};
        return archive(
            rows, cols, zpp::bits::bytes(std::span(m.data(), static_cast<std::size_t>(rows * cols)))
        );
    } else {
        Eigen::Index rows;
        Eigen::Index cols;
        if (auto result = archive(rows, cols); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        m.resize(rows, cols);
        return archive(
            zpp::bits::bytes(std::span(m.data(), static_cast<std::size_t>(rows * cols)))
        );
    }
}

template <typename Scalar, typename Index>
[[using gnu: always_inline]]
constexpr auto serialize(auto& archive, const Triplet<Scalar, Index>& t) -> zpp::bits::errc {
    const Index row{t.row()};
    const Index col{t.col()};
    const Scalar value{t.value()};
    return archive(row, col, value);
}

template <typename Scalar, typename Index>
[[using gnu: always_inline]]
constexpr auto serialize(auto& archive, Triplet<Scalar, Index>& t) -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        const Index row{t.row()};
        const Index col{t.col()};
        const Scalar value{t.value()};
        return archive(row, col, value);
    } else {
        Index row;
        Index col;
        Scalar value;
        if (auto result = archive(row, col, value); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        t = Triplet<Scalar, Index>(row, col, value);
        return {};
    }
}

template <typename Scalar, int Options, typename Index>
[[using gnu: always_inline]]
constexpr auto serialize(auto& archive, const SparseMatrix<Scalar, Options, Index>& m)
    -> zpp::bits::errc {
    const Index rows{m.rows()};
    const Index cols{m.cols()};
    const Index non_zeros{m.nonZeros()};
    const Index outer_size = m.outerSize();
    std::vector<Triplet<Scalar>> triplets;
    triplets.reserve(static_cast<std::size_t>(non_zeros));
    for (Index i{0}; i < outer_size; ++i) {
        for (typename SparseMatrix<Scalar, Options, Index>::InnerIterator it(m, i); it; ++it) {
            triplets.emplace_back(it.row(), it.col(), it.value());
        }
    }
    return archive(rows, cols, non_zeros, triplets);
}

template <typename Scalar, int Options, typename Index>
[[using gnu: always_inline]]
constexpr auto serialize(auto& archive, SparseMatrix<Scalar, Options, Index>& m)
    -> zpp::bits::errc {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    if constexpr (archive_type::kind() == zpp::bits::kind::out) {
        const Index rows{m.rows()};
        const Index cols{m.cols()};
        const Index non_zeros{m.nonZeros()};
        const Index outer_size{m.outerSize()};
        std::vector<Triplet<Scalar>> triplets;
        triplets.reserve(static_cast<std::size_t>(non_zeros));
        for (Index i{0}; i < outer_size; ++i) {
            for (typename SparseMatrix<Scalar, Options, Index>::InnerIterator it(m, i); it; ++it) {
                triplets.emplace_back(it.row(), it.col(), it.value());
            }
        }
        return archive(rows, cols, non_zeros, triplets);
    } else {
        Index rows;
        Index cols;
        Index non_zeros;
        std::vector<Triplet<Scalar>> triplets;
        if (auto result = archive(rows, cols, non_zeros); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        triplets.reserve(static_cast<std::size_t>(non_zeros));
        if (auto result = archive(triplets); zpp::bits::failure(result)) [[unlikely]] {
            return result;
        }
        m.resize(rows, cols);
        m.reserve(non_zeros);
        m.setFromTriplets(triplets.begin(), triplets.end());
        return {};
    }
}

} // namespace Eigen
