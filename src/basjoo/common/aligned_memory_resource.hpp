/**
 * @file aligned_memory_resource.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-06-25
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <memory_resource>
#include <new>
#include <stdexcept>

#include "zpp_bits.h"

namespace basjoo::common::pmr {

class AlignedMemoryResource final : public std::pmr::memory_resource {
  public:
    constexpr AlignedMemoryResource(const AlignedMemoryResource& other) noexcept = default;
    constexpr auto operator=(const AlignedMemoryResource& other) noexcept
        -> AlignedMemoryResource& = default;
    constexpr AlignedMemoryResource(AlignedMemoryResource&& other) noexcept = default;
    constexpr auto operator=(AlignedMemoryResource&& other) noexcept
        -> AlignedMemoryResource& = default;
    ~AlignedMemoryResource() noexcept override = default;

    [[using gnu: always_inline]]
    constexpr explicit AlignedMemoryResource(std::size_t alignment = 32)
        : m_alignment{alignment} {
        if ((alignment & (alignment - 1)) != 0) [[unlikely]] {
            throw std::invalid_argument("Alignment must be a power of 2.");
        }
    }

  protected:
    [[using gnu: always_inline, hot]]
    auto do_allocate(std::size_t bytes, std::size_t alignment) -> void* override {
        alignment = std::max(m_alignment, alignment);
        bytes = bytes + (alignment - bytes % alignment) % alignment;
        void* ptr{nullptr};
#ifdef _MSC_VER
        ptr = _aligned_malloc(bytes, alignment);
#else
        ptr = std::aligned_alloc(alignment, bytes);
#endif
        if (ptr == nullptr) [[unlikely]] {
            throw std::bad_alloc();
        }
        return ptr;
    }

    [[using gnu: always_inline, hot]]
    auto do_deallocate(
        void* ptr, [[maybe_unused]] std::size_t bytes, [[maybe_unused]] std::size_t alignment
    ) -> void override {
#ifdef _MSC_VER
        _aligned_free(ptr);
#else
        std::free(ptr);
#endif
        return;
    }

    [[using gnu: pure, always_inline, hot]]
    auto do_is_equal(const std::pmr::memory_resource& other) const noexcept -> bool override {
        return this == &other;
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    std::size_t m_alignment;
};

template <std::size_t Alignment = 32>
[[using gnu: always_inline, hot]]
inline constexpr auto getAlignedMemoryResource() noexcept -> AlignedMemoryResource* {
    static AlignedMemoryResource memory_resource(Alignment);
    return &memory_resource;
}

} // namespace basjoo::common::pmr
