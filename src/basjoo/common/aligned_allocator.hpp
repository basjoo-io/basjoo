/**
 * @file aligned_allocator.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief
 * @version 0.1
 * @date 2025-03-24
 *
 * @copyright Copyright (c) 2025 basjoo development team.
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdlib>
#include <limits>
#include <memory>
#include <new>
#include <type_traits>

namespace basjoo::common {

// The widest SIMD register of the build in bytes, the default alignment of an allocation.
inline constexpr std::size_t kSimdAlignment{
#if defined(__AVX512F__)
    64
#elif defined(__AVX__)
    32
#elif defined(__SSE2__) || defined(__ARM_NEON) || defined(_M_X64) || defined(_M_ARM64)
    16
#else
    alignof(std::max_align_t)
#endif
};

// Constant evaluation allocates through std::allocator instead, the only allocation it
// admits; the memory never outlives the evaluation, so its alignment does not matter there.
template <typename T, std::size_t Alignment = kSimdAlignment>
class AlignedAllocator {
  public:
    using value_type = T;
    using pointer = value_type*;
    using const_pointer = const value_type*;
    using reference = value_type&;
    using const_reference = const value_type&;
    using size_type = std::size_t;
    using difference_type = std::ptrdiff_t;

    template <class U>
    struct rebind {
        using other = AlignedAllocator<U, Alignment>;
    };

    constexpr AlignedAllocator() noexcept = default;
    constexpr AlignedAllocator(const AlignedAllocator& other) noexcept = default;
    constexpr auto operator=(const AlignedAllocator& other) noexcept -> AlignedAllocator& = default;
    constexpr AlignedAllocator(AlignedAllocator&& other) noexcept = default;
    constexpr auto operator=(AlignedAllocator&& other) noexcept -> AlignedAllocator& = default;
    constexpr ~AlignedAllocator() noexcept = default;

    template <typename U>
    constexpr AlignedAllocator(
        [[maybe_unused]] const AlignedAllocator<U, Alignment>& other
    ) noexcept {}

    template <typename U, std::size_t OtherAlignment>
    constexpr auto operator==(
        [[maybe_unused]] const AlignedAllocator<U, OtherAlignment>& other
    ) const noexcept -> bool {
        return std::is_same_v<T, U> && Alignment == OtherAlignment;
    }

    [[using gnu: always_inline]]
    static constexpr auto max_size() noexcept -> size_type {
        return std::numeric_limits<std::ptrdiff_t>::max() / sizeof(T);
    }

    // Custom allocator intentionally uses low-level memory allocation
    // NOLINTBEGIN(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)

    [[using gnu: always_inline, hot]]
    constexpr auto allocate(size_type num) -> pointer {
        if consteval {
            return std::allocator<value_type>{}.allocate(num);
        }
        constexpr size_type alignment{std::max(Alignment, alignof(value_type))};
        size_type bytes{num * sizeof(value_type)};
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
        return static_cast<pointer>(ptr);
    }

    [[using gnu: always_inline, hot]]
    constexpr auto deallocate(pointer p, size_type num) noexcept -> void {
        if consteval {
            std::allocator<value_type>{}.deallocate(p, num);
            return;
        }
#ifdef _MSC_VER
        _aligned_free(p);
#else
        std::free(p);
#endif
        return;
    }

    // NOLINTEND(cppcoreguidelines-owning-memory,cppcoreguidelines-no-malloc,hicpp-no-malloc)

    constexpr auto select_on_container_copy_construction() const noexcept -> AlignedAllocator {
        return *this;
    }
};

} // namespace basjoo::common
