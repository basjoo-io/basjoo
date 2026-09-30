/**
 * @file math/dense/quaternion.hpp
 * @author Houchen Li (houchen_li@hotmail.com)
 * @brief Hamilton quaternion of spatial rotations, convertible to and from Mat33.
 * @version 0.1
 * @date 2026-11-04
 *
 * @copyright Copyright (c) 2026 basjoo development team
 *            All rights reserved.
 *
 */

#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <concepts>
#include <cstddef>
#include <limits>
#include <mdspan>
#include <stdexcept>
#include <tuple>
#include <type_traits>
#include <utility>

#if defined(__x86_64__) || defined(_M_X64)
#include <immintrin.h>
#elif defined(__aarch64__) || defined(_M_ARM64)
#include <arm_neon.h>
#endif

#include "zpp_bits.h"

#include "basjoo/math/concepts.hpp"
#include "basjoo/math/dense/dense_arithmetic_base.hpp"
#include "basjoo/math/dense/dense_lanes.hpp"
#include "basjoo/math/dense/dense_trait.hpp"
#include "basjoo/math/dense/mat33.hpp"
#include "basjoo/math/dense/vec3.hpp"

namespace basjoo::math {

// x i + y j + z k + w with Hamilton's ij = k, stored and constructed as (x, y, z, w) like tf2 and
// the ROS messages. As a rotation it maps v to q v q*, so that q * p applies p first. Unit norm is
// not an invariant: the algebra takes any quaternion; rotate(), slerp() and Mat33::fromQuaternion()
// require a unit one, checked against √ε when parameters are; the axis, angle, rotation vector,
// roll-pitch-yaw and angular distance do not depend on the norm. q and -q are the same rotation but
// compare unequal. Default construction leaves the components uninitialized, as Vec3 does. The
// sum, difference, negation and scaling are those of DenseArithmeticBase, element-wise over the
// four components at data().
template <std::floating_point Scalar>
class alignas(4 * sizeof(Scalar)) Quaternion final : public DenseArithmeticBase {
  public:
    using value_type = Scalar;
    using size_type = std::size_t;
    using extents_type = std::extents<size_type, 4>;

    [[using gnu: always_inline]]
    constexpr Quaternion() noexcept = default;
    [[using gnu: always_inline]]
    constexpr Quaternion(value_type cx, value_type cy, value_type cz, value_type cw) noexcept
        : m_data{cx, cy, cz, cw} {}
    [[using gnu: always_inline]]
    constexpr Quaternion(const Quaternion& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(const Quaternion& other) noexcept -> Quaternion& = default;
    [[using gnu: always_inline]]
    constexpr Quaternion(Quaternion&& other) noexcept = default;
    [[using gnu: always_inline]]
    constexpr auto operator=(Quaternion&& other) noexcept -> Quaternion& = default;
    ~Quaternion() noexcept = default;

    template <std::floating_point U>
    [[using gnu: always_inline]]
    constexpr explicit Quaternion(const Quaternion<U>& other) noexcept
        : m_data{
              static_cast<value_type>(other.x()), static_cast<value_type>(other.y()),
              static_cast<value_type>(other.z()), static_cast<value_type>(other.w())
          } {}

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto x(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[0];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto y(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[1];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto z(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[2];
    }

    template <typename Self>
    [[using gnu: pure, always_inline]]
    constexpr auto w(this Self&& self) noexcept -> decltype(auto) {
        return self.m_data[3];
    }

    [[using gnu: const, always_inline, leaf]]
    static constexpr auto size() noexcept -> size_type {
        return 4;
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() noexcept -> value_type* {
        return m_data.data();
    }

    [[using gnu: pure, always_inline]]
    constexpr auto data() const noexcept -> const value_type* {
        return m_data.data();
    }

    [[using gnu: const, always_inline]]
    static constexpr auto identity() noexcept -> Quaternion {
        return Quaternion{value_type(0.0), value_type(0.0), value_type(0.0), value_type(1.0)};
    }

    // The rotation by `angle` about `axis`, which need not be of unit length. A zero axis throws
    // when parameters are checked and yields NaNs otherwise.
    // Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
    template <std::integral Index, Allocatory Alloc>
    [[using gnu: always_inline]]
    static constexpr auto fromAxisAngle(
        const Vector<value_type, Index, 3, Alloc>& axis, value_type angle
    ) noexcept(!BASJOO_CHECK_PARAMS) -> Quaternion {
#if BASJOO_CHECK_PARAMS == 1
        if (axis.euclideanSqr() == value_type(0.0)) [[unlikely]] {
            throw std::invalid_argument("Quaternion::fromAxisAngle: zero-length axis.");
        }
#endif
        const value_type half_angle{value_type(0.5) * angle};
        const value_type s{std::sin(half_angle) / axis.euclidean()};
        return Quaternion{s * axis.x(), s * axis.y(), s * axis.z(), std::cos(half_angle)};
    }

    // The exponential map: the rotation by the length of the rotation vector (rx, ry, rz) about its
    // direction.
    [[using gnu: pure, always_inline]]
    static constexpr auto fromRotationVector(value_type rx, value_type ry, value_type rz) noexcept
        -> Quaternion {
        constexpr value_type kEps{std::numeric_limits<value_type>::epsilon()};
        const value_type theta_sqr{rx * rx + ry * ry + rz * rz};
        // Below ε, sin(θ / 2) / θ = 1 / 2 - θ^2 / 48 and cos(θ / 2) = 1 - θ^2 / 8 round to their
        // leading terms, while the general form divides 0 by 0 at θ = 0.
        if (theta_sqr < kEps * kEps) {
            const value_type half(0.5);
            return Quaternion{half * rx, half * ry, half * rz, value_type(1.0)};
        }
        const value_type theta{std::sqrt(theta_sqr)};
        const value_type half_theta{value_type(0.5) * theta};
        const value_type s{std::sin(half_theta) / theta};
        return Quaternion{s * rx, s * ry, s * rz, std::cos(half_theta)};
    }

    // REP-103's roll, pitch and yaw about the fixed x, y and z axes, applied in this order:
    // R = Rz(yaw) Ry(pitch) Rx(roll).
    [[using gnu: pure, always_inline]]
    static constexpr auto fromRpy(value_type roll, value_type pitch, value_type yaw) noexcept
        -> Quaternion {
        const value_type half(0.5);
        const value_type cr{std::cos(half * roll)};
        const value_type sr{std::sin(half * roll)};
        const value_type cp{std::cos(half * pitch)};
        const value_type sp{std::sin(half * pitch)};
        const value_type cy{std::cos(half * yaw)};
        const value_type sy{std::sin(half * yaw)};
        return Quaternion{
            sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy
        };
    }

    // The minimal rotation turning the direction of `from` into that of `to`, the one of
    // Mat33::fromTwoVectors(); neither needs to be of unit length. A zero vector throws when
    // parameters are checked and yields NaNs otherwise.
    // Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
    template <std::integral Index, Allocatory Alloc>
    static constexpr auto fromTwoVectors(
        const Vector<value_type, Index, 3, Alloc>& from,
        const Vector<value_type, Index, 3, Alloc>& to
    ) noexcept(!BASJOO_CHECK_PARAMS) -> Quaternion {
        using vector_type = Vector<value_type, Index, 3, Alloc>;

#if BASJOO_CHECK_PARAMS == 1
        if (from.euclideanSqr() == value_type(0.0) || to.euclideanSqr() == value_type(0.0))
            [[unlikely]] {
            throw std::invalid_argument("Quaternion::fromTwoVectors: zero-length vector.");
        }
#endif
        const vector_type a{from.normalized()};
        const vector_type b{to.normalized()};
        // (a x b, 1 + a.b), once normalized, rotates a onto b.
        const value_type real{value_type(1.0) + a.dot(b)};
        if (real <= std::numeric_limits<value_type>::epsilon()) {
            // Antiparallel: half a turn about an axis perpendicular to `from`, its cross product
            // with the coordinate axis least aligned with it.
            const value_type ax{std::abs(a.x())};
            const value_type ay{std::abs(a.y())};
            const value_type az{std::abs(a.z())};
            const value_type one(1.0);
            const value_type zero(0.0);
            const vector_type coordinate_axis{
                ax <= ay && ax <= az ? vector_type{one, zero, zero}
                : ay <= az           ? vector_type{zero, one, zero}
                                     : vector_type{zero, zero, one}
            };
            const vector_type u{a.cross(coordinate_axis).normalized()};
            return Quaternion{u.x(), u.y(), u.z(), zero};
        }
        const vector_type v{a.cross(b)};
        const value_type n{std::sqrt(real * real + v.euclideanSqr())};
        return Quaternion{v.x() / n, v.y() / n, v.z() / n, real / n};
    }

    // The quaternion with w >= 0 of a rotation matrix, by Shepperd's method: from the trace when it
    // is positive, else from the largest diagonal element, so that the square root taken is at
    // least 1. A matrix that is not a rotation within √ε throws when parameters are checked.
    // Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
    template <std::integral Index, DenseLayout Layout, Allocatory Alloc>
    static constexpr auto fromRotationMatrix(
        const Matrix<value_type, Index, 3, 3, Layout, Alloc>& m
    ) noexcept(!BASJOO_CHECK_PARAMS) -> Quaternion {
#if BASJOO_CHECK_PARAMS == 1
        if (!m.isRotation(std::sqrt(std::numeric_limits<value_type>::epsilon()))) [[unlikely]] {
            throw std::invalid_argument("Quaternion::fromRotationMatrix: not a rotation matrix.");
        }
#endif
        const value_type one(1.0);
        const value_type half(0.5);
        const value_type trace{m.trace()};
        // Every branch assigns q.
        Quaternion q;
        if (trace > value_type(0.0)) {
            // s = 2 w
            const value_type s{std::sqrt(trace + one)};
            const value_type f{half / s};
            q = Quaternion{
                (m[2, 1] - m[1, 2]) * f, (m[0, 2] - m[2, 0]) * f, (m[1, 0] - m[0, 1]) * f, half * s
            };
        } else if (m[0, 0] >= m[1, 1] && m[0, 0] >= m[2, 2]) {
            // s = 2 |x|
            const value_type s{std::sqrt(one + m[0, 0] - m[1, 1] - m[2, 2])};
            const value_type f{half / s};
            q = Quaternion{
                half * s, (m[1, 0] + m[0, 1]) * f, (m[0, 2] + m[2, 0]) * f, (m[2, 1] - m[1, 2]) * f
            };
        } else if (m[1, 1] >= m[2, 2]) {
            // s = 2 |y|
            const value_type s{std::sqrt(one + m[1, 1] - m[0, 0] - m[2, 2])};
            const value_type f{half / s};
            q = Quaternion{
                (m[1, 0] + m[0, 1]) * f, half * s, (m[2, 1] + m[1, 2]) * f, (m[0, 2] - m[2, 0]) * f
            };
        } else {
            // s = 2 |z|
            const value_type s{std::sqrt(one + m[2, 2] - m[0, 0] - m[1, 1])};
            const value_type f{half / s};
            q = Quaternion{
                (m[0, 2] + m[2, 0]) * f, (m[2, 1] + m[1, 2]) * f, half * s, (m[1, 0] - m[0, 1]) * f
            };
        }
        // The branch taken decides the sign; q and -q are the same rotation.
        return q.w() < value_type(0.0) ? -q : q;
    }

    // The unit axis, oriented so that the angle is within [0, π]; (1, 0, 0) for the identity.
    [[using gnu: pure, always_inline]]
    constexpr auto axis() const noexcept -> Vector<value_type, std::size_t, 3> {
        const value_type n{std::sqrt(x() * x() + y() * y() + z() * z())};
        if (n == value_type(0.0)) [[unlikely]] {
            return Vector<value_type, std::size_t, 3>{
                value_type(1.0), value_type(0.0), value_type(0.0)
            };
        }
        const value_type f{(w() < value_type(0.0) ? value_type(-1.0) : value_type(1.0)) / n};
        return Vector<value_type, std::size_t, 3>{x() * f, y() * f, z() * f};
    }

    // Within [0, π], whichever of q and -q *this is.
    [[using gnu: pure, always_inline]]
    constexpr auto angle() const noexcept -> value_type {
        return value_type(2.0) *
               std::atan2(std::sqrt(x() * x() + y() * y() + z() * z()), std::abs(w()));
    }

    // The logarithm map: the axis scaled by the angle, within [0, π], to be taken apart as
    // `const auto [rx, ry, rz] = q.rotationVector();`.
    [[using gnu: pure, always_inline]]
    constexpr auto rotationVector() const noexcept
        -> std::tuple<value_type, value_type, value_type> {
        constexpr value_type kEps{std::numeric_limits<value_type>::epsilon()};
        const value_type n_sqr{x() * x() + y() * y() + z() * z()};
        // The angle over n is 2 atan2(n, |w|) / n, signed like w so that the angle stays within
        // [0, π] for q and -q alike. Once n / |w| is below ε it rounds to 2 / |w|: the next term,
        // -2 n^2 / (3 |w|^3), is too small to count.
        if (n_sqr <= kEps * kEps * w() * w()) {
            const value_type f{value_type(2.0) / w()};
            return {x() * f, y() * f, z() * f};
        }
        const value_type n{std::sqrt(n_sqr)};
        const value_type f{value_type(2.0) * std::atan2(n, std::abs(w())) / n};
        const value_type signed_f{w() < value_type(0.0) ? -f : f};
        return {x() * signed_f, y() * signed_f, z() * signed_f};
    }

    // The angles of fromRpy(), to be taken apart as `const auto [roll, pitch, yaw] = q.rpy();`:
    // pitch within [-π/2, π/2], roll and yaw within [-π, π]; at gimbal lock yaw is 0 and roll
    // takes the combined angle. These are the formulas of Eigen's
    // eulerAngles(2, 1, 0) without its folding of the first angle into [0, π]: yaw comes first and
    // roll from what is left once yaw is undone, so that the three angles rebuild the rotation to
    // rounding even next to the lock. The matrix entries are kept as forms of degree two in q, all
    // scaled by |q|^2, which the angles cancel.
    [[using gnu: pure]]
    constexpr auto rpy() const noexcept -> std::tuple<value_type, value_type, value_type> {
        constexpr value_type kEps{std::numeric_limits<value_type>::epsilon()};
        const value_type two(2.0);
        const value_type ww{w() * w()};
        const value_type xx{x() * x()};
        const value_type yy{y() * y()};
        const value_type zz{z() * z()};
        const value_type m00{ww + xx - yy - zz};
        const value_type m01{two * (x() * y() - w() * z())};
        const value_type m02{two * (x() * z() + w() * y())};
        const value_type m10{two * (x() * y() + w() * z())};
        const value_type m11{ww - xx + yy - zz};
        const value_type m12{two * (y() * z() - w() * x())};
        const value_type m20{two * (x() * z() - w() * y())};
        const value_type m21{two * (y() * z() + w() * x())};
        const value_type m22{ww - xx - yy + zz};

        const value_type pitch{std::atan2(-m20, std::hypot(m21, m22))};
        // |q|^2 cos(pitch). At the lock it is down to rounding, a few ε |q|^2, and yaw is
        // undetermined.
        const value_type cos_pitch{std::hypot(m00, m10)};
        value_type yaw(0.0);
        value_type cos_yaw(1.0);
        value_type sin_yaw(0.0);
        if (cos_pitch > value_type(8.0) * kEps * (ww + xx + yy + zz)) {
            yaw = std::atan2(m10, m00);
            cos_yaw = m00 / cos_pitch;
            sin_yaw = m10 / cos_pitch;
        }
        // Rz(-yaw) R = Ry(pitch) Rx(roll), whose second row is (0, cos(roll), -sin(roll)).
        const value_type roll{
            std::atan2(sin_yaw * m02 - cos_yaw * m12, cos_yaw * m11 - sin_yaw * m01)
        };
        return {roll, pitch, yaw};
    }

    // DenseArithmeticBase's scaling, which the Hamilton product of the same name would hide.
    using DenseArithmeticBase::operator*=;
    using DenseArithmeticBase::operator*;

    // The Hamilton product, *this on the left. With a = *this and b = obj, the SIMD kernels form
    // aw b + ax (bw, -bz, by, -bx) + ay (bz, bw, -bx, -by) + az (-by, bx, bw, -bz), which adds the
    // terms of each component in the order of the expressions below. Both operands are loaded
    // before the result is stored, so that obj may be *this.
    [[using gnu: always_inline, hot]]
    constexpr auto operator*=(const Quaternion& obj) noexcept -> Quaternion& {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 a{_mm_load_ps(data())};
                const __m128 b{_mm_load_ps(obj.data())};
                // The swizzles of b, their signs flipped by XOR.
                const __m128 t1{_mm_xor_ps(
                    _mm_shuffle_ps(b, b, _MM_SHUFFLE(0, 1, 2, 3)),
                    _mm_set_ps(-0.0F, 0.0F, -0.0F, 0.0F)
                )};
                const __m128 t2{_mm_xor_ps(
                    _mm_shuffle_ps(b, b, _MM_SHUFFLE(1, 0, 3, 2)),
                    _mm_set_ps(-0.0F, -0.0F, 0.0F, 0.0F)
                )};
                const __m128 t3{_mm_xor_ps(
                    _mm_shuffle_ps(b, b, _MM_SHUFFLE(2, 3, 0, 1)),
                    _mm_set_ps(-0.0F, 0.0F, 0.0F, -0.0F)
                )};
                _mm_store_ps(
                    data(),
                    _mm_add_ps(
                        _mm_add_ps(
                            _mm_add_ps(
                                _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(3, 3, 3, 3)), b),
                                _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(0, 0, 0, 0)), t1)
                            ),
                            _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(1, 1, 1, 1)), t2)
                        ),
                        _mm_mul_ps(_mm_shuffle_ps(a, a, _MM_SHUFFLE(2, 2, 2, 2)), t3)
                    )
                );
                return *this;
#elif defined(__ARM_NEON)
                const float32x4_t a{vld1q_f32(data())};
                const float32x4_t b{vld1q_f32(obj.data())};
                // The swizzles of b by rev64 and ext, (by, bx, bw, bz) and (bz, bw, bx, by), their
                // signs flipped by XOR.
                const float32x4_t swapped{vrev64q_f32(b)};
                const float32x4_t t1{vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(vextq_f32(swapped, swapped, 2)),
                    vreinterpretq_u32_u64(vdupq_n_u64(0x8000000000000000ULL))
                ))};
                const float32x4_t t2{vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(vextq_f32(b, b, 2)),
                    vcombine_u32(vdup_n_u32(0U), vdup_n_u32(0x80000000U))
                ))};
                const float32x4_t t3{vreinterpretq_f32_u32(veorq_u32(
                    vreinterpretq_u32_f32(swapped),
                    vcombine_u32(vcreate_u32(0x80000000ULL), vcreate_u32(0x8000000000000000ULL))
                ))};
                vst1q_f32(
                    data(), vaddq_f32(
                                vaddq_f32(
                                    vaddq_f32(vmulq_laneq_f32(b, a, 3), vmulq_laneq_f32(t1, a, 0)),
                                    vmulq_laneq_f32(t2, a, 1)
                                ),
                                vmulq_laneq_f32(t3, a, 2)
                            )
                );
                return *this;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                // *this is loaded whole and its components broadcast from (ax, ay | ax, ay) and
                // (az, aw | az, aw): read one by one, GCC reloads them from a stack copy of *this,
                // which stalls on store forwarding.
                const __m256d a{_mm256_load_pd(data())};
                const __m256d a_lo{_mm256_permute2f128_pd(a, a, 0x00)};
                const __m256d a_hi{_mm256_permute2f128_pd(a, a, 0x11)};
                const __m256d b{_mm256_load_pd(obj.data())};
                // (bz, bw, bx, by) swaps the halves of b; swapping within them too gives
                // (bw, bz, by, bx), and swapping within them only (by, bx, bw, bz).
                const __m256d halves{_mm256_permute2f128_pd(b, b, 0x01)};
                const __m256d t1{_mm256_xor_pd(
                    _mm256_permute_pd(halves, 0b0101), _mm256_set_pd(-0.0, 0.0, -0.0, 0.0)
                )};
                const __m256d t2{_mm256_xor_pd(halves, _mm256_set_pd(-0.0, -0.0, 0.0, 0.0))};
                const __m256d t3{
                    _mm256_xor_pd(_mm256_permute_pd(b, 0b0101), _mm256_set_pd(-0.0, 0.0, 0.0, -0.0))
                };
                const __m256d r{_mm256_add_pd(
                    _mm256_add_pd(
                        _mm256_add_pd(
                            _mm256_mul_pd(_mm256_permute_pd(a_hi, 0b1111), b),
                            _mm256_mul_pd(_mm256_permute_pd(a_lo, 0b0000), t1)
                        ),
                        _mm256_mul_pd(_mm256_permute_pd(a_lo, 0b1111), t2)
                    ),
                    _mm256_mul_pd(_mm256_permute_pd(a_hi, 0b0000), t3)
                )};
                _mm256_store_pd(data(), r);
                return *this;
#elif defined(__SSE2__) || defined(_M_X64)
                // The terms in halves, from (bx, by), (bz, bw) and their swaps; `upper` flips the
                // sign of the upper lane, `lower` that of the lower one.
                const __m128d b_lo{_mm_load_pd(obj.data())};
                const __m128d b_hi{_mm_load_pd(obj.data() + 2)};
                const __m128d swapped_lo{_mm_shuffle_pd(b_lo, b_lo, 0b01)};
                const __m128d swapped_hi{_mm_shuffle_pd(b_hi, b_hi, 0b01)};
                const __m128d upper{_mm_set_pd(-0.0, 0.0)};
                const __m128d lower{_mm_set_pd(0.0, -0.0)};
                // *this is loaded in halves, as under AVX, and its components broadcast from them.
                const __m128d a_lo{_mm_load_pd(data())};
                const __m128d a_hi{_mm_load_pd(data() + 2)};
                const __m128d ax{_mm_unpacklo_pd(a_lo, a_lo)};
                const __m128d ay{_mm_unpackhi_pd(a_lo, a_lo)};
                const __m128d az{_mm_unpacklo_pd(a_hi, a_hi)};
                const __m128d aw{_mm_unpackhi_pd(a_hi, a_hi)};
                const __m128d r_lo{_mm_add_pd(
                    _mm_add_pd(
                        _mm_add_pd(
                            _mm_mul_pd(aw, b_lo), _mm_mul_pd(ax, _mm_xor_pd(swapped_hi, upper))
                        ),
                        _mm_mul_pd(ay, b_hi)
                    ),
                    _mm_mul_pd(az, _mm_xor_pd(swapped_lo, lower))
                )};
                const __m128d r_hi{_mm_add_pd(
                    _mm_add_pd(
                        _mm_add_pd(
                            _mm_mul_pd(aw, b_hi), _mm_mul_pd(ax, _mm_xor_pd(swapped_lo, upper))
                        ),
                        _mm_mul_pd(ay, _mm_xor_pd(b_lo, _mm_set1_pd(-0.0)))
                    ),
                    _mm_mul_pd(az, _mm_xor_pd(swapped_hi, upper))
                )};
                _mm_store_pd(data(), r_lo);
                _mm_store_pd(data() + 2, r_hi);
                return *this;
#elif defined(__ARM_NEON)
                // The terms in halves, from (bx, by), (bz, bw) and their swaps; `upper` flips the
                // sign of the upper lane, `lower` that of the lower one.
                const float64x2_t b_lo{vld1q_f64(obj.data())};
                const float64x2_t b_hi{vld1q_f64(obj.data() + 2)};
                const float64x2_t swapped_lo{vextq_f64(b_lo, b_lo, 1)};
                const float64x2_t swapped_hi{vextq_f64(b_hi, b_hi, 1)};
                const uint64x2_t upper{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 1)};
                const uint64x2_t lower{vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)};
                // *this is loaded in halves, as under AVX, and multiplied by their lanes.
                const float64x2_t a_lo{vld1q_f64(data())};
                const float64x2_t a_hi{vld1q_f64(data() + 2)};
                const float64x2_t r_lo{vaddq_f64(
                    vaddq_f64(
                        vaddq_f64(
                            vmulq_laneq_f64(b_lo, a_hi, 1),
                            vmulq_laneq_f64(
                                vreinterpretq_f64_u64(
                                    veorq_u64(vreinterpretq_u64_f64(swapped_hi), upper)
                                ),
                                a_lo, 0
                            )
                        ),
                        vmulq_laneq_f64(b_hi, a_lo, 1)
                    ),
                    vmulq_laneq_f64(
                        vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(swapped_lo), lower)),
                        a_hi, 0
                    )
                )};
                const float64x2_t r_hi{vaddq_f64(
                    vaddq_f64(
                        vaddq_f64(
                            vmulq_laneq_f64(b_hi, a_hi, 1),
                            vmulq_laneq_f64(
                                vreinterpretq_f64_u64(
                                    veorq_u64(vreinterpretq_u64_f64(swapped_lo), upper)
                                ),
                                a_lo, 0
                            )
                        ),
                        vmulq_laneq_f64(vnegq_f64(b_lo), a_lo, 1)
                    ),
                    vmulq_laneq_f64(
                        vreinterpretq_f64_u64(veorq_u64(vreinterpretq_u64_f64(swapped_hi), upper)),
                        a_hi, 0
                    )
                )};
                vst1q_f64(data(), r_lo);
                vst1q_f64(data() + 2, r_hi);
                return *this;
#endif
            }
        }
        *this = Quaternion{
            w() * obj.x() + x() * obj.w() + y() * obj.z() - z() * obj.y(),
            w() * obj.y() - x() * obj.z() + y() * obj.w() + z() * obj.x(),
            w() * obj.z() + x() * obj.y() - y() * obj.x() + z() * obj.w(),
            w() * obj.w() - x() * obj.x() - y() * obj.y() - z() * obj.z()
        };
        return *this;
    }
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto operator*(this Self&& self, const Quaternion& obj) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self *= obj;
            return static_cast<Self&&>(self);
        } else {
            Quaternion result{self};
            result *= obj;
            return result;
        }
    }

    // Component by component. It hides DenseArithmeticBase's ==, which compares extents first.
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto operator==(const Quaternion& other) const noexcept -> bool {
        return DenseLanes<value_type>::equal(4, data(), other.data());
    }

    // The inner product of the four components.
    [[using gnu: pure, always_inline, hot]]
    constexpr auto dot(const Quaternion& obj) const noexcept -> value_type {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 prod{_mm_mul_ps(_mm_load_ps(data()), _mm_load_ps(obj.data()))};
                const __m128 pairs{_mm_add_ps(prod, _mm_movehl_ps(prod, prod))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                return vaddvq_f32(vmulq_f32(vld1q_f32(data()), vld1q_f32(obj.data())));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d prod{
                    _mm256_mul_pd(_mm256_load_pd(data()), _mm256_load_pd(obj.data()))
                };
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(prod), _mm256_extractf128_pd(prod, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                const __m128d pairs{_mm_add_pd(
                    _mm_mul_pd(_mm_load_pd(data()), _mm_load_pd(obj.data())),
                    _mm_mul_pd(_mm_load_pd(data() + 2), _mm_load_pd(obj.data() + 2))
                )};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                return vaddvq_f64(vaddq_f64(
                    vmulq_f64(vld1q_f64(data()), vld1q_f64(obj.data())),
                    vmulq_f64(vld1q_f64(data() + 2), vld1q_f64(obj.data() + 2))
                ));
#endif
            }
        }
        return x() * obj.x() + y() * obj.y() + z() * obj.z() + w() * obj.w();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto euclidean() const noexcept -> value_type {
        return std::sqrt(euclideanSqr());
    }
    [[using gnu: pure, always_inline, leaf, hot]]
    constexpr auto euclideanSqr() const noexcept -> value_type {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                const __m128 v{_mm_load_ps(data())};
                const __m128 sq{_mm_mul_ps(v, v)};
                const __m128 pairs{_mm_add_ps(sq, _mm_movehl_ps(sq, sq))};
                return _mm_cvtss_f32(
                    _mm_add_ss(pairs, _mm_shuffle_ps(pairs, pairs, _MM_SHUFFLE(1, 1, 1, 1)))
                );
#elif defined(__ARM_NEON)
                const float32x4_t v{vld1q_f32(data())};
                return vaddvq_f32(vmulq_f32(v, v));
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                const __m256d v{_mm256_load_pd(data())};
                const __m256d sq{_mm256_mul_pd(v, v)};
                const __m128d pairs{
                    _mm_add_pd(_mm256_castpd256_pd128(sq), _mm256_extractf128_pd(sq, 1))
                };
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__SSE2__) || defined(_M_X64)
                const __m128d lo{_mm_load_pd(data())};
                const __m128d hi{_mm_load_pd(data() + 2)};
                const __m128d pairs{_mm_add_pd(_mm_mul_pd(lo, lo), _mm_mul_pd(hi, hi))};
                return _mm_cvtsd_f64(_mm_add_sd(pairs, _mm_unpackhi_pd(pairs, pairs)));
#elif defined(__ARM_NEON)
                const float64x2_t lo{vld1q_f64(data())};
                const float64x2_t hi{vld1q_f64(data() + 2)};
                return vaddvq_f64(vaddq_f64(vmulq_f64(lo, lo), vmulq_f64(hi, hi)));
#endif
            }
        }
        return x() * x() + y() * y() + z() * z() + w() * w();
    }

    [[using gnu: pure, always_inline, hot]]
    constexpr auto identicalTo(const Quaternion& obj, value_type tol = 1E-8) const noexcept
        -> bool {
        const value_type one(1.0);
        return std::max(
                   {std::abs(x() - obj.x()) / std::max(std::abs(x()), one),
                    std::abs(y() - obj.y()) / std::max(std::abs(y()), one),
                    std::abs(z() - obj.z()) / std::max(std::abs(z()), one),
                    std::abs(w() - obj.w()) / std::max(std::abs(w()), one)}
               ) < tol;
    }

    // | |q|^2 - 1 | within `tol`, by default the √ε, about half the digits, that the rotations
    // check their operands with.
    [[using gnu: pure, always_inline]]
    constexpr auto isUnit(
        value_type tol = std::sqrt(std::numeric_limits<value_type>::epsilon())
    ) const noexcept -> bool {
        return std::abs(euclideanSqr() - value_type(1.0)) <= tol;
    }

    [[using gnu: always_inline, hot]]
    constexpr auto selfNormalized() noexcept -> Quaternion& {
        return *this /= euclidean();
    }
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto normalized(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfNormalized();
            return static_cast<Self&&>(self);
        } else {
            Quaternion result{self};
            result.selfNormalized();
            return result;
        }
    }

    // Negates x, y and z; the SIMD kernels flip their sign bits by XOR, as negation does.
    [[using gnu: always_inline, hot]]
    constexpr auto selfConjugated() noexcept -> Quaternion& {
        if !consteval {
            if constexpr (std::is_same_v<value_type, float>) {
#if defined(__SSE2__) || defined(_M_X64)
                _mm_store_ps(
                    data(), _mm_xor_ps(_mm_load_ps(data()), _mm_set_ps(0.0F, -0.0F, -0.0F, -0.0F))
                );
                return *this;
#elif defined(__ARM_NEON)
                vst1q_f32(
                    data(), vreinterpretq_f32_u32(veorq_u32(
                                vreinterpretq_u32_f32(vld1q_f32(data())),
                                vsetq_lane_u32(0U, vdupq_n_u32(0x80000000U), 3)
                            ))
                );
                return *this;
#endif
            } else if constexpr (std::is_same_v<value_type, double>) {
#if defined(__AVX__)
                _mm256_store_pd(
                    data(),
                    _mm256_xor_pd(_mm256_load_pd(data()), _mm256_set_pd(0.0, -0.0, -0.0, -0.0))
                );
                return *this;
#elif defined(__SSE2__) || defined(_M_X64)
                _mm_store_pd(data(), _mm_xor_pd(_mm_load_pd(data()), _mm_set1_pd(-0.0)));
                _mm_store_pd(
                    data() + 2, _mm_xor_pd(_mm_load_pd(data() + 2), _mm_set_pd(0.0, -0.0))
                );
                return *this;
#elif defined(__ARM_NEON)
                vst1q_f64(data(), vnegq_f64(vld1q_f64(data())));
                vst1q_f64(
                    data() + 2, vreinterpretq_f64_u64(veorq_u64(
                                    vreinterpretq_u64_f64(vld1q_f64(data() + 2)),
                                    vsetq_lane_u64(0x8000000000000000ULL, vdupq_n_u64(0), 0)
                                ))
                );
                return *this;
#endif
            }
        }
        x() = -x();
        y() = -y();
        z() = -z();
        return *this;
    }
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto conjugated(this Self&& self) noexcept -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfConjugated();
            return static_cast<Self&&>(self);
        } else {
            Quaternion result{self};
            result.selfConjugated();
            return result;
        }
    }

    // The multiplicative inverse q* / |q|^2, which is q* for a unit quaternion. A zero quaternion
    // throws when parameters are checked and yields NaNs otherwise.
    [[using gnu: always_inline, hot]]
    constexpr auto selfInversed() noexcept(!BASJOO_CHECK_PARAMS) -> Quaternion& {
        const value_type norm_sqr{euclideanSqr()};
#if BASJOO_CHECK_PARAMS == 1
        if (norm_sqr == value_type(0.0)) [[unlikely]] {
            throw std::runtime_error("Quaternion::selfInversed: zero quaternion.");
        }
#endif
        selfConjugated();
        return *this /= norm_sqr;
    }
    template <typename Self>
    [[using gnu: always_inline, hot]]
    constexpr auto inversed(this Self&& self) noexcept(!BASJOO_CHECK_PARAMS) -> decltype(auto) {
        if constexpr (Reusable<Self>) {
            self.selfInversed();
            return static_cast<Self&&>(self);
        } else {
            Quaternion result{self};
            result.selfInversed();
            return result;
        }
    }

    // q v q*, as v + w t + u x t with u = (x, y, z) and t = 2 u x v: Eigen's expansion, cheaper
    // than the two Hamilton products, which takes |q| = 1. A quaternion off unit norm throws when
    // parameters are checked and otherwise yields (1 - |q|^2) v + |q|^2 R v.
    // Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
    template <std::integral Index, Allocatory Alloc>
    [[using gnu: always_inline, hot]]
    constexpr auto rotate(const Vector<value_type, Index, 3, Alloc>& v) const noexcept(
        !BASJOO_CHECK_PARAMS
    ) -> Vector<value_type, Index, 3, Alloc> {
        using vector_type = Vector<value_type, Index, 3, Alloc>;

#if BASJOO_CHECK_PARAMS == 1
        if (!isUnit()) [[unlikely]] {
            throw std::invalid_argument("Quaternion::rotate: not a unit quaternion.");
        }
#endif
        const vector_type u{x(), y(), z()};
        const vector_type t{value_type(2.0) * u.cross(v)};
        return v + w() * t + u.cross(t);
    }

    // The angle within [0, π] of the rotation taking *this to `other`.
    [[using gnu: pure, always_inline]]
    constexpr auto angularDistance(const Quaternion& other) const noexcept -> value_type {
        return (*this * other.conjugated()).angle();
    }

  private:
    friend zpp::bits::access;
    using serialize = zpp::bits::members<1>;

    std::array<value_type, 4> m_data;
};

template <typename T>
struct isQuaternionArithmetic : std::false_type {};

template <std::floating_point T>
struct isQuaternionArithmetic<Quaternion<T>> : std::true_type {};

template <typename T>
inline constexpr bool isQuaternionArithmeticV = isQuaternionArithmetic<T>::value;

template <typename T>
concept QuaternionArithmetic = isQuaternionArithmeticV<T>;

// Spherical linear interpolation from `start` at ratio 0 to `end` at ratio 1, along the shorter of
// the two arcs, at a constant angular rate; the counterpart of lerp() for rotations. Either
// quaternion off unit norm throws when parameters are checked.
// Not gnu::pure: GCC deletes an unused call to a pure function together with its exception.
template <QuaternionArithmetic Q, std::floating_point U = double>
[[using gnu: hot]]
inline constexpr auto slerp(Q start, Q end, U ratio) noexcept(!BASJOO_CHECK_PARAMS) -> Q {
    using value_type = typename Q::value_type;

#if BASJOO_CHECK_PARAMS == 1
    if (!start.isUnit() || !end.isUnit()) [[unlikely]] {
        throw std::invalid_argument("basjoo::math::slerp(): not a unit quaternion.");
    }
#endif
    const value_type one(1.0);
    const value_type d{start.dot(end)};
    // Heading for -end when it is the nearer of the two takes the shorter arc.
    const value_type sign{d < value_type(0.0) ? value_type(-1.0) : one};
    const value_type cos_theta{std::abs(d)};
    // The weighted sum is formed in the storage of the two parameters.
    if (cos_theta >= one - std::numeric_limits<value_type>::epsilon()) {
        // sin(θ) vanishes: interpolate linearly instead.
        start *= 1 - ratio;
        end *= sign * ratio;
        start += end;
        start.selfNormalized();
        return start;
    }
    const value_type theta{std::acos(cos_theta)};
    const value_type sin_theta{std::sin(theta)};
    start *= std::sin((1 - ratio) * theta) / sin_theta;
    end *= sign * std::sin(ratio * theta) / sin_theta;
    start += end;
    return start;
}

} // namespace basjoo::math

// NOLINTBEGIN(cert-dcl58-cpp)

namespace std {

// Like std::abs on std::complex, the magnitude.
template <::basjoo::math::QuaternionArithmetic Q>
[[using gnu: pure, always_inline]]
inline constexpr auto abs(const Q& obj) noexcept -> typename Q::value_type {
    return obj.euclidean();
}

// Like std::norm on std::complex, the squared magnitude.
template <::basjoo::math::QuaternionArithmetic Q>
[[using gnu: pure, always_inline]]
inline constexpr auto norm(const Q& obj) noexcept -> typename Q::value_type {
    return obj.euclideanSqr();
}

template <typename T>
    requires ::basjoo::math::QuaternionArithmetic<std::remove_cvref_t<T>>
[[using gnu: always_inline]]
inline constexpr auto conj(T&& quaternion) noexcept -> decltype(auto) {
    return std::forward<T>(quaternion).conjugated();
}

} // namespace std

// NOLINTEND(cert-dcl58-cpp)
