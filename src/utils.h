#ifndef ROCKET_AVIONICS_UTILS_H
#define ROCKET_AVIONICS_UTILS_H

#include <array>

#include "states.h"
#if USE_GPS
#include "gps.h"
#endif

void wait(int milliseconds);

// Component Identifications
struct Vec3;
struct Quat;

static double clamp(double v, double lo, double hi);

// 3D Vector
typedef struct Vec3 {
    double x, y, z;

    Vec3 operator*(const double s) const {
        return { x*s, y*s, z*s };
    }
    Vec3& operator*=(const double s) {
        x *= s; y *= s; z *= s;
        return *this;
    }

    Vec3 operator/(const double s) const {
        return { x/s, y/s, z/s };
    }
    Vec3& operator/=(const double s) {
        x /= s; y /= s; z /= s;
        return *this;
    }

    Vec3 operator+(const Vec3 &v) const {
        return { x+v.x, y+v.y, z+v.z };
    }
    Vec3& operator+=(const Vec3 &v) {
        x += v.x; y += v.y; z += v.z;
        return *this;
    }

    Vec3 operator-(const Vec3 &v) const {
        return { x-v.x, y-v.y, z-v.z };
    }
    Vec3& operator-=(const Vec3 &v) {
        x -= v.x; y -= v.y; z -= v.z;
        return *this;
    }

    [[nodiscard]] double dot(const Vec3 &v) const;
    [[nodiscard]] Vec3 cross(const Vec3 &v) const;

    [[nodiscard]] double norm3() const;

    [[nodiscard]] double mag() const;

    [[nodiscard]] Quat toQuat(double w) const;
} Vec3;

// 4D Vector Base (for Quat and Grad4)
template<typename T, typename Derived>
struct Vec4Base {
    T w, x, y, z;
    Vec4Base() = default;
    constexpr Vec4Base(T w_, T x_, T y_, T z_)
        : w(w_), x(x_), y(y_), z(z_) {}

    inline Derived& derived() {
        return static_cast<Derived&>(*this);
    }

    [[nodiscard]] inline const Derived& derived() const {
        return static_cast<const Derived&>(*this);
    }

    // Scalar product
    Derived operator*(const T s) const {
        return Derived{ w*s, x*s, y*s, z*s };
    }
    Derived& operator*=(const T s) {
        w *= s; x *= s; y *= s; z *= s;
        return derived();
    }

    Derived operator/(const T s) const {
        return Derived{ w/s, x/s, y/s, z/s };
    }
    Derived& operator/=(const T s) {
        w /= s; x /= s; y /= s; z /= s;
        return derived();
    }

    Derived operator+(const Vec4Base &q) const {
        return Derived{ w+q.w, x+q.x, y+q.y, z+q.z };
    }
    Derived& operator+=(const Vec4Base &q) {
        w += q.w; x += q.x; y += q.y; z += q.z;
        return derived();
    }

    Derived operator-(const Vec4Base &q) const {
        return Derived{ w-q.w, x-q.x, y-q.y, z-q.z };
    }
    Derived& operator-=(const Vec4Base &q) {
        w -= q.w; x -= q.x; y -= q.y; z -= q.z;
        return derived();
    }

    [[nodiscard]] T norm4() const { return std::sqrt(w*w + x*x + y*y + z*z); }
    [[nodiscard]] T norm() const { return norm4(); }
    [[nodiscard]] T mag() const { return norm4(); }
};

// Quaternion Containers
typedef struct Quat : Vec4Base<double, Quat> {
    using Vec4Base::Vec4Base;
    // Bring scalar operators from Vec4Base into Quat scope
    // so they aren't hidden by the Hamilton products below.
    using Vec4Base::operator*;
    using Vec4Base::operator*=;
    using Vec4Base::operator/;
    using Vec4Base::operator/=;

    // Hamilton product p ⊗ q
    Quat operator*(const Quat& q) const {
        return {
            w*q.w - x*q.x - y*q.y - z*q.z,
            w*q.x + x*q.w + y*q.z - z*q.y,
            w*q.y - x*q.z + y*q.w + z*q.x,
            w*q.z + x*q.y - y*q.x + z*q.w
        };
    }
    Quat& operator*=(const Quat& q) {
        const Quat p = *this;
        w = p.w*q.w - p.x*q.x - p.y*q.y - p.z*q.z;
        x = p.w*q.x + p.x*q.w + p.y*q.z - p.z*q.y;
        y = p.w*q.y - p.x*q.z + p.y*q.w + p.z*q.x;
        z = p.w*q.z + p.x*q.y - p.y*q.x + p.z*q.w;
        return *this;
    }

    [[nodiscard]] Quat conjugate() const;

    void normalise();

    [[nodiscard]] std::array<std::array<double,3>,3> toMatrix() const;
    Quat() = default;
} Quat;

struct Grad4 : Vec4Base<double, Grad4> {
    using Vec4Base::Vec4Base;
};

#endif