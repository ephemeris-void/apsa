#pragma once
#include "math_assert.hpp"
#include "math_attributes.hpp"
#include <array>
#include <cmath>
#include <cstdint>
#include <complex>
#include <random>
#include <iostream>

using scalar = float;
constexpr scalar kNormSqThreshold{1e-12};
constexpr scalar kDetThreshold{1e-12};
constexpr scalar kDegToRad{0.017453292519943295f};

// MATH CORE — VEC3 + MAT3 + QUATERNION


struct Vec3 {
    std::array<scalar,3> data{};

    constexpr Vec3() = default;
    constexpr Vec3(scalar x_, scalar y_, scalar z_) noexcept : data{x_, y_, z_} {}

    [[nodiscard]] constexpr scalar& operator[](std::size_t i) noexcept {
    MATH_ASSERT(i < 3U);
    return data[i];
    }

    [[nodiscard]] constexpr scalar operator[](std::size_t i) const noexcept {
    MATH_ASSERT(i < 3U);
    return data[i];
   }

    [[nodiscard]] constexpr scalar& x() noexcept { return data[0]; }
    [[nodiscard]] constexpr scalar& y() noexcept { return data[1]; }
    [[nodiscard]] constexpr scalar& z() noexcept { return data[2]; }

    [[nodiscard]] constexpr scalar x() const noexcept { return data[0]; }
    [[nodiscard]] constexpr scalar y() const noexcept { return data[1]; }
    [[nodiscard]] constexpr scalar z() const noexcept { return data[2]; }
};

struct Mat3 {
    std::array<scalar,9> data{};

    constexpr Mat3() = default;
    constexpr Mat3(std::array<scalar,9> d) noexcept : data(d) {}

    [[nodiscard]] constexpr scalar& at(uint32_t i, uint32_t j) noexcept {
        MATH_ASSERT(i < 3U);
        MATH_ASSERT(j < 3U);
        return data[(i * 3) + j];
    }

    [[nodiscard]] constexpr scalar at(uint32_t i, uint32_t j) const noexcept {
        MATH_ASSERT(i < 3U);
        MATH_ASSERT(j < 3U);
        return data[(i * 3) + j];
    }
};

// Vec3 Operators 

[[nodiscard, APSA_PURE]] constexpr Vec3 operator+(const Vec3& a, const Vec3& b) noexcept {
    return Vec3{a.x() + b.x(), a.y() + b.y(), a.z() + b.z()};
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator-(const Vec3& a, const Vec3& b) noexcept {
    return Vec3{a.x() - b.x(), a.y() - b.y(), a.z() - b.z()};
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator-(const Vec3& a) noexcept {
    return Vec3{-a.x(), -a.y(), -a.z()};
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator/(const Vec3& v, scalar s) noexcept {
    return Vec3{v.x() / s, v.y() / s, v.z() / s};
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator*(const Mat3& a, const Vec3& b) noexcept {
    return Vec3{
        std::fma(a.at(0,0), b.x(), std::fma(a.at(0,1), b.y(), std::fma(a.at(0,2), b.z(), 0.0f))),
        std::fma(a.at(1,0), b.x(), std::fma(a.at(1,1), b.y(), std::fma(a.at(1,2), b.z(), 0.0f))),
        std::fma(a.at(2,0), b.x(), std::fma(a.at(2,1), b.y(), std::fma(a.at(2,2), b.z(), 0.0f)))
    };
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator*(const Vec3& b, const Mat3& a) noexcept {
    return Vec3{
        std::fma(b.x(), a.at(0,0), std::fma(b.y(), a.at(1,0), std::fma(b.z(), a.at(2,0), 0.0f))),
        std::fma(b.x(), a.at(0,1), std::fma(b.y(), a.at(1,1), std::fma(b.z(), a.at(2,1), 0.0f))),
        std::fma(b.x(), a.at(0,2), std::fma(b.y(), a.at(1,2), std::fma(b.z(), a.at(2,2), 0.0f)))
    };
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator*(scalar b, const Vec3& v) noexcept {
    return Vec3{b * v.x(), b * v.y(), b * v.z()};
}

[[nodiscard, APSA_PURE]] constexpr Vec3 operator*(const Vec3& v, scalar b) noexcept {
    return Vec3{v.x() * b, v.y() * b, v.z() * b};
}

//  Vec3 Math 

[[nodiscard, APSA_PURE]] constexpr scalar dot(const Vec3& a, const Vec3& b) noexcept {
    return std::fma(a.x(), b.x(), std::fma(a.y(), b.y(), std::fma(a.z(), b.z(), 0.0f)));
}

[[nodiscard, APSA_PURE]] constexpr Vec3 cross(const Vec3& a, const Vec3& b) noexcept {
    return Vec3{
        std::fma(a.y(), b.z(), -a.z() * b.y()),
        std::fma(a.z(), b.x(), -a.x() * b.z()),
        std::fma(a.x(), b.y(), -a.y() * b.x())
    };
}

[[nodiscard, APSA_PURE]] constexpr scalar norm_sq(const Vec3& a) noexcept {
    return std::fma(a.x(), a.x(), std::fma(a.y(), a.y(), std::fma(a.z(), a.z(), 0.0f)));
}

[[nodiscard]] inline constexpr Vec3 normalize(const Vec3& v) noexcept {
    const scalar n_sq = norm_sq(v);
    if (n_sq < kNormSqThreshold) APSA_UNLIKELY {
        return Vec3{};
    }
    const scalar inv_norm = scalar{1} / std::sqrt(n_sq);
    return Vec3{v.x() * inv_norm, v.y() * inv_norm, v.z() * inv_norm};
}

// Mat3 Operators 

[[nodiscard, APSA_PURE]] constexpr Mat3 operator*(const Mat3& a, const Mat3& b) noexcept {
    Mat3 c{};
    for (uint32_t i{0}; i < 3U; ++i) {
        for (uint32_t k{0}; k < 3U; ++k) {
            for (uint32_t j{0}; j < 3U; ++j) {
                c.at(i,j) = std::fma(a.at(i,k), b.at(k,j), c.at(i,j));
            }
        }
    }
    return c;
}

[[nodiscard, APSA_PURE]] constexpr Mat3 operator+(const Mat3& a, const Mat3& b) noexcept {
    return Mat3{{
        a.at(0,0) + b.at(0,0), a.at(0,1) + b.at(0,1), a.at(0,2) + b.at(0,2),
        a.at(1,0) + b.at(1,0), a.at(1,1) + b.at(1,1), a.at(1,2) + b.at(1,2),
        a.at(2,0) + b.at(2,0), a.at(2,1) + b.at(2,1), a.at(2,2) + b.at(2,2)
    }};
}

[[nodiscard, APSA_PURE]] constexpr Mat3 operator-(const Mat3& a, const Mat3& b) noexcept {
    return Mat3{{
        a.at(0,0) - b.at(0,0), a.at(0,1) - b.at(0,1), a.at(0,2) - b.at(0,2),
        a.at(1,0) - b.at(1,0), a.at(1,1) - b.at(1,1), a.at(1,2) - b.at(1,2),
        a.at(2,0) - b.at(2,0), a.at(2,1) - b.at(2,1), a.at(2,2) - b.at(2,2)
    }};
}

[[nodiscard, APSA_PURE]] constexpr Mat3 operator*(const Mat3& m, scalar b) noexcept {
    return Mat3{{
        m.at(0,0) * b, m.at(0,1) * b, m.at(0,2) * b,
        m.at(1,0) * b, m.at(1,1) * b, m.at(1,2) * b,
        m.at(2,0) * b, m.at(2,1) * b, m.at(2,2) * b
    }};
}

[[nodiscard, APSA_PURE]] constexpr Mat3 transpose(const Mat3& m) noexcept {
    return Mat3{{
        m.at(0,0), m.at(1,0), m.at(2,0),
        m.at(0,1), m.at(1,1), m.at(2,1),
        m.at(0,2), m.at(1,2), m.at(2,2)
    }};
}

[[nodiscard, APSA_PURE]] constexpr scalar determinant(const Mat3& m) noexcept {
    return std::fma(m.at(0,0), std::fma(m.at(1,1), m.at(2,2), -m.at(1,2) * m.at(2,1)),
           std::fma(-m.at(0,1), std::fma(m.at(1,0), m.at(2,2), -m.at(1,2) * m.at(2,0)),
           std::fma(m.at(0,2), std::fma(m.at(1,0), m.at(2,1), -m.at(1,1) * m.at(2,0)), 0.0f)));
}

[[nodiscard]] inline constexpr bool inverse(const Mat3& m, Mat3& out_inv) noexcept {
    
    const scalar det = determinant(m);
    if (std::abs(det) < kDetThreshold) APSA_UNLIKELY {
        return false;
    }

    const scalar inv_det = scalar{1} / det;
    out_inv = Mat3{{
        std::fma(m.at(1,1), m.at(2,2), -m.at(1,2)*m.at(2,1)) * inv_det,
        std::fma(m.at(0,2), m.at(2,1), -m.at(0,1)*m.at(2,2)) * inv_det,
        std::fma(m.at(0,1), m.at(1,2), -m.at(0,2)*m.at(1,1)) * inv_det,
        std::fma(m.at(1,2), m.at(2,0), -m.at(1,0)*m.at(2,2)) * inv_det,
        std::fma(m.at(0,0), m.at(2,2), -m.at(0,2)*m.at(2,0)) * inv_det,
        std::fma(m.at(0,2), m.at(1,0), -m.at(0,0)*m.at(1,2)) * inv_det,
        std::fma(m.at(1,0), m.at(2,1), -m.at(1,1)*m.at(2,0)) * inv_det,
        std::fma(m.at(0,1), m.at(2,0), -m.at(0,0)*m.at(2,1)) * inv_det,
        std::fma(m.at(0,0), m.at(1,1), -m.at(0,1)*m.at(1,0)) * inv_det
    }};
    return true;
}

[[nodiscard, APSA_PURE]] constexpr Mat3 triple_product(const Mat3& F, const Mat3& P) noexcept {const Mat3 FP = F * P;const Mat3 Ft = transpose(F); return FP * Ft;}


// QUATERNION


class Quaternion {

private:
    scalar w{1.0f};
    scalar x{0.0f};
    scalar y{0.0f};
    scalar z{0.0f};

public:
    constexpr Quaternion() = default;
    constexpr Quaternion(scalar w_, scalar x_, scalar y_, scalar z_) noexcept
        : w(w_), x(x_), y(y_), z(z_) {}

    [[nodiscard]] constexpr scalar get_x() const noexcept { return x; }
    [[nodiscard]] constexpr scalar get_y() const noexcept { return y; }
    [[nodiscard]] constexpr scalar get_z() const noexcept { return z; }
    [[nodiscard]] constexpr scalar get_w() const noexcept { return w; }

    friend constexpr bool operator==(const Quaternion&, const Quaternion&) noexcept = delete;

    [[nodiscard]] friend constexpr Quaternion operator*(const Quaternion& a, const Quaternion& b) noexcept {
        return Quaternion{
            std::fma(a.w, b.w, -std::fma(a.x, b.x, std::fma(a.y, b.y, a.z*b.z))),
            std::fma(a.w, b.x,  std::fma(a.x, b.w, std::fma(a.y, b.z, -a.z*b.y))),
            std::fma(a.w, b.y,  std::fma(a.y, b.w, std::fma(a.z, b.x, -a.x*b.z))),
            std::fma(a.w, b.z,  std::fma(a.z, b.w, std::fma(a.x, b.y, -a.y*b.x)))
        };
    }

    [[nodiscard]] constexpr Quaternion conjugate() const noexcept {
        return Quaternion{w, -x, -y, -z};
    }

    [[nodiscard, APSA_PURE]] friend constexpr scalar norm_sq(const Quaternion& a) noexcept {
        return std::fma(a.w, a.w, std::fma(a.x, a.x, std::fma(a.y, a.y, std::fma(a.z, a.z, 0.0f))));
    }

    [[nodiscard, APSA_PURE]] friend constexpr scalar norm(const Quaternion& a) noexcept {
        return std::sqrt(norm_sq(a));
    }

    [[nodiscard]] constexpr Quaternion normalize() const noexcept {
        const scalar n_sq = norm_sq(*this);
        if (n_sq < kNormSqThreshold) APSA_UNLIKELY {

           return Quaternion{0.0f, 0.0f, 0.0f, 0.0f};
        }
        const scalar inv_n = scalar{1} / std::sqrt(n_sq);
        return Quaternion{w * inv_n, x * inv_n, y * inv_n, z * inv_n};
    }

    [[nodiscard, APSA_PURE]] friend constexpr Vec3 rotate(const Quaternion& q, const Vec3& v) noexcept {
        
          if (norm_sq(q) < kNormSqThreshold) APSA_UNLIKELY {return Vec3{};}

        if (norm_sq(v) < kNormSqThreshold) APSA_UNLIKELY {return Vec3{}; }

        const Quaternion q_normalized = q.normalize();
        const Quaternion qv{0.0f, v.x(), v.y(), v.z()};
        const Quaternion qv_rot = q * qv * q_normalized.conjugate();
        return Vec3{qv_rot.get_x(), qv_rot.get_y(), qv_rot.get_z()};
    }
    
};


[[nodiscard]] inline constexpr bool quaternion_derivative(const Quaternion& q, const Vec3& omega,Quaternion& q_dot) noexcept {
   
    const Quaternion omega_q{
        0.0f,
        omega.x(),
        omega.y(),
        omega.z()
    };

    const Quaternion result = omega_q * q;
    const scalar half = scalar{0.5f};

    q_dot = Quaternion{
        half * result.get_w(),
        half * result.get_x(),
        half * result.get_y(),
        half * result.get_z()
    };

    return true;
}

[[nodiscard]] inline constexpr bool quaternion_integrate(const Quaternion& q0,const Vec3& omega,const scalar dt,Quaternion& q1) noexcept {
    
    const scalar w_mag = std::sqrt(dot(omega, omega));

    if (dt <= kNormSqThreshold) APSA_UNLIKELY {
        return false;
    }

    if (w_mag <= kNormSqThreshold) {
       
        q1 = q0;
        return true;
    }

    const scalar half_angle = scalar{0.5f} * w_mag * dt;
    const scalar c = std::cos(half_angle);
    const scalar s = std::sin(half_angle);
    const scalar inv_mag = scalar{1.0f} / w_mag;

    const Quaternion dq{
        c,
        s * omega.x() * inv_mag,
        s * omega.y() * inv_mag,
        s * omega.z() * inv_mag
    };

    q1 = dq * q0;
    return true;
}

//---------------------------------------------------
//---------------------------------------------------


// DYNAMIC MATRIX + VECTOR

template<std::size_t MatR,std::size_t MatC>
struct Matrix_Dynamic {
private:
    std::array<scalar,MatR*MatC> mat{};
    std::size_t rows_{MatR};
    std::size_t cols_{MatC};

public:
    constexpr Matrix_Dynamic() noexcept = default;

    Matrix_Dynamic(std::size_t row,std::size_t col) noexcept : rows_(row), cols_(col) {
        MATH_ASSERT(row <= MatR);
        MATH_ASSERT(col <= MatC);
    }

    [[nodiscard]] constexpr scalar& at(std::size_t row,std::size_t col) noexcept {
        MATH_ASSERT(row < rows_);
        MATH_ASSERT(col < cols_);
        return mat[row*cols_+col];
    }

    [[nodiscard]] constexpr scalar at(std::size_t row,std::size_t col) const noexcept {
        MATH_ASSERT(row < rows_);
        MATH_ASSERT(col < cols_);
        return mat[row*cols_+col];
    }

    [[nodiscard]] constexpr std::size_t rows() const noexcept { return rows_; }
    [[nodiscard]] constexpr std::size_t cols() const noexcept { return cols_; }
    [[nodiscard]] constexpr std::size_t size() const noexcept { return rows_*cols_; }

    [[nodiscard]] constexpr scalar data_at(std::size_t idx) const noexcept {
        return mat[idx];
    }

    template<std::size_t OtherR, std::size_t OtherC>
    Matrix_Dynamic& operator=(const Matrix_Dynamic<OtherR, OtherC>& other) noexcept {
        rows_ = other.rows();
        cols_ = other.cols();
        const std::size_t n = rows_ * cols_;
        for (std::size_t i = 0; i < n && i < MatR*MatC; ++i) {
            mat[i] = other.data_at(i);
        }
        return *this;
    }
};

template<std::size_t VecN>
struct Vector_Dynamic {
private:
    std::array<scalar,VecN> vec{};
    std::size_t N_{VecN};

public:
    constexpr Vector_Dynamic() noexcept = default;
    constexpr explicit Vector_Dynamic(std::size_t N) noexcept : N_(N) {
        MATH_ASSERT(N <= VecN);
    }

    [[nodiscard]] constexpr scalar& at(std::size_t n) noexcept {
        MATH_ASSERT(n < N_);
        return vec[n];
    }

    [[nodiscard]] constexpr scalar at(std::size_t n) const noexcept {
        MATH_ASSERT(n < N_);
        return vec[n];
    }

    [[nodiscard]] constexpr std::size_t size() const noexcept { return N_; }
};

// Dynamic Vector Math 



template<std::size_t N>
[[nodiscard]] inline bool inverse_dynamic(const Matrix_Dynamic<N,N>& m,Matrix_Dynamic<N,N>& out_inv) noexcept {
   
    Matrix_Dynamic<N,N> L{N,N};
    Matrix_Dynamic<N,N> U{N,N};
    const std::size_t n = m.rows();
    Matrix_Dynamic<N,N> P{n, n};

    if (!lu_decompose(m, L, U, P)) return false;

    out_inv = Matrix_Dynamic<N,N>{n, n};

    for (std::size_t col{0}; col < n; ++col) {
        Vector_Dynamic<N> e{n};
        e.at(col) = scalar{1.0f};

        Vector_Dynamic<N> x{n};
        if (!solve(m, e, x)) return false;

        for (std::size_t row{0}; row < n; ++row) {
            out_inv.at(row, col) = x.at(row);
        }
    }

    return true;
}


template<std::size_t N>
[[nodiscard]] inline constexpr scalar norm_sq_dynamic(const Vector_Dynamic<N>& v) noexcept {
    scalar sum{0.0f};
    for (std::size_t i{0}; i < v.size(); ++i) {
        sum = std::fma(v.at(i), v.at(i), sum);
    }
    return sum;
}

template<std::size_t N>
[[nodiscard]] inline constexpr scalar dot_dynamic(const Vector_Dynamic<N>& a,const Vector_Dynamic<N>& b) noexcept {
    scalar sum{0.0f};
    for (std::size_t i{0}; i < a.size(); ++i) {
        sum = std::fma(a.at(i), b.at(i), sum);
    }
    return sum;
}

template<std::size_t N>
[[nodiscard]] inline constexpr Vector_Dynamic<N> operator*(scalar s,const Vector_Dynamic<N>& v) noexcept {
   
    Vector_Dynamic<N> result{v.size()};
    for (std::size_t i{0}; i < v.size(); ++i) {
        result.at(i) = s * v.at(i);
    }
    return result;
}

template<std::size_t N>
[[nodiscard]] inline constexpr Vector_Dynamic<N> operator*(const Vector_Dynamic<N>& v,scalar s) noexcept {
    return s * v;
}

template<std::size_t N>
[[nodiscard]] inline constexpr Vector_Dynamic<N> operator-(const Vector_Dynamic<N>& a,const Vector_Dynamic<N>& b) noexcept {
    MATH_ASSERT(a.size() == b.size());

    Vector_Dynamic<N> result{a.size()};
    for (std::size_t i{0}; i < a.size(); ++i) {
        result.at(i) = a.at(i) - b.at(i);
    }
    return result;
}

template<std::size_t N>
[[nodiscard]] inline constexpr Vector_Dynamic<N> operator+(const Vector_Dynamic<N>& a,const Vector_Dynamic<N>& b) noexcept {
    MATH_ASSERT(a.size() == b.size());

    Vector_Dynamic<N> result{a.size()};
    for (std::size_t i{0}; i < a.size(); ++i) {
        result.at(i) = a.at(i) + b.at(i);
    }
    return result;
}

// ─── Dynamic Matrix Operators ───

template<std::size_t MatR,std::size_t MatC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<MatR,MatC> operator+(const Matrix_Dynamic<MatR,MatC>& a,const Matrix_Dynamic<MatR,MatC>& b) noexcept {
    
    MATH_ASSERT(a.rows()==b.rows());
    MATH_ASSERT(a.cols()==b.cols());

    Matrix_Dynamic<MatR,MatC> result{a.rows(),a.cols()};

    for(std::size_t i{0};i<result.rows();++i) {
        for(std::size_t j{0};j<result.cols();++j) {
            result.at(i,j) = a.at(i,j) + b.at(i,j);
        }
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<MatR,MatC> operator-(const Matrix_Dynamic<MatR,MatC>& a,const Matrix_Dynamic<MatR,MatC>& b) noexcept {
    
    MATH_ASSERT(a.rows()==b.rows());
    MATH_ASSERT(a.cols()==b.cols());

    Matrix_Dynamic<MatR,MatC> result{a.rows(),a.cols()};

    for(std::size_t i{0};i<result.rows();++i) {
        for(std::size_t j{0};j<result.cols();++j) {
            result.at(i,j) = a.at(i,j) - b.at(i,j);
        }
    }
    return result;
}


template<std::size_t MatR, std::size_t MatK, std::size_t MatC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<MatR,MatC> operator*(const Matrix_Dynamic<MatR,MatK>& a, const Matrix_Dynamic<MatK,MatC>& b) noexcept {
    
    Matrix_Dynamic<MatR,MatC> result{a.rows(), b.cols()};

    for (std::size_t i{0}; i < a.rows(); ++i) {
        for (std::size_t k{0}; k < a.cols(); ++k) {
            for (std::size_t j{0}; j < b.cols(); ++j) {
                result.at(i,j) = std::fma(a.at(i,k), b.at(k,j), result.at(i,j));
            }
        }
    }
    return result;
}

template<std::size_t MatR, std::size_t MatC>
[[nodiscard, APSA_PURE]] constexpr Matrix_Dynamic<MatR, MatC> operator*( const Matrix_Dynamic<MatR, MatC>& m, scalar s) noexcept {
    
    Matrix_Dynamic<MatR, MatC> result{m.rows(), m.cols()};
    for (std::size_t i{0}; i < m.rows(); ++i) {
        for (std::size_t j{0}; j < m.cols(); ++j) {
            result.at(i, j) = m.at(i, j) * s;
        }
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC,std::size_t VecN>
[[nodiscard,APSA_PURE]] constexpr Vector_Dynamic<MatR> operator*(const Matrix_Dynamic<MatR,MatC>& a,const Vector_Dynamic<VecN>& b) noexcept {
   
    MATH_ASSERT(a.cols()==b.size());

    Vector_Dynamic<MatR> result{a.rows()};

    for(std::size_t i{0};i<a.rows();++i) {
        for(std::size_t j{0};j<a.cols();++j) {
            result.at(i) = std::fma(a.at(i,j), b.at(j), result.at(i));
        }
    }
    return result;
}

// ─── Dynamic Matrix Utilities ───

template<std::size_t MatR,std::size_t MatC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<MatC,MatR> transpose_Dynamic(const Matrix_Dynamic<MatR,MatC>& a) noexcept {
    
    Matrix_Dynamic<MatC,MatR> result{a.cols(),a.rows()};

    for(std::size_t i{0};i<result.rows();++i) {
        for(std::size_t j{0};j<result.cols();++j) {
            result.at(i,j) = a.at(j,i);
        }
    }
    return result;
}

// this function is using for save another code file wrong implemention
template<std::size_t MatR, std::size_t MatC>
[[nodiscard, APSA_PURE]] constexpr Matrix_Dynamic<MatC, MatR> transpose(const Matrix_Dynamic<MatR, MatC>& a) noexcept {
    
    return transpose_Dynamic(a);
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<N,N> Identity_Dynamic() noexcept {
    
    Matrix_Dynamic<N,N> result{N,N};

    for(std::size_t i{0};i<N;++i) {
        for(std::size_t j{0};j<N;++j) {
            if(i==j) APSA_UNLIKELY {
                result.at(i,j) = scalar{1.0f};
            } else {
                result.at(i,j) = scalar{0.0f};
            }
        }
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<MatR,MatC> Zero_Dynamic() noexcept {
    Matrix_Dynamic<MatR,MatC> result{MatR,MatC};

    for(std::size_t i{0};i<MatR;++i) {
        for(std::size_t j{0};j<MatC;++j) {
            result.at(i,j) = scalar{0.0f};
        }
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<MatR,MatC> One_Dynamic() noexcept {
    
    Matrix_Dynamic<MatR,MatC> result{MatR,MatC};

    for(std::size_t i{0};i<MatR;++i) {
        for(std::size_t j{0};j<MatC;++j) {
            result.at(i,j) = scalar{1.0f};
        }
    }
    return result;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<N,N> Diagonal_Dynamic(const Vector_Dynamic<N>& a) noexcept {
   
    const std::size_t n = a.size();
    Matrix_Dynamic<N,N> result{n, n};

    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            if(i==j) APSA_UNLIKELY {
                result.at(i,j) = a.at(i);
            } else {
                result.at(i,j) = scalar{0.0f};
            }
        }
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC,std::size_t BlockR,std::size_t BlockC>
[[nodiscard,APSA_PURE]] constexpr Matrix_Dynamic<BlockR,BlockC> submatrix(const Matrix_Dynamic<MatR,MatC>& a,std::size_t Block_start_row, std::size_t Block_start_col) noexcept {
    
    Matrix_Dynamic<BlockR,BlockC> result{BlockR,BlockC};

    for(std::size_t i{0};i<BlockR;++i) {
        for(std::size_t j{0};j<BlockC;++j) {
            result.at(i,j) = a.at(Block_start_row+i, Block_start_col+j);
        }
    }
    return result;
}

//Column/Row Access 

template<std::size_t MatR,std::size_t MatC>
[[nodiscard]] inline constexpr Vector_Dynamic<MatR> get_column(const Matrix_Dynamic<MatR,MatC>& A,std::size_t col) noexcept {
    
    MATH_ASSERT(col < MatC);

    Vector_Dynamic<MatR> result{MatR};

    for(std::size_t i{0};i<MatR;++i) {
        result.at(i) = A.at(i,col);
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC>
[[nodiscard]] inline constexpr Vector_Dynamic<MatC> get_row(const Matrix_Dynamic<MatR,MatC>& A,std::size_t row) noexcept {
   
    MATH_ASSERT(row < MatR);

    Vector_Dynamic<MatC> result{MatC};

    for(std::size_t j{0};j<MatC;++j) {
        result.at(j) = A.at(row,j);
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC>
[[nodiscard]] inline constexpr Vector_Dynamic<MatR> get_sub_column( const Matrix_Dynamic<MatR,MatC>& A, const std::size_t start_row,std::size_t col) noexcept {
    
    MATH_ASSERT(start_row < MatR);
    MATH_ASSERT(col < MatC);

    const std::size_t size = MatR - start_row;
    Vector_Dynamic<MatR> result{size};

    for(std::size_t i{start_row};i<MatR;++i) {
        result.at(i-start_row) = A.at(i,col);
    }
    return result;
}

template<std::size_t MatR,std::size_t MatC>
[[nodiscard]] inline constexpr Vector_Dynamic<MatC> get_sub_row(const Matrix_Dynamic<MatR,MatC>& A, const std::size_t start_col, std::size_t row) noexcept {
   
    MATH_ASSERT(start_col < MatC);
    MATH_ASSERT(row < MatR);

    const std::size_t size = MatC - start_col;
    Vector_Dynamic<MatC> result{size};

    for(std::size_t j{start_col};j<MatC;++j) {
        result.at(j-start_col) = A.at(row,j);
    }
    return result;
}


// MATRIX CHECKS


template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_symmetric(const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{i+1};j<n;++j) {
            if(std::abs(a.at(i,j)-a.at(j,i)) > kNormSqThreshold) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_diagonal( const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            if(i != j && std::abs(a.at(i,j)) > kNormSqThreshold) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_zero(const Matrix_Dynamic<N,N>& a) noexcept {
    
    const std::size_t n = a.rows();
    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            if(std::abs(a.at(i,j)) > kNormSqThreshold) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_nan(const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            if(std::isnan(a.at(i,j))) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_inf( const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            if(std::isinf(a.at(i,j))) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_finite(const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            if(!std::isfinite(a.at(i,j))) {
                return false;
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard,APSA_PURE]] constexpr bool is_orthogonal(const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    Matrix_Dynamic<N,N> ATA = transpose_Dynamic(a) * a;

    for(std::size_t i{0};i<n;++i) {
        for(std::size_t j{0};j<n;++j) {
            scalar identity_value{0.0f};
            if(i == j) APSA_UNLIKELY {
                identity_value = scalar{1.0f};
            }

            if(std::abs(ATA.at(i,j) - identity_value) > kNormSqThreshold) {
                return false;
            }
        }
    }
    return true;
}

//------------------------------------------------------------
//------------------------------------------------------------



// ROW/COLUMN UTILITIES


template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr bool swap_rows(Matrix_Dynamic<MatR, MatC>& m,const std::size_t row_a, const std::size_t row_b) noexcept {
    
    if (row_a == row_b) {
        return true;
    }
    MATH_ASSERT(row_a < MatR);
    MATH_ASSERT(row_b < MatR);

    for (std::size_t c{0}; c < MatC; ++c) {
        const scalar temp = m.at(row_a, c);
        m.at(row_a, c) = m.at(row_b, c);
        m.at(row_b, c) = temp;
    }
    return true;
}

template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr bool swap_columns(Matrix_Dynamic<MatR, MatC>& m,const std::size_t col_a,const std::size_t col_b) noexcept {
   
    if (col_a == col_b) {
        return true;
    }
    MATH_ASSERT(col_a < MatC);
    MATH_ASSERT(col_b < MatC);

    for (std::size_t r{0}; r < MatR; ++r) {
        const scalar temp = m.at(r, col_a);
        m.at(r, col_a) = m.at(r, col_b);
        m.at(r, col_b) = temp;
    }
    return true;
}

template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr std::size_t find_max_in_column(const Matrix_Dynamic<MatR, MatC>& m,const std::size_t col, const std::size_t start_row = 0) noexcept {
    
    MATH_ASSERT(col < MatC);
    MATH_ASSERT(start_row < MatR);

    std::size_t max_row = start_row;
    scalar max_val = std::abs(m.at(start_row, col));

    for (std::size_t r = start_row + 1; r < MatR; ++r) {
        const scalar candidate = std::abs(m.at(r, col));
        if (candidate > max_val) {
            max_val = candidate;
            max_row = r;
        }
    }
    return max_row;
}

template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr std::size_t find_max_in_row(const Matrix_Dynamic<MatR, MatC>& m, const std::size_t row,const std::size_t start_col = 0) noexcept {
    
    MATH_ASSERT(row < MatR);
    MATH_ASSERT(start_col < MatC);

    std::size_t max_col = start_col;
    scalar max_val = std::abs(m.at(row, start_col));

    for (std::size_t c = start_col + 1; c < MatC; ++c) {
        const scalar candidate = std::abs(m.at(row, c));
        if (candidate > max_val) {
            max_val = candidate;
            max_col = c;
        }
    }
    return max_col;
}


// CHOLESKY DECOMPOSITION
// SPD matrix → L×Lᵀ


template<std::size_t N>
[[nodiscard]] inline constexpr bool cholesky( const Matrix_Dynamic<N,N>& a, Matrix_Dynamic<N,N>& L) noexcept {
   
    const std::size_t n = a.rows();
    L = Matrix_Dynamic<N,N>{n, n};

    if(!is_symmetric(a)) APSA_UNLIKELY {
        return false;
    }

    for(std::size_t i{0}; i<n; ++i) {
        for(std::size_t j{0}; j<=i; ++j) {
            if(i==j) APSA_UNLIKELY {
                scalar sum{0.0f};
                for(std::size_t k{0}; k<i; ++k) {
                    sum += L.at(i,k) * L.at(i,k);
                }

                const scalar discriminant = a.at(i,j) - sum;
                if(discriminant <= kNormSqThreshold) APSA_UNLIKELY {
                    return false;
                }

                L.at(i,j) = std::sqrt(discriminant);
            } else {
                scalar sum{0.0f};
                for(std::size_t k{0}; k<j; ++k) {
                    sum += L.at(i,k) * L.at(j,k);
                }
                L.at(i,j) = (a.at(i,j) - sum) / L.at(j,j);
            }
        }
    }
    return true;
}

template<std::size_t N>
[[nodiscard]] inline constexpr bool is_positive_definite(const Matrix_Dynamic<N,N>& a) noexcept {
   
    const std::size_t n = a.rows();
    Matrix_Dynamic<N,N> L{n, n};
    return cholesky(a, L);
}


// LU DECOMPOSITION
// general matrix → P×L×U


template<std::size_t N>
[[nodiscard]] inline constexpr bool lu_decompose(const Matrix_Dynamic<N,N>& a,Matrix_Dynamic<N,N>& L, Matrix_Dynamic<N,N>& U,Matrix_Dynamic<N,N>& P) noexcept {
   
    const std::size_t n = a.rows();
    L = Matrix_Dynamic<N,N>{n, n};
    U = Matrix_Dynamic<N,N>{n, n};
    P = Matrix_Dynamic<N,N>{n, n};

    for (std::size_t i{0}; i < n; ++i) {
        P.at(i,i) = scalar{1.0f};
    }

    Matrix_Dynamic<N,N> a_copy{a.rows(), a.cols()};
    for (std::size_t ii{0}; ii < n; ++ii) {
        for (std::size_t jj{0}; jj < n; ++jj) {
            a_copy.at(ii,jj) = a.at(ii,jj);
        }
    }

    for (std::size_t i{0}; i < n; ++i) {
        const std::size_t pivot_row = find_max_in_column(a_copy, i, i);

        if (std::abs(a_copy.at(pivot_row, i)) <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }

        if (pivot_row != i) {
            if (!swap_rows(a_copy, i, pivot_row)) APSA_UNLIKELY {
                return false;
            }
            if (!swap_rows(P, i, pivot_row)) APSA_UNLIKELY {
                return false;
            }
        }

        for (std::size_t j{0}; j < n; ++j) {
            if (j < i) {
                scalar sum{0.0f};
                for (std::size_t k{0}; k < j; ++k) {
                    sum += L.at(i,k) * U.at(k,j);
                }
                L.at(i,j) = (a_copy.at(i,j) - sum) / U.at(j,j);
            } else {
                scalar sum{0.0f};
                for (std::size_t k{0}; k < i; ++k) {
                    sum += L.at(i,k) * U.at(k,j);
                }
                U.at(i,j) = a_copy.at(i,j) - sum;
                if (i == j) {
                    L.at(i,j) = scalar{1.0f};
                }
            }
        }
    }

    scalar det{1.0f};
    for (std::size_t i{0}; i < n; ++i) {
        det *= U.at(i,i);
    }

    if (std::abs(det) < kDetThreshold) APSA_UNLIKELY {
        return false;
    }

    return true;
}


// FORWARD + BACK SUBSTITUTION
// triangular solve


template<std::size_t N>
[[nodiscard]] inline constexpr bool forward_substitute(const Vector_Dynamic<N>& b,const Matrix_Dynamic<N,N>& L, Vector_Dynamic<N>& y) noexcept {
    
    const std::size_t n = b.size();
    y = Vector_Dynamic<N>{n};

    for(std::size_t i{0}; i<n; ++i) {
        if(L.at(i,i) <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }

        scalar sum{0.0f};
        for(std::size_t k{0}; k<i; ++k) {
            sum += L.at(i,k) * y.at(k);
        }
        y.at(i) = (b.at(i) - sum) / L.at(i,i);
    }
    return true;
}

template<std::size_t N>
[[nodiscard]] inline constexpr bool backward_substitute(const Vector_Dynamic<N>& b,const Matrix_Dynamic<N,N>& U,Vector_Dynamic<N>& x) noexcept {
   
    const std::size_t n = b.size();
    x = Vector_Dynamic<N>{n};

    std::size_t i = n;
    while (i > 0) {
        --i;

        if(U.at(i,i) <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }

        scalar sum{0.0f};
        for(std::size_t k{i+1}; k<n; ++k) {
            sum += U.at(i,k) * x.at(k);
        }
        x.at(i) = (b.at(i) - sum) / U.at(i,i);
    }
    return true;
}


// SOLVERS
// general + SPD solve


template<std::size_t N>
[[nodiscard]] inline constexpr bool solve(const Matrix_Dynamic<N,N>& A,const Vector_Dynamic<N>& b,Vector_Dynamic<N>& x) noexcept {
    
    Matrix_Dynamic<N,N> L{N,N};
    Matrix_Dynamic<N,N> U{N,N};
    Matrix_Dynamic<N,N> P{N,N};

    if (!lu_decompose(A, L, U, P)) return false;

    Vector_Dynamic<N> Pb{N};
    Pb = P * b;

    Vector_Dynamic<N> y{N};
    if (!forward_substitute(Pb, L, y)) return false;

    if (!backward_substitute(y, U, x)) return false;

    return true;
}

template<std::size_t N>
[[nodiscard]] inline constexpr bool solve_psd(const Matrix_Dynamic<N,N>& A,const Vector_Dynamic<N>& b,Vector_Dynamic<N>& x) noexcept {
    
    const std::size_t n = A.rows();
    Matrix_Dynamic<N,N> L{n, n};

    if(!cholesky(A, L)) APSA_UNLIKELY {
        return false;
    }

    Vector_Dynamic<N> y{n};
    if(!forward_substitute(b, L, y)) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<N,N> Lt = transpose_Dynamic(L);
    if(!backward_substitute(y, Lt, x)) APSA_UNLIKELY {
        return false;
    }
    return true;
}

//--------------------------------------------------------
//--------------------------------------------------------


// QR DECOMPOSITION
// Householder — A → Q×R


template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr bool qr_decompose( const Matrix_Dynamic<MatR, MatC>& A,Matrix_Dynamic<MatR, MatR>& Q,Matrix_Dynamic<MatR, MatC>& R) noexcept {
   
    if (!(MatR >= MatC)) APSA_UNLIKELY {
        return false;
    }

    R = A;
    Q = Identity_Dynamic<MatR>();

    for (std::size_t col{0}; col < MatC; ++col) {
        const std::size_t sub_start = col;
        const std::size_t sub_size = MatR - col;

        Vector_Dynamic<MatR> x = get_sub_column(R, sub_start, col);

        const scalar norm_x = std::sqrt(norm_sq_dynamic(x));
        if (norm_x <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }

        Vector_Dynamic<MatR> e{sub_size};
        e.at(0) = scalar{1.0f};

        Vector_Dynamic<MatR> v{sub_size};
        v = x - norm_x * e;

        const scalar vtv = dot_dynamic(v, v);
        if (vtv <= kNormSqThreshold) APSA_UNLIKELY {
            return false;
        }

        const scalar beta = scalar{2.0f} / vtv;

        // R update
        for (std::size_t j{col}; j < MatC; ++j) {
            scalar dot_v{0.0f};
            for (std::size_t i{0}; i < sub_size; ++i) {
                dot_v = std::fma(v.at(i), R.at(i + sub_start, j), dot_v);
            }

            for (std::size_t i{0}; i < sub_size; ++i) {
                R.at(i + sub_start, j) =
                    std::fma(-beta * v.at(i), dot_v, R.at(i + sub_start, j));
            }
        }

        // Q update
        for (std::size_t i{0}; i < MatR; ++i) {
            scalar dot_q{0.0f};
            for (std::size_t k{0}; k < sub_size; ++k) {
                dot_q = std::fma(Q.at(i, k + sub_start), v.at(k), dot_q);
            }

            for (std::size_t k{0}; k < sub_size; ++k) {
                Q.at(i, k + sub_start) =
                    std::fma(-beta * dot_q, v.at(k), Q.at(i, k + sub_start));
            }
        }
    }

    return true;
}



// BIDIAGONALIZATION
// SVD-র আগে A → bidiagonal


template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr bool bidiagonalize(const Matrix_Dynamic<MatR,MatC>& A,Matrix_Dynamic<MatR,MatC>& B) noexcept {
    
    if (MatR < MatC) APSA_UNLIKELY {
        return false;
    }

    B = A;

    for (std::size_t i{0}; i < MatC; ++i) {
        const std::size_t sub_size = MatR - i;
        const std::size_t col_index = i;
        const std::size_t start_row = i;

        // Left Householder
        Vector_Dynamic<MatR> x = get_sub_column(B, start_row, col_index);

        const scalar norm_x = std::sqrt(norm_sq_dynamic(x));

        if (norm_x <= kNormSqThreshold) APSA_UNLIKELY {
            continue;
        }

        Vector_Dynamic<MatR> e{sub_size};
        e.at(0) = scalar{1.0f};

        Vector_Dynamic<MatR> v{sub_size};
        v = x - norm_x * e;

        const scalar vtv = dot_dynamic(v, v);

        if (vtv <= kNormSqThreshold) APSA_UNLIKELY {
            continue;
        }

        const scalar beta = scalar{2.0f} / vtv;

        // Left apply — column
        for (std::size_t j{col_index}; j < MatC; ++j) {
            scalar dot_v{0.0f};
            for (std::size_t k{0}; k < sub_size; ++k) {
                dot_v = std::fma(v.at(k), B.at(k + start_row, j), dot_v);
            }
            for (std::size_t k{0}; k < sub_size; ++k) {
                B.at(k + start_row, j) =
                    std::fma(-beta * v.at(k), dot_v, B.at(k + start_row, j));
            }
        }

        // Right Householder
        if (i < MatC - 1) {
            const std::size_t row_index = i;
            const std::size_t start_col = i + 1;
            const std::size_t sub_size_r = MatC - start_col;

            Vector_Dynamic<MatC> xr = get_sub_row(B, start_col, row_index);

            const scalar norm_xr = std::sqrt(norm_sq_dynamic(xr));

            if (norm_xr <= kNormSqThreshold) APSA_UNLIKELY {
                continue;
            }

            Vector_Dynamic<MatC> er{sub_size_r};
            er.at(0) = scalar{1.0f};

            Vector_Dynamic<MatC> vr{sub_size_r};
            vr = xr - norm_xr * er;

            const scalar vtv_r = dot_dynamic(vr, vr);

            if (vtv_r <= kNormSqThreshold) APSA_UNLIKELY {
                continue;
            }

            const scalar beta_r = scalar{2.0f} / vtv_r;

            // Right apply — row
            for (std::size_t r{0}; r < MatR; ++r) {
                scalar dot_r{0.0f};
                for (std::size_t c{0}; c < sub_size_r; ++c) {
                    dot_r = std::fma(vr.at(c), B.at(r, c + start_col), dot_r);
                }
                for (std::size_t c{0}; c < sub_size_r; ++c) {
                    B.at(r, c + start_col) =
                        std::fma(-beta_r * vr.at(c), dot_r, B.at(r, c + start_col));
                }
            }
        }
    }

    return true;
}


// SVD
// A → U × Σ × Vᵀ

template<std::size_t MatR, std::size_t MatC>
[[nodiscard]] inline constexpr bool svd( const Matrix_Dynamic<MatR,MatC>& A, Matrix_Dynamic<MatR,MatR>& U,Matrix_Dynamic<MatC,MatC>& V, Vector_Dynamic<MatC>& singular_values) noexcept {
   
    if (MatR < MatC) APSA_UNLIKELY {
        return false;
    }

    Matrix_Dynamic<MatR,MatC> B{MatR, MatC};

    if (!bidiagonalize(A, B)) {
        return false;
    }

    U = Identity_Dynamic<MatR>();
    V = Identity_Dynamic<MatC>();

    constexpr std::size_t kMaxIter{100};
    constexpr scalar kTol{1e-12};

    for (std::size_t iter{0}; iter < kMaxIter; ++iter) {
        bool converged = true;
        for (std::size_t i{0}; i < MatC - 1; ++i) {
            const scalar threshold = kTol *
                (std::abs(B.at(i,i)) + std::abs(B.at(i+1,i+1)));

            if (std::abs(B.at(i, i+1)) > threshold) {
                converged = false;
                break;
            }
        }

        if (converged) {
            break;
        }

        for (std::size_t i{0}; i < MatC - 1; ++i) {
            const scalar d0 = B.at(i, i);
            const scalar e0 = B.at(i, i + 1);
            const scalar d1 = B.at(i + 1, i + 1);

            const scalar shift = d1;
            const scalar a = d0 - shift;
            const scalar b = e0;

            const scalar r = std::sqrt(std::fma(a, a, b * b));
            if (r <= kTol) APSA_UNLIKELY {
                continue;
            }

            const scalar c = a / r;
            const scalar s = b / r;

            // Left rotation
            for (std::size_t col{0}; col < MatC; ++col) {
                const scalar t0 = B.at(i, col);
                const scalar t1 = B.at(i + 1, col);

                B.at(i, col)     =  c * t0 + s * t1;
                B.at(i + 1, col) = -s * t0 + c * t1;
            }

            for (std::size_t col{0}; col < MatR; ++col) {
                const scalar t0 = U.at(col, i);
                const scalar t1 = U.at(col, i + 1);

                U.at(col, i)     =  c * t0 + s * t1;
                U.at(col, i + 1) = -s * t0 + c * t1;
            }

            // Right rotation
            for (std::size_t row{0}; row < MatR; ++row) {
                const scalar t0 = B.at(row, i);
                const scalar t1 = B.at(row, i + 1);

                B.at(row, i)     =  c * t0 + s * t1;
                B.at(row, i + 1) = -s * t0 + c * t1;
            }

            for (std::size_t row{0}; row < MatC; ++row) {
                const scalar t0 = V.at(row, i);
                const scalar t1 = V.at(row, i + 1);

                V.at(row, i)     =  c * t0 + s * t1;
                V.at(row, i + 1) = -s * t0 + c * t1;
            }
        }
    }

    for (std::size_t i{0}; i < MatC; ++i) {
        singular_values.at(i) = std::abs(B.at(i,i));
    }

    return true;
}

// rank — threshold-র বেশি singular values count

template<std::size_t N>
[[nodiscard]] inline constexpr std::size_t rank( const Vector_Dynamic<N>& sigma_vector) noexcept {
    
    std::size_t result{0};

    scalar sigma_max{0.0f};
    for (std::size_t i{0}; i < sigma_vector.size(); ++i) {
        sigma_max = std::max(sigma_max, std::abs(sigma_vector.at(i)));
    }

    if (sigma_max <= kNormSqThreshold) {
        return 0;
    }

    const scalar threshold = kNormSqThreshold * sigma_max;

    for (std::size_t i{0}; i < sigma_vector.size(); ++i) {
        if (std::abs(sigma_vector.at(i)) > threshold) {
            ++result;
        }
    }

    return result;
}

//------------------------------------------------------
//------------------------------------------------------



// 1D LOOKUP TABLE

template<std::size_t N>
[[nodiscard]] inline constexpr bool lookup_table(const Vector_Dynamic<N>& x, const Vector_Dynamic<N>& y,const scalar x_query,scalar& y_result) noexcept {
   
    if (x.size() != y.size() || x.size() < 2) APSA_UNLIKELY {
        return false;
    }

    if (x_query < x.at(0) || x_query > x.at(x.size() - 1)) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{1}; i < x.size(); ++i) {
        if (x.at(i) <= x.at(i - 1)) APSA_UNLIKELY {
            return false;
        }
    }

    for (std::size_t i{0}; i < x.size() - 1; ++i) {
        if (x_query >= x.at(i) && x_query <= x.at(i + 1)) {
            const scalar dx = x.at(i + 1) - x.at(i);

            if (dx <= kNormSqThreshold) APSA_UNLIKELY {
                return false;
            }

            const scalar slope = (y.at(i + 1) - y.at(i)) / dx;
            y_result = std::fma(x_query - x.at(i), slope, y.at(i));
            return true;
        }
    }

    return false;
}



template<std::size_t N>
[[nodiscard]] inline constexpr bool lookup_table(const std::array<scalar, N>& x, const std::array<scalar, N>& y,const scalar x_query,scalar& y_result) noexcept {
    if (N < 2) APSA_UNLIKELY {
        return false;
    }

    if (x_query < x[0] || x_query > x[N - 1]) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{1}; i < N; ++i) {
        if (x[i] <= x[i - 1]) APSA_UNLIKELY {
            return false;
        }
    }

    for (std::size_t i{0}; i < N - 1; ++i) {
        if (x_query >= x[i] && x_query <= x[i + 1]) {
            const scalar dx = x[i + 1] - x[i];

            if (dx <= kNormSqThreshold) APSA_UNLIKELY {
                return false;
            }

            const scalar slope = (y[i + 1] - y[i]) / dx;
            y_result = std::fma(x_query - x[i], slope, y[i]);
            return true;
        }
    }

    return false;
}




// N-D LOOKUP — STATIC


template<std::size_t Dim, std::size_t MaxAxisLen, std::size_t MaxDataLen>
class LookupNDStatic {
public:
    LookupNDStatic() = default;

    [[nodiscard]] bool init(const std::array<std::array<scalar, MaxAxisLen>, Dim>& axes,const std::size_t axes_len,const std::array<scalar, MaxDataLen>& data,const std::size_t data_len) noexcept {
        
        if (axes_len > MaxAxisLen) return false;
        if (data_len > MaxDataLen) return false;

        axes_ = axes;
        axes_len_ = axes_len;
        data_ = data;
        data_len_ = data_len;
        return true;
    }

[[nodiscard]] inline bool evaluate(const std::array<scalar, Dim>& query,scalar& result) const noexcept {
    
    if (axes_len_ == 0 || data_len_ == 0) return false;

    std::size_t lo[Dim]{};
    scalar t[Dim]{};

    for (std::size_t d{0}; d < Dim; ++d) {
        if (!find_interval(axes_[d], axes_len_, query[d], lo[d], t[d])) {
            return false;
        }
    }

    constexpr std::size_t corners = 1U << Dim;
    scalar values[corners]{};

    for (std::size_t c{0}; c < corners; ++c) {
        std::size_t idx = 0;
        std::size_t stride = 1;

        for (std::size_t d = Dim; d-- > 0;) {
            const std::size_t offset = (c >> d) & 1U;
            idx += (lo[d] + offset) * stride;
            stride *= axes_len_;
        }

        if (idx >= data_len_) return false;
        values[c] = data_[idx];
    }

    for (std::size_t d{0}; d < Dim; ++d) {
        const std::size_t step = 1U << (Dim - 1U - d);
        for (std::size_t c{0}; c < step; ++c) {
            values[c] = values[2U * c] +
                        t[d] * (values[2U * c + 1U] - values[2U * c]);
        }
    }

    result = values[0];
    return true;
}


private:
    [[nodiscard]] static bool find_interval(const std::array<scalar, MaxAxisLen>& axis,const std::size_t len,const scalar q,std::size_t& lo,scalar& t) noexcept {
       
        if ((q < axis[0]) || (q > axis[len - 1U])) return false;

        std::size_t low{0};
        std::size_t high = len - 1U;

        while ((high - low) > 1U) {
            const std::size_t mid = low + ((high - low) / 2U);
            if (axis[mid] <= q) {
                low = mid;
            } else {
                high = mid;
            }
        }

        const scalar denom = axis[high] - axis[low];
        if (denom <= kNormSqThreshold) return false;

        lo = low;
        t = (q - axis[low]) / denom;
        return true;
    }

    std::array<std::array<scalar, MaxAxisLen>, Dim> axes_{};
    std::array<scalar, MaxDataLen> data_{};
    std::size_t axes_len_{0};
    std::size_t data_len_{0};
};


// N-D LOOKUP — RUNTIME


template<std::size_t MaxDim, std::size_t MaxAxisLen, std::size_t MaxDataLen>
class LookupNDRuntime {
public:
    LookupNDRuntime() = default;

    [[nodiscard]] bool configure(const std::size_t dim,const std::size_t axis_len,const std::size_t data_len) noexcept {
       
        if ((dim == 0U) || (dim > MaxDim)) return false;
        if ((axis_len < 2U) || (axis_len > MaxAxisLen)) return false;
        if ((data_len == 0U) || (data_len > MaxDataLen)) return false;

        dim_ = dim;
        axis_len_ = axis_len;
        data_len_ = data_len;
        return true;
    }

    [[nodiscard]] bool set_axis( const std::size_t dim_idx,const std::array<scalar, MaxAxisLen>& axis) noexcept {
       
        if (dim_idx >= dim_) return false;
        axes_[dim_idx] = axis;
        return true;
    }

    [[nodiscard]] bool set_data( const std::array<scalar, MaxDataLen>& data) noexcept {
        data_ = data;
        return true;
    }

    [[nodiscard]] bool evaluate(const std::array<scalar, MaxDim>& query,scalar& result) const noexcept {
        
        if ((dim_ == 0U) || (axis_len_ < 2U) || (data_len_ == 0U)) {
            return false;
        }

        std::size_t lo[MaxDim]{};
        scalar t[MaxDim]{};

        for (std::size_t d{0}; d < dim_; ++d) {
            if (!find_interval(axes_[d], axis_len_, query[d], lo[d], t[d])) {
                return false;
            }
        }

        constexpr std::size_t max_corners = 1U << MaxDim;
        scalar values[max_corners]{};
        const std::size_t corners = 1U << dim_;

        for (std::size_t c{0}; c < corners; ++c) {
            std::size_t idx = 0;
            std::size_t stride = 1;

            for (std::size_t d{0}; d < dim_; ++d) {
                const std::size_t offset = (c >> d) & 1U;
                idx += (lo[d] + offset) * stride;
                stride *= axis_len_;
            }

            if (idx >= data_len_) return false;
            values[c] = data_[idx];
        }

        for (std::size_t d{0}; d < dim_; ++d) {
            const std::size_t step = 1U << (dim_ - 1U - d);
            for (std::size_t c{0}; c < step; ++c) {
                values[c] = values[2U * c] +
                            t[d] * (values[2U * c + 1U] - values[2U * c]);
            }
        }

        result = values[0];
        return true;
    }

private:
    [[nodiscard]] static bool find_interval(const std::array<scalar, MaxAxisLen>& axis,const std::size_t len, const scalar q,std::size_t& lo,scalar& t) noexcept {
       
        if ((q < axis[0]) || (q > axis[len - 1U])) return false;

        std::size_t low{0};
        std::size_t high = len - 1U;

        while ((high - low) > 1U) {
            const std::size_t mid = low + ((high - low) / 2U);
            if (axis[mid] <= q) {
                low = mid;
            } else {
                high = mid;
            }
        }

        const scalar denom = axis[high] - axis[low];
        if (denom <= kNormSqThreshold) return false;

        lo = low;
        t = (q - axis[low]) / denom;
        return true;
    }

    std::array<std::array<scalar, MaxAxisLen>, MaxDim> axes_{};
    std::array<scalar, MaxDataLen> data_{};
    std::size_t dim_{0U};
    std::size_t axis_len_{0U};
    std::size_t data_len_{0U};
};

// factory — static table build
template<std::size_t Dim, std::size_t AxisLen, std::size_t DataLen>
inline constexpr auto make_lookup(const std::array<std::array<scalar, AxisLen>, Dim>& axes, const std::array<scalar, DataLen>& data)
{
    LookupNDStatic<Dim, AxisLen, DataLen> t;
    const bool ok = t.init(axes, AxisLen, data, DataLen);
    MATH_ASSERT(ok);
    return t;
}


// GAUSSIAN RNG
// seedable normal distribution


class GaussRNG {
private:
    std::mt19937 engine_;
    std::normal_distribution<scalar> dist_;

public:
    constexpr GaussRNG() = delete;

    GaussRNG(const scalar mean,const scalar stddev,const std::uint32_t seed ) : engine_(seed), dist_(mean, stddev) {
       
        MATH_ASSERT(stddev > scalar{0.0f});
    }

    [[nodiscard]] scalar next() noexcept { return dist_(engine_); }
};


// STATISTICS


template<std::size_t N>
[[nodiscard]] inline constexpr bool mean(
    const Vector_Dynamic<N>& a,scalar& mean_value) noexcept {
    
    const std::size_t n = a.size();
    if (n == 0) APSA_UNLIKELY { return false; }

    scalar sum{0.0f};
    for (std::size_t i{0}; i < n; ++i) {
        sum += a.at(i);
    }

    mean_value = sum / static_cast<scalar>(n);
    return true;
}

template<std::size_t N>
[[nodiscard]] inline constexpr bool variance(const Vector_Dynamic<N>& state,scalar& variance_value) noexcept {
    
    const std::size_t n = state.size();
    if (n == 0 || n == 1) APSA_UNLIKELY { return false; }

    scalar mu{0.0f};
    if (!mean(state, mu)) APSA_UNLIKELY { return false; }

    scalar sum{0.0f};
    for (std::size_t i{0}; i < n; ++i) {
        const scalar diff = state.at(i) - mu;
        sum += diff * diff;
    }
    
    variance_value = sum / static_cast<scalar>(n - 1);
    return true;
}

template<std::size_t N>
[[nodiscard]] inline bool covariance(const Vector_Dynamic<N>& state_x,const Vector_Dynamic<N>& state_y,scalar& covariance_value) noexcept {
   
    const std::size_t n = state_x.size();
    if (n == 0 || n == 1) APSA_UNLIKELY { return false; }
    if (state_x.size() != state_y.size()) APSA_UNLIKELY { return false; }

    scalar x_mean{0.0f};
    scalar y_mean{0.0f};
    if (!mean(state_x, x_mean)) APSA_UNLIKELY { return false; }
    if (!mean(state_y, y_mean)) APSA_UNLIKELY { return false; }

    scalar sum{0.0f};
    for (std::size_t i{0}; i < n; ++i) {
        const scalar x_diff = state_x.at(i) - x_mean;
        const scalar y_diff = state_y.at(i) - y_mean;
        sum += x_diff * y_diff;
    }

    covariance_value = sum / static_cast<scalar>(n - 1);
    return true;
}

template<std::size_t N>
[[nodiscard]] inline bool correlation(const Vector_Dynamic<N>& state_x,const Vector_Dynamic<N>& state_y,scalar& correlation_value) noexcept {
    
    scalar cov_xy{0.0f};
    if (!covariance(state_x, state_y, cov_xy)) APSA_UNLIKELY { return false; }

    scalar variance_x{0.0f};
    if (!variance(state_x, variance_x)) APSA_UNLIKELY { return false; }

    scalar variance_y{0.0f};
    if (!variance(state_y, variance_y)) APSA_UNLIKELY { return false; }

    const scalar deviation = std::sqrt(variance_x) * std::sqrt(variance_y);
    if (deviation <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    correlation_value = cov_xy / deviation;
    return true;
}


// heavy-tail distribution — hypersonic noise


constexpr scalar kPi{3.14159265358979323846f};
constexpr scalar kTwoPi{6.28318530717958647692f};
constexpr scalar kHalfPi{1.57079632679489661923f};
constexpr scalar kLightSpeed{299792458.0f};
constexpr scalar kGravity{9.80665f};

[[nodiscard]] inline bool student_coefficient(const scalar v,scalar& coefficient_result) noexcept {
   
    if (v <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar numerator = std::tgamma(scalar{0.5f} * v + scalar{1.0f});
    const scalar v_pi = v * kPi;
    const scalar sqrt_v = std::sqrt(v_pi);
    const scalar gamma_value = v / scalar{2};
    const scalar denominator = sqrt_v * std::tgamma(gamma_value);

    coefficient_result = numerator / denominator;
    return true;
}


[[nodiscard]] inline bool student_t_pdf(const scalar t, const scalar v, const scalar coefficient,scalar& result) noexcept {
   
    if (v <= kNormSqThreshold) APSA_UNLIKELY { return false; }

    const scalar inv_v = scalar{1.0f} / v;
    const scalar t_sqr = t * t;
    const scalar base = std::fma(t_sqr, inv_v, scalar{1.0f});

    if (base <= scalar{0.0f}) APSA_UNLIKELY { return false; }

    const scalar exponent = -(scalar{0.5f} * v + scalar{1.0f});
    const scalar part_pdf = std::exp(exponent * std::log(base));

    result = coefficient * part_pdf;

    if (!std::isfinite(result)) APSA_UNLIKELY {
        return false;
    }

    return true;
}

//------------------------------------------------------



[[nodiscard]] inline bool quaternion_normalize(const Quaternion& q_in, Quaternion& q_out) noexcept {
    
    const scalar n_sq = norm_sq(q_in);
    if (n_sq < kNormSqThreshold) return false;
    const scalar inv_n = 1.0f / std::sqrt(n_sq);

    q_out = Quaternion{

        q_in.get_w()*inv_n, q_in.get_x()*inv_n,
        q_in.get_y()*inv_n, q_in.get_z()*inv_n
    };

    return true;
}


[[nodiscard]] inline bool up_direction(const Quaternion& q,Vec3& up) noexcept {
   
    up = rotate(q, Vec3{0.0f, 0.0f, 1.0f});
    return true;
}

[[nodiscard]] inline Mat3 rotation_matrix_from_quaternion(const Quaternion& q) noexcept {
    
    const scalar w = q.get_w(), x = q.get_x();
    const scalar y = q.get_y(), z = q.get_z();
    
    return Mat3{{
        1.0f-2.0f*(y*y+z*z), 2.0f*(x*y-w*z), 2.0f*(x*z+w*y),
        2.0f*(x*y+w*z), 1.0f-2.0f*(x*x+z*z), 2.0f*(y*z-w*x),
        2.0f*(x*z-w*y), 2.0f*(y*z+w*x), 1.0f-2.0f*(x*x+y*y)
    }};
}


