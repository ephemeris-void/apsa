#pragma once
#include "apsa_math_engine.hpp"
#include <cstdint>  
#include <array>
#include <cmath>

// (removed stray slash)

// Dimensionless + Quantity


using scalar = float;

template<std::int32_t... Dims>
struct Quantity {
private:
    scalar value{0.0f};
public:
    constexpr Quantity() = default;
    constexpr explicit Quantity(scalar val) noexcept : value(val) {}

    [[nodiscard]] constexpr scalar get() const noexcept { return value; }
};

using Dimensionless = Quantity<0,0,0>;

// Dimension Math

template<typename A, typename B>
struct DimAdd;

template<std::int32_t... A, std::int32_t... B>
struct DimAdd<Quantity<A...>, Quantity<B...>> {
    using type = Quantity<(A + B)...>;
};

template<typename A, typename B>
struct DimSub;

template<std::int32_t... A, std::int32_t... B>
struct DimSub<Quantity<A...>, Quantity<B...>> {
    using type = Quantity<(A - B)...>;
};

// Quantity Operators


template<std::int32_t... Dims>
[[nodiscard]] constexpr Quantity<Dims...> operator+( Quantity<Dims...> a,Quantity<Dims...> b) noexcept {
    return Quantity<Dims...>{a.get() + b.get()};
}

template<std::int32_t... Dims>
[[nodiscard]] constexpr Quantity<Dims...> operator-( Quantity<Dims...> a,Quantity<Dims...> b) noexcept {
    return Quantity<Dims...>{a.get() - b.get()};
}

template<std::int32_t... A, std::int32_t... B>
[[nodiscard]] constexpr auto operator*(Quantity<A...> a,Quantity<B...> b) noexcept {

    using Result = typename DimAdd<Quantity<A...>, Quantity<B...>>::type;
    return Result{a.get() * b.get()};
}

template<std::int32_t... A, std::int32_t... B>
[[nodiscard]] constexpr auto operator/( Quantity<A...> a,Quantity<B...> b) noexcept {
    
    using Result = typename DimSub<Quantity<A...>, Quantity<B...>>::type;
    return Result{a.get() / b.get()};
}

// Physical Unit Types 

using Mass        = Quantity<1,0,0>;
using Length      = Quantity<0,1,0>;
using Time        = Quantity<0,0,1>;

using Temperature = Quantity<0,0,0,1>;
using Voltage     = Quantity<1,2,-3,-1>;
using Current     = Quantity<0,0,-1,1>;

using Volume = Quantity<0,3,0>;  
using Velocity     = Quantity<0,1,-1>;
using Acceleration = Quantity<0,1,-2>;
using Force        = Quantity<1,1,-2>;
using Torque       = Quantity<1,2,-2>;
using AngularVel   = Quantity<0,0,-1>;
using Momentum     = Quantity<1,1,-1>;
using AngularMom = Quantity<1,2,-1>;         
using SpecificAngularMom = Quantity<0,2,-1>;  
using Density      = Quantity<1,-3,0>;
using Pressure     = Quantity<1,-1,-2>;
using Energy       = Quantity<1,2,-2>;
using Power        = Quantity<1,2,-3>;

using GravitationalParameter = Quantity<0,3,-2>;
using SpecificImpulse       = Time;
using MassFlowRate          = Quantity<1,0,-1>;
using MomentInertia         = Quantity<1,2,0>;

using Radian = Quantity<0,0,0>;
using Degree = Quantity<0,0,0>;
using Angle = Radian;

using HeatFlux = Quantity<1,-3,0>;
using Viscosity = Quantity<1,-1,-1>;
using Enthalpy = Quantity<0,2,-2>;            
using SpecificHeat = Quantity<0,2,-2,-1>;     
using ThermalConductivity = Quantity<1,1,-3,-1>;
using VelocityGradient = Quantity<0,0,-1>;
using PyrolysisRate = Quantity<1,-3,-1>;
using Frequency = Quantity<0,0,-1>;
using MassFlux = Quantity<1,-2,-1>;
using SurfaceTempRate = Quantity<0,0,-1,1>;
using ConvectionCoefficient = Quantity<1,0,-3,-1>;

using Stress = Quantity<1,-1,-2>;        
using ExpansionCoeff = Quantity<0,0,0,-1>; 
using InertiaTensor = Quantity<1,2,0>;
using KinematicViscosity = Quantity<0,2,-1>;
using EnergyRate = Quantity<1,2,-3>;  
using Area = Quantity<0,2,0>;  
using AngularAccel = Quantity<0,0,-2>;
using MagneticField = Quantity<1,0,-2,-1>;

// ─── Dimension Helpers ───

template<typename Unit>
using DimSq = typename DimAdd<Unit, Unit>::type;

template<typename Unit>
using DimCube = typename DimAdd<DimSq<Unit>, Unit>::type;

template<typename Unit>
using DimInv = typename DimSub<Dimensionless, Unit>::type;

template<typename Unit>
struct DimSqrtImpl;

template<std::int32_t... Dims>
struct DimSqrtImpl<Quantity<Dims...>> {
    using type = Quantity<(Dims / 2)...>;
};

template<typename Unit>
using DimSqrt = typename DimSqrtImpl<Unit>::type;

template<typename Unit, std::int32_t N>
struct DimScaleImpl;

template<std::int32_t... Dims, std::int32_t N>
struct DimScaleImpl<Quantity<Dims...>, N> {
    using type = Quantity<(Dims * N)...>;
};

template<typename Unit, std::int32_t N>
using DimScale = typename DimScaleImpl<Unit, N>::type;

// UnitVec3


template<typename Unit>
struct UnitVec3 {
    Vec3 data{};

    constexpr UnitVec3() = default;
    constexpr explicit UnitVec3(Vec3 v) noexcept : data(v) {}
    
    [[nodiscard]] constexpr scalar& x() noexcept { return data.x(); }
    [[nodiscard]] constexpr scalar& y() noexcept { return data.y(); }
    [[nodiscard]] constexpr scalar& z() noexcept { return data.z(); }
    [[nodiscard]] constexpr scalar x() const noexcept { return data.x(); }
    [[nodiscard]] constexpr scalar y() const noexcept { return data.y(); }
    [[nodiscard]] constexpr scalar z() const noexcept { return data.z(); }
};

//UnitVec3 Operators 

template<typename Unit>
[[nodiscard]] constexpr UnitVec3<Unit> operator+( UnitVec3<Unit> a,const UnitVec3<Unit>& b) noexcept {
    return UnitVec3<Unit>{a.data + b.data};
}

template<typename Unit>
[[nodiscard]] constexpr UnitVec3<Unit> operator-(UnitVec3<Unit> a,const UnitVec3<Unit>& b) noexcept {
    return UnitVec3<Unit>{a.data - b.data};
}

template<typename Unit>
[[nodiscard]] constexpr UnitVec3<Unit> operator*(UnitVec3<Unit> v,scalar s) noexcept {
    return UnitVec3<Unit>{v.data * s};
}

template<typename Unit>
[[nodiscard]] constexpr UnitVec3<Unit> operator*(scalar s,UnitVec3<Unit> v) noexcept {
    return UnitVec3<Unit>{v.data * s};
}
// UnitVec3<Unit> × Time - UnitVec3<Unit×Time>
template<typename Unit>
[[nodiscard]] constexpr auto operator*(UnitVec3<Unit> v,const Time t) noexcept {
    using ResultUnit = typename DimAdd<Unit, Time>::type;
    return UnitVec3<ResultUnit>{v.data * t.get()};
}

// Time × UnitVec3<Unit> - UnitVec3<Unit×Time>
template<typename Unit>
[[nodiscard]] constexpr auto operator*(const Time t,UnitVec3<Unit> v) noexcept {
    using ResultUnit = typename DimAdd<Unit, Time>::type;
    return UnitVec3<ResultUnit>{v.data * t.get()};
}

template<typename Unit>
[[nodiscard]] constexpr UnitVec3<Unit> operator/(UnitVec3<Unit> v,scalar s) noexcept {
    
    return UnitVec3<Unit>{v.data / s};
}

//UnitVec3 Dot + Cross

template<typename UnitA, typename UnitB>
[[nodiscard]] constexpr auto dot(UnitVec3<UnitA> a,UnitVec3<UnitB> b) noexcept {
    using ResultUnit = typename DimAdd<UnitA, UnitB>::type;
    return ResultUnit{dot(a.data, b.data)};
}

template<typename UnitA, typename UnitB>
[[nodiscard]] constexpr auto cross(UnitVec3<UnitA> a,UnitVec3<UnitB> b) noexcept {
    using ResultUnit = typename DimAdd<UnitA, UnitB>::type;
    using ResultVec = UnitVec3<ResultUnit>;
    return ResultVec{cross(a.data, b.data)};
}

// UnitVec3 Norm


template<typename Unit>
[[nodiscard]] constexpr DimSq<Unit> norm_sq(const UnitVec3<Unit>& v) noexcept {
    return DimSq<Unit>{dot(v.data, v.data)};
}

template<typename Unit>
[[nodiscard]] constexpr Unit norm(const UnitVec3<Unit>& v) noexcept {
    return Unit{std::sqrt(norm_sq(v).get())};
}

template<typename Unit>
[[nodiscard]] constexpr UnitVec3<Dimensionless> normalize(const UnitVec3<Unit>& v) noexcept {
    
    const scalar n = norm(v).get();

    if (n <= kNormSqThreshold) APSA_UNLIKELY {
        return UnitVec3<Dimensionless>{Vec3{}};
    }

    const scalar inv = scalar{1.0f} / n;
    return UnitVec3<Dimensionless>{v.data * inv};
}

// ─── sqrt for Quantity ───


template<typename Unit>
[[nodiscard]] constexpr DimSqrt<Unit> sqrt_q(const Unit& u) noexcept {
    return DimSqrt<Unit>{std::sqrt(u.get())};
}

// UnitVectorDynamic


template<std::size_t N, typename Unit>
struct UnitVectorDynamic {
    Vector_Dynamic<N> data{};

    UnitVectorDynamic() = default;
    explicit UnitVectorDynamic(std::size_t size) noexcept : data(size) {}

    [[nodiscard]] std::size_t size() const noexcept { return data.size(); }
    [[nodiscard]] constexpr scalar& at(std::size_t i) noexcept { return data.at(i); }
    [[nodiscard]] constexpr scalar at(std::size_t i) const noexcept { return data.at(i); }
};

//UnitVectorDynamic Operators

template<std::size_t N, typename Unit>
[[nodiscard]] constexpr UnitVectorDynamic<N, Unit> operator+(UnitVectorDynamic<N, Unit> a,const UnitVectorDynamic<N, Unit>& b) noexcept {
    UnitVectorDynamic<N, Unit> result{a.size()};
    for (std::size_t i{0}; i < a.size(); ++i) {
        result.at(i) = a.at(i) + b.at(i);
    }
    return result;
}

template<std::size_t N, typename Unit>
[[nodiscard]] constexpr UnitVectorDynamic<N, Unit> operator-(UnitVectorDynamic<N, Unit> a,const UnitVectorDynamic<N, Unit>& b) noexcept {
    UnitVectorDynamic<N, Unit> result{a.size()};
    for (std::size_t i{0}; i < a.size(); ++i) {
        result.at(i) = a.at(i) - b.at(i);
    }
    return result;
}

template<std::size_t N, typename Unit>
[[nodiscard]] constexpr UnitVectorDynamic<N, Unit> operator*(UnitVectorDynamic<N, Unit> v,scalar s) noexcept {
    UnitVectorDynamic<N, Unit> result{v.size()};
    for (std::size_t i{0}; i < v.size(); ++i) {
        result.at(i) = v.at(i) * s;
    }
    return result;
}

template<std::size_t N, typename Unit>
[[nodiscard]] constexpr UnitVectorDynamic<N, Unit> operator/(UnitVectorDynamic<N, Unit> v, scalar s) noexcept {
    UnitVectorDynamic<N, Unit> result{v.size()};
    for (std::size_t i{0}; i < v.size(); ++i) {
        result.at(i) = v.at(i) / s;
    }
    return result;
}

//UnitMatrixDynamic 


template<std::size_t R, std::size_t C, typename Unit>
struct UnitMatrixDynamic {
    Matrix_Dynamic<R,C> data{};

    UnitMatrixDynamic() = default;
    UnitMatrixDynamic(std::size_t rows, std::size_t cols) noexcept : data(rows, cols) {}

    [[nodiscard]] std::size_t rows() const noexcept { return data.rows(); }
    [[nodiscard]] std::size_t cols() const noexcept { return data.cols(); }
    [[nodiscard]] constexpr scalar& at(std::size_t i, std::size_t j) noexcept { return data.at(i,j); }
    [[nodiscard]] constexpr scalar at(std::size_t i, std::size_t j) const noexcept { return data.at(i,j); }
};

//UnitMatrixDynamic Operators

template<std::size_t R, std::size_t C, typename Unit>
[[nodiscard]] constexpr UnitMatrixDynamic<R, C, Unit> operator+(UnitMatrixDynamic<R, C, Unit> a,const UnitMatrixDynamic<R, C, Unit>& b) noexcept {
    return UnitMatrixDynamic<R, C, Unit>{a.data + b.data};
}

template<std::size_t R, std::size_t C, typename Unit>
[[nodiscard]] constexpr UnitMatrixDynamic<R, C, Unit> operator-(UnitMatrixDynamic<R, C, Unit> a,const UnitMatrixDynamic<R, C, Unit>& b) noexcept {
    
    return UnitMatrixDynamic<R, C, Unit>{a.data - b.data};
}

template<std::size_t R, std::size_t C, typename Unit>
[[nodiscard]] constexpr UnitMatrixDynamic<R, C, Unit> operator*(UnitMatrixDynamic<R, C, Unit> m,scalar s) noexcept {
    
    return UnitMatrixDynamic<R, C, Unit>{m.data * s};
}

// Matrix × Vector


template<std::size_t R, std::size_t C, typename UnitM, typename UnitV>
[[nodiscard]] constexpr auto operator*(const UnitMatrixDynamic<R, C, UnitM>& m, const UnitVectorDynamic<C, UnitV>& v) noexcept {
    using ResultUnit = typename DimAdd<UnitM, UnitV>::type;
    UnitVectorDynamic<R, ResultUnit> result{m.rows()};

    for (std::size_t i{0}; i < m.rows(); ++i) {
        scalar sum{0.0f};
        for (std::size_t j{0}; j < m.cols(); ++j) {
            sum = std::fma(m.at(i,j), v.at(j), sum);
        }
        result.at(i) = sum;
    }
    return result;
}

// mat_vec_mul — explicit version 

template<std::size_t R, std::size_t C, typename UnitA, typename UnitB>
[[nodiscard]] constexpr auto mat_vec_mul( const UnitMatrixDynamic<R,C,UnitA>& m,const UnitVectorDynamic<C,UnitB>& v) noexcept -> UnitVectorDynamic<R, typename DimAdd<UnitA,UnitB>::type> {
    using ResultUnit = typename DimAdd<UnitA, UnitB>::type;
    UnitVectorDynamic<R, ResultUnit> result{m.rows()};

    for (std::size_t i{0}; i < m.rows(); ++i) {
        scalar sum{0.0f};
        for (std::size_t j{0}; j < m.cols(); ++j) {
            sum = std::fma(m.at(i,j), v.at(j), sum);
        }
        result.at(i) = sum;
    }
    return result;
}


[[nodiscard]] constexpr scalar km_to_m(const scalar km) noexcept { return km * 1000.0f; }
[[nodiscard]] constexpr scalar m_to_km(const scalar m) noexcept { return m / 1000.0f; }
[[nodiscard]] constexpr scalar deg_to_rad(const scalar deg) noexcept { return deg * kPi / 180.0f; }
[[nodiscard]] constexpr scalar rad_to_deg(const scalar rad) noexcept { return rad * 180.0f / kPi; }


//------------------------------------------------------
//-------------------------------------------------------


// 1D LOOKUP — UNIT-TYPED


template<std::size_t N, typename UnitX, typename UnitY>
[[nodiscard]] constexpr bool lookup_table_typed(const std::array<UnitX, N>& x, const std::array<UnitY, N>& y, const UnitX& x_query,UnitY& y_result) noexcept {
    if constexpr (N < 2) {
        return false;
    }

    const scalar q = x_query.get();

    if (!std::isfinite(q)) APSA_UNLIKELY {
        return false;
    }

    if (q < x[0].get() || q > x[N - 1].get()) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{1}; i < N; ++i) {
        if (x[i].get() <= x[i - 1].get()) APSA_UNLIKELY {
            return false;
        }
    }

    for (std::size_t i{0}; i < N - 1; ++i) {
        const scalar x0 = x[i].get();
        const scalar x1 = x[i + 1].get();

        if (q >= x0 && q <= x1) {
            const scalar dx = x1 - x0;

            if (dx <= kNormSqThreshold) APSA_UNLIKELY {
                return false;
            }

            const scalar y0 = y[i].get();
            const scalar y1 = y[i + 1].get();

            const scalar slope = (y1 - y0) / dx;
            const scalar interp = std::fma(q - x0, slope, y0);

            if (!std::isfinite(interp)) APSA_UNLIKELY {
                return false;
            }

            y_result = UnitY{interp};
            return true;
        }
    }

    return false;
}


// 1D LOOKUP — DYNAMIC UNIT-TYPED


template<std::size_t N, typename UnitX, typename UnitY>
[[nodiscard]] constexpr bool lookup_table_typed_dynamic( const UnitVectorDynamic<N, UnitX>& x,const UnitVectorDynamic<N, UnitY>& y,const UnitX& x_query, UnitY& y_result) noexcept {
    const std::size_t len = x.size();

    if (len < 2U || len != y.size()) APSA_UNLIKELY {
        return false;
    }

    const scalar q = x_query.get();

    if (!std::isfinite(q)) APSA_UNLIKELY {
        return false;
    }

    if (q < x.at(0).get() || q > x.at(len - 1U).get()) APSA_UNLIKELY {
        return false;
    }

    for (std::size_t i{1}; i < len; ++i) {
        if (x.at(i).get() <= x.at(i - 1U).get()) APSA_UNLIKELY {
            return false;
        }
    }

    for (std::size_t i{0}; i < len - 1U; ++i) {
        const scalar x0 = x.at(i).get();
        const scalar x1 = x.at(i + 1U).get();

        if (q >= x0 && q <= x1) {
            const scalar dx = x1 - x0;

            if (dx <= kNormSqThreshold) APSA_UNLIKELY {
                return false;
            }

            const scalar y0 = y.at(i).get();
            const scalar y1 = y.at(i + 1U).get();

            const scalar slope = (y1 - y0) / dx;
            const scalar interp = std::fma(q - x0, slope, y0);

            if (!std::isfinite(interp)) APSA_UNLIKELY {
                return false;
            }

            y_result = UnitY{interp};
            return true;
        }
    }

    return false;
}

