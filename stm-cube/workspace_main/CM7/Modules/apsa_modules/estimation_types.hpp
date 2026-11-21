#pragma once
#include "apsa_math_engine.hpp"

// ═══════════════════════════════════════════════════════════
// APSA ESTIMATION — SHARED FUNCTION POINTER TYPES
// Universal | DO-178C Level A | No Domain-Specific
// ═══════════════════════════════════════════════════════════

// State transition: x' = f(x, u)
template<std::size_t N, std::size_t U = 6>
using StateTransitionFnT = Vector_Dynamic<N> (*)(
    const Vector_Dynamic<N>& x,
    const Vector_Dynamic<U>& u
) noexcept;

// Measurement: z = h(x)
template<std::size_t N, std::size_t M>
using MeasurementFnT = Vector_Dynamic<M> (*)(
    const Vector_Dynamic<N>& x
) noexcept;

// State Jacobian: F = ∂f/∂x
template<std::size_t N, std::size_t U = 6>
using StateJacobianFnT = Matrix_Dynamic<N, N> (*)(
    const Vector_Dynamic<N>& x,
    const Vector_Dynamic<U>& u
) noexcept;

// Measurement Jacobian: H = ∂h/∂x
template<std::size_t N, std::size_t M>
using MeasurementJacobianFnT = Matrix_Dynamic<M, N> (*)(
    const Vector_Dynamic<N>& x
) noexcept;
