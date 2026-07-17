#pragma once

#include <cstdint>

// shared low level vocabulary no domain knowledge lives here, only the
// primitive aliases every layer above uses

using byte = unsigned char;

// vec_t is the scalar precision of a stage geometry stages (csg, bsp) select
// double for robustness against imprecise editor input; lighting stages (vis,
// rad) select float for speed and memory a stage sets this before including
// the math headers, so vec3<vec_t> resolves to the right precision
#ifndef HLTOOLS_DOUBLE_PRECISION
using vec_t = float;
#else
using vec_t = double;
#endif
