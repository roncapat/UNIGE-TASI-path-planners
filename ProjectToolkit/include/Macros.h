/** @file */
#ifndef RONCAPAT_PLANNER_TOOLKIT_MACROS_H
#define RONCAPAT_PLANNER_TOOLKIT_MACROS_H
/** Approximation of the squared root of 2 */
extern const float SQRT2;

/**
 * Compute the square of a number (single-precision float) */
#define SQUARE(x) (float(x)*float(x))

/** Compute the cathetus of a triangle, same as sqrt(x^2-y^2) (single-precision float) */
#define CATH(x, y) std::sqrt(float(SQUARE((x))- SQUARE((y))))

/** Compute the hypotenuse of a triangle, same as sqrt(x^2+y^2) (single-precision float) */
#define HYPOT(x, y) (float)std::hypotf(x,y)

/**
 * Interpolate among two values, given an offset from the "from" position.
 * This version is used for interpolant that have exaclty distance 1.
 * */
#define INTERP_1(from, to, delta) ((from) + ((to)-(from))*(delta))

/**
 * Interpolate among two values, given an offset from the "from" position.
 * This version is used for interpolant that in principle can be at distances greater or lowe than 1.
 * */
#define INTERP_ABS(from, to, delta) ((from) + ((to)-(from))/std::abs((to)-(from))*(delta))

/** With debug disabled, this is the same as using return statement */
/** With debug enabled, positiveness of x is asserted before returning the value */
#ifdef NDEBUG
#define RETURN_CHECK_POSITIVE(x) return(x)
#else
#define RETURN_CHECK_POSITIVE(x) auto _y = x; assert(_y>0); return(_y);
#endif

/** With debug disabled, this is the same as using return statement */
/** With debug enabled, positiveness and finitness of x is asserted before returning the value */
#ifdef NDEBUG
#define RETURN_CHECK_POSITIVE_LIMITED(x) return(x)
#else
#define RETURN_CHECK_POSITIVE_LIMITED(x) auto _y = x; assert(_y>0 and _y<INFINITY); return(_y);
#endif

#if __cplusplus > 201703L // std::optional comes with C++17
#include <optional>
using std::optional;
using std::nullopt;
#else // use code from P0798R0 proposal (to permit C++11 compilation - eg. RTEMS 5)
#include <tl/optional.hpp>
using tl::optional;
using tl::nullopt;
#endif

#endif //RONCAPAT_PLANNER_TOOLKIT_MACROS_H
