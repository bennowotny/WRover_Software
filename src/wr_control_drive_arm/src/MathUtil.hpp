#ifndef MATH_UTIL_H
#define MATH_UTIL_H

#include <cmath>

namespace MathUtil {
/**
 * @brief Mathematically correct modulus function
 *
 * @param dividend Division dividend
 * @param divisor Division divisor
 * @return double The result of dividend (mod divisor) on the interval [0, abs(divisor))
 */
auto corrMod(double dividend, double divisor) -> double;

/// Helper constant for circle measurements
static constexpr double RADIANS_PER_ROTATION{2 * M_PI};
} // namespace MathUtil

#endif