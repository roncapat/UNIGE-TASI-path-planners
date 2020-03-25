#ifndef NODEUTILS_HPP
#define NODEUTILS_HPP

#include <type_traits>
#include <cmath>
#include <tuple>

namespace igvc {
/**
Calculates euclidian distance between two points

@tparam T the data type of the input points to calculate the euclidian distance for
@param[in] x1 x value of first point
@param[in] y1 y value of first point
@param[in] x2 x value of second point
@param[in] y2 y value of second point
@return the euclidian distance between both points
*/
template<typename T>
inline T get_distance(T x1, T y1, T x2, T y2) {
    return std::hypot(x2 - x1, y2 - y1);
}

/**
Calculates euclidian distance between two points, taking tuples for each
(x,y) point as arguments

@tparam T the data type contained within each input tuple
@param[in] p1 the first point
@param[in] p2 the second point
@return the euclidian distance between both points
*/
template<typename T>
inline T get_distance(const std::tuple<T, T> &p1, const std::tuple<T, T> &p2) {
    return igvc::get_distance(std::get<0>(p1), std::get<1>(p1), std::get<0>(p2), std::get<1>(p2));
}

/**
symmetric round up
Bias: away from zero

@tparam T the data type to round up
@param[in] the value to round up
@return the value rounded away from zero
*/
template<typename T>
T ceil0(const T &value) {
    return (value < 0.0) ? std::floor(value) : std::ceil(value);
}

/**
Adjust angle to lie within the polar range [-PI, PI]
*/
inline void fit_to_polar(double &angle) {
    angle = std::fmod(angle, 2 * M_PI);
    if (angle > M_PI) {
        angle -= 2 * M_PI;
    } else if (angle < -M_PI) {
        angle += 2 * M_PI;
    }
}
}
#endif
