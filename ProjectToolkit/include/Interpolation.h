#ifndef RONCAPAT_PLANNER_TOOLKIT_BLERP_H
#define RONCAPAT_PLANNER_TOOLKIT_BLERP_H

#include "Graph.h"
/*
   |    |     |             |
y2 |---q12----+------------q22--
   |    |     |             |
   |    |     |             |
 y |----+-----P-------------+---
   |    |     |             |
y1 |---q11----+------------q21--
   |    |     |             |
   |----+-----+-------------+---
        x1    x             x2
 */

/**
 * Bilinear interpolation
 * @param q11 value at (x1, y1)
 * @param q12 value at (x1, y2)
 * @param q21 value at (x2, y1)
 * @param q22 value at (x2, y2)
 * @param x1 first abscissa
 * @param x2 second abscissa
 * @param y1 first ordinate
 * @param y2 second ordinate
 * @param x abscissa for the interpolation
 * @param y ordinate for the interpolation
 * @return the interpolated value at (x, y)
 */
float bilinear_interp(float q11, float q12, float q21, float q22,
                            float x1, float x2, float y1, float y2,
                            float x, float y);

/**
 * Bilinear interpolation
 * @param q11 value at (x1, y1)
 * @param q12 value at (x1, y2)
 * @param q21 value at (x2, y1)
 * @param q22 value at (x2, y2)
 * @param x1 first abscissa
 * @param x2 second abscissa
 * @param y1 first ordinate
 * @param y2 second ordinate
 * @param p position (x, y) for the interpolation
 * @return the interpolated value at (x, y)
 */
float bilinear_interp(float q11, float q12, float q21, float q22,
                            float x1, float x2, float y1, float y2,
                            const Position &p);
/*
   |    |     |             |
y2 |--(p12)---+------------p22--
   |    |     |             |
   |    |     |             |
 y |----+-----P-------------+---
   |    |     |             |
y1 |---p11----+-----------(p21)-
   |    |     |             |
   |----+-----+-------------+---
        x1    x             x2
 */

/**
 * Bilinear interpolation among 4 positions
 * @param q11 value at (x1, y1)
 * @param q12 value at (x1, y2)
 * @param q21 value at (x2, y1)
 * @param q22 value at (x2, y2)
 * @param p11 position with (x1, y1)
 * @param p22 position with (x2, y2)
 * @param p position (x, y) for the interpolation
 * @return the interpolated value at (x, y)
 */
float bilinear_interp(float q11, float q12, float q21, float q22,
                            const Position &p11, const Position &p22,
                            const Position &p);

/**
 * Bilinear interpolation among 4 cells
 * @param q11 value at (x1, y1) (cell c11)
 * @param q12 value at (x1, y2) (cell c12)
 * @param q21 value at (x2, y1) (cell c21)
 * @param q22 value at (x2, y2) (cell c22)
 * @param c11 cell with center (x1, y1)
 * @param c22 cell with center (x2, y2)
 * @param p position (x, y) for the interpolation
 * @return the interpolated value at (x, y)
 */
float bilinear_interp(float q11, float q12, float q21, float q22,
                            const Cell &c11, const Cell &c22,
                            const Position &p);

#endif //RONCAPAT_PLANNER_TOOLKIT_BLERP_H
