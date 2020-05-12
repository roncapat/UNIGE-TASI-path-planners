#ifndef RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H
#define RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H

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

float BilinearInterpolation(float q11, float q12, float q21, float q22,
                            float x1, float x2, float y1, float y2,
                            float x, float y);

float BilinearInterpolation(float q11, float q12, float q21, float q22,
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
float BilinearInterpolation(float q11, float q12, float q21, float q22,
                            const Position &p11, const Position &p22,
                            const Position &p);

float BilinearInterpolation(float q11, float q12, float q21, float q22,
                            const Cell &p11, const Cell &p22,
                            const Position &p);

#endif //RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H
