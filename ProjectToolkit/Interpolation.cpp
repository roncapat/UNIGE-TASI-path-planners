#include "Interpolation.h"

float bilinear_interp(float q11, float q12, float q21, float q22,
                            float x1, float x2, float y1, float y2,
                            float x, float y) {
    float x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;
    y2y1 = y2 - y1;
    x2x = x2 - x;
    y2y = y2 - y;
    yy1 = y - y1;
    xx1 = x - x1;
    return (q11 * x2x * y2y +
        q21 * xx1 * y2y +
        q12 * x2x * yy1 +
        q22 * xx1 * yy1
    ) / (x2x1 * y2y1);
}
float bilinear_interp(float q11, float q12, float q21, float q22,
                            float x1, float x2, float y1, float y2,
                            const Position &p) {
    return bilinear_interp(q11, q12, q21, q22, x1, x2, y1, y2, p.x, p.y);
}
float bilinear_interp(float q11, float q12, float q21, float q22,
                            const Position &p11, const Position &p22,
                            const Position &p) {
    return bilinear_interp(q11, q12, q21, q22, p11.x, p22.x, p11.y, p22.y, p.x, p.y);
}
float bilinear_interp(float q11, float q12, float q21, float q22,
                            const Cell &p11, const Cell &p22,
                            const Position &p) {
    return bilinear_interp(q11, q12, q21, q22,
                                 (float) p11.x, (float) p22.x, (float) p11.y, (float) p22.y,
                                 p.x - 0.5f, p.y - 0.5f);
}
