//
// Created by patrick on 06/04/20.
//

#include "Graph.h"
#include "interpolation.h"

const float SQRT2 = 1.41421356237309504880168872420969807856967187537694f;

namespace TraversalTypeI {
namespace Corner {
float cost(TraversalParams &t) {
    return t.g2 + t.b + CATH(t.c, t.b);
}

bool cond(TraversalParams &t) {
    return t.c > (t.b * SQRT2);
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float x = 1 - t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= 1);
    if (t.p0.x == t.p1.x) { // p lies on a vertical edge
        assert(t.p0.y != t.p1.y);
        return {{t.p0.x, INTERP_1(t.p0.y, t.p1.y, x)}, t.p2};
    } else {            // p lies on a horizontal edge
        assert(t.p0.x != t.p1.x);
        return {{INTERP_1(t.p0.x, t.p1.x, x), t.p0.y}, t.p2};
    }
}
}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    return t.g2 + (1 - t.q) * t.b + CATH(t.c, t.b);
}

bool cond(TraversalParams &t) {
    return t.c > (t.b * HYPOT(1, 1 / (1 - t.q)));
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float x = 1 - t.q - t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= (1 - t.q));
    if (t.p0.x == t.p1.x) { // p lies on a vertical edge
        assert(t.p0.y != t.p1.y);
        return {{t.p0.x, INTERP_ABS(t.p0.y, t.p1.y, x)}, t.p2};
    } else {            // p lies on a horizontal edge
        assert(t.p0.x != t.p1.x);
        return {{INTERP_ABS(t.p0.x, t.p1.x, x), t.p0.y}, t.p2};
    }
}
}
namespace OppositeEdge {
bool cond(TraversalParams &t) {
    return t.c > (t.b * HYPOT(1, 1 + t.p));
}

float cost(TraversalParams &t) {
    return t.g2 + t.b + (1 + t.p) * CATH(t.c, t.b);
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float x = 1 - (1 + t.p) * t.b / CATH(t.c, t.b);
    float v = (1 - x) * t.p / (t.p + 1);
    assert(x >= 0 and x <= 1);
    assert(v >= 0 and v <= 1);
    assert((x + v) <= 1);
    if (t.p0.x == t.p1.x) {
        return {{t.p0.x, INTERP_1(t.p0.y, t.p1.y, v)},
                {t.p0.x, INTERP_1(t.p0.y, t.p1.y, v + x)},
                t.p2};
    } else {
        return {{INTERP_1(t.p0.x, t.p1.x, v), t.p0.y},
                {INTERP_1(t.p0.x, t.p1.x, v + x), t.p0.y},
                t.p2};
    }
}
}
}

namespace TraversalTypeII {
namespace Corner {
float cost(TraversalParams &t) {
    return t.g1 + CATH(t.c, t.f);
}

bool cond(TraversalParams &t) {
    return t.c > (t.f * SQRT2);
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float y = t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    if (t.p0.x == t.p1.x) {
        assert(t.p1.x != t.p2.x);
        return {{INTERP_1(t.p1.x, t.p2.x, y), t.p1.y}};
    } else {            // p lies on a horizontal edge
        assert(t.p1.y != t.p2.y);
        return {{t.p1.x, INTERP_1(t.p1.y, t.p2.y, y)}};
    }
}
}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    return t.g1 + (1 - t.q) * CATH(t.c, t.f);
}

bool cond(TraversalParams &t) {
    return (t.f > 0) and (t.c > t.f * HYPOT(1, 1 - t.q));
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float y = (1 - t.q) * t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    if (t.p0.x == t.p1.x) {
        assert(t.p1.x != t.p2.x);
        return {{INTERP_ABS(t.p1.x, t.p2.x, y), t.p1.y}};
    } else {
        assert(t.p1.y != t.p2.y);
        return {{t.p1.x, INTERP_ABS(t.p1.y, t.p2.y, y)}};
    }
}
}
namespace OppositeEdge {
float cost(TraversalParams &t) {
    return t.g2 + CATH(t.c, t.f) + (1 - t.p) * t.f;
}

bool cond(TraversalParams &t) {
    return (t.f > 0) and (t.c > (t.f * HYPOT(1, 1 / (1 - t.p))));
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float y = t.p + t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    if (t.p0.x == t.p1.x) {
        assert(t.p1.x != t.p2.x);
        return {{INTERP_1(t.p1.x, t.p2.x, y), t.p1.y}};
    } else {
        assert(t.p1.y != t.p2.y);
        return {{t.p1.x, INTERP_1(t.p1.y, t.p2.y, y)}};
    }
}
}
}

namespace TraversalTypeIII {
namespace Corner {
float cost(TraversalParams &t) {
    return t.g1 + t.b;
}

bool cond(TraversalParams &t) {
    return t.c > t.b;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p1};
}
}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    return t.g1 + (1 - t.q) * t.b;
}

bool cond(TraversalParams &t) {
    return t.c > t.b;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p1};
}
}

namespace OppositeEdge {
float cost(TraversalParams &t) {
    return t.g1 + t.b + t.p * CATH(t.c, t.b);
}

bool cond(TraversalParams &t) {
    return t.c > t.b * HYPOT(1, t.p);
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float x = t.p * t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= 1);
    if (t.p0.x == t.p1.x) {
        return {{t.p0.x, INTERP_1(t.p0.y, t.p1.y, x)}, t.p1};
    } else {            // p lies on a horizontal edge
        return {{INTERP_1(t.p0.x, t.p1.x, x), t.p0.y}, t.p1};
    }
}
}
}

namespace TraversalTypeA {
namespace Corner {
float cost(TraversalParams &t) {
    return t.g2 + t.c * SQRT2;
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p2};
}
}
namespace ContiguousEdge {

float cost(TraversalParams &t) {
    return t.g2 + t.c * HYPOT(1, 1 - t.q);
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p2};
}
}
namespace OppositeEdge {
float cost(TraversalParams &t) {
    return t.g2 + t.c * HYPOT(1 - t.p, 1);
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p2};
}
}
}

namespace TraversalTypeB {
namespace Corner {
float cost(TraversalParams &t) {
    return t.g1 + t.c;
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p1};
}

}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    return t.g1 + t.c * (1 - t.q);
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p1};
}
}
namespace OppositeEdge {
float cost(TraversalParams &t) {
    return t.g1 + t.c * HYPOT(t.p, 1);
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {t.p1};
}
}
}