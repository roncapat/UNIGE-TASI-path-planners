//
// Created by patrick on 06/04/20.
//

#include <numeric>
#include <algorithm>
#include "Graph.h"
#include "LinearTraversalCostInterpolation.h"

namespace TraversalTypeI {
namespace Corner {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + t.b + CATH(t.c, t.b));
}

float stepcost(TraversalParams &t) {
    float x = 1 - t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= 1);
    RETURN_CHECK_POSITIVE_LIMITED(x * t.b + HYPOT(1 - x, 1) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float x = 1 - t.b / CATH(t.c, t.b);
    return {x * t.b, HYPOT(1 - x, 1) * t.c};
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
        return {{t.p0.x, INTERP_1(t.p0.y, t.p1.y, x)}, Position(t.p2)};
    } else {            // p lies on a horizontal edge
        assert(t.p0.x != t.p1.x);
        return {{INTERP_1(t.p0.x, t.p1.x, x), t.p0.y}, Position(t.p2)};
    }
}
}

namespace ContiguousEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + (1 - t.q) * t.b + CATH(t.c, t.b));
}

float stepcost(TraversalParams &t) {
    float x = 1 - t.q - t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= (1 - t.q));
    RETURN_CHECK_POSITIVE_LIMITED(x * t.b + HYPOT(1 - t.q - x, 1) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float x = 1 - t.q - t.b / CATH(t.c, t.b);
    return {x * t.b, HYPOT(1 - t.q - x, 1) * t.c};
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
        return {{t.p0.x, INTERP_ABS(t.p0.y, t.p1.y, x)}, Position(t.p2)};
    } else {            // p lies on a horizontal edge
        assert(t.p0.x != t.p1.x);
        return {{INTERP_ABS(t.p0.x, t.p1.x, x), t.p0.y}, Position(t.p2)};
    }
}
}
namespace OppositeEdge {
bool cond(TraversalParams &t) {
    return t.c > (t.b * HYPOT(1, 1 + t.p));
}

float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + t.b + (1 + t.p) * CATH(t.c, t.b));
}

float stepcost(TraversalParams &t) {
    float x = 1 - (1 + t.p) * t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= (1));
    RETURN_CHECK_POSITIVE_LIMITED(x * t.b + HYPOT(1 - x, 1 + t.p) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float x = 1 - (1 + t.p) * t.b / CATH(t.c, t.b);
    return {x * t.b, HYPOT(1 - x, 1 + t.p) * t.c};
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
                Position(t.p2)};
    } else {
        return {{INTERP_1(t.p0.x, t.p1.x, v), t.p0.y},
                {INTERP_1(t.p0.x, t.p1.x, v + x), t.p0.y},
                Position(t.p2)};
    }
}
}
}

namespace TraversalTypeII {
namespace Corner {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + CATH(t.c, t.f));
}

bool cond(TraversalParams &t) {
    return t.c > (t.f * SQRT2);
}

float stepcost(TraversalParams &t) {
    float y = t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    RETURN_CHECK_POSITIVE_LIMITED(HYPOT(1, y) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float y = t.f / CATH(t.c, t.f);
    return {HYPOT(1, y) * t.c};
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    float y = t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    if (t.p0.x == t.p1.x) {
        assert(t.p1.x != t.p2.x);
        return {{INTERP_1(t.p1.x, t.p2.x, y), (float) t.p1.y}};
    } else {            // p lies on a horizontal edge
        assert(t.p1.y != t.p2.y);
        return {{(float) t.p1.x, INTERP_1(t.p1.y, t.p2.y, y)}};
    }
}
}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + (1 - t.q) * CATH(t.c, t.f));
}

float stepcost(TraversalParams &t) {
    float y = (1 - t.q) * t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    RETURN_CHECK_POSITIVE_LIMITED(HYPOT(1 - t.q, y) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float y = (1 - t.q) * t.f / CATH(t.c, t.f);
    return {HYPOT(1 - t.q, y) * t.c};
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
        return {{INTERP_ABS(t.p1.x, t.p2.x, y), (float) t.p1.y}};
    } else {
        assert(t.p1.y != t.p2.y);
        return {{(float) t.p1.x, INTERP_ABS(t.p1.y, t.p2.y, y)}};
    }
}
}
namespace OppositeEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + CATH(t.c, t.f) + (1 - t.p) * t.f);
}

float stepcost(TraversalParams &t) {
    float y = t.p + t.f / CATH(t.c, t.f);
    assert(y >= 0 and y <= 1);
    RETURN_CHECK_POSITIVE_LIMITED(HYPOT(1, y - t.p) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float y = t.p + t.f / CATH(t.c, t.f);
    return {HYPOT(1, y - t.p) * t.c};
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
        return {{INTERP_1(t.p1.x, t.p2.x, y), (float) t.p1.y}};
    } else {
        assert(t.p1.y != t.p2.y);
        return {{(float) t.p1.x, INTERP_1(t.p1.y, t.p2.y, y)}};
    }
}
}
}

namespace TraversalTypeIII {
namespace Corner {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + t.b);
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED(t.b);
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {t.b};
}

bool cond(TraversalParams &t) {
    return t.c > t.b;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p1)};
}
}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + (1 - t.q) * t.b);
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED((1 - t.q) * t.b);
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {(1 - t.q) * t.b};
}

bool cond(TraversalParams &t) {
    return t.c > t.b;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p1)};
}
}

namespace OppositeEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + t.b + t.p * CATH(t.c, t.b));
}

float stepcost(TraversalParams &t) {
    float x = t.p * t.b / CATH(t.c, t.b);
    assert(x >= 0 and x <= 1);
    RETURN_CHECK_POSITIVE_LIMITED(HYPOT(x, t.p) * t.c + (1 - x) * t.b);
}

std::vector<float> stepcosts(TraversalParams &t) {
    float x = t.p * t.b / CATH(t.c, t.b);
    return {HYPOT(x, t.p) * t.c, (1 - x) * t.b};
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
        return {{t.p0.x, INTERP_1(t.p0.y, t.p1.y, x)}, Position(t.p1)};
    } else {            // p lies on a horizontal edge
        return {{INTERP_1(t.p0.x, t.p1.x, x), t.p0.y}, Position(t.p1)};
    }
}
}
}

namespace TraversalTypeA {
namespace Corner {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + t.c * SQRT2);
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED(t.c * SQRT2);
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {t.c * SQRT2};
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p2)};
}
}
namespace ContiguousEdge {

float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + t.c * HYPOT(1, 1 - t.q));
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED(t.c * HYPOT(1, 1 - t.q));
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {t.c * HYPOT(1, 1 - t.q)};
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p2)};
}
}
namespace OppositeEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g2 + t.c * HYPOT(1 - t.p, 1));
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED(t.c * HYPOT(1 - t.p, 1));
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {t.c * HYPOT(1 - t.p, 1)};
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p2)};
}
}
}

namespace TraversalTypeB {
namespace Corner {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + t.c);
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED(t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {t.c};
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p1)};
}

}
namespace ContiguousEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + t.c * (1 - t.q));
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED((1 - t.q) * t.c);
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {(1 - t.q) * t.c};
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p1)};
}
}
namespace OppositeEdge {
float cost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE(t.g1 + t.c * HYPOT(t.p, 1));
}

float stepcost(TraversalParams &t) {
    RETURN_CHECK_POSITIVE_LIMITED(t.c * HYPOT(t.p, 1));
}

std::vector<float> stepcosts(TraversalParams &t) {
    return {t.c * HYPOT(t.p, 1)};
}

bool cond(TraversalParams &) {
    return true;
}

float condcost(TraversalParams &t) {
    return (cond(t)) ? cost(t) : INFINITY;
}

std::vector<Position> additions(TraversalParams &t) {
    return {Position(t.p1)};
}
}
}

namespace InterpolatedTraversal {

path_additions traversalFromCorner(TraversalParams &cell, float &step_cost) {
    if (cell.g1 == INFINITY && cell.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    if (cell.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell.f = cell.g1 - cell.g2;

    int type;
    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    enum { TYPE_I, TYPE_II, TYPE_III, TYPE_A, TYPE_B };

    if (cell.c > cell.b) {
        if ((cell.f <= 0) or (SQUARE(cell.f) <= CATH(cell.c, cell.b))) {
            min_cost = TraversalTypeIII::Corner::cost(cell);
            type = TYPE_III;
        } else if ((cell.f <= cell.b) and (cell.c > (cell.f * SQRT2))) {
            min_cost = TraversalTypeII::Corner::cost(cell);
            type = TYPE_II;
        } else if ((cell.f > cell.b) and (cell.c > (cell.b * SQRT2))) {
            min_cost = TraversalTypeI::Corner::cost(cell);
            type = TYPE_I;
        } else {
            min_cost = TraversalTypeA::Corner::cost(cell);
            type = TYPE_A;
        }
    } else {
        if (cell.f <= 0) {
            min_cost = TraversalTypeB::Corner::cost(cell);
            type = TYPE_B;
        } else if ((cell.f * SQRT2) < cell.c) {
            min_cost = TraversalTypeII::Corner::cost(cell);
            type = TYPE_II;
        } else {
            min_cost = TraversalTypeA::Corner::cost(cell);
            type = TYPE_A;
        }
    }

    if (type == TYPE_I) {
        positions = TraversalTypeI::Corner::additions(cell);
        step_costs = TraversalTypeI::Corner::stepcosts(cell);
    } else if (type == TYPE_II) {
        positions = TraversalTypeII::Corner::additions(cell);
        step_costs = TraversalTypeII::Corner::stepcosts(cell);
    } else if (type == TYPE_III) {
        positions = TraversalTypeIII::Corner::additions(cell);
        step_costs = TraversalTypeIII::Corner::stepcosts(cell);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::Corner::additions(cell);
        step_costs = TraversalTypeA::Corner::stepcosts(cell);
    } else {
        positions = TraversalTypeB::Corner::additions(cell);
        step_costs = TraversalTypeB::Corner::stepcosts(cell);
    }

    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}
path_additions traversalFromContiguousEdge(TraversalParams &cell1, float &step_cost) {
    if (cell1.g1 == INFINITY && cell1.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    if (cell1.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell1.f = cell1.g1 - cell1.g2;
    assert(cell1.q > 0 and cell1.q < 1);
    assert(cell1.g1 >= 0 and cell1.g2 >= 0); //Goal has g=0
    assert(cell1.b > 0 and cell1.c > 0);

    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    enum { TYPE_I, TYPE_II, TYPE_III, TYPE_A, TYPE_B };
    std::array<float, 5> costs{};
    costs.fill(INFINITY);

    costs[TYPE_I] = TraversalTypeI::ContiguousEdge::condcost(cell1);
    costs[TYPE_II] = TraversalTypeII::ContiguousEdge::condcost(cell1);
    costs[TYPE_III] = TraversalTypeIII::ContiguousEdge::condcost(cell1);
    costs[TYPE_A] = TraversalTypeA::ContiguousEdge::cost(cell1);
    costs[TYPE_B] = TraversalTypeB::ContiguousEdge::cost(cell1);

    auto min = std::min_element(costs.begin(), costs.end());
    int type = std::distance(costs.begin(), min);
    min_cost = *min;

    if (type == TYPE_I) {
        positions = TraversalTypeI::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeI::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_II) {
        positions = TraversalTypeII::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeII::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_III) {
        positions = TraversalTypeIII::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeIII::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeA::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_B) {
        positions = TraversalTypeB::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeB::ContiguousEdge::stepcosts(cell1);
    }
    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}
path_additions traversalFromOppositeEdge(TraversalParams &cell1, TraversalParams &cell2, float &step_cost) {
    // FOR EXTRACTION IN SEPARATE LIBRARY
    if (cell1.g1 == INFINITY && cell2.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    if (cell1.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell1.f = cell1.g1 - cell1.g2;
    cell2.f = -cell1.f;
    assert(cell1.c == cell2.c);
    assert(cell1.p > 0 and cell1.p < 1);
    assert((cell1.p + cell2.p) == 1);
    assert(cell1.g1 >= 0 and cell1.g2 >= 0); // goal has g=0
    assert(cell1.b > 0 and cell2.b > 0 and cell1.c > 0);

    // Use _f1, _p1, _b1, _g_s2;
    // Use neg(_f), 1-_p1, _b2, _g_s1;
    // NOTE: neg(_f1)=_f2, (1-_p1)=_p2
    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    enum { TYPE_I__1, TYPE_I__2, TYPE_II__1, TYPE_II__2, TYPE_III__1, TYPE_III__2, TYPE_A__1, TYPE_A__2 };
    std::array<float, 8> costs{};
    costs.fill(INFINITY);
    costs[TYPE_I__1] = TraversalTypeI::OppositeEdge::condcost(cell1);
    costs[TYPE_I__2] = TraversalTypeI::OppositeEdge::condcost(cell2);
    costs[TYPE_II__1] = TraversalTypeII::OppositeEdge::condcost(cell1);
    costs[TYPE_II__2] = TraversalTypeII::OppositeEdge::condcost(cell2);
    costs[TYPE_III__1] = TraversalTypeIII::OppositeEdge::condcost(cell1);
    costs[TYPE_III__2] = TraversalTypeIII::OppositeEdge::condcost(cell2);
    costs[TYPE_A__1] = TraversalTypeA::OppositeEdge::cost(cell1);
    costs[TYPE_A__2] = TraversalTypeA::OppositeEdge::cost(cell2);

    auto min = std::min_element(costs.begin(), costs.end());
    int type = std::distance(costs.begin(), min);
    min_cost = *min;

    if (type == TYPE_I__1) {
        positions = TraversalTypeI::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeI::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_I__2) {
        positions = TraversalTypeI::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeI::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_II__1) {
        positions = TraversalTypeII::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeII::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_II__2) {
        positions = TraversalTypeII::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeII::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_III__1) {
        positions = TraversalTypeIII::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeIII::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_III__2) {
        positions = TraversalTypeIII::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeIII::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_A__1) {
        positions = TraversalTypeA::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeA::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_A__2) {
        positions = TraversalTypeA::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeA::OppositeEdge::stepcosts(cell2);
    }
    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}

path_additions directTraversalFromCorner(TraversalParams &cell, float &step_cost) {
    if (cell.g1 == INFINITY && cell.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    if (cell.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell.f = cell.g1 - cell.g2;

    int type;
    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    enum { TYPE_I, TYPE_II, TYPE_III, TYPE_A, TYPE_B };

    if (cell.f <= 0) {
        min_cost = TraversalTypeB::Corner::cost(cell);
        type = TYPE_B;
    } else if ((cell.f * SQRT2) < cell.c) {
        min_cost = TraversalTypeII::Corner::cost(cell);
        type = TYPE_II;
    } else {
        min_cost = TraversalTypeA::Corner::cost(cell);
        type = TYPE_A;
    }

     if (type == TYPE_II) {
        positions = TraversalTypeII::Corner::additions(cell);
        step_costs = TraversalTypeII::Corner::stepcosts(cell);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::Corner::additions(cell);
        step_costs = TraversalTypeA::Corner::stepcosts(cell);
    } else {
        positions = TraversalTypeB::Corner::additions(cell);
        step_costs = TraversalTypeB::Corner::stepcosts(cell);
    }

    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}

path_additions directTraversalFromContiguousEdge(TraversalParams &cell1, float &step_cost) {
    if (cell1.g1 == INFINITY && cell1.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    if (cell1.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell1.f = cell1.g1 - cell1.g2;
    assert(cell1.q > 0 and cell1.q < 1);
    assert(cell1.g1 >= 0 and cell1.g2 >= 0); //Goal has g=0
    assert(cell1.b > 0 and cell1.c > 0);

    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    enum {TYPE_II, TYPE_A, TYPE_B };
    std::array<float, 3> costs{};
    costs.fill(INFINITY);

    costs[TYPE_II] = TraversalTypeII::ContiguousEdge::condcost(cell1);
    costs[TYPE_A] = TraversalTypeA::ContiguousEdge::cost(cell1);
    costs[TYPE_B] = TraversalTypeB::ContiguousEdge::cost(cell1);

    auto min = std::min_element(costs.begin(), costs.end());
    int type = std::distance(costs.begin(), min);
    min_cost = *min;

    if (type == TYPE_II) {
        positions = TraversalTypeII::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeII::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeA::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_B) {
        positions = TraversalTypeB::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeB::ContiguousEdge::stepcosts(cell1);
    }
    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}

path_additions directTraversalFromOppositeEdge(TraversalParams &cell1, TraversalParams &cell2, float &step_cost) {
    // FOR EXTRACTION IN SEPARATE LIBRARY
    if (cell1.g1 == INFINITY && cell2.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    if (cell1.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell1.f = cell1.g1 - cell1.g2;
    cell2.f = -cell1.f;
    assert(cell1.c == cell2.c);
    assert(cell1.p > 0 and cell1.p < 1);
    assert((cell1.p + cell2.p) == 1);
    assert(cell1.g1 >= 0 and cell1.g2 >= 0); // goal has g=0
    assert(cell1.b > 0 and cell2.b > 0 and cell1.c > 0);

    // Use _f1, _p1, _b1, _g_s2;
    // Use neg(_f), 1-_p1, _b2, _g_s1;
    // NOTE: neg(_f1)=_f2, (1-_p1)=_p2
    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    enum {TYPE_II__1, TYPE_II__2, TYPE_A__1, TYPE_A__2 };
    std::array<float, 4> costs{};
    costs.fill(INFINITY);
    costs[TYPE_II__1] = TraversalTypeII::OppositeEdge::condcost(cell1);
    costs[TYPE_II__2] = TraversalTypeII::OppositeEdge::condcost(cell2);
    costs[TYPE_A__1] = TraversalTypeA::OppositeEdge::cost(cell1);
    costs[TYPE_A__2] = TraversalTypeA::OppositeEdge::cost(cell2);

    auto min = std::min_element(costs.begin(), costs.end());
    int type = std::distance(costs.begin(), min);
    min_cost = *min;

    if (type == TYPE_II__1) {
        positions = TraversalTypeII::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeII::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_II__2) {
        positions = TraversalTypeII::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeII::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_A__1) {
        positions = TraversalTypeA::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeA::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_A__2) {
        positions = TraversalTypeA::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeA::OppositeEdge::stepcosts(cell2);
    }
    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}
}