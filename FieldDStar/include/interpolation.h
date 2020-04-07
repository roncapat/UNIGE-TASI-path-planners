//
// Created by patrick on 06/04/20.
//

#include "Graph.h"
#ifndef RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H
#define RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H

extern const float SQRT2;
#define SQUARE(x) ((x)*(x))
#define CATH(x, y) std::sqrt(SQUARE((x))- SQUARE((y)))
#define HYPOT(x, y) std::hypot(x,y)
#define INTERP_1(from, to, delta) ((from) + ((to)-(from))*(delta))
#define INTERP_ABS(from, to, delta) ((from) + ((to)-(from))*(delta))

struct TraversalParams {
  Position p0, p1, p2;
  float b, c, f, g1, g2, p, q;
};

namespace TraversalTypeI {
namespace Corner {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}

namespace ContiguousEdge {
float cost(TraversalParams &t);

bool cond(TraversalParams &t);

float condcost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}

namespace OppositeEdge {
bool cond(TraversalParams &t);

float cost(TraversalParams &t);

float condcost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeII {
namespace Corner {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace ContiguousEdge {
float cost(TraversalParams &t);

bool cond(TraversalParams &t);

float condcost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace OppositeEdge {
float cost(TraversalParams &t);

bool cond(TraversalParams &t);

float condcost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeIII {
namespace Corner {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace ContiguousEdge {
float cost(TraversalParams &t);

bool cond(TraversalParams &t);

float condcost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace OppositeEdge {
float cost(TraversalParams &t);

bool cond(TraversalParams &t);

float condcost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeA {
namespace Corner {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace ContiguousEdge {

float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace OppositeEdge {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeB {
namespace Corner {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);

}
namespace ContiguousEdge {
float cost(TraversalParams &t);

std::vector<Position> additions(TraversalParams &t);
}
namespace OppositeEdge {
std::vector<Position> additions(TraversalParams &t);
}
}
#endif //RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H
