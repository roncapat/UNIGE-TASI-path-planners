//
// Created by patrick on 06/04/20.
//

#include "Graph.h"
#ifndef RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H
#define RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H

extern const float SQRT2;
#define SQUARE(x) ((x)*(x))
#define CATH(x, y) std::sqrt(SQUARE((x))- SQUARE((y)))
#define HYPOT(x, y) (float)std::hypot(x,y)
#define INTERP_1(from, to, delta) ((from) + ((to)-(from))*(delta))
#define INTERP_ABS(from, to, delta) ((from) + ((to)-(from))*(delta))
#ifdef NDEBUG
#define RETURN_CHECK_POSITIVE(x) return(x)
#define RETURN_CHECK_POSITIVE_LIMITED(x) return(x)
#else
#define RETURN_CHECK_POSITIVE(x) auto _y = x; assert(_y>0); return(_y);
#define RETURN_CHECK_POSITIVE_LIMITED(x) auto _y = x; assert(_y>0 and _y<INFINITY); return(_y);
#endif

struct TraversalParams {
  Position p0, p1, p2;
  float b, c, f, g1, g2, p, q;
};

namespace TraversalTypeI {
namespace Corner {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}

namespace ContiguousEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}

namespace OppositeEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeII {
namespace Corner {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
namespace ContiguousEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
namespace OppositeEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeIII {
namespace Corner {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
namespace ContiguousEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
namespace OppositeEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeA {
namespace Corner {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}

namespace ContiguousEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}

namespace OppositeEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
}

namespace TraversalTypeB {
namespace Corner {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}

namespace ContiguousEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}

namespace OppositeEdge {
bool cond(TraversalParams &t);
float cost(TraversalParams &t);
float stepcost(TraversalParams &t);
std::vector<float> stepcosts(TraversalParams &t);
float condcost(TraversalParams &t);
std::vector<Position> additions(TraversalParams &t);
}
}
#endif //RONCAPAT_GLOBAL_PLANNERS_INTERPOLATION_H
