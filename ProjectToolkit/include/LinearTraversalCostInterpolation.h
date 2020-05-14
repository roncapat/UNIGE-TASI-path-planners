//
// Created by patrick on 06/04/20.
//

#include "Graph.h"
#include "Macros.h"
#ifndef RONCAPAT_GLOBAL_PLANNERS_TRAVERSAL_INTERPOLATION_H
#define RONCAPAT_GLOBAL_PLANNERS_TRAVERSAL_INTERPOLATION_H

struct TraversalParams {
  Position p0;
  Node p1, p2;
  float b, c, f, g1, g2, p, q;
};

class path_additions {
 public:
  std::vector<Position> steps;
  std::vector<float> stepcosts;
  float cost_to_goal;
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

namespace InterpolatedTraversal {
path_additions traversalFromCorner(TraversalParams &cell,
                                   float &step_cost);
path_additions traversalFromContiguousEdge(TraversalParams &cell1,
                                           float &step_cost);
path_additions traversalFromOppositeEdge(TraversalParams &cell1,
                                         TraversalParams &cell2,
                                         float &step_cost);

path_additions directTraversalFromCorner(TraversalParams &cell,
                                   float &step_cost);
path_additions directTraversalFromContiguousEdge(TraversalParams &cell1,
                                           float &step_cost);
path_additions directTraversalFromOppositeEdge(TraversalParams &cell1,
                                         TraversalParams &cell2,
                                         float &step_cost);
}

#endif //RONCAPAT_GLOBAL_PLANNERS_TRAVERSAL_INTERPOLATION_H
