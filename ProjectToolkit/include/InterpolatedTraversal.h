#ifndef RONCAPAT_PLANNER_TOOLKIT_INTERP_TRAVERSAL_H
#define RONCAPAT_PLANNER_TOOLKIT_INTERP_TRAVERSAL_H

#include "Graph.h"
#include "Macros.h"

/**
 * Contains parameters needed for linear interpolation of cell traversal cost.
 * Refer to the original Field D* paper for reference of the labels.
 */
struct TraversalParams {
  /** Point aligned with p1, but not p2 */
  Position p0;
  /** Nodes of the edge to traverse */
  Node p1, p2;
  /** Traversal cost of the current cell */
  float b;
  /** Traversal cost of the adjacent cell */
  float c;
  /** g(p1) - g(p2) */
  float f;
  /** The costs of the two nodes of the considered edge */
  float g1, g2;
  /** Offsets of the starting point inside the cell (cell has unitary size) */
  float p, q;
};

/**
 * A piece of trajectory. Encapsulates spatial steps, their cost and the remaining
 * cost to goal.
 */
struct PathAdditions {
  /** A sequence of Positions that describe a piece of path */
  std::vector<Position> steps;
  /** Each step has an associated cost */
  std::vector<float> stepcosts;
  /** Remaining cost to goal */
  float cost_to_goal;
};


//TODO use CRPT to define classes with these functions as interface
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

//TODO move output parameter step_cost in PathAdditions
namespace InterpolatedTraversal {
PathAdditions traversalFromCorner(TraversalParams &cell,
                                  float &step_cost);
PathAdditions traversalFromContiguousEdge(TraversalParams &cell1,
                                          float &step_cost);
PathAdditions traversalFromOppositeEdge(TraversalParams &cell1,
                                        TraversalParams &cell2,
                                        float &step_cost);

PathAdditions directTraversalFromCorner(TraversalParams &cell,
                                        float &step_cost);
PathAdditions directTraversalFromContiguousEdge(TraversalParams &cell1,
                                                float &step_cost);
PathAdditions directTraversalFromOppositeEdge(TraversalParams &cell1,
                                              TraversalParams &cell2,
                                              float &step_cost);
}

#endif //RONCAPAT_PLANNER_TOOLKIT_INTERP_TRAVERSAL_H
