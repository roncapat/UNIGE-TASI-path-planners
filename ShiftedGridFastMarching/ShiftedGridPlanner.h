#ifndef ShiftedGridPlanner_H
#define ShiftedGridPlanner_H

#include <cassert>
#include <cmath>
#include <limits>
#include <tuple>
#include <vector>
#include <utility>

#include "Macros.h"
#include "Graph.h"
#include "ExpandedMap.h"
#include "PriorityQueue.h"
#include "LinearTraversalCostInterpolation.h"
#include "planner_base.h"

#define LOOP_OK 0
#define LOOP_FAILURE_NO_GRAPH -1
#define LOOP_FAILURE_NO_GOAL -2

template<int OptimizationLevel>
class ShiftedGridPlanner : public ReplannerBase<ShiftedGridPlanner<OptimizationLevel>, Node, Node, std::pair<float, float>> {
typedef ReplannerBase<ShiftedGridPlanner<OptimizationLevel>, Node, Node, std::pair<float, float>> Base;
friend Base;
typedef typename Base::Key Key;
typedef typename Base::Queue Queue;
typedef typename Base::Map Map;
public:
using Base::grid;
using Base::priority_queue;
using Base::map;
public:
    ShiftedGridPlanner() = default;
    void set_start(const Position &pos);

protected:
    void init();
    void update();
    void plan();
    Key calculateKey(const Node &s, float g, float rhs);


private:
    std::vector<Node> start_nodes;

    float computeOptimalCost(const Node &n, const Node &p_a, const Node &p_b, float ga, float gb);

    float computeOptimalCost(const Node &n, const Node &p_a, const Node &p_b);

    bool end_condition();

    Key calculateKey(const Node &s);

    Key calculateKey(const Node &s, float cost_so_far);

    float minRHS(const Node &s);
    float minRHS(const Node &s, Node &bptr);

    bool new_start;

    void getC(TraversalParams &t);
};

template<>
void ShiftedGridPlanner<0>::plan();
template<>
float ShiftedGridPlanner<0>::minRHS(const Node &);

template<>
void ShiftedGridPlanner<1>::plan();
template<>
float ShiftedGridPlanner<1>::minRHS(const Node &, Node &bptr);

template<>
void ShiftedGridPlanner<2>::plan();
template<>
float ShiftedGridPlanner<2>::minRHS(const Node &, Node &bptr);
#include "ShiftedGridPlanner_impl.h"
#endif
