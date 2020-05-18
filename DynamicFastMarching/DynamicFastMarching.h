#ifndef FIELDDPLANNER_H
#define FIELDDPLANNER_H

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

template<int OptimizationLevel>
class DFMPlanner : public ReplannerBase<DFMPlanner<OptimizationLevel>, Cell, std::pair<Cell,Cell>, float>{
    typedef ReplannerBase<DFMPlanner<OptimizationLevel>, Cell, std::pair<Cell,Cell>, float> Base;
    friend Base;
    typedef typename Base::Key Key;
    typedef typename Base::Queue Queue;
    typedef typename Base::Map Map;
public:
    using Base::grid;
    using Base::priority_queue;
    using Base::map;
public:
    DFMPlanner() = default;
    void set_start(const Position &pos);

protected:
    void init();
    void update();
    void plan();
    Key calculateKey(const Cell &s, float g, float rhs);

private:
    std::vector<Cell> start_nodes;

    float computeOptimalCost(const Cell &p, std::pair<Cell, Cell> &bptrs);

    bool end_condition();

    Key calculateKey(const Cell &s);

    Key calculateKey(const Cell &s, float cost_so_far);

    void updateCell(const Cell &cell);

    unsigned long computeShortestPath();

    unsigned long updateCells();

    std::pair<Cell, float> minCost(const Cell &a, const Cell &b);

    void getBC(TraversalParams &t);

    void updateCellDecreasedNeighbor(const Cell &cell, const Cell &nbr);

    float computeOptimalCostDecreasedNeighbor(const Cell &c, const Cell &nbr, std::pair<Cell, Cell> &bptrs);

    std::pair<float, std::pair<Cell, Cell>> FMcost(Cell &ca1, Cell &cb1, float ga1, float gb1, float tau, float h);
};
#include "DynamicFastMarching_impl.h"
#endif
