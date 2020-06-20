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
#include "InterpolatedTraversal.h"
#include "ReplannerBase.h"

template<int OptimizationLevel>
class DFMPlanner : public ReplannerBase<DFMPlanner<OptimizationLevel>, Cell, std::pair<Cell, Cell>, float> {
 public:
    typedef ReplannerBase<DFMPlanner<OptimizationLevel>, Cell, std::pair<Cell, Cell>, float> Base;
    friend Base;
    typedef typename Base::Key Key;
    typedef typename Base::Queue Queue;
    typedef typename Base::Map Map;
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

    Key calculate_key(const Cell &s, float g, float rhs) const;

private:
    typename decltype(map)::nodeptr start_cell_it;

    float min_rhs(const Cell &c) const;

    float min_rhs(const Cell &p, std::pair<Cell, Cell> &bptrs) const;

    bool end_condition() const;

    Key calculate_key(const Cell &s) const;

    Key calculate_key(const Cell &s, float cost_so_far) const;

    void update_cell(const Cell &cell);

    std::pair<Cell, float> best_cell(const Cell &a, const Cell &b) const;

    float min_rhs_decreased_neighbor(const Cell &c, const Cell &nbr, std::pair<Cell, Cell> &bptrs) const;

    std::pair<float, std::pair<Cell, Cell>>
    compute_optimal_cost(Cell &ca1, Cell &cb1, float ga1, float gb1, float tau, float h) const;
};

template<>
void DFMPlanner<0>::plan();

template<>
float DFMPlanner<0>::min_rhs(const Cell &) const;

template<>
void DFMPlanner<1>::plan();

template<>
float DFMPlanner<1>::min_rhs(const Cell &, std::pair<Cell, Cell> &bptrs) const;

#include "DynamicFastMarching_impl.h"

#endif
