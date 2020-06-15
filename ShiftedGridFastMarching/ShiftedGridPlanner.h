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
#include "InterpolatedTraversal.h"
#include "ReplannerBase.h"

template<int OptimizationLevel>
class ShiftedGridPlanner
        : public ReplannerBase<ShiftedGridPlanner<OptimizationLevel>, Node, Node, std::pair<float, float>> {
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

    Key calculate_key(const Node &s, float g, float rhs);


private:
    std::vector<Node> start_nodes;

    float compute_optimal_cost(const Node &n, const Node &p_a, const Node &p_b, float ga, float gb);

    float compute_optimal_cost(const Node &n, const Node &p_a, const Node &p_b);

    bool end_condition();

    Key calculate_key(const Node &s);

    Key calculate_key(const Node &s, float cost_so_far);

    float min_rhs(const Node &s);

    float min_rhs(const Node &s, Node &bptr);

    float min_rhs_decreased_neighbor(const Node &sp, const Node &s, Node &bptr);

    void fill_traversal_costs(TraversalParams &t);

    void update_node(const Node &node);

    float min_rhs_decreased_diag_neighbor(const Node &sp, const Node &s, Node &bptr);

    float min_rhs_decreased_ortho_neigbor(const Node &sp, const Node &s, Node &bptr);
};

template<>
void ShiftedGridPlanner<0>::plan();

template<>
float ShiftedGridPlanner<0>::min_rhs(const Node &);

template<>
void ShiftedGridPlanner<1>::plan();

template<>
float ShiftedGridPlanner<1>::min_rhs(const Node &, Node &bptr);

template<>
float ShiftedGridPlanner<1>::min_rhs_decreased_neighbor(const Node &sp, const Node &s, Node &bptr);

template<>
void ShiftedGridPlanner<2>::plan();

template<>
float ShiftedGridPlanner<2>::min_rhs(const Node &, Node &bptr);

template<>
float ShiftedGridPlanner<2>::min_rhs_decreased_ortho_neigbor(const Node &sp, const Node &s, Node &bptr);

template<>
float ShiftedGridPlanner<2>::min_rhs_decreased_diag_neighbor(const Node &sp, const Node &s, Node &bptr);
#include "ShiftedGridPlanner_impl.h"

#endif
