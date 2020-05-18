//
// Created by patrick on 18/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_DIRECTLINEARINTERPOLATIONPATHEXTRACTORCELLS_H
#define RONCAPAT_GLOBAL_PLANNERS_DIRECTLINEARINTERPOLATIONPATHEXTRACTORCELLS_H

#include "Node.h"
#include "Graph.h"
#include "ExpandedMap.h"

template<typename T>
class DirectLinearInterpolationPathExtractorCells {
public:
    DirectLinearInterpolationPathExtractorCells(const ExpandedMap<Cell, T> &map, const Graph &grid);

    void extract_path();

    std::vector<Position> path_{};
    std::vector<float> cost_{};
    float total_cost = 0;
    float total_dist = 0;
    bool lookahead = true;
    float e_time = 0;
private:
    const ExpandedMap<Cell, T> &map;
    const Graph &grid;

    path_additions getPathAdditions(const Position &p, const bool &do_lookahead, float &step_cost);

    path_additions traversalFromEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);

    path_additions traversalFromOppositeEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);

    path_additions traversalFromContiguousEdge(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);

    path_additions traversalFromCorner(const Position &p, const Node &p_a, const Node &p_b, float &step_cost);

    bool goalReached(const Position &p);

    void getBC(TraversalParams &t);

};

#include "DirectLinearInterpolationPathExtractorCells_impl.h"

#endif //RONCAPAT_GLOBAL_PLANNERS_LINEARINTERPOLATIONPATHEXTRACTORCELLS_H
