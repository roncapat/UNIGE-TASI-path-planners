//
// Created by patrick on 18/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_DIRECTLINEARINTERPOLATIONPATHEXTRACTOR_IMPL_H
#define RONCAPAT_GLOBAL_PLANNERS_DIRECTLINEARINTERPOLATIONPATHEXTRACTOR_IMPL_H

#include "DirectLinearInterpolationPathExtractor.h"

template<typename T>
DirectLinearInterpolationPathExtractor<T>::DirectLinearInterpolationPathExtractor(
        const ExpandedMap<Node,T> &map, const Graph & grid) : map(map), grid(grid){}

template<typename T>
void DirectLinearInterpolationPathExtractor<T>::extract_path() {
    auto begin = std::chrono::steady_clock::now();
    path_.clear();
    cost_.clear();
    total_cost = 0;
    total_dist = 0;

    float min_cost;
    path_additions pa;

    int curr_step = 0;
    // TODO do something better than this sh*t
    int max_steps = 20000;

    float step_cost, step_dist;
    path_.push_back(grid.start_pos_);
    Position last = path_.back();
    do {
        // move one step and calculate the optimal path additions
        pa = getPathAdditions(last, lookahead, step_cost);
        assert(step_cost <= 255 * SQRT2);
        path_.reserve(path_.size() + pa.steps.size());
        cost_.reserve(cost_.size() + pa.stepcosts.size());
        auto it = path_.insert(path_.end(), pa.steps.begin(), pa.steps.end());
        cost_.insert(cost_.end(), pa.stepcosts.begin(), pa.stepcosts.end());
        min_cost = pa.cost_to_goal;
        step_dist = 0;
        for (auto p = it - 1; p < (path_.end() - 1); ++p) {
            step_dist += p->distance(*(p + 1));
        }
        total_cost += step_cost;
        total_dist += step_dist;
        curr_step += 1;
        last = path_.back();
    } while (!goalReached(last) && (min_cost != INFINITY) &&
             (curr_step < max_steps));

    if (min_cost == INFINITY) {
        std::cerr << "[Extraction] No valid path exists" << std::endl;
        path_.clear();
    } else if (curr_step >= max_steps) {
        std::cerr << "[Extraction] Maximum step number reached" << std::endl;
        path_.clear();
    }
    auto end = std::chrono::steady_clock::now();
    e_time = std::chrono::duration<float, std::milli>(end - begin).count();

    //std::cout << "Found path. Cost: " << total_cost << " Distance: " << total_dist << std::endl;
}

template<typename T>
path_additions DirectLinearInterpolationPathExtractor<T>::traversalFromCorner(const Position &p,
                                                       const Node &p_a,
                                                       const Node &p_b,
                                                       float &step_cost) {
    assert(grid.isValidVertex(p));

    TraversalParams cell{};
    cell.p0 = p;
    bool cond = p.aligned(p_a);
    cell.p1 = cond ? p_a : p_b;
    cell.p2 = cond ? p_b : p_a;

    assert(cell.p0.aligned(cell.p1));
    assert(not cell.p0.aligned(cell.p2));

    cell.g1 = map.getRHS(cell.p1);
    cell.g2 = map.getRHS(cell.p2);
    getC(cell);

    return InterpolatedTraversal::directTraversalFromCorner(cell, step_cost);
}

template<typename T>
path_additions DirectLinearInterpolationPathExtractor<T>::traversalFromContiguousEdge(const Position &p,
                                                               const Node &p_a,
                                                               const Node &p_b,
                                                               float &step_cost) {
    TraversalParams cell1{};
    bool cond = p.aligned(p_a);
    cell1.p0 = p;
    cell1.p1 = cond ? p_a : p_b; // lies on the same edge of p
    cell1.p2 = cond ? p_b : p_a;

    assert(cell1.p0.aligned(cell1.p1));
    assert(not cell1.p0.aligned(cell1.p2));

    cell1.g1 = map.getRHS(Node(cell1.p1));
    cell1.g2 = map.getRHS(Node(cell1.p2));
    getC(cell1);
    cell1.q = 1 - std::abs(cell1.p1.y - p.y) - std::abs(cell1.p1.x - p.x);

    return InterpolatedTraversal::directTraversalFromContiguousEdge(cell1, step_cost);
}

template<typename T>
path_additions DirectLinearInterpolationPathExtractor<T>::traversalFromOppositeEdge(const Position &p,
                                                             const Node &p_a,
                                                             const Node &p_b,
                                                             float &step_cost) {

    TraversalParams cell1{}, cell2{};
    cell1.p1 = cell2.p2 = p_a;
    cell1.p2 = cell2.p1 = p_b;

    // align p with p_1 and p_2 before getBC();
    // vertical edge, p must move up or down
    // horizontal edge, p must move left or right

    cell1.p0 = cell2.p0 = p;
    bool vertical = p_a.x == p_b.x;
    if (vertical) {
        cell1.p0.y = p_a.y;
        cell2.p0.y = p_b.y;
    } else {
        cell1.p0.x = p_a.x;
        cell2.p0.x = p_b.x;
    }

    cell1.g1 = cell2.g2 = map.getRHS(p_a);
    cell1.g2 = cell2.g1 = map.getRHS(p_b);
    getC(cell1);
    cell2.c = cell1.c;
    cell1.p = std::abs(p.y - cell1.p0.y) + std::abs(p.x - cell1.p0.x);
    cell2.p = 1 - cell1.p;

    return InterpolatedTraversal::directTraversalFromOppositeEdge(cell1, cell2, step_cost);
}

template<typename T>
path_additions DirectLinearInterpolationPathExtractor<T>::traversalFromEdge(const Position &p,
                                                     const Node &p_a,
                                                     const Node &p_b,
                                                     float &step_cost) {

    assert(!grid.isValidVertex(p));

    bool cond_1 = (p.x == p_a.x || p.y == p_a.y);
    bool cond_2 = (p.x == p_b.x || p.y == p_b.y);
    if (cond_1 || cond_2) {
        assert(cond_1 xor cond_2);
        return traversalFromContiguousEdge(p, p_a, p_b, step_cost);
    } else {
        return traversalFromOppositeEdge(p, p_a, p_b, step_cost);
    }
}

template<typename T>
path_additions DirectLinearInterpolationPathExtractor<T>::getPathAdditions(const Position &p,
                                                    const bool &do_lookahead,
                                                    float &step_cost) {
    float min_cost = INFINITY;
    path_additions min_pa = {};
    path_additions temp_pa;
    float lookahead_cost;
#ifdef VERBOSE_EXTRACTION
    if (lookahead and not do_lookahead)
        std::cout << "\t";
        std::cout << "p     " << std::to_string(p.x) << ", " << std::to_string(p.y)
              << (isVertex(p) ? " (Corner)" : " (Edge)") << std::endl << std::endl;
#endif

    for (const auto &[p_a, p_b] : grid.consecutiveNeighbors(p)) {
        float cur_step_cost = INFINITY;

        if (grid.isValidVertex(p))
            temp_pa = traversalFromCorner(p, p_a, p_b, cur_step_cost);
        else
            temp_pa = traversalFromEdge(p, p_a, p_b, cur_step_cost);

#ifdef VERBOSE_EXTRACTION
        if (lookahead and not do_lookahead) std::cout << "\t";
        std::cout << "X:" << p_a.x << ", Y:" << p_a.y
                  << ", G:" << expanded_map.getG(p_a.castToNode()) << ", RHS:" << expanded_map.getRHS(p_a.castToNode()) << " | "
                  << "X:" << p_b.x << ", Y:" << p_b.y
                  << ", G:" << expanded_map.getG(p_b.castToNode()) << ", RHS:" << expanded_map.getRHS(p_b.castToNode())
                  << " || cost: " << temp_pa.cost_to_goal << std::endl;
        for (auto addition: temp_pa.steps) {
            if (lookahead and not do_lookahead) std::cout << "\t";
            std::cout << "step  " << std::to_string(addition.x) << ", " << std::to_string(addition.y) << std::endl;
        }
        std::cout << std::endl;
#endif

        if (temp_pa.steps.empty()) continue;

        // LOOKAHEAD PROCEDURE documented in
        // Field D* path-finding on weighted triangulated and tetrahedral meshes (Perkins et al. 2013), Section 3
        // Only needed if next point is on edge
        float dummy;
        if (do_lookahead and not grid.isValidVertex(temp_pa.steps.back())) {
            lookahead_cost = getPathAdditions(temp_pa.steps.back(), false, dummy).cost_to_goal;
            if (lookahead_cost > temp_pa.cost_to_goal) { // Lookahead test failed
#ifdef VERBOSE_EXTRACTION
                std::cout << "Lookahead test failed" << std::endl;
#endif
                continue;
            }
        }
        if (temp_pa.cost_to_goal < min_cost) { // Promote as best solution
            min_cost = temp_pa.cost_to_goal;
            min_pa = temp_pa;
            step_cost = cur_step_cost;
        }
    }

#ifdef VERBOSE_EXTRACTION
    if (lookahead and not do_lookahead) std::cout << "\t";
    std::cout << "Final choice for X:" << std::to_string(p.x) << ", Y:" << std::to_string(p.y)
              << " || cost: " << std::to_string(min_pa.cost_to_goal) << std::endl;
    for (auto addition: min_pa.steps) {
        if (lookahead and not do_lookahead) std::cout << "\t";
        std::cout << "step  " << std::to_string(addition.x) << ", " << std::to_string(addition.y) << std::endl
                  << std::endl;
    }
#endif
    return min_pa;
}

template<typename T>
bool DirectLinearInterpolationPathExtractor<T>::goalReached(const Position &p) {
    return grid.goal_pos_.x == p.x && grid.goal_pos_.y == p.y;
}

template <typename T>
void DirectLinearInterpolationPathExtractor<T>::getC(TraversalParams &t) {
    Cell cell_ind_c;

    if (t.p0.x == t.p1.x) {
        cell_ind_c = t.p1.neighborCell(t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cell_ind_c = t.p1.neighborCell(t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    t.c = grid.getCost(cell_ind_c);
}


#endif //RONCAPAT_GLOBAL_PLANNERS_DIRECTLINEARINTERPOLATIONPATHEXTRACTOR_IMPL_H
