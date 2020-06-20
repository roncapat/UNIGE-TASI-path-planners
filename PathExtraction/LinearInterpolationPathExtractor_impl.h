//
// Created by patrick on 18/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_LINEARINTERPOLATIONPATHEXTRACTOR_IMPL_H
#define RONCAPAT_GLOBAL_PLANNERS_LINEARINTERPOLATIONPATHEXTRACTOR_IMPL_H

template<typename E, typename T>
LinearInterpolationPathExtractor<E, T>::LinearInterpolationPathExtractor(
    const ExpandedMap<E, T> &map, const Graph &grid) : map(map), grid(grid) {}

template<typename E, typename T>
void LinearInterpolationPathExtractor<E, T>::extract_path() {
    auto begin = std::chrono::steady_clock::now();
    path_.clear();
    cost_.clear();
    total_cost = 0;
    total_dist = 0;

    float min_cost;
    PathAdditions pa;

    int curr_step = 0;

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
        //std::cerr << "[Extraction] Maximum step number reached" << std::endl;
        //path_.clear();
    }
    auto end = std::chrono::steady_clock::now();
    e_time = std::chrono::duration<float, std::milli>(end - begin).count();
    //std::cout << "Found path. Cost: " << total_cost << " Distance: " << total_dist << std::endl;
}

template<typename E, typename T>
PathAdditions LinearInterpolationPathExtractor<E, T>::traversalFromCorner(const Position &p,
                                                                          const Node &p_a,
                                                                          const Node &p_b,
                                                                          float &step_cost) {
    assert(grid.is_valid_vertex(p));

    TraversalParams cell{};
    cell.p0 = p;
    bool cond = p.aligned(p_a);
    cell.p1 = cond ? p_a : p_b;
    cell.p2 = cond ? p_b : p_a;

    assert(cell.p0.aligned(cell.p1));
    assert(not cell.p0.aligned(cell.p2));

    cell.g1 = map.get_interp_rhs(cell.p1);
    cell.g2 = map.get_interp_rhs(cell.p2);
    fill_traversal_costs(cell);

    if (allow_indirect_traversals)
        return InterpolatedTraversal::traversalFromCorner(cell, step_cost);
    else
        return InterpolatedTraversal::directTraversalFromCorner(cell, step_cost);
}

template<typename E, typename T>
PathAdditions LinearInterpolationPathExtractor<E, T>::traversalFromContiguousEdge(const Position &p,
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

    cell1.g1 = map.get_interp_rhs(Node(cell1.p1));
    cell1.g2 = map.get_interp_rhs(Node(cell1.p2));
    fill_traversal_costs(cell1);
    cell1.q = 1 - std::abs(cell1.p1.y - p.y) - std::abs(cell1.p1.x - p.x);

    if (allow_indirect_traversals)
        return InterpolatedTraversal::traversalFromContiguousEdge(cell1, step_cost);
    else
        return InterpolatedTraversal::directTraversalFromContiguousEdge(cell1, step_cost);
}

template<typename E, typename T>
PathAdditions LinearInterpolationPathExtractor<E, T>::traversalFromOppositeEdge(const Position &p,
                                                                                const Node &p_a,
                                                                                const Node &p_b,
                                                                                float &step_cost) {

    TraversalParams cell1{}, cell2{};
    cell1.p1 = cell2.p2 = p_a;
    cell1.p2 = cell2.p1 = p_b;

    // align p with p_1 and p_2 before fill_traversal_costs();
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

    cell1.g1 = cell2.g2 = map.get_interp_rhs(p_a);
    cell1.g2 = cell2.g1 = map.get_interp_rhs(p_b);
    fill_traversal_costs(cell1);
    fill_traversal_costs(cell2);
    cell1.p = std::abs(p.y - cell1.p0.y) + std::abs(p.x - cell1.p0.x);
    cell2.p = 1 - cell1.p;

    if (allow_indirect_traversals)
        return InterpolatedTraversal::traversalFromOppositeEdge(cell1, cell2, step_cost);
    else
        return InterpolatedTraversal::directTraversalFromOppositeEdge(cell1, cell2, step_cost);
}

template<typename E, typename T>
PathAdditions LinearInterpolationPathExtractor<E, T>::traversalFromEdge(const Position &p,
                                                                        const Node &p_a,
                                                                        const Node &p_b,
                                                                        float &step_cost) {

    assert(!grid.is_valid_vertex(p));

    bool cond_1 = (p.x == p_a.x || p.y == p_a.y);
    bool cond_2 = (p.x == p_b.x || p.y == p_b.y);
    if (cond_1 || cond_2) {
        assert(cond_1 xor cond_2);
        return traversalFromContiguousEdge(p, p_a, p_b, step_cost);
    } else {
        return traversalFromOppositeEdge(p, p_a, p_b, step_cost);
    }
}

template<typename E, typename T>
PathAdditions LinearInterpolationPathExtractor<E, T>::getPathAdditions(const Position &p,
                                                                       const bool &do_lookahead,
                                                                       float &step_cost) {
    float min_cost = INFINITY;
    PathAdditions min_pa = {};
    PathAdditions temp_pa;
    float lookahead_cost;

    for (const auto &edge : grid.consecutive_neighbors(p)) {
        float cur_step_cost = INFINITY;

        if (grid.is_valid_vertex(p))
            temp_pa = traversalFromCorner(p, edge.first, edge.second, cur_step_cost);
        else
            temp_pa = traversalFromEdge(p, edge.first, edge.second, cur_step_cost);

        if (temp_pa.steps.empty()) continue;

        // LOOKAHEAD PROCEDURE documented in
        // Field D* path-finding on weighted triangulated and tetrahedral meshes (Perkins et al. 2013), Section 3
        // Only needed if next point is on edge
        float dummy;
        if (do_lookahead and not grid.is_valid_vertex(temp_pa.steps.back())) {
            lookahead_cost = getPathAdditions(temp_pa.steps.back(), false, dummy).cost_to_goal;
            if (lookahead_cost > temp_pa.cost_to_goal) { // Lookahead test failed
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

template<typename E, typename T>
bool LinearInterpolationPathExtractor<E, T>::goalReached(const Position &p) {
    return grid.goal_pos_.x == p.x && grid.goal_pos_.y == p.y;
}

// p must be aligned with p_1, p_1 aligned with p_2, p and P_2 diagonal neighbors
template<typename E, typename T>
void LinearInterpolationPathExtractor<E, T>::fill_traversal_costs(TraversalParams &t) {
    Cell cell_ind_b, cell_ind_c;

    if (t.p0.x == t.p1.x) {
        cell_ind_b = t.p1.neighbor_cell(t.p2.x > t.p1.x, t.p0.y > t.p1.y);
        cell_ind_c = t.p1.neighbor_cell(t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cell_ind_b = t.p1.neighbor_cell(t.p0.x < t.p1.x, t.p2.y < t.p1.y);
        cell_ind_c = t.p1.neighbor_cell(t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    t.b = grid.get_cost(cell_ind_b);
    t.c = grid.get_cost(cell_ind_c);
}

#endif //RONCAPAT_GLOBAL_PLANNERS_LINEARINTERPOLATIONPATHEXTRACTOR_IMPL_H
