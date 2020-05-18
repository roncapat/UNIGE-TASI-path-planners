#include <cmath>

template<int O>
void ShiftedGridPlanner<O>::set_start(const Position &pos){
    Base::set_start(pos);
    start_nodes = grid.start_cell_.cornerNodes();
}

template<int O>
void ShiftedGridPlanner<O>::init() {
    for (const auto &node: start_nodes)
        map.insert_or_assign(node, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_node_, INFINITY, 0.0f);
    priority_queue.insert(grid.goal_node_, calculateKey(grid.goal_node_, 0));
}

template<>
void ShiftedGridPlanner<0>::plan() {

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Node s = priority_queue.topValue();
        priority_queue.pop();
        ++expanded;

        // Get reference to the node
        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Node &nbr : grid.neighbors_8(s)){
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_node_)
                    RHS(nbr_it) = minRHS(nbr);
                enqueueIfInconsistent(nbr_it);
            }
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Node &nbr : grid.neighbors_8(s)){
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_node_)
                    RHS(nbr_it) = minRHS(nbr);
                enqueueIfInconsistent(nbr_it);
            }
            if (s != grid.goal_node_)
                RHS(s_it) = minRHS(s);
            enqueueIfInconsistent(s_it);
        }
    }
    num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}

template<>
void ShiftedGridPlanner<1>::plan() {
    float cost1, cost2;
    Node cn, ccn, bptr;

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Node s = priority_queue.topValue();
        ++expanded;

        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) {
            G(s_it) = RHS(s_it);
            priority_queue.pop();
            for (Node &sp : grid.neighbors_8(s)) {
                auto sp_it = map.find_or_init(sp);

                ccn = grid.counterClockwiseNeighbor(sp, s);
                cn = grid.clockwiseNeighbor(sp, s);
                cost1 = ccn.isValid() ? computeOptimalCost(sp, s, ccn) : INFINITY;
                cost2 = cn.isValid() ? computeOptimalCost(sp, cn, s) : INFINITY;
                if ((cost1 <= cost2) and (RHS(sp_it) > cost1)) {
                    RHS(sp_it) = cost1;
                    INFO(sp_it) = s;
                }
                if ((cost1 > cost2) and (RHS(sp_it) > cost2)) {
                    RHS(sp_it) = cost2;
                    INFO(sp_it) = cn;
                }
                enqueueIfInconsistent(sp_it);
            }
        } else {
            G(s_it) = INFINITY;
            for (Node &sp : grid.neighbors_8(s)) {
                auto sp_it = map.find(sp);
                assert(sp_it != map.end());

                if (INFO(sp_it) == s or INFO(sp_it) == grid.clockwiseNeighbor(sp, s)) {
                    RHS(sp_it) = minRHS(sp, bptr);
                    if (RHS(sp_it) < INFINITY) INFO(sp_it) = bptr;
                    enqueueIfInconsistent(sp_it);
                }
            }
            enqueueIfInconsistent(s_it);
        }
    }
    num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}

template<>
void ShiftedGridPlanner<2>::plan() {
    float cost1, cost2;
    Node cn, ccn, bptr;

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Node s = priority_queue.topValue();
        ++expanded;

        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) {
            G(s_it) = RHS(s_it);
            priority_queue.pop();
            float g_ccn, g_cn, g_s, cost = INFINITY;
            bool valid1, valid2;

            for (Node &sp : grid.neighbors_diag_4(s)) {
                auto sp_it = map.find_or_init(sp);

                ccn = grid.counterClockwiseNeighbor(sp, s);
                cn = grid.clockwiseNeighbor(sp, s);
                if (initialize_search) {
                    g_ccn = map.getRHS(ccn);
                    g_cn = map.getRHS(cn);
                    g_s = map.getRHS(s);
                } else {
                    g_ccn = map.getG(ccn);
                    g_cn = map.getG(cn);
                    g_s = map.getG(s);
                }
                valid1 = ccn.isValid();
                valid2 = cn.isValid();

                if (valid1 and ((not valid2) or (g_ccn <= g_cn))) {
                    cost = computeOptimalCost(sp, s, ccn, g_s, g_ccn);
                    bptr = s;
                } else if (valid2 and ((not valid1) or (g_ccn > g_cn))) {
                    cost = computeOptimalCost(sp, s, cn, g_s, g_cn);
                    bptr = cn;
                }
                if (RHS(sp_it) > cost) {
                    RHS(sp_it) = cost;
                    INFO(sp_it) = bptr;
                }
                enqueueIfInconsistent(sp_it);
            }
            for (Node &sp : grid.neighbors_4(s)) {
                auto sp_it = map.find_or_init(sp);

                ccn = grid.counterClockwiseNeighbor(sp, s);
                cn = grid.clockwiseNeighbor(sp, s);
                if (initialize_search) {
                    g_s = map.getRHS(s);
                    g_cn = map.getRHS(cn);
                    g_ccn = map.getRHS(ccn);
                } else {
                    g_s = map.getG(s);
                    g_cn = map.getG(cn);
                    g_ccn = map.getG(ccn);
                }
                cost1 = ccn.isValid() ? computeOptimalCost(sp, s, ccn, g_s, g_ccn) : INFINITY;
                cost2 = cn.isValid() ? computeOptimalCost(sp, s, cn, g_s, g_cn) : INFINITY;
                if ((cost1 <= cost2) and (RHS(sp_it) > cost1)) {
                    RHS(sp_it) = cost1;
                    INFO(sp_it) = s;
                }
                if ((cost1 > cost2) and (RHS(sp_it) > cost2)) {
                    RHS(sp_it) = cost2;
                    INFO(sp_it) = cn;
                }
                enqueueIfInconsistent(sp_it);
            }
        } else {
            G(s_it) = INFINITY;
            for (Node &sp : grid.neighbors_8(s)) {
                auto sp_it = map.find(sp);
                assert(sp_it != map.end());

                if (INFO(sp_it) == s or INFO(sp_it) == grid.clockwiseNeighbor(sp, s)) {
                    RHS(sp_it) = minRHS(sp, bptr);
                    if (RHS(sp_it) < INFINITY) INFO(sp_it) = bptr;
                    enqueueIfInconsistent(sp_it);
                }
            }
            enqueueIfInconsistent(s_it);
        }
    }
    num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}


template<int O>
void ShiftedGridPlanner<O>::update() {
    Queue new_queue;
    for (const auto &elem: priority_queue)
        // Only heuristic changes, so either G or RHS is kept the same
        new_queue.insert(elem.elem, calculateKey(elem.elem, elem.key.second));
    priority_queue.swap(new_queue);

    robin_hood::unordered_flat_set<Node> to_update;
    std::vector<Node> updates;
    // construct a set of all updated nodes
    for (const Cell &cell : grid.updated_cells_) {
        updates = cell.cornerNodes();
        to_update.insert(updates.begin(), updates.end());
    }

    for (const Node &s : to_update) {
        if constexpr(O == 0) {
            auto s_it = map.find_or_init(s);
            if (s != grid.goal_node_)
                RHS(s_it) = minRHS(s);
            enqueueIfInconsistent(s_it);
        } else if constexpr(O == 1) {
            auto s_it = map.find_or_init(s);
            if (s != grid.goal_node_) {
                Node bptr;
                RHS(s_it) = minRHS(s, bptr);
                if (RHS(s_it) < INFINITY) INFO(s_it) = bptr;
                this->enqueueIfInconsistent(s_it);
            }
        } else if constexpr(O == 2) {
            auto s_it = map.find_or_init(s);
            if (s != grid.goal_node_) {
                Node bptr;
                RHS(s_it) = minRHS(s, bptr);
                if (RHS(s_it) < INFINITY) INFO(s_it) = bptr;
                this->enqueueIfInconsistent(s_it);
            }
        } else static_assert(always_false<Base>, "ShiftedGridPlanner supports Optimization LVLs 0,1,2");
    }

    Base::num_nodes_updated = to_update.size();
    //std::cout << num_nodes_updated << " nodes updated" << std::endl;
}


template<int O>
typename ShiftedGridPlanner<O>::Key ShiftedGridPlanner<O>::calculateKey(const Node &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return calculateKey(s, g, rhs);
}

template<int O>
typename ShiftedGridPlanner<O>::Key ShiftedGridPlanner<O>::calculateKey(const Node &s, const float g, const float rhs) {
    return calculateKey(s, std::min(g, rhs));
}

template<int O>
typename ShiftedGridPlanner<O>::Key ShiftedGridPlanner<O>::calculateKey(const Node &s, const float cost_so_far) {
    auto dist = grid.start_pos_.distance(s);
    return {cost_so_far + this->heuristic_multiplier * dist, cost_so_far};
}

template<>
float ShiftedGridPlanner<0>::minRHS(const Node &s) {
    float rhs = INFINITY;
    for (auto &[nbr1, nbr2] : grid.consecutiveNeighbors(s))
        rhs = std::min(rhs, computeOptimalCost(s, nbr1, nbr2));
    return rhs;
}

template<>
float ShiftedGridPlanner<1>::minRHS(const Node &s, Node &bptr) {
    float rhs = INFINITY, cost;
    for (const auto &sp : grid.neighbors_8(s)) {
        auto ccn = grid.counterClockwiseNeighbor(s, sp);
        if (ccn.isValid()) {
            rhs = std::min(rhs, cost = computeOptimalCost(s, sp, ccn));
            if (rhs == cost)
                bptr = sp;
        }
    }
    return rhs;
}

template<>
float ShiftedGridPlanner<2>::minRHS(const Node &s, Node &bptr) {
//    std::cout << "V2" << std::endl;
    float rhs = INFINITY, cost;
    for (const auto &sp : grid.neighbors_diag_4(s)) {
        auto ccn = grid.counterClockwiseNeighbor(s, sp);
        auto cn = grid.clockwiseNeighbor(s, sp);
        float ccn_g = map.getG(ccn);
        float cn_g = map.getG(cn);
        float sp_g = map.getG(sp);
        bool valid1 = ccn.isValid();
        bool valid2 = cn.isValid();

        if (valid1 and ((not valid2) or (ccn_g <= cn_g))) {
            rhs = std::min(rhs, cost = computeOptimalCost(s, sp, ccn, sp_g, ccn_g));
//                std::cout << "SP " << sp.x << " " << sp.y << " G " << map.getG(sp)
//                          << " CCN " << ccn.x << " " << ccn.y << " G " << map.getG(ccn)
//                          << " COST " << cost << std::endl;
            if (rhs == cost)
                bptr = sp;
        } else if (valid2 and ((not valid1) or (ccn_g > cn_g))) {
            rhs = std::min(rhs, cost = computeOptimalCost(s, sp, cn, sp_g, cn_g));
//                    std::cout << "SP " << cn.x << " " << cn.y << " G " << map.getG(cn)
//                              << " CCN " << sp.x << " " << sp.y << " G " << map.getG(sp)
//                              << " COST " << cost << std::endl;
            if (rhs == cost)
                bptr = cn;
        }
    }
//    std::cout << "CHOICE " << bptr.x << " " << bptr.y << " COST " << rhs << std::endl;
    return rhs;
}

template<int O>
bool ShiftedGridPlanner<O>::end_condition() {
    // We need to check expansion until all 4 corners of start cell
    // used early stop from D* LITE
    auto top_key = priority_queue.topKey();
    Key max_start_key = {0, 0};
/*
    std::cout << std::endl;
    for (auto &node: start_nodes) {
        auto[g, rhs] = map.getGandRHS(node);
        auto key = calculateKey(node, g, rhs);
        std::cout << g << " " << rhs << " " << key.first << std::endl;
    }
*/
    for (auto &node: start_nodes) {
        auto[g, rhs] = map.getGandRHS(node);
        auto key = calculateKey(node, g, rhs);
        if (rhs != INFINITY and key.first != INFINITY) {
            max_start_key = std::max(max_start_key, key);
            if (rhs > g)
                return false; //Start node underconsistent
        }
    }
    if (max_start_key.first == 0) return false; //Start node not reached
    return max_start_key <= top_key; //Start node surpassed
}

template <int O>
float ShiftedGridPlanner<O>::computeOptimalCost(const Node &n,
                                             const Node &p_a,
                                             const Node &p_b) {
    float ga, gb;
    if (this->initialize_search) {
        ga = map.getRHS(p_a);
        gb = map.getRHS(p_b);
    } else {
        ga = map.getG(p_a);
        gb = map.getG(p_b);
    }

    return computeOptimalCost(n, p_a, p_b, ga, gb);
}

template <int O>
float ShiftedGridPlanner<O>::computeOptimalCost(const Node &n,
                                             const Node &p_a,
                                             const Node &p_b, float ga, float gb) {

    Position p(n);
    std::vector<Position> positions;
    float min_cost;

    assert(grid.isValidVertex(p));

    TraversalParams cell{};
    cell.p0 = p;
    bool cond = p.aligned(p_a);
    cell.p1 = cond ? p_a : p_b;
    cell.p2 = cond ? p_b : p_a;

    assert((cell.p0.x == cell.p1.x) || (cell.p0.y == cell.p1.y));
    assert((cell.p0.x != cell.p2.x) && (cell.p0.y != cell.p2.y));

    cell.g1 = cond ? ga : gb;
    cell.g2 = cond ? gb : ga;

    if (cell.g1 == INFINITY && cell.g2 == INFINITY)
        return INFINITY;
    getC(cell);
    if (cell.c == INFINITY)
        return INFINITY;
    cell.f = cell.g1 - cell.g2;

    if (cell.f <= 0) {
        min_cost = TraversalTypeB::Corner::cost(cell);
    } else if ((cell.f * SQRT2) <= cell.c) {
        min_cost = TraversalTypeII::Corner::cost(cell);
    } else {
        min_cost = TraversalTypeA::Corner::cost(cell);
    }
    return min_cost;
}

// p must be aligned with p_1, p_1 aligned with p_2, p and P_2 diagonal neighbors
template <int O>
void ShiftedGridPlanner<O>::getC(TraversalParams &t) {
    Cell cell_ind_c;

    if (t.p0.x == t.p1.x) {
        cell_ind_c = t.p1.neighborCell(t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cell_ind_c = t.p1.neighborCell(t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    t.c = grid.getCost(cell_ind_c);
}