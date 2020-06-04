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

    //TODO check initial point for traversability
    Node bptr;
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
            for (const Node &sp : grid.neighbors_8(s)) {
                auto sp_it = map.find_or_init(sp);
                float rhs = minRHSDecreasedNeighbor(sp, s, bptr);
                if (rhs < RHS(sp_it)) {
                    RHS(sp_it) = rhs;
                    INFO(sp_it) = bptr;
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
                    if (RHS(sp_it) < INFINITY) //TODO understand theese are necessary
                        INFO(sp_it) = bptr;
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

    //TODO check initial point for traversability

    Node bptr;
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
            for (Node &sp : grid.neighbors_diag_4(s)) {
                auto sp_it = map.find_or_init(sp);
                float rhs = minRHSDecreasedDiagNeighbor(sp, s, bptr);
                if (rhs < RHS(sp_it)) {
                    RHS(sp_it) = rhs;
                    INFO(sp_it) = bptr;
                }
                enqueueIfInconsistent(sp_it);
            }
            for (Node &sp : grid.neighbors_4(s)) {
                auto sp_it = map.find_or_init(sp);
                float rhs = minRHSDecreasedOrthoNeighbor(sp, s, bptr);
                if (rhs < RHS(sp_it)) {
                    RHS(sp_it) = rhs;
                    INFO(sp_it) = bptr;
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
    #ifndef NO_HEURISTIC
    Queue new_queue;
    for (const auto &elem: priority_queue)
        // Only heuristic changes, so either G or RHS is kept the same
        new_queue.insert(elem.elem, calculateKey(elem.elem, elem.key.second));
    priority_queue.swap(new_queue);
    #endif

    robin_hood::unordered_flat_set<Node> to_update;
    std::vector<Node> updates;
    // construct a set of all updated nodes
    for (const Cell &cell : grid.updated_cells_) {
        updates = cell.cornerNodes();
        to_update.insert(updates.begin(), updates.end());
    }

    for (const Node &s : to_update)
        updateNode(s);

    this->num_nodes_updated = to_update.size();
    //std::cout << num_nodes_updated << " nodes updated" << std::endl;
}

template<>
void ShiftedGridPlanner<0>::updateNode(const Node &node) {
    auto s_it = map.find_or_init(node);

    if (node != grid.goal_node_)
        RHS(s_it) = minRHS(node);

    enqueueIfInconsistent(s_it);
}

template<>
void ShiftedGridPlanner<1>::updateNode(const Node &node) {
    auto s_it = map.find_or_init(node);

    if (node != grid.goal_node_) {
        Node bptr;
        RHS(s_it) = minRHS(node, bptr);
        if (RHS(s_it) < INFINITY) //TODO check if this branch is necessary
            INFO(s_it) = bptr;
        enqueueIfInconsistent(s_it);
    }
}

template<>
void ShiftedGridPlanner<2>::updateNode(const Node &node) {
    auto s_it = map.find_or_init(node);

    if (node != grid.goal_node_) {
        Node bptr;
        RHS(s_it) = minRHS(node, bptr);
        if (RHS(s_it) < INFINITY) //TODO check if this branch is necessary
            INFO(s_it) = bptr;
        enqueueIfInconsistent(s_it);
    }
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
    #ifdef NO_HEURISTIC
        return {cost_so_far, cost_so_far};
    #else
    auto dist = grid.start_pos_.distance(s);
    return {cost_so_far + this->heuristic_multiplier * dist, cost_so_far};
    #endif
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
            if (rhs == cost)
                bptr = sp;
        } else if (valid2 and ((not valid1) or (ccn_g > cn_g))) {
            rhs = std::min(rhs, cost = computeOptimalCost(s, sp, cn, sp_g, cn_g));
            if (rhs == cost)
                bptr = cn;
        }
    }
    return rhs;
}

template <>
float ShiftedGridPlanner<1>::minRHSDecreasedNeighbor(const Node &sp, const Node &s, Node &bptr){
    auto ccn = grid.counterClockwiseNeighbor(sp, s);
    auto cn = grid.clockwiseNeighbor(sp, s);
    float cost1 = ccn.isValid() ? computeOptimalCost(sp, s, ccn) : INFINITY;
    float cost2 = cn.isValid() ? computeOptimalCost(sp, cn, s) : INFINITY;
    if (cost1<=cost2){
        bptr = s;
        return cost1;
    } else {
        bptr = cn;
        return cost2;
    }
}

template <>
float ShiftedGridPlanner<2>::minRHSDecreasedOrthoNeighbor(const Node &sp, const Node &s, Node &bptr){
    auto ccn = grid.counterClockwiseNeighbor(sp, s);
    auto cn = grid.clockwiseNeighbor(sp, s);
    float cost1 = ccn.isValid() ? computeOptimalCost(sp, s, ccn) : INFINITY;
    float cost2 = cn.isValid() ? computeOptimalCost(sp, cn, s) : INFINITY;
    if (cost1<=cost2){
        bptr = s;
        return cost1;
    } else {
        bptr = cn;
        return cost2;
    }
}


template <>
float ShiftedGridPlanner<2>::minRHSDecreasedDiagNeighbor(const Node &sp, const Node &s, Node &bptr){
    auto ccn = grid.counterClockwiseNeighbor(sp, s);
    auto cn = grid.clockwiseNeighbor(sp, s);
    auto g_ccn = map.getG(ccn);
    auto g_cn = map.getG(cn);
    auto g_s = map.getG(s);
    auto valid1 = ccn.isValid();
    auto valid2 = cn.isValid();

    if (valid1 and ((not valid2) or (g_ccn <= g_cn))) {
        bptr = s;
        return computeOptimalCost(sp, s, ccn, g_s, g_ccn);
    } else if (valid2 and ((not valid1) or (g_ccn > g_cn))) {
        bptr = cn;
        return computeOptimalCost(sp, s, cn, g_s, g_cn);
    }
    return INFINITY;
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
    ga = map.getG(p_a);
    gb = map.getG(p_b);

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