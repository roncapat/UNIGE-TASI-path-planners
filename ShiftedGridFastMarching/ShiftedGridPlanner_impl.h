#include <cmath>

template<int O>
void ShiftedGridPlanner<O>::set_start(const Position &pos){
    Base::set_start(pos);
    start_nodes = grid.start_cell_.corners();
}

template<int O>
void ShiftedGridPlanner<O>::init() {
    for (const auto &node: start_nodes)
        map.insert_or_assign(node, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_node_, INFINITY, 0.0f);
    priority_queue.insert(grid.goal_node_, calculate_key(grid.goal_node_, 0));
}

template<>
void ShiftedGridPlanner<0>::plan() {

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Node s = priority_queue.top_value();
        priority_queue.pop();
        ++expanded;

        // Get reference to the node
        auto s_it_opt = map.find(s);
        assert(s_it_opt.has_value());
        auto s_it = s_it_opt.value();

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Node &nbr : grid.neighbors_8(s)){
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_node_)
                    RHS(nbr_it) = min_rhs(nbr);
                enqueue_if_inconsistent(nbr_it);
            }
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Node &nbr : grid.neighbors_8(s)){
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_node_)
                    RHS(nbr_it) = min_rhs(nbr);
                enqueue_if_inconsistent(nbr_it);
            }
            if (s != grid.goal_node_)
                RHS(s_it) = min_rhs(s);
            enqueue_if_inconsistent(s_it);
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
        Node s = priority_queue.top_value();
        ++expanded;

        auto s_it_opt = map.find(s);
        assert(s_it_opt.has_value());
        auto s_it = s_it_opt.value();

        if (G(s_it) > RHS(s_it)) {
            G(s_it) = RHS(s_it);
            priority_queue.pop();
            for (const Node &sp : grid.neighbors_8(s)) {
                auto sp_it = map.find_or_init(sp);
                float rhs = min_rhs_decreased_neighbor(sp, s, bptr);
                if (rhs < RHS(sp_it)) {
                    RHS(sp_it) = rhs;
                    INFO(sp_it) = bptr;
                }
                enqueue_if_inconsistent(sp_it);
            }
        } else {
            G(s_it) = INFINITY;
            for (Node &sp : grid.neighbors_8(s)) {
                auto sp_it_opt = map.find(sp);
                assert(sp_it_opt.has_value());
                auto sp_it = sp_it_opt.value();
                if (INFO(sp_it) == s or INFO(sp_it) == grid.cw_neighbor(sp, s)) {
                    RHS(sp_it) = min_rhs(sp, bptr);
                    if (RHS(sp_it) < INFINITY) //TODO understand theese are necessary
                        INFO(sp_it) = bptr;
                    enqueue_if_inconsistent(sp_it);
                }
            }
            enqueue_if_inconsistent(s_it);
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
        Node s = priority_queue.top_value();
        ++expanded;

        auto s_it_opt = map.find(s);
        assert(s_it_opt.has_value());
        auto s_it = s_it_opt.value();

        if (G(s_it) > RHS(s_it)) {
            G(s_it) = RHS(s_it);
            priority_queue.pop();
            for (Node &sp : grid.neighbors_diag_4(s)) {
                auto sp_it = map.find_or_init(sp);
                float rhs = min_rhs_decreased_diag_neighbor(sp, s, bptr);
                if (rhs < RHS(sp_it)) {
                    RHS(sp_it) = rhs;
                    INFO(sp_it) = bptr;
                }
                enqueue_if_inconsistent(sp_it);
            }
            for (Node &sp : grid.neighbors_4(s)) {
                auto sp_it = map.find_or_init(sp);
                float rhs = min_rhs_decreased_ortho_neigbor(sp, s, bptr);
                if (rhs < RHS(sp_it)) {
                    RHS(sp_it) = rhs;
                    INFO(sp_it) = bptr;
                }
                enqueue_if_inconsistent(sp_it);
            }
        } else {
            G(s_it) = INFINITY;
            for (Node &sp : grid.neighbors_8(s)) {
                auto sp_it_opt = map.find(sp);
                assert(sp_it_opt.has_value());
                auto sp_it = sp_it_opt.value();

                if (INFO(sp_it) == s or INFO(sp_it) == grid.cw_neighbor(sp, s)) {
                    RHS(sp_it) = min_rhs(sp, bptr);
                    if (RHS(sp_it) < INFINITY) INFO(sp_it) = bptr;
                    enqueue_if_inconsistent(sp_it);
                }
            }
            enqueue_if_inconsistent(s_it);
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
        new_queue.insert(elem.elem, calculate_key(elem.elem, elem.key.second));
    priority_queue.swap(new_queue);
    #endif

    robin_hood::unordered_flat_set<Node> to_update;
    std::vector<Node> updates;
    // construct a set of all updated nodes
    for (const Cell &cell : grid.updated_cells_) {
        updates = cell.corners();
        to_update.insert(updates.begin(), updates.end());
    }

    for (const Node &s : to_update)
        update_node(s);

    this->num_nodes_updated = to_update.size();
    //std::cout << num_nodes_updated << " nodes updated" << std::endl;
}

template<>
void ShiftedGridPlanner<0>::update_node(const Node &node) {
    auto s_it = map.find_or_init(node);

    if (node != grid.goal_node_)
        RHS(s_it) = min_rhs(node);

    enqueue_if_inconsistent(s_it);
}

template<>
void ShiftedGridPlanner<1>::update_node(const Node &node) {
    auto s_it = map.find_or_init(node);

    if (node != grid.goal_node_) {
        Node bptr;
        RHS(s_it) = min_rhs(node, bptr);
        if (RHS(s_it) < INFINITY) //TODO check if this branch is necessary
            INFO(s_it) = bptr;
        enqueue_if_inconsistent(s_it);
    }
}

template<>
void ShiftedGridPlanner<2>::update_node(const Node &node) {
    auto s_it = map.find_or_init(node);

    if (node != grid.goal_node_) {
        Node bptr;
        RHS(s_it) = min_rhs(node, bptr);
        if (RHS(s_it) < INFINITY) //TODO check if this branch is necessary
            INFO(s_it) = bptr;
        enqueue_if_inconsistent(s_it);
    }
}

template<int O>
typename ShiftedGridPlanner<O>::Key ShiftedGridPlanner<O>::calculate_key(const Node &s) {
    auto[g, rhs] = map.get_g_rhs(s);
    return calculate_key(s, g, rhs);
}

template<int O>
typename ShiftedGridPlanner<O>::Key ShiftedGridPlanner<O>::calculate_key(const Node &s, const float g, const float rhs) {
    return calculate_key(s, std::min(g, rhs));
}

template<int O>
typename ShiftedGridPlanner<O>::Key ShiftedGridPlanner<O>::calculate_key(const Node &s, const float cost_so_far) {
    (void)s;
    #ifdef NO_HEURISTIC
     return {cost_so_far, cost_so_far};
    #else
    auto dist = grid.start_pos_.distance(s);
    return {cost_so_far + this->heuristic_multiplier * dist, cost_so_far};
    #endif
}

template<>
float ShiftedGridPlanner<0>::min_rhs(const Node &s) {
    float rhs = INFINITY;
    for (auto &[nbr1, nbr2] : grid.consecutive_neighbors(s))
        rhs = std::min(rhs, compute_optimal_cost(s, nbr1, nbr2));
    return rhs;
}

template<>
float ShiftedGridPlanner<1>::min_rhs(const Node &s, Node &bptr) {
    float rhs = INFINITY, cost;
    for (const auto &sp : grid.neighbors_8(s)) {
        auto ccn = grid.ccw_neighbor(s, sp);
        if (ccn.is_valid()) {
            rhs = std::min(rhs, cost = compute_optimal_cost(s, sp, ccn));
            if (rhs == cost)
                bptr = sp;
        }
    }
    return rhs;
}

template<>
float ShiftedGridPlanner<2>::min_rhs(const Node &s, Node &bptr) {
    float rhs = INFINITY, cost;
    for (const auto &sp : grid.neighbors_diag_4(s)) {
        auto ccn = grid.ccw_neighbor(s, sp);
        auto cn = grid.cw_neighbor(s, sp);
        float ccn_g = map.get_g(ccn);
        float cn_g = map.get_g(cn);
        float sp_g = map.get_g(sp);
        bool valid1 = ccn.is_valid();
        bool valid2 = cn.is_valid();

        if (valid1 and ((not valid2) or (ccn_g <= cn_g))) {
            rhs = std::min(rhs, cost = compute_optimal_cost(s, sp, ccn, sp_g, ccn_g));
            if (rhs == cost)
                bptr = sp;
        } else if (valid2 and ((not valid1) or (ccn_g > cn_g))) {
            rhs = std::min(rhs, cost = compute_optimal_cost(s, sp, cn, sp_g, cn_g));
            if (rhs == cost)
                bptr = cn;
        }
    }
    return rhs;
}

template <>
float ShiftedGridPlanner<1>::min_rhs_decreased_neighbor(const Node &sp, const Node &s, Node &bptr){
    auto ccn = grid.ccw_neighbor(sp, s);
    auto cn = grid.cw_neighbor(sp, s);
    float cost1 = ccn.is_valid() ? compute_optimal_cost(sp, s, ccn) : INFINITY;
    float cost2 = cn.is_valid() ? compute_optimal_cost(sp, cn, s) : INFINITY;
    if (cost1<=cost2){
        bptr = s;
        return cost1;
    } else {
        bptr = cn;
        return cost2;
    }
}

template <>
float ShiftedGridPlanner<2>::min_rhs_decreased_ortho_neigbor(const Node &sp, const Node &s, Node &bptr){
    auto ccn = grid.ccw_neighbor(sp, s);
    auto cn = grid.cw_neighbor(sp, s);
    float cost1 = ccn.is_valid() ? compute_optimal_cost(sp, s, ccn) : INFINITY;
    float cost2 = cn.is_valid() ? compute_optimal_cost(sp, cn, s) : INFINITY;
    if (cost1<=cost2){
        bptr = s;
        return cost1;
    } else {
        bptr = cn;
        return cost2;
    }
}


template <>
float ShiftedGridPlanner<2>::min_rhs_decreased_diag_neighbor(const Node &sp, const Node &s, Node &bptr){
    auto ccn = grid.ccw_neighbor(sp, s);
    auto cn = grid.cw_neighbor(sp, s);
    auto g_ccn = map.get_g(ccn);
    auto g_cn = map.get_g(cn);
    auto g_s = map.get_g(s);
    auto valid1 = ccn.is_valid();
    auto valid2 = cn.is_valid();

    if (valid1 and ((not valid2) or (g_ccn <= g_cn))) {
        bptr = s;
        return compute_optimal_cost(sp, s, ccn, g_s, g_ccn);
    } else if (valid2 and ((not valid1) or (g_ccn > g_cn))) {
        bptr = cn;
        return compute_optimal_cost(sp, s, cn, g_s, g_cn);
    }
    return INFINITY;
}



template<int O>
bool ShiftedGridPlanner<O>::end_condition() {
    // We need to check expansion until all 4 corners of start cell
    // used early stop from D* LITE
    auto top_key = priority_queue.top_key();
    Key max_start_key = {0, 0};
/*
    std::cout << std::endl;
    for (auto &node: start_nodes) {
        auto[g, rhs] = map.get_g_rhs(node);
        auto key = calculate_key(node, g, rhs);
        std::cout << g << " " << rhs << " " << key.first << std::endl;
    }
*/
    for (auto &node: start_nodes) {
        auto[g, rhs] = map.get_g_rhs(node);
        auto key = calculate_key(node, g, rhs);
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
float ShiftedGridPlanner<O>::compute_optimal_cost(const Node &n,
                                             const Node &p_a,
                                             const Node &p_b) {
    float ga, gb;
    ga = map.get_g(p_a);
    gb = map.get_g(p_b);

    return compute_optimal_cost(n, p_a, p_b, ga, gb);
}

template <int O>
float ShiftedGridPlanner<O>::compute_optimal_cost(const Node &n,
                                             const Node &p_a,
                                             const Node &p_b, float ga, float gb) {

    Position p(n);
    std::vector<Position> positions;
    float min_cost;

    assert(grid.is_valid_vertex(p));

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
    fill_traversal_costs(cell);
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
void ShiftedGridPlanner<O>::fill_traversal_costs(TraversalParams &t) {
    Cell cell_ind_c;

    if (t.p0.x == t.p1.x) {
        cell_ind_c = t.p1.neighbor_cell(t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cell_ind_c = t.p1.neighbor_cell(t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    t.c = grid.get_cost(cell_ind_c);
}