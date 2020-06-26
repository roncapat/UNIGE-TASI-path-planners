template<int O>
void DFMPlanner<O>::set_start(const Position &pos) {
    Base::set_start(pos);
}

template<int O>
void DFMPlanner<O>::init() {
    start_cell_it = map.insert_or_assign(grid.start_cell_, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_cell_, INFINITY, 0);
    priority_queue.insert(grid.goal_cell_, calculate_key(grid.goal_cell_, 0));
}

template<>
void DFMPlanner<0>::plan() {

    //TODO check initial point for traversability

    int expanded = 0;
    start_cell_it = map.find_or_init(grid.start_cell_);
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Cell s = priority_queue.top_value();
        priority_queue.pop();
        ++expanded;

        // Get reference to the node
        auto s_it_opt = map.find(s);
        assert(s_it_opt.has_value());
        auto s_it = s_it_opt.value();

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Cell &nbr : grid.neighbors_8(s)) {
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_cell_)
                    RHS(nbr_it) = min_rhs(nbr);
                enqueue_if_inconsistent(nbr_it);
            }
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Cell &nbr : grid.neighbors_8(s)) {
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_cell_)
                    RHS(nbr_it) = min_rhs(nbr);
                enqueue_if_inconsistent(nbr_it);
            }
            if (s != grid.goal_cell_)
                RHS(s_it) = min_rhs(s);
            enqueue_if_inconsistent(s_it);
        }
    }
    num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}

template<>
void DFMPlanner<1>::plan() {

    //TODO check initial point for traversability
    std::pair<Cell, Cell> bptrs;
    int expanded = 0;
    start_cell_it = map.find_or_init(grid.start_cell_);
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Cell s = priority_queue.top_value();
        ++expanded;

        // Get reference to the node
        auto s_it_opt = map.find(s);
        assert(s_it_opt.has_value());
        auto s_it = s_it_opt.value();

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            priority_queue.pop();
            for (const Cell &nbr : grid.neighbors_8(s)) {
                auto nbr_it = map.find_or_init(nbr);
                if (nbr != grid.goal_cell_) { //TODO understand why these checks are not necessary in FD*
                    float rhs = min_rhs_decreased_neighbor(nbr, s, bptrs);
                    if (rhs < RHS(nbr_it)) {
                        RHS(nbr_it) = rhs;
                        INFO(nbr_it) = bptrs;
                    }
                }
                enqueue_if_inconsistent(nbr_it);
            }
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Cell &nbr : grid.neighbors_8(s)) {
                auto nbr_it_opt = map.find(nbr);
                assert(nbr_it_opt.has_value());
                auto nbr_it = nbr_it_opt.value();
                if (INFO(nbr_it).first == s or INFO(nbr_it).second == s) {
                    if (nbr != grid.goal_cell_) //TODO understand why these checks are not necessary in FD*
                        RHS(nbr_it) = min_rhs(nbr, INFO(nbr_it));
                    enqueue_if_inconsistent(nbr_it);
                }
            }
            enqueue_if_inconsistent(s_it);
        }
    }
    this->num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}

template<int O>
void DFMPlanner<O>::update() {
    for (const Cell &cell : grid.updated_cells_)
        update_cell(cell);
    this->num_nodes_updated = grid.updated_cells_.size();
    //std::cout << num_cells_updated << " cells updated" << std::endl;
}

template<>
void DFMPlanner<0>::update_cell(const Cell &cell) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_cell_)
        RHS(s_it) = min_rhs(cell);

    enqueue_if_inconsistent(s_it);
}

template<>
void DFMPlanner<1>::update_cell(const Cell &cell) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_cell_)
        RHS(s_it) = min_rhs(cell, INFO(s_it));

    enqueue_if_inconsistent(s_it);
}

template<int O>
typename DFMPlanner<O>::Key DFMPlanner<O>::calculate_key(const Cell &s) const {
    float g, rhs;
    std::tie(g, rhs) = map.get_g_rhs(s);
    return calculate_key(s, g, rhs);
}

template<int O>
typename DFMPlanner<O>::Key DFMPlanner<O>::calculate_key(const Cell &s, float g, float rhs) const {
    return calculate_key(s, std::min(g, rhs));
}

template<int O>
typename DFMPlanner<O>::Key DFMPlanner<O>::calculate_key(const Cell &s, float cost_so_far) const {
#ifdef NO_HEURISTIC
    (void)s;
    return cost_so_far;
#else
    auto dist = grid.start_cell_.distance(s);
    return {cost_so_far + this->heuristic_multiplier * dist, cost_so_far};
#endif
}

template<>
float DFMPlanner<0>::min_rhs(const Cell &c) const {
    float tau = grid.get_cost(c);
    if (tau == INFINITY) return INFINITY;

    float stencil_ortho_cost = INFINITY;
    float stencil_diago_cost = INFINITY;

    std::pair<Cell, Cell> ortho_bptrs{}, diago_bptrs{};

    Cell ca1, cb1;
    float g_a_1, g_b_1;
    std::tie(ca1, g_a_1) = best_cell(c.top_cell(), c.bottom_cell());
    std::tie(cb1, g_b_1) = best_cell(c.left_cell(), c.right_cell());
#ifdef VERBOSE_EXTRACTION
    std::cout << "Expanding " << c.x << " " << c.y << std::endl;
    /*
    std::cout << "X- " << c.top_cell().x << " " << c.top_cell().y << "   cost " << map.get_g(c.top_cell()) << std::endl;
    std::cout << "X+ " << c.bottom_cell().x << " " << c.bottom_cell().y << "   cost " << map.get_g(c.bottom_cell())
              << std::endl;
    std::cout << "Y- " << c.left_cell().x << " " << c.left_cell().y << "   cost " << map.get_g(c.left_cell()) << std::endl;
    std::cout << "Y+ " << c.right_cell().x << " " << c.right_cell().y << "   cost " << map.get_g(c.right_cell())
              << std::endl;
    std::cout << "X min " << ca1.x << " " << ca1.y << "   cost " << g_a_1 << std::endl;
    std::cout << "Y min " << cb1.x << " " << cb1.y << "   cost " << g_b_1 << std::endl;
*/
     #endif
    std::tie(stencil_ortho_cost, ortho_bptrs) = compute_optimal_cost(ca1, cb1, g_a_1, g_b_1, tau, 1);

    Cell cc1, cd1;
    float g_c_1, g_d_1;
    std::tie(cc1, g_c_1) = best_cell(c.top_left_cell(), c.bottom_right_cell());
    std::tie(cd1, g_d_1) = best_cell(c.bottom_left_cell(), c.top_right_cell());
#ifdef VERBOSE_EXTRACTION
    std::cout << "Expanding " << c.x << " " << c.y << std::endl;
    /*
    std::cout << "X-Y- " << c.top_left_cell().x << " " << c.top_left_cell().y << "   cost " << map.get_g(c.top_left_cell()) << std::endl;
    std::cout << "X+Y+ " << c.bottom_right_cell().x << " " << c.bottom_right_cell().y << "   cost " << map.get_g(c.bottom_right_cell())
              << std::endl;
    std::cout << "X+Y- " << c.bottom_left_cell().x << " " << c.bottom_left_cell().y << "   cost " << map.get_g(c.bottom_left_cell()) << std::endl;
    std::cout << "X-Y+ " << c.top_right_cell().x << " " << c.top_right_cell().y << "   cost " << map.get_g(c.top_right_cell())
              << std::endl;
    std::cout << "D1 min " << cc1.x << " " << cc1.y << "   cost " << g_c_1 << std::endl;
    std::cout << "D2 min " << cd1.x << " " << cd1.y << "   cost " << g_d_1 << std::endl;
*/
     #endif
    std::tie(stencil_diago_cost, diago_bptrs) = compute_optimal_cost(cc1, cd1, g_c_1, g_d_1, tau, SQRT2);

    if (stencil_diago_cost < stencil_ortho_cost) {
        return stencil_diago_cost;
    } else {
        return stencil_ortho_cost;
    }
}

template<>
float DFMPlanner<1>::min_rhs(const Cell &c, std::pair<Cell, Cell> &bptrs) const {
    float tau = grid.get_cost(c);
    bptrs = {};
    if (tau == INFINITY) return INFINITY;

    float stencil_ortho_cost = INFINITY;
    float stencil_diago_cost = INFINITY;

    std::pair<Cell, Cell> ortho_bptrs{}, diago_bptrs{};

    Cell ca1, cb1;
    float g_a_1, g_b_1;
    std::tie(ca1, g_a_1) = best_cell(c.top_cell(), c.bottom_cell());
    std::tie(cb1, g_b_1) = best_cell(c.left_cell(), c.right_cell());
#ifdef VERBOSE_EXTRACTION
    /*
    std::cout << std::endl << "Expanding " << c.x << " " << c.y << std::endl;
    std::cout << "X- " << c.top_cell().x << " " << c.top_cell().y << "   cost " << map.get_g(c.top_cell()) << std::endl;
    std::cout << "X+ " << c.bottom_cell().x << " " << c.bottom_cell().y << "   cost " << map.get_g(c.bottom_cell())
              << std::endl;
    std::cout << "Y- " << c.left_cell().x << " " << c.left_cell().y << "   cost " << map.get_g(c.left_cell()) << std::endl;
    std::cout << "Y+ " << c.right_cell().x << " " << c.right_cell().y << "   cost " << map.get_g(c.right_cell())
              << std::endl;
    std::cout << "X min " << ca1.x << " " << ca1.y << "   cost " << g_a_1 << std::endl;
    std::cout << "Y min " << cb1.x << " " << cb1.y << "   cost " << g_b_1 << std::endl;
*/
#endif
    std::tie(stencil_ortho_cost, ortho_bptrs) = compute_optimal_cost(ca1, cb1, g_a_1, g_b_1, tau, 1);

    Cell cc1, cd1;
    float g_c_1, g_d_1;
    std::tie(cc1, g_c_1) = best_cell(c.top_left_cell(), c.bottom_right_cell());
    std::tie(cd1, g_d_1) = best_cell(c.bottom_left_cell(), c.top_right_cell());
#ifdef VERBOSE_EXTRACTION
    /*
    std::cout << std::endl << "Expanding " << c.x << " " << c.y << std::endl;
    std::cout << "X-Y- " << c.top_left_cell().x << " " << c.top_left_cell().y << "   cost " << map.get_g(c.top_left_cell()) << std::endl;
    std::cout << "X+Y+ " << c.bottom_right_cell().x << " " << c.bottom_right_cell().y << "   cost " << map.get_g(c.bottom_right_cell())
              << std::endl;
    std::cout << "X+Y- " << c.bottom_left_cell().x << " " << c.bottom_left_cell().y << "   cost " << map.get_g(c.bottom_left_cell()) << std::endl;
    std::cout << "X-Y+ " << c.top_right_cell().x << " " << c.top_right_cell().y << "   cost " << map.get_g(c.top_right_cell())
              << std::endl;
    std::cout << "D1 min " << cc1.x << " " << cc1.y << "   cost " << g_c_1 << std::endl;
    std::cout << "D2 min " << cd1.x << " " << cd1.y << "   cost " << g_d_1 << std::endl;
*/
#endif
    std::tie(stencil_diago_cost, diago_bptrs) = compute_optimal_cost(cc1, cd1, g_c_1, g_d_1, tau, SQRT2);

    if (stencil_diago_cost < stencil_ortho_cost) {
        bptrs = std::move(diago_bptrs);
        return stencil_diago_cost;
    } else {
        bptrs = std::move(ortho_bptrs);
        return stencil_ortho_cost;
    }
}

template<int O>
float DFMPlanner<O>::min_rhs_decreased_neighbor(const Cell &c, const Cell &nbr, std::pair<Cell, Cell> &bptrs) const {
    float tau = grid.get_cost(c);
    bptrs = {};
    if (tau == INFINITY) return INFINITY;

    float g_a_1 = map.get_g(nbr), g_b_1;
    Cell ca1(nbr), cb1;

    int dx = nbr.x - c.x;
    int dy = nbr.y - c.y;

    if (dx * dy == 0) {
        if (dx != 0) {
            std::tie(cb1, g_b_1) = best_cell(c.left_cell(), c.right_cell());
        } else {
            std::tie(cb1, g_b_1) = best_cell(c.top_cell(), c.bottom_cell());
        }
    } else {
        if (dx != dy) {
            std::tie(cb1, g_b_1) = best_cell(c.top_left_cell(), c.bottom_right_cell());
        } else {
            std::tie(cb1, g_b_1) = best_cell(c.bottom_left_cell(), c.top_right_cell());
        }
    }

    float stencil_cost = INFINITY;

#ifdef VERBOSE_EXTRACTION
    /*
    std::cout << std::endl << "Expanding " << c.x << " " << c.y << std::endl;
    std::cout << "X- " << c.top_cell().x << " " << c.top_cell().y << "   cost " << map.get_g(c.top_cell()) << std::endl;
    std::cout << "X+ " << c.bottom_cell().x << " " << c.bottom_cell().y << "   cost " << map.get_g(c.bottom_cell())
              << std::endl;
    std::cout << "Y- " << c.left_cell().x << " " << c.left_cell().y << "   cost " << map.get_g(c.left_cell()) << std::endl;
    std::cout << "Y+ " << c.right_cell().x << " " << c.right_cell().y << "   cost " << map.get_g(c.right_cell())
              << std::endl;
    std::cout << "X min " << ca1.x << " " << ca1.y << "   cost " << g_a_1 << std::endl;
    std::cout << "Y min " << cb1.x << " " << cb1.y << "   cost " << g_b_1 << std::endl;
*/
#endif
    std::tie(stencil_cost, bptrs) = compute_optimal_cost(ca1, cb1, g_a_1, g_b_1, tau, HYPOT(dx, dy));
    return stencil_cost;
}

template<int O>
bool DFMPlanner<O>::end_condition() const {
    auto top_key = priority_queue.top_key();
    return CONSISTENT(start_cell_it)
    and top_key >= calculate_key(grid.start_cell_, G(start_cell_it), RHS(start_cell_it));
}

template<int O>
std::pair<float, std::pair<Cell, Cell>>
DFMPlanner<O>::compute_optimal_cost(Cell &ca1, Cell &cb1, float ga1, float gb1, float tau, float h) const {
    if (ga1 > gb1) {
        std::swap(ga1, gb1);
        std::swap(ca1, cb1);
    }
    float stencil_cost;
    std::pair<Cell, Cell> bptrs;
    if (ga1 == INFINITY and gb1 == INFINITY) {
        bptrs = {};
        stencil_cost = INFINITY;
    } else if ((tau * h) > (gb1 - ga1)) {
        bptrs = {ca1, cb1};
        stencil_cost = (ga1 + gb1 + std::sqrt(float(2 * SQUARE(tau * h) - SQUARE(gb1 - ga1)))) * 0.5f;
    } else {
        bptrs = {ca1, {}};
        stencil_cost = ga1 + tau * h;
    }
    return {stencil_cost, bptrs};
}

template<int O>
std::pair<Cell, float> DFMPlanner<O>::best_cell(const Cell &a, const Cell &b) const {
    auto ca = map.get_g(a), cb = map.get_g(b);
    if (ca < cb)
        return {a, ca};
    else
        return {b, cb};
}