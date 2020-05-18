template<int O>
void DFMPlanner<O>::set_start(const Position &pos) {
    Base::set_start(pos);
    start_nodes = {Node(pos).cellTopRight(),
                   Node(pos).cellTopLeft(),
                   Node(pos).cellBottomRight(),
                   Node(pos).cellBottomLeft()};
}

template<int O>
void DFMPlanner<O>::init() {
    map.insert_or_assign(grid.start_cell_, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_cell_, INFINITY, 0);
    priority_queue.insert(grid.goal_cell_, calculateKey(grid.goal_cell_, 0));
}


template<>
void DFMPlanner<0>::plan() {

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Cell s = priority_queue.topValue();
        priority_queue.pop();
        ++expanded;
        // Get reference to the node
        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Cell &nbr : grid.neighbors_8(s)) {
                updateCell(nbr);
            }
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Cell &nbr : grid.neighbors_8(s)) {
                updateCell(nbr);
            }
            updateCell(s);
        }
    }
    this->num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}

template<>
void DFMPlanner<1>::plan() {

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Cell s = priority_queue.topValue();
        priority_queue.pop();
        ++expanded;
        // Get reference to the node
        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Cell &nbr : grid.neighbors_8(s)) {
                updateCellDecreasedNeighbor(nbr, s);
            }
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Cell &nbr : grid.neighbors_8(s)) {
                auto nbr_it = map.find(nbr);
                assert(nbr_it != map.end());
                if (INFO(nbr_it).first == s or INFO(nbr_it).second == s)
                    updateCell(nbr);
            }
            updateCell(s);
        }
    }
    this->num_nodes_expanded = expanded;
    //std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
}


template<int O>
void DFMPlanner<O>::update() {
    for (const Cell &cell : grid.updated_cells_)
        updateCell(cell);
    this->num_nodes_updated = grid.updated_cells_.size();
    //std::cout << num_cells_updated << " cells updated" << std::endl;
}


template<int O>
typename DFMPlanner<O>::Key DFMPlanner<O>::calculateKey(const Cell &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return calculateKey(s, g, rhs);
}

template<int O>
typename DFMPlanner<O>::Key DFMPlanner<O>::calculateKey(const Cell &s, float g, float rhs) {
    return calculateKey(s, std::min(g, rhs));
}

template<int O>
typename DFMPlanner<O>::Key DFMPlanner<O>::calculateKey(const Cell &, float cost_so_far) {
    return cost_so_far;
}


template<int O>
bool DFMPlanner<O>::end_condition() {
    auto top_key = priority_queue.topKey();
    auto[g, rhs] = map.getGandRHS(grid.start_cell_);
    return ((top_key >= calculateKey(grid.start_cell_, g, rhs)) and (rhs == g));
}

template<int O>
void DFMPlanner<O>::updateCellDecreasedNeighbor(const Cell &cell, const Cell &nbr) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_cell_) {
        std::pair<Cell, Cell> bptrs;
        float rhs = computeOptimalCostDecreasedNeighbor(cell, nbr, bptrs);
        if (rhs < RHS(s_it)) {
            RHS(s_it) = rhs;
            INFO(s_it) = bptrs;
        }
    }
    this->enqueueIfInconsistent(s_it);
}

template<int O>
float DFMPlanner<O>::computeOptimalCostDecreasedNeighbor(const Cell &c, const Cell &nbr, std::pair<Cell, Cell> &bptrs) {
    float tau = grid.getCost(c);
    bptrs = {};
    if (tau == INFINITY) return INFINITY;


    float g_a_1 = map.getG(nbr), g_b_1;
    Cell ca1(nbr), cb1;

    int dx = nbr.x - c.x;
    int dy = nbr.y - c.y;

    if (dx * dy == 0) {
        if (dx != 0) {
            std::tie(cb1, g_b_1) = minCost(c.leftCell(), c.rightCell());
        } else {
            std::tie(cb1, g_b_1) = minCost(c.topCell(), c.bottomCell());
        }
    } else {
        if (dx != dy) {
            std::tie(cb1, g_b_1) = minCost(c.topLeftCell(), c.bottomRightCell());
        } else {
            std::tie(cb1, g_b_1) = minCost(c.bottomLeftCell(), c.topRightCell());
        }
    }


    float stencil_cost = INFINITY;

#ifdef VERBOSE_EXTRACTION
    std::cout << std::endl << "Expanding " << c.x << " " << c.y << std::endl;
    std::cout << "X- " << c.topCell().x << " " << c.topCell().y << "   cost " << map.getG(c.topCell()) << std::endl;
    std::cout << "X+ " << c.bottomCell().x << " " << c.bottomCell().y << "   cost " << map.getG(c.bottomCell())
              << std::endl;
    std::cout << "Y- " << c.leftCell().x << " " << c.leftCell().y << "   cost " << map.getG(c.leftCell()) << std::endl;
    std::cout << "Y+ " << c.rightCell().x << " " << c.rightCell().y << "   cost " << map.getG(c.rightCell())
              << std::endl;
    std::cout << "X min " << ca1.x << " " << ca1.y << "   cost " << g_a_1 << std::endl;
    std::cout << "Y min " << cb1.x << " " << cb1.y << "   cost " << g_b_1 << std::endl;
#endif
    std::tie(stencil_cost, bptrs) = FMcost(ca1, cb1, g_a_1, g_b_1, tau, HYPOT(dx, dy));
    return stencil_cost;
}

template<int O>
std::pair<float, std::pair<Cell, Cell>>
DFMPlanner<O>::FMcost(Cell &ca1, Cell &cb1, float ga1, float gb1, float tau, float h) {
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
        stencil_cost = (ga1 + gb1 + std::sqrt(2 * SQUARE(tau * h) - SQUARE(gb1 - ga1))) / 2.0f;
    } else {
        bptrs = {ca1, {}};
        stencil_cost = ga1 + tau * h;
    }
    return {stencil_cost, bptrs};
}

template<int O>
void DFMPlanner<O>::updateCell(const Cell &cell) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_cell_)
        RHS(s_it) = computeOptimalCost(cell, INFO(s_it));

    this->enqueueIfInconsistent(s_it);
}

template<int O>
float DFMPlanner<O>::computeOptimalCost(const Cell &c, std::pair<Cell, Cell> &bptrs) {
    float tau = grid.getCost(c);
    bptrs = {};
    if (tau == INFINITY) return INFINITY;

    float stencil_ortho_cost = INFINITY;
    float stencil_diago_cost = INFINITY;

    std::pair<Cell, Cell> ortho_bptrs{}, diago_bptrs{};

    auto[ca1, g_a_1] = minCost(c.topCell(), c.bottomCell());
    auto[cb1, g_b_1] = minCost(c.leftCell(), c.rightCell());
#ifdef VERBOSE_EXTRACTION
    std::cout << std::endl << "Expanding " << c.x << " " << c.y << std::endl;
    std::cout << "X- " << c.topCell().x << " " << c.topCell().y << "   cost " << map.getG(c.topCell()) << std::endl;
    std::cout << "X+ " << c.bottomCell().x << " " << c.bottomCell().y << "   cost " << map.getG(c.bottomCell())
              << std::endl;
    std::cout << "Y- " << c.leftCell().x << " " << c.leftCell().y << "   cost " << map.getG(c.leftCell()) << std::endl;
    std::cout << "Y+ " << c.rightCell().x << " " << c.rightCell().y << "   cost " << map.getG(c.rightCell())
              << std::endl;
    std::cout << "X min " << ca1.x << " " << ca1.y << "   cost " << g_a_1 << std::endl;
    std::cout << "Y min " << cb1.x << " " << cb1.y << "   cost " << g_b_1 << std::endl;
#endif
    std::tie(stencil_ortho_cost, ortho_bptrs) = FMcost(ca1, cb1, g_a_1, g_b_1, tau, 1);

    auto[cc1, g_c_1] = minCost(c.topLeftCell(), c.bottomRightCell());
    auto[cd1, g_d_1] = minCost(c.bottomLeftCell(), c.topRightCell());
#ifdef VERBOSE_EXTRACTION
    std::cout << std::endl << "Expanding " << c.x << " " << c.y << std::endl;
    std::cout << "X-Y- " << c.topLeftCell().x << " " << c.topLeftCell().y << "   cost " << map.getG(c.topLeftCell()) << std::endl;
    std::cout << "X+Y+ " << c.bottomRightCell().x << " " << c.bottomRightCell().y << "   cost " << map.getG(c.bottomRightCell())
              << std::endl;
    std::cout << "X+Y- " << c.bottomLeftCell().x << " " << c.bottomLeftCell().y << "   cost " << map.getG(c.bottomLeftCell()) << std::endl;
    std::cout << "X-Y+ " << c.topRightCell().x << " " << c.topRightCell().y << "   cost " << map.getG(c.topRightCell())
              << std::endl;
    std::cout << "D1 min " << cc1.x << " " << cc1.y << "   cost " << g_c_1 << std::endl;
    std::cout << "D2 min " << cd1.x << " " << cd1.y << "   cost " << g_d_1 << std::endl;
#endif
    std::tie(stencil_diago_cost, diago_bptrs) = FMcost(cc1, cd1, g_c_1, g_d_1, tau, SQRT2);

    if (stencil_diago_cost < stencil_ortho_cost) {
        bptrs = std::move(diago_bptrs);
        return stencil_diago_cost;
    } else {
        bptrs = std::move(ortho_bptrs);
        return stencil_ortho_cost;
    }
}

template<int O>
std::pair<Cell, float> DFMPlanner<O>::minCost(const Cell &a, const Cell &b) {
    auto ca = map.getG(a), cb = map.getG(b);
    if (ca < cb)
        return {a, ca};
    else
        return {b, cb};
}

//FIXME use only getC
template<int O>
void DFMPlanner<O>::getBC(TraversalParams &t) {
    Cell cb, cc;

    if (t.p0.x == t.p1.x) {
        cb = t.p1.neighborCell(t.p2.x > t.p1.x, t.p0.y > t.p1.y);
        cc = t.p1.neighborCell(t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cb = t.p1.neighborCell(t.p0.x < t.p1.x, t.p2.y < t.p1.y);
        cc = t.p1.neighborCell(t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    std::vector b{grid.getCost(cb), grid.getCost(cb.rightCell()), grid.getCost(cb.bottomCell()),
                  grid.getCost(cb.bottomRightCell())};
    std::vector c{grid.getCost(cc), grid.getCost(cc.rightCell()), grid.getCost(cc.bottomCell()),
                  grid.getCost(cc.bottomRightCell())};

    t.b = *std::min_element(b.begin(), b.end());
    t.c = *std::min_element(c.begin(), c.end());
}