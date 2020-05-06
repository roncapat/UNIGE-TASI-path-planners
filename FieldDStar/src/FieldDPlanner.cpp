#include "FieldDPlanner.h"
#include <cmath>
#include <array>
#include <numeric>
#include <chrono>
#include <iostream>

FieldDPlanner::FieldDPlanner() = default;

void FieldDPlanner::init() {
    initialize_search = true;
}

int FieldDPlanner::step() {
    // don't plan unless the map has been initialized and a goal node has been set
    if (initialize_graph_) return LOOP_FAILURE_NO_GRAPH;

    // don't plan unless a goal node has been set
    if (not goal_set_) return LOOP_FAILURE_NO_GOAL;

    auto begin = std::chrono::steady_clock::now();
    if (initialize_search or new_goal_) {
        new_goal_ = false;
        initialize_search = false;
        initializeSearch();
    } else if (new_start) {
        new_start = false;
        Queue new_queue;
        for (const auto &elem: priority_queue)
            // Only heuristic changes, so either G or RHS is kept the same
            new_queue.insert(elem.elem, calculateKey(elem.elem, elem.key.second));
        priority_queue.swap(new_queue);
        // gather cells with updated edge costs and update affected nodes
        updateNodesAroundUpdatedCells();
    }
    auto end = std::chrono::steady_clock::now();
    u_time = std::chrono::duration<float, std::milli>(end - begin).count();

    // only update the graph if nodes have been updated
    begin = std::chrono::steady_clock::now();
    if ((num_nodes_updated > 0)) {
        if (optimization_lvl == 0) {
            computeShortestPath_0();
        } else {
            computeShortestPath_1();
        }
    } else {
        num_nodes_expanded = 0;
    }
    end = std::chrono::steady_clock::now();
    p_time = std::chrono::duration<float, std::milli>(end - begin).count();

    begin = std::chrono::steady_clock::now();
    constructOptimalPath();
    end = std::chrono::steady_clock::now();
    e_time = std::chrono::duration<float, std::milli>(end - begin).count();
    std::cout << "Update time     = " << u_time << " ms" << std::endl;
    std::cout << "Planning time   = " << p_time << " ms" << std::endl;
    std::cout << "Extraction time = " << e_time << " ms" << std::endl;

    return LOOP_OK;
}

void FieldDPlanner::set_map(const std::shared_ptr<uint8_t[]> &m, int w, int l) {
    grid.initializeGraph(m, w, l);
    initialize_graph_ = false;
}

void FieldDPlanner::set_goal(const Position &point) {
    Node new_goal(point);
    if (grid.goal_node_ != new_goal)
        new_goal_ = true;
    grid.setGoal(new_goal);
    goal_set_ = true;
}

bool FieldDPlanner::isVertex(const Position &p) {
    bool is_vertex = (ceilf(p.x) == p.x) && (ceilf(p.y) == p.y);
    bool satisfies_bounds = grid.isValid(p);
    return is_vertex && satisfies_bounds;
}

FieldDPlanner::Queue::Key FieldDPlanner::calculateKey(const Node &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return calculateKey(s, g, rhs);
}

FieldDPlanner::Queue::Key FieldDPlanner::calculateKey(const Node &s, const float g, const float rhs) {
    return calculateKey(s, std::min(g, rhs));
}

FieldDPlanner::Queue::Key FieldDPlanner::calculateKey(const Node &s, const float cost_so_far) {
    auto dist = grid.start_pos_.distance(s);
    return {cost_so_far + heuristic_multiplier * dist, cost_so_far};
}

void FieldDPlanner::initializeSearch() {
    num_nodes_updated = 0;
    num_nodes_expanded = 0;
    map.clear();
    priority_queue.clear();
    grid.updated_cells_.clear();
    for (const auto &node: start_nodes)
        map.insert_or_assign(node, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_node_, INFINITY, 0.0f);
    priority_queue.insert(grid.goal_node_, calculateKey(grid.goal_node_, 0));
    num_nodes_updated = 1;
}

unsigned long FieldDPlanner::computeShortestPath_1() {
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
                    BPTR(sp_it) = s;
                }
                if ((cost1 > cost2) and (RHS(sp_it) > cost2)) {
                    RHS(sp_it) = cost2;
                    BPTR(sp_it) = cn;
                }
                enqueueIfInconsistent(sp_it);
            }
        } else {
            G(s_it) = INFINITY;
            for (Node &sp : grid.neighbors_8(s)) {
                auto sp_it = map.find(sp);
                assert(sp_it != map.end());

                if (BPTR(sp_it) == s or BPTR(sp_it) == grid.clockwiseNeighbor(sp, s)) {
                    RHS(sp_it) = minRHS_1(sp, bptr);
                    if (RHS(sp_it) < INFINITY) BPTR(sp_it) = bptr;
                    enqueueIfInconsistent(sp_it);
                }
            }
            enqueueIfInconsistent(s_it);
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    return num_nodes_expanded;
}

float FieldDPlanner::minRHS_1(const Node &s, Node &bptr) {
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

bool FieldDPlanner::end_condition() {
    // We need to check expansion until all 4 corners of start cell
    // used early stop from D* LITE
    auto top_key = priority_queue.topKey();
    for (auto &node: start_nodes) {
        auto[g, rhs] = map.getGandRHS(node);
        if ((top_key < calculateKey(node, g, rhs)) or (rhs > g)) {
            return false;
        }
    }
    return true; //STOP: all 4 conditions met
}

unsigned long FieldDPlanner::computeShortestPath_0() {

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
            for (const Node &nbr : grid.neighbors_8(s))
                updateNode_0(nbr);
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Node &nbr : grid.neighbors_8(s))
                updateNode_0(nbr);
            updateNode_0(s);
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    return num_nodes_expanded;
}

void FieldDPlanner::updateNode_0(const Node &s) {
    auto s_it = map.find_or_init(s);

    if (s != grid.goal_node_)
        RHS(s_it) = minRHS_0(s);

    enqueueIfInconsistent(s_it);
}

float FieldDPlanner::minRHS_0(const Node &s) {
    float rhs = INFINITY;
    for (auto &[nbr1, nbr2] : grid.consecutiveNeighbors(s))
        rhs = std::min(rhs, computeOptimalCost(s, nbr1, nbr2));
    return rhs;
}

unsigned long FieldDPlanner::updateNodesAroundUpdatedCells() {
    std::unordered_set<Node> to_update;
    std::vector<Node> updates;
    // construct a set of all updated nodes
    for (const Cell &cell : grid.updated_cells_) {
        updates = cell.corners();
        to_update.insert(updates.begin(), updates.end());
    }

    for (const Node &s : to_update) {
        if (optimization_lvl == 0) {
            updateNode_0(s);
        } else {
            auto s_it = map.find_or_init(s);

            if (s != grid.goal_node_) {
                Node bptr;
                RHS(s_it) = minRHS_1(s, bptr);
                if (RHS(s_it) < INFINITY) BPTR(s_it) = bptr;
                enqueueIfInconsistent(s_it);
            }
        }
    }

    num_nodes_updated = to_update.size();
    std::cout << num_nodes_updated << " nodes updated" << std::endl;
    return num_nodes_updated;
}

void FieldDPlanner::enqueueIfInconsistent(ExpandedMap::iterator it) {
    if (G(it) != RHS(it))
        priority_queue.insert_or_update(NODE(it), calculateKey(NODE(it), G(it), RHS(it)));
    else
        priority_queue.remove_if_present(NODE(it));
}

void FieldDPlanner::constructOptimalPath() {
    path_.clear();
    cost_.clear();
    total_cost = 0;
    total_dist = 0;

    float min_cost;
    path_additions pa;

    int curr_step = 0;
    // TODO do something better than this sh*t
    int max_steps = 2000;

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
            step_dist += p->distance(*(p+1));
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

    std::cout << "Found path. Cost: " << total_cost << " Distance: " << total_dist << std::endl;
}

float FieldDPlanner::computeOptimalCost(const Node &n,
                                        const Node &p_a,
                                        const Node &p_b) {

    Position p(n);
    std::vector<Position> positions;
    float min_cost;

    assert(isVertex(p));

    TraversalParams cell{};
    cell.p0 = p;
    bool cond = p.aligned(p_a);
    cell.p1 = cond ? p_a : p_b;
    cell.p2 = cond ? p_b : p_a;

    assert((cell.p0.x == cell.p1.x) || (cell.p0.y == cell.p1.y));
    assert((cell.p0.x != cell.p2.x) && (cell.p0.y != cell.p2.y));

    if (first_run_trick and initialize_search) {
        cell.g1 = map.getRHS(cell.p1);
        cell.g2 = map.getRHS(cell.p2);
    } else {
        cell.g1 = map.getG(cell.p1);
        cell.g2 = map.getG(cell.p2);
    }
    if (cell.g1 == INFINITY && cell.g2 == INFINITY)
        return INFINITY;
    getBC(cell);
    if (cell.c == INFINITY)
        return INFINITY;
    cell.f = cell.g1 - cell.g2;

    if (cell.c > cell.b) {
        if ((cell.f <= 0) or (SQUARE(cell.f) <= CATH(cell.c, cell.b))) {
            min_cost = TraversalTypeIII::Corner::cost(cell);
        } else if ((cell.f <= cell.b) and (cell.c > (cell.f * SQRT2))) {
            min_cost = TraversalTypeII::Corner::cost(cell);
        } else if ((cell.f > cell.b) and (cell.c > (cell.b * SQRT2))) {
            min_cost = TraversalTypeI::Corner::cost(cell);
        } else {
            min_cost = TraversalTypeA::Corner::cost(cell);
        }
    } else {
        if (cell.f <= 0) {
            min_cost = TraversalTypeB::Corner::cost(cell);
        } else if ((cell.f * SQRT2) < cell.c) {
            min_cost = TraversalTypeII::Corner::cost(cell);
        } else {
            min_cost = TraversalTypeA::Corner::cost(cell);
        }
    }

    return min_cost;
}

path_additions FieldDPlanner::traversalFromCorner(const Position &p,
                                                  const Node &p_a,
                                                  const Node &p_b,
                                                  float &step_cost) {
    assert(isVertex(p));

    TraversalParams cell{};
    cell.p0 = p;
    bool cond = p.aligned(p_a);
    cell.p1 = cond ? p_a : p_b;
    cell.p2 = cond ? p_b : p_a;

    assert(cell.p0.aligned(cell.p1));
    assert(not cell.p0.aligned(cell.p2));

    cell.g1 = map.getRHS(cell.p1);
    cell.g2 = map.getRHS(cell.p2);
    getBC(cell);

    return InterpolatedTraversal::traversalFromCorner(cell, step_cost);
}

path_additions FieldDPlanner::traversalFromContiguousEdge(const Position &p,
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
    getBC(cell1);
    cell1.q = 1 - std::abs(cell1.p1.y - p.y) - std::abs(cell1.p1.x - p.x);

    return InterpolatedTraversal::traversalFromContiguousEdge(cell1, step_cost);
}

path_additions FieldDPlanner::traversalFromOppositeEdge(const Position &p,
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
    getBC(cell1);
    getBC(cell2);
    cell1.p = std::abs(p.y - cell1.p0.y) + std::abs(p.x - cell1.p0.x);
    cell2.p = 1 - cell1.p;

    return InterpolatedTraversal::traversalFromOppositeEdge(cell1, cell2, step_cost);
}

path_additions FieldDPlanner::traversalFromEdge(const Position &p,
                                                const Node &p_a,
                                                const Node &p_b,
                                                float &step_cost) {

    assert(!isVertex(p));

    bool cond_1 = (p.x == p_a.x || p.y == p_a.y);
    bool cond_2 = (p.x == p_b.x || p.y == p_b.y);
    if (cond_1 || cond_2) {
        assert(cond_1 xor cond_2);
        return traversalFromContiguousEdge(p, p_a, p_b, step_cost);
    } else {
        return traversalFromOppositeEdge(p, p_a, p_b, step_cost);
    }
}

path_additions FieldDPlanner::getPathAdditions(const Position &p,
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

        if (isVertex(p))
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
        if (do_lookahead and not isVertex(temp_pa.steps.back())) {
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

bool FieldDPlanner::goalReached(const Position &p) {
    return grid.goal_pos_.x == p.x && grid.goal_pos_.y == p.y;
}

FieldDPlanner::ExpandedMap::iterator
FieldDPlanner::ExpandedMap::find_or_init(const Node &n) {
    iterator it = find(n);
    if (it == end()) { // Init node if not yet considered
        it = emplace(n, std::make_tuple(INFINITY, INFINITY, NULLNODE)).first;
    }
    return it;
}

FieldDPlanner::ExpandedMap::iterator
FieldDPlanner::ExpandedMap::insert_or_assign(const Node &s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    auto it = find(s);
    if (it != end()) {
        std::get<0>(it->second) = g;
        std::get<1>(it->second) = rhs;
        return it;
    } else {
        [[maybe_unused]] auto[it, ok] = emplace(s, std::make_tuple(g, rhs, NULLNODE));
        assert(ok);
        return it;
    }
}

std::pair<float, float> FieldDPlanner::ExpandedMap::getGandRHS(const Node &s) {
    ExpandedMap::iterator it;
    if ((it = find(s)) != end())
        return {G(it), RHS(it)};
    else
        return {INFINITY, INFINITY};
}

float FieldDPlanner::ExpandedMap::getG(const Node &s) {
    ExpandedMap::iterator it;
    if ((it = find(s)) != end())
        return G(it);
    else
        return INFINITY;
}

float FieldDPlanner::ExpandedMap::getRHS(const Node &s) {
    ExpandedMap::iterator it;
    if ((it = find(s)) != end())
        return RHS(it);
    else
        return INFINITY;
}

bool FieldDPlanner::consistent(const Node &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return g == rhs;
}

bool FieldDPlanner::consistent(const ExpandedMap::iterator &it) {
    return G(it) == RHS(it);
}

void FieldDPlanner::set_start(const Position &pos) {
    grid.setStart(pos);
    start_nodes = grid.start_cell_.corners();
    new_start = true;
}

void FieldDPlanner::patch_map(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h) {
    grid.updateGraph(patch, x, y, w, h);
}

void FieldDPlanner::set_occupancy_threshold(float threshold) { grid.setOccupancyThreshold(threshold); }
void FieldDPlanner::set_heuristic_multiplier(float mult) { heuristic_multiplier = mult; }
void FieldDPlanner::set_lookahead(bool use_lookahead) { lookahead = use_lookahead; }
void FieldDPlanner::set_optimization_lvl(int lvl) { optimization_lvl = lvl; }
void FieldDPlanner::set_first_run_trick(bool enable) { first_run_trick = enable; }

// p must be aligned with p_1, p_1 aligned with p_2, p and P_2 diagonal neighbors
void FieldDPlanner::getBC(TraversalParams &t) {
    Cell cell_ind_b, cell_ind_c;

    if (t.p0.x == t.p1.x) {
        cell_ind_b = t.p1.neighborCell(t.p2.x > t.p1.x, t.p0.y > t.p1.y);
        cell_ind_c = t.p1.neighborCell(t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cell_ind_b = t.p1.neighborCell(t.p0.x < t.p1.x, t.p2.y < t.p1.y);
        cell_ind_c = t.p1.neighborCell(t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    t.b = grid.getTraversalCost(cell_ind_b);
    t.c = grid.getTraversalCost(cell_ind_c);
}