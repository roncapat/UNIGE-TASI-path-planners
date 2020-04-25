#include "FieldDPlanner.h"
#include <cmath>
#include <array>
#include <numeric>
#include <chrono>

FieldDPlanner::FieldDPlanner() = default;

void FieldDPlanner::init() {
    grid.setOccupancyThreshold(occupancy_threshold_);
    setGoalDistance(static_cast<float>(goal_range_));
    num_nodes_updated = 0;
    num_nodes_expanded = 0;
    initialize_search = true;
}

int FieldDPlanner::step() {
    // don't plan unless the map has been initialized and a goal node has been set
    if (initialize_graph_) return LOOP_FAILURE_NO_GRAPH;

    // don't plan unless a goal node has been set
    if (not goal_set_) return LOOP_FAILURE_NO_GOAL;

    if (initialize_search or goal_changed_) {
        std::cout << "Initializing Search..." << std::endl;
        goal_changed_ = false;
        initializeSearch();
    }

    // gather cells with updated edge costs and update affected nodes
    auto begin = std::chrono::steady_clock::now();
    updateNodesAroundUpdatedCells();
    auto end = std::chrono::steady_clock::now();

    auto time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    auto time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    auto u_time = time_ms + time_us / 1000.0f;

    // only update the graph if nodes have been updated
    begin = std::chrono::steady_clock::now();
    if ((num_nodes_updated > 0) or initialize_search) {
        if (optimization_lvl == 0) {
            computeShortestPath_0();
        } else {
            computeShortestPath_1();
        }
    } else {
        num_nodes_expanded = 0;
    }
    end = std::chrono::steady_clock::now();

    time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    auto p_time = time_ms + time_us / 1000.0f;

    begin = std::chrono::steady_clock::now();
    constructOptimalPath();
    end = std::chrono::steady_clock::now();

    time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count();
    time_us = std::chrono::duration_cast<std::chrono::microseconds>(end - begin).count();
    auto e_time = time_ms + time_us / 1000.0f;
    std::cout << "Update time     = " << u_time << " ms" << std::endl;
    std::cout << "Planning time   = " << p_time << " ms" << std::endl;
    std::cout << "Extraction time = " << e_time << " ms" << std::endl;

    initialize_search = false;
    return LOOP_OK;
}

void FieldDPlanner::publish_expanded_set() {

    float max_g = -INFINITY;

    for (std::pair<Node, std::tuple<float, float>> e : expanded_map) {
        // ignore infinite g-values when getting the max value
        if (std::get<0>(e.second) == INFINITY)
            continue;
        max_g = std::max(max_g, std::get<0>(e.second));
    }

    std::vector<std::tuple<int, int, float>> expanded;
    expanded.reserve(expanded_map.size());
    for (auto e : expanded_map) {
        auto x = static_cast<float>(std::get<0>(e.first.getIndex()) - x_initial_) * map_->resolution;
        auto y = static_cast<float>(std::get<1>(e.first.getIndex()) - y_initial_) * map_->resolution;
        auto g = std::get<0>(e.second) / max_g * 255.f;
        expanded.emplace_back(x, y, g);
    }
    expanded_cb({expanded, num_nodes_expanded, num_nodes_updated});
}

void FieldDPlanner::publish_path() {
    size_t num_poses = path_.size();
    std::vector<Pose> poses;
    poses.reserve(num_poses);

    if (num_poses > 1 || !follow_old_path_) {
        for (size_t i = 0; i < num_poses; i++) {
            Pose pose;

            // calculate the position in the "/odom" frame
            pose.x = (path_[i].x - x_initial_) * grid.resolution_;
            pose.y = (path_[i].y - y_initial_) * grid.resolution_;

            // calculate the orientation
            double delta_x = ((path_[i + 1].x - x_initial_) * grid.resolution_) - pose.x;
            double delta_y = ((path_[i + 1].y - y_initial_) * grid.resolution_) - pose.y;
            double heading = atan2(delta_y, delta_x);
            pose.orientation = heading;

            poses.push_back(pose);
        }

        poses.back().orientation = poses[num_poses - 2].orientation;
        poses_cb(poses, total_dist, total_cost);
    }
}

void FieldDPlanner::set_map(const MapPtr &msg) {
    map_ = msg;  // update current map

    if (initialize_graph_) {
        x_initial_ = map_->x_initial;
        y_initial_ = map_->y_initial;
        grid.initializeGraph(map_);
        start_pos = Position(msg->x, msg->y);
        initialize_graph_ = false;
    } else {
        //node_grid_.updateGraph(map_); // FIXME
    }
}

void FieldDPlanner::set_goal(std::pair<float, float> point) {
    int goal_x, goal_y;
    goal_x = static_cast<int>(std::round(point.first / grid.resolution_)) + x_initial_;
    goal_y = static_cast<int>(std::round(point.second / grid.resolution_)) + y_initial_;

    Node new_goal(goal_x, goal_y);

    if (grid.goal_ != new_goal)
        goal_changed_ = true;

    grid.setGoal(new_goal);

    float distance_to_goal = grid.euclideanHeuristic(new_goal) * grid.resolution_;

    std::cout << (goal_changed_ ? "New" : "Same") << " waypoint received. Search Problem Goal = " << grid.goal_
              << ". Distance: " << distance_to_goal << "m." << std::endl;

    if (distance_to_goal > maximum_distance_) {
        std::cout << "Planning to waypoint more than " << maximum_distance_
                  << "m. away - distance = " << distance_to_goal << std::endl;
        goal_set_ = false;
    } else if (distance_to_goal < goal_range_) {
        std::cout << "Waiting for new waypoint..." << std::endl;
        goal_set_ = false;
    } else {
        goal_set_ = true;
    }
}

void FieldDPlanner::setGoalDistance(float goal_dist) {
    this->goal_dist_ = goal_dist;
}

bool FieldDPlanner::isVertex(const Position &p) {
    bool is_vertex = (ceilf(p.x) == p.x) && (ceilf(p.y) == p.y);
    bool satisfies_bounds = grid.isValidPosition(p);
    return is_vertex && satisfies_bounds;
}

Key FieldDPlanner::calculateKey(const Node &s) {
    // obtain g-values and rhs-values for node s
    float cost_so_far = std::min(getG(s), getRHS(s));
    // calculate the key to order the node in the priority_queue_ with. key_modifier_ is the
    // key modifier, a value which corrects for the distance traveled by the robot
    // since the search began (source: D* Lite)
    auto[x, y] = s.getIndex();
    auto dist = std::hypot(start_pos.x - x, start_pos.y - y);
    // TODO: reason on key_modifier scaling with heuristic_multiplier
    return {cost_so_far + heuristic_multiplier * dist + grid.key_modifier_,
            cost_so_far};
}

void FieldDPlanner::initializeSearch() {
    grid.key_modifier_ = 0.0f;
    expanded_map.clear();
    priority_queue.clear();
    grid.updated_cells_.clear();

    insert_or_assign(grid.start_, INFINITY, INFINITY);
    insert_or_assign(grid.goal_, INFINITY, 0.0f);
    priority_queue.insert(grid.goal_, calculateKey(grid.goal_));
}

int FieldDPlanner::computeShortestPath_1() {
    float cost1, cost2, g_sp, rhs_sp, rhs_s, g_s;
    Node cn, ccn;

    // if the start node is occupied, return immediately. No path exists
    if (grid.getTraversalCost(grid.start_.getIndex()) == INFINITY) {
        std::cerr << "Start node occupied. No path is possible." << std::endl; //FIXME test
        return 0;
    }

    int expanded = 0;
    while ((not priority_queue.empty()) and end_condition()) {
        Node s = priority_queue.topNode();
        expanded++;

        g_s = getG(s);
        rhs_s = getRHS(s);
        if (g_s > rhs_s) {
            insert_or_assign(s, rhs_s, rhs_s);
            priority_queue.pop();
            for (Node &sp : grid.neighbors(s)) {
                if (expanded_map.find(sp) == expanded_map.end()) {
                    insert_or_assign(sp, INFINITY, INFINITY);
                }
                ccn = grid.counterClockwiseNeighbor(sp, s);
                cn = grid.clockwiseNeighbor(sp, s);
                cost1 = ccn.valid ? computeOptimalCost(sp, s, ccn) : INFINITY;
                cost2 = cn.valid ? computeOptimalCost(sp, cn, s) : INFINITY;
                if (cost1 <= cost2) {
                    rhs_sp = getRHS(sp);
                    if (rhs_sp > cost1) {
                        sp.setBptr(s.getIndex());
                        g_sp = getG(sp);
                        insert_or_assign(sp, g_sp, cost1);
                        priority_queue.remove(sp);
                        if (g_sp != cost1) {
                            priority_queue.insert(sp, calculateKey(sp));
                        }
                    } else if (rhs_sp > cost2) {
                        sp.setBptr(cn.getIndex());
                        g_sp = getG(sp);
                        insert_or_assign(sp, g_sp, cost2);
                        priority_queue.remove(sp);
                        if (g_sp != cost2) {
                            priority_queue.insert(sp, calculateKey(sp));
                        }
                    }
                }
            }
        } else {
            insert_or_assign(s, INFINITY, rhs_s);
            for (Node &sp : grid.neighbors(s)) {
                auto sp_bptr = sp.getBptr();
                if (sp_bptr == s.getIndex() or sp_bptr == grid.clockwiseNeighbor(sp, s).getIndex()) {
                    float min_rhs = INFINITY;
                    for (const auto &spp : grid.neighbors(sp)) {
                        ccn = grid.counterClockwiseNeighbor(sp, spp);
                        if (ccn.valid) {
                            cost1 = computeOptimalCost(sp, spp, ccn);
                            min_rhs = std::min(min_rhs, cost1);
                            if (min_rhs == cost1) sp.setBptr(spp.getIndex());
                        }
                    }
                    g_sp = getG(sp);
                    insert_or_assign(sp, g_sp, min_rhs);
                    priority_queue.remove(sp);
                    if (g_sp != min_rhs) {
                        priority_queue.insert(sp, calculateKey(sp));
                    }
                }
            }
            priority_queue.remove(s);
            if (g_s != rhs_s) {
                priority_queue.insert(s, calculateKey(s));
            }
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    return num_nodes_expanded;
}

bool FieldDPlanner::end_condition() {
    // Given a position, the integer part of its indices is the cell index,
    // and the remainder is the internal offsets
    Cell start_cell(std::floor(start_pos.x), std::floor(start_pos.y));
    // We need to check expansion until all 4 corners of start cell
    // used early stof from D* LITE
    for (auto &node: grid.getNodesAroundCell(start_cell)) {
        if (not((priority_queue.topKey() >= calculateKey(node)) and (getRHS(node) <= getG(node)))) {
            return true;
        }
    }
    return false; //STOP: all 4 conditions met
}

int FieldDPlanner::computeShortestPath_0() {
    // if the start node is occupied, return immediately. No path exists
    if (grid.getTraversalCost(grid.start_.getIndex()) == INFINITY) {
        std::cerr << "Start node occupied. No path is possible." << std::endl; //FIXME test
        return 0;
    }
    int skipped = 0;
    int expanded = 0;
    while ((not priority_queue.empty()) and end_condition()) {
        // Pop head of queue and its key
        Node s = priority_queue.topNode();
        Key old_key = priority_queue.topKey();
        priority_queue.pop();
        ++expanded;

        // Handle update of key (see D* Lite paper with key modifier)
        Key new_key = calculateKey(s);
        if (old_key < new_key) {
            skipped += 1;
            priority_queue.insert(s, new_key);
            continue;
        }

        // Get reference to the node
        auto top_node_it = expanded_map.find(s);
        assert(top_node_it != expanded_map.end());

        float g = G(top_node_it);
        float rhs = RHS(top_node_it);
        if (g > rhs) { // Overconsistent
            G(top_node_it) = rhs;
            for (const Node &nbr : grid.neighbors(s))
                updateNode_0(nbr);
        } else { // Underconsistent
            G(top_node_it) = INFINITY;
            for (const Node &nbr : grid.neighbors(s))
                updateNode_0(nbr);
            updateNode_0(s);
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    std::cout << skipped << " nodes skipped" << std::endl;
    return num_nodes_expanded;
}

void FieldDPlanner::updateNode_0(const Node &s) {
    auto s_it = expanded_map.find(s);
    if (s_it == expanded_map.end()) { // Init node if not yet considered
        s_it = expanded_map.emplace(s, std::make_pair(INFINITY, INFINITY)).first;
    } else { // Otherwise, remove it from the priority queue, if present
        priority_queue.remove(s);
    }

    float g = G(s_it);
    float rhs = RHS(s_it);
    if (s != grid.goal_) {
        rhs = INFINITY;
        for (auto &[nbr1, nbr2] : grid.consecutiveNeighbors(s))
            rhs = std::min(rhs, computeOptimalCost(s, nbr1, nbr2));
        RHS(s_it) = rhs;
    }

    if (g != rhs) {
        priority_queue.insert(s, calculateKey(s));
    }
}

int FieldDPlanner::updateNodesAroundUpdatedCells() {
    std::unordered_set<Node> to_update;
    std::vector<Node> updates;
    // construct a set of all updated nodes
    for (const Cell &cell : grid.updated_cells_) {
        updates = grid.getNodesAroundCell(cell);
        to_update.insert(updates.begin(), updates.end());
    }

    for (Node s : to_update) {
        if (optimization_lvl == 0) {
            updateNode_0(s);
        } else {
            float cost = INFINITY, min_rhs = INFINITY;
            if (expanded_map.find(s) == expanded_map.end()) {
                insert_or_assign(s, INFINITY, INFINITY);
            }
            if (s == grid.goal_) continue;
            for (const auto &sp : grid.neighbors(s)) {
                auto ccn = grid.counterClockwiseNeighbor(s, sp);
                if (ccn.valid) {
                    cost = computeOptimalCost(s, sp, ccn);
                    min_rhs = std::min(min_rhs, cost);
                    if (min_rhs == cost) s.setBptr(sp.getIndex());
                }
            }
            auto g_sp = getG(s);
            insert_or_assign(s, g_sp, min_rhs);
            priority_queue.remove(s);
            if (g_sp != min_rhs) {
                priority_queue.insert(s, calculateKey(s));
            }
        }
    }

    num_nodes_updated = to_update.size();
    std::cout << num_nodes_updated << " nodes updated" << std::endl;
    return num_nodes_updated;
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
    int max_steps = static_cast<int>(20000.00f / (this->grid.resolution_));

    float step_cost, step_dist;
    path_.push_back(start_pos);
    Position last = path_.back();
    do {
        // move one step and calculate the optimal path additions
        pa = getPathAdditions(last, lookahead, step_cost);
        path_.reserve(path_.size() + pa.steps.size());
        cost_.reserve(cost_.size() + pa.stepcosts.size());
        auto it = path_.insert(path_.end(), pa.steps.begin(), pa.steps.end());
        cost_.insert(cost_.end(), pa.stepcosts.begin(), pa.stepcosts.end());
        min_cost = pa.cost_to_goal;
        step_dist = 0;
        for (auto p = it - 1; p < (path_.end() - 1); ++p) {
            step_dist += std::hypot(p->x - (p + 1)->x, p->y - (p + 1)->y);
        }
        total_cost += step_cost;
        total_dist += step_dist;
        curr_step += 1;
        last = path_.back();
    } while (!isWithinRangeOfGoal(last) && (min_cost != INFINITY) &&
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

std::tuple<float, float> cell_idx_bottom_left(const Position &p) { return {p.x, p.y - 1}; }
std::tuple<float, float> cell_idx_bottom_right(const Position &p) { return {p.x, p.y}; }
std::tuple<float, float> cell_idx_top_left(const Position &p) { return {p.x - 1, p.y - 1}; }
std::tuple<float, float> cell_idx_top_right(const Position &p) { return {p.x - 1, p.y}; }

std::tuple<float, float> cell_idx_4n(const Position &p, bool bottom_TOP, bool left_RIGHT) {
    if (bottom_TOP)
        return left_RIGHT ? cell_idx_top_right(p) : cell_idx_top_left(p);
    else
        return left_RIGHT ? cell_idx_bottom_right(p) : cell_idx_bottom_left(p);
}

// p must be aligned with p_1, p_1 aligned with p_2, p and P_2 diagonal neighbors
std::pair<float, float> FieldDPlanner::getBC(TraversalParams &t) {
    std::tuple<int, int> cell_ind_b, cell_ind_c;

    if (t.p0.x == t.p1.x) {
        cell_ind_b = cell_idx_4n(t.p1, t.p2.x > t.p1.x, t.p0.y > t.p1.y);
        cell_ind_c = cell_idx_4n(t.p1, t.p2.x < t.p1.x, t.p0.y > t.p1.y);
    } else {
        cell_ind_b = cell_idx_4n(t.p1, t.p0.x < t.p1.x, t.p2.y < t.p1.y);
        cell_ind_c = cell_idx_4n(t.p1, t.p0.x < t.p1.x, t.p2.y > t.p1.y);
    }

    t.b = grid.getTraversalCost(cell_ind_b);
    t.c = grid.getTraversalCost(cell_ind_c);
    if (t.b >= 255 * occupancy_threshold_) t.b = INFINITY;
    if (t.c >= 255 * occupancy_threshold_) t.c = INFINITY;
    return {t.b, t.c};
}

float FieldDPlanner::computeOptimalCost(const Position &p,
                                        const Position &p_a,
                                        const Position &p_b) {
    std::vector<Position> positions;
    float min_cost;

    assert(isVertex(p));
    assert(isVertex(p_a));
    assert(isVertex(p_b));

    TraversalParams cell;
    cell.p0 = p;
    bool cond = grid.isDiagonalContinuous(p, p_a);
    cell.p1 = cond ? p_b : p_a;
    cell.p2 = cond ? p_a : p_b;

    assert((cell.p0.x == cell.p1.x) || (cell.p0.y == cell.p1.y));
    assert((cell.p0.x != cell.p2.x) && (cell.p0.y != cell.p2.y));

    if (first_run_trick and initialize_search) {
        cell.g1 = getRHS(cell.p1.castToNode());
        cell.g2 = getRHS(cell.p2.castToNode());
    } else {
        cell.g1 = getG(cell.p1.castToNode());
        cell.g2 = getG(cell.p2.castToNode());
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

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromCorner(const Position &p,
                                                                                   const Position &p_a,
                                                                                   const Position &p_b,
                                                                                   float &step_cost) {
    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;

    assert(isVertex(p));
    assert(isVertex(p_a));
    assert(isVertex(p_b));

    TraversalParams cell;
    cell.p0 = p;
    bool cond = grid.isDiagonalContinuous(p, p_a);
    cell.p1 = cond ? p_b : p_a;
    cell.p2 = cond ? p_a : p_b;

    assert((cell.p0.x == cell.p1.x) || (cell.p0.y == cell.p1.y));
    assert((cell.p0.x != cell.p2.x) && (cell.p0.y != cell.p2.y));

    if (first_run_trick and initialize_search) {
        cell.g1 = getRHS(cell.p1.castToNode());
        cell.g2 = getRHS(cell.p2.castToNode());
    } else {
        cell.g1 = getG(cell.p1.castToNode());
        cell.g2 = getG(cell.p2.castToNode());
    }
    if (cell.g1 == INFINITY && cell.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    getBC(cell);
    if (cell.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell.f = cell.g1 - cell.g2;

    int type;
    enum { TYPE_I, TYPE_II, TYPE_III, TYPE_A, TYPE_B };

    if (cell.c > cell.b) {
        if ((cell.f <= 0) or (SQUARE(cell.f) <= CATH(cell.c, cell.b))) {
            min_cost = TraversalTypeIII::Corner::cost(cell);
            type = TYPE_III;
        } else if ((cell.f <= cell.b) and (cell.c > (cell.f * SQRT2))) {
            min_cost = TraversalTypeII::Corner::cost(cell);
            type = TYPE_II;
        } else if ((cell.f > cell.b) and (cell.c > (cell.b * SQRT2))) {
            min_cost = TraversalTypeI::Corner::cost(cell);
            type = TYPE_I;
        } else {
            min_cost = TraversalTypeA::Corner::cost(cell);
            type = TYPE_A;
        }
    } else {
        if (cell.f <= 0) {
            min_cost = TraversalTypeB::Corner::cost(cell);
            type = TYPE_B;
        } else if ((cell.f * SQRT2) < cell.c) {
            min_cost = TraversalTypeII::Corner::cost(cell);
            type = TYPE_II;
        } else {
            min_cost = TraversalTypeA::Corner::cost(cell);
            type = TYPE_A;
        }
    }

    if (type == TYPE_I) {
        positions = TraversalTypeI::Corner::additions(cell);
        step_costs = TraversalTypeI::Corner::stepcosts(cell);
    } else if (type == TYPE_II) {
        positions = TraversalTypeII::Corner::additions(cell);
        step_costs = TraversalTypeII::Corner::stepcosts(cell);
    } else if (type == TYPE_III) {
        positions = TraversalTypeIII::Corner::additions(cell);
        step_costs = TraversalTypeIII::Corner::stepcosts(cell);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::Corner::additions(cell);
        step_costs = TraversalTypeA::Corner::stepcosts(cell);
    } else if (type == TYPE_B) {
        positions = TraversalTypeB::Corner::additions(cell);
        step_costs = TraversalTypeB::Corner::stepcosts(cell);
    }

    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromContiguousEdge(const Position &p,
                                                                                           const Position &p_a,
                                                                                           const Position &p_b,
                                                                                           float &step_cost) {
    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    TraversalParams cell1;
    bool cond = (p.x == p_a.x || p.y == p_a.y);
    cell1.p0 = p;
    cell1.p1 = cond ? p_a : p_b; // lies on the same edge of p
    cell1.p2 = cond ? p_b : p_a;

    if (first_run_trick and initialize_search) {
        cell1.g1 = getRHS(cell1.p1.castToNode());
        cell1.g2 = getRHS(cell1.p2.castToNode());
    } else {
        cell1.g1 = getG(cell1.p1.castToNode());
        cell1.g2 = getG(cell1.p2.castToNode());
    }
    if (cell1.g1 == INFINITY && cell1.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    getBC(cell1);
    if (cell1.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell1.f = cell1.g1 - cell1.g2;
    cell1.q = 1 - std::abs(cell1.p1.y - p.y) - std::abs(cell1.p1.x - p.x);
    assert(cell1.q > 0 and cell1.q < 1);
    assert(cell1.g1 >= 0 and cell1.g2 >= 0); //Goal has g=0
    assert(cell1.b > 0 and cell1.c > 0);

    enum { TYPE_I, TYPE_II, TYPE_III, TYPE_A, TYPE_B };
    std::array<float, 5> costs{};
    costs.fill(INFINITY);

    costs[TYPE_I] = TraversalTypeI::ContiguousEdge::condcost(cell1);
    costs[TYPE_II] = TraversalTypeII::ContiguousEdge::condcost(cell1);
    costs[TYPE_III] = TraversalTypeIII::ContiguousEdge::condcost(cell1);
    costs[TYPE_A] = TraversalTypeA::ContiguousEdge::cost(cell1);
    costs[TYPE_B] = TraversalTypeB::ContiguousEdge::cost(cell1);

    auto min = std::min_element(costs.begin(), costs.end());
    int type = std::distance(costs.begin(), min);
    min_cost = *min;

    if (type == TYPE_I) {
        positions = TraversalTypeI::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeI::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_II) {
        positions = TraversalTypeII::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeII::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_III) {
        positions = TraversalTypeIII::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeIII::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeA::ContiguousEdge::stepcosts(cell1);
    } else if (type == TYPE_B) {
        positions = TraversalTypeB::ContiguousEdge::additions(cell1);
        step_costs = TraversalTypeB::ContiguousEdge::stepcosts(cell1);
    }
    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromOppositeEdge(const Position &p,
                                                                                         const Position &p_a,
                                                                                         const Position &p_b,
                                                                                         float &step_cost) {

    std::vector<float> step_costs;
    std::vector<Position> positions;
    float min_cost;
    TraversalParams cell1, cell2;
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

    if (first_run_trick and initialize_search) {
        cell1.g1 = cell2.g2 = getRHS(p_a.castToNode());
        cell1.g2 = cell2.g1 = getRHS(p_b.castToNode());
    } else {
        cell1.g1 = cell2.g2 = getG(p_a.castToNode());
        cell1.g2 = cell2.g1 = getG(p_b.castToNode());
    }

    if (cell1.g1 == INFINITY && cell2.g2 == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    getBC(cell1);
    getBC(cell2);
    if (cell1.c == INFINITY)
        return {{/*EMPTY*/}, {/*EMPTY*/}, INFINITY};
    cell1.f = cell1.g1 - cell1.g2;
    cell2.f = -cell1.f;
    cell1.p = std::abs(p.y - cell1.p0.y) + std::abs(p.x - cell1.p0.x);
    cell2.p = 1 - cell1.p;
    assert(cell1.c == cell2.c);
    assert(cell1.p > 0 and cell1.p < 1);
    assert(cell1.g1 >= 0 and cell1.g2 >= 0); // goal has g=0
    assert(cell1.b > 0 and cell2.b > 0 and cell1.c > 0);

    // Use _f1, _p1, _b1, _g_s2;
    // Use neg(_f), 1-_p1, _b2, _g_s1;
    // NOTE: neg(_f1)=_f2, (1-_p1)=_p2

    enum { TYPE_I__1, TYPE_I__2, TYPE_II__1, TYPE_II__2, TYPE_III__1, TYPE_III__2, TYPE_A__1, TYPE_A__2 };
    std::array<float, 8> costs{};
    costs.fill(INFINITY);
    costs[TYPE_I__1] = TraversalTypeI::OppositeEdge::condcost(cell1);
    costs[TYPE_I__2] = TraversalTypeI::OppositeEdge::condcost(cell2);
    costs[TYPE_II__1] = TraversalTypeII::OppositeEdge::condcost(cell1);
    costs[TYPE_II__2] = TraversalTypeII::OppositeEdge::condcost(cell2);
    costs[TYPE_III__1] = TraversalTypeIII::OppositeEdge::condcost(cell1);
    costs[TYPE_III__2] = TraversalTypeIII::OppositeEdge::condcost(cell2);
    costs[TYPE_A__1] = TraversalTypeA::OppositeEdge::cost(cell1);
    costs[TYPE_A__2] = TraversalTypeA::OppositeEdge::cost(cell2);

    auto min = std::min_element(costs.begin(), costs.end());
    int type = std::distance(costs.begin(), min);
    min_cost = *min;

    if (type == TYPE_I__1) {
        positions = TraversalTypeI::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeI::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_I__2) {
        positions = TraversalTypeI::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeI::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_II__1) {
        positions = TraversalTypeII::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeII::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_II__2) {
        positions = TraversalTypeII::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeII::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_III__1) {
        positions = TraversalTypeIII::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeIII::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_III__2) {
        positions = TraversalTypeIII::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeIII::OppositeEdge::stepcosts(cell2);
    } else if (type == TYPE_A__1) {
        positions = TraversalTypeA::OppositeEdge::additions(cell1);
        step_costs = TraversalTypeA::OppositeEdge::stepcosts(cell1);
    } else if (type == TYPE_A__2) {
        positions = TraversalTypeA::OppositeEdge::additions(cell2);
        step_costs = TraversalTypeA::OppositeEdge::stepcosts(cell2);
    }
    step_cost = std::accumulate(step_costs.begin(), step_costs.end(), .0f);
    return {positions, step_costs, min_cost};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromEdge(const Position &p,
                                                                                 const Position &p_a,
                                                                                 const Position &p_b,
                                                                                 float &step_cost) {

    assert(!isVertex(p));
    assert(isVertex(p_a));
    assert(isVertex(p_b));

    bool cond_1 = (p.x == p_a.x || p.y == p_a.y);
    bool cond_2 = (p.x == p_b.x || p.y == p_b.y);
    if (cond_1 || cond_2) {
        assert(cond_1 xor cond_2);
        return computeOptimalCellTraversalFromContiguousEdge(p, p_a, p_b, step_cost);
    } else {
        return computeOptimalCellTraversalFromOppositeEdge(p, p_a, p_b, step_cost);
    }
}

FieldDPlanner::path_additions FieldDPlanner::getPathAdditions(const Position &p,
                                                              const bool &do_lookahead,
                                                              float &step_cost) {
    float min_cost = INFINITY;
    path_additions min_pa;
    path_additions temp_pa;
    float lookahead_cost;
    #ifdef VERBOSE_EXTRACTION
    if (lookahead and not do_lookahead) std::cout << "\t";
    std::cout << "p     " << std::to_string(p.x) << ", " << std::to_string(p.y)
              << (isVertex(p) ? " (Corner)" : " (Edge)") << std::endl << std::endl;
    #endif

    for (const auto &[p_a, p_b] : grid.consecutiveNeighbors(p)) {
        float cur_step_cost = INFINITY;
        /* POSSIBLE WAY TO AVOID INCONSISTENT NODES IN EXTRACTION?*/
        // Initial observations show that it does not always work
        /*
        if (!((consistent(p_a.castToNode()) and (consistent(p_b.castToNode()) or getG(p_b.castToNode()) == INFINITY))
            or (consistent(p_b.castToNode())
                and (consistent(p_a.castToNode()) or getG(p_a.castToNode()) == INFINITY))))
            continue;
        */

        if (isVertex(p))
            temp_pa = computeOptimalCellTraversalFromCorner(p, p_a, p_b, cur_step_cost);
        else
            temp_pa = computeOptimalCellTraversalFromEdge(p, p_a, p_b, cur_step_cost);

        #ifdef VERBOSE_EXTRACTION
        if (lookahead and not do_lookahead) std::cout << "\t";
        std::cout << "X:" << p_a.x << ", Y:" << p_a.y
                  << ", G:" << getG(p_a.castToNode()) << ", RHS:" << getRHS(p_a.castToNode()) << " | "
                  << "X:" << p_b.x << ", Y:" << p_b.y
                  << ", G:" << getG(p_b.castToNode()) << ", RHS:" << getRHS(p_b.castToNode())
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

bool FieldDPlanner::isWithinRangeOfGoal(const Position &p) {
    auto[x, y] = grid.goal_.getIndex();
    return x == p.x && y == p.y;
}

std::unordered_map<const Node, Key>::iterator
FieldDPlanner::insert_or_assign(const Node &s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    auto it = expanded_map.find(s);
    if (it != expanded_map.end()) {
        std::get<0>(it->second) = g;
        std::get<1>(it->second) = rhs;
        return it;
    } else {
        auto[it, ok] = expanded_map.emplace(s, std::make_pair(g, rhs));
        return it;
    }
}

float FieldDPlanner::getG(const Node &s) {
    // return g value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (expanded_map.find(s) != expanded_map.end())
        return std::get<0>(expanded_map.at(s));
    else
        return INFINITY;
}

float FieldDPlanner::getRHS(const Node &s) {
    // return rhs value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (expanded_map.find(s) != expanded_map.end())
        return std::get<1>(expanded_map.at(s));
    else
        return INFINITY;
}

bool FieldDPlanner::consistent(const Node &s) {
    return getG(s) == getRHS(s);
}