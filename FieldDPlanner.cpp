#include "FieldDPlanner.h"
#include <cmath>
#include <array>

FieldDPlanner::FieldDPlanner() = default;

void FieldDPlanner::init() {
    node_grid_.setConfigurationSpace(static_cast<float>(configuration_space_));
    node_grid_.setOccupancyThreshold(static_cast<float>(occupancy_threshold_));
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

    if (initialize_search) {
        std::cout << "Initializing Search..." << std::endl;
        initializeSearch();
    } else if (goal_changed_) {
        std::cout << "New Goal Received. Initializing Search..." << std::endl;
        goal_changed_ = false;
        initializeSearch();
    }

    // gather cells with updated edge costs and update affected nodes
    updateNodesAroundUpdatedCells();

    // only update the graph if nodes have been updated
    if ((num_nodes_updated > 0) or initialize_search) {
        //TODO if(initialize_search), use Update-reducing rhs' tweak to speed up first planning run
        computeShortestPath();
        initialize_search = false;
    } else {
        num_nodes_expanded = 0;
    }

    if (expanded_cb) {
        publish_expanded_set();
    }

    if (poses_cb) {
        constructOptimalPath();
        publish_path();
    }

    return LOOP_OK;
}

void FieldDPlanner::publish_expanded_set() {

    float max_g = -INFINITY;

    for (std::pair<Node, std::tuple<float, float>> e : expanded_map_) {
        // ignore infinite g-values when getting the max value
        if (std::get<0>(e.second) == INFINITY)
            continue;
        max_g = std::max(max_g, std::get<0>(e.second));
    }

    std::vector<std::tuple<int, int, float>> expanded;
    for (auto e : expanded_map_) {
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

    if (num_poses > 1 || !follow_old_path_) {
        for (size_t i = 0; i < num_poses; i++) {
            Pose pose;

            // calculate the position in the "/odom" frame
            pose.x = (path_[i].x - x_initial_) * node_grid_.resolution_;
            pose.y = (path_[i].y - y_initial_) * node_grid_.resolution_;

            // calculate the orientation
            double delta_x = ((path_[i + 1].x - x_initial_) * node_grid_.resolution_) - pose.x;
            double delta_y = ((path_[i + 1].y - y_initial_) * node_grid_.resolution_) - pose.y;
            double heading = atan2(delta_y, delta_x);
            pose.orientation = heading;

            poses.push_back(pose);
        }

        poses.back().orientation = poses[num_poses - 2].orientation;
        poses_cb(poses);
    }
}

void FieldDPlanner::set_map(const MapPtr &msg) {
    map_ = msg;  // update current map

    if (initialize_graph_) {
        x_initial_ = map_->x_initial;
        y_initial_ = map_->y_initial;
        node_grid_.initializeGraph(map_);
        initialize_graph_ = false;
    } else {
        node_grid_.updateGraph(map_);
    }
}

void FieldDPlanner::set_goal(std::pair<float, float> point) {
    int goal_x, goal_y;
    goal_x = static_cast<int>(std::round(point.first / node_grid_.resolution_)) + x_initial_;
    goal_y = static_cast<int>(std::round(point.second / node_grid_.resolution_)) + y_initial_;

    Node new_goal(goal_x, goal_y);

    if (node_grid_.goal_ != new_goal)
        goal_changed_ = true;

    node_grid_.setGoal(new_goal);

    float distance_to_goal = node_grid_.euclidianHeuristic(new_goal) * node_grid_.resolution_;

    std::cout << (goal_changed_ ? "New" : "Same") << " waypoint received. Search Problem Goal = " << node_grid_.goal_
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
    bool satisfies_bounds = node_grid_.isValidPosition(p);
    return is_vertex && satisfies_bounds;
}

Key FieldDPlanner::calculateKey(const Node &s) {
    // obtain g-values and rhs-values for node s
    float cost_so_far = std::min(getG(s), getRHS(s));
    // calculate the key to order the node in the priority_queue_ with. key_modifier_ is the
    // key modifier, a value which corrects for the distance traveled by the robot
    // since the search began (source: D* Lite)
    return Key(std::round(cost_so_far + node_grid_.euclidianHeuristic(s.getIndex()) + node_grid_.key_modifier_),
               std::round(cost_so_far));
}

void FieldDPlanner::initializeSearch() {
    node_grid_.key_modifier_ = 0.0f;
    expanded_map_.clear();
    priority_queue_.clear();
    node_grid_.updated_cells_.clear();

    insert_or_assign(node_grid_.start_, INFINITY, INFINITY);
    insert_or_assign(node_grid_.goal_, INFINITY, 0.0f);
    priority_queue_.insert(node_grid_.goal_, calculateKey(node_grid_.goal_));
}

void FieldDPlanner::updateNode(const Node &s) {
    // s never visited before, add to unordered map with g(s) = rhs(s) = inf
    if (expanded_map_.find(s) == expanded_map_.end()) {
        insert_or_assign(s, INFINITY, INFINITY);
    } else {
        /**
        looks for a node in the priority queue and removes it if found
        same as calling: if priority_queue_.contains(s) priority_queue_.remove(s);
        */
        priority_queue_.remove(s);
    }

    // update rhs value of Node s
    if (s != node_grid_.goal_) {
        float min_rhs = INFINITY;
        for (auto connbr : node_grid_.consecutiveNeighbors(s))
            //min_rhs = std::min(min_rhs, this->computeCost(s, std::get<0>(connbr), std::get<1>(connbr)).cost);
            min_rhs = std::min(min_rhs,
                               computeOptimalCellTraversalFromCorner(s, std::get<0>(connbr), std::get<1>(connbr))
                                   .second);;

        insert_or_assign(s, getG(s), min_rhs);
    }

    // insert node into priority queue if it is locally inconsistent
    if (getG(s) != getRHS(s)) {
        priority_queue_.insert(s, calculateKey(s));
    }
}

int FieldDPlanner::computeShortestPath() {
    // if the start node is occupied, return immediately. No path exists
    if (node_grid_.getMinTraversalCost(node_grid_.start_) == INFINITY) {
        std::cerr << "Start node occupied. No path is possible." << std::endl; //FIXME test
        return 0;
    }

    int expanded = 0;
    while (((priority_queue_.topKey() < calculateKey(node_grid_.start_)) ||
        (std::fabs(getRHS(node_grid_.start_) - getG(node_grid_.start_)) > 1e-5)) &&
        (!priority_queue_.empty())) {
        Node top_node = priority_queue_.topNode();
        priority_queue_.pop();
        expanded++;

        if (getG(top_node) > getRHS(top_node)) {
            // locally overconsistent case. This node is now more favorable.
            // make node locally consistent by setting g = rhs and propagate
            // changes to neighboring nodes
            insert_or_assign(top_node, getRHS(top_node), getRHS(top_node));
            for (Node nbr : node_grid_.nbrs(top_node))
                updateNode(nbr);
        } else {
            // locally underconsistent case. This node is now less favorable.
            // make node locally consistent or overconsistent by setting g = inf
            // and propagate changes to {neighbors} U {top_node}
            insert_or_assign(top_node, INFINITY, getRHS(top_node));
            for (Node nbr : node_grid_.nbrs(top_node))
                updateNode(nbr);
            updateNode(top_node);
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    return num_nodes_expanded;
}

int FieldDPlanner::updateNodesAroundUpdatedCells() {
    std::unordered_set<Node> to_update;
    std::vector<Node> updates;
    // construct a set of all updated nodes
    for (Cell cell : node_grid_.updated_cells_) {
        updates = node_grid_.getNodesAroundCellWithConfigurationSpace(cell);
        to_update.insert(updates.begin(), updates.end());
    }

    for (Node n : to_update) {
        if (expanded_map_.find(n) != expanded_map_.end())  // Only update explored nodes
            updateNode(n);
    }

    num_nodes_updated = to_update.size();
    std::cout << num_nodes_updated << " nodes updated" << std::endl;
    return num_nodes_updated;
}

void FieldDPlanner::constructOptimalPath() {
    path_.clear();

    Position curr_pos(node_grid_.start_);
    path_.push_back(curr_pos);

    float min_cost;
    path_additions pa;

    int curr_step = 0;
    // TODO do something better than this sh*t
    int max_steps = static_cast<int>(20000.00f / (this->node_grid_.resolution_));

    do {
        // move one step and calculate the optimal path additions
        pa = getPathAdditions(curr_pos, lookahead_dist_);
        path_.insert(path_.end(), pa.first.begin(), pa.first.end());
        min_cost = pa.second;
        curr_pos = path_.back();
        curr_step += 1;
    } while (!isWithinRangeOfGoal(curr_pos) && (min_cost != INFINITY) &&
        (curr_step < max_steps));

    if (min_cost == INFINITY) {
        std::cerr << "[Extraction] No valid path exists" << std::endl;
        path_.clear();
    } else if (curr_step >= max_steps) {
        std::cerr << "[Extraction] Maximum step number reached" << std::endl;
        path_.clear();
    }
}

std::tuple<float, float> cell_idx_bottom_left(const Position &p) { return {p.x - 1, p.y - 1}; }
std::tuple<float, float> cell_idx_bottom_right(const Position &p) { return {p.x, p.y - 1}; }
std::tuple<float, float> cell_idx_top_left(const Position &p) { return {p.x - 1, p.y}; }
std::tuple<float, float> cell_idx_top_right(const Position &p) { return {p.x, p.y}; }

std::tuple<float, float> cell_idx_4n(const Position &p, bool bottom_TOP, bool left_RIGHT) {
    if (bottom_TOP)
        return left_RIGHT ? cell_idx_top_right(p) : cell_idx_top_left(p);
    else
        return left_RIGHT ? cell_idx_bottom_right(p) : cell_idx_bottom_left(p);
}

// p must be aligned with p_1, p_1 aligned with p_2, p and P_2 diagonal neighbors
std::pair<float, float> FieldDPlanner::getBC(TraversalParams &t) {
    std::tuple<int, int> cell_ind_b, cell_ind_c;
    /* Code for clear understanding of synteshis. Please KEEP (at least for some time)
    if (p.x == p_1.x) { // p lies on a vertical edge
        if (p_2.x > p_1.x) { // we need to traverse the cell on the right
            if (p.y > p_1.y) { // we are over the considered edge
                // "b" is the cost of the cell on the top-left of p_1
                cell_ind_b = cell_idx_top_left(p_1);
                cell_ind_c = cell_idx_top_right(p_1);
            } else { // we are under the considered edge
                // "b" is the cost of the cell on the bottom-left of p_1
                cell_ind_b = cell_idx_bottom_left(p_1);
                cell_ind_c = cell_idx_bottom_right(p_1);
            }
        } else { // we need to traverse the cell on the left
            if (p.y > p_1.y) { // we are over the considered edge
                // "b" is the cost of the cell on the top-right of p_1
                cell_ind_b = cell_idx_top_right(p_1);
                cell_ind_c = cell_idx_top_left(p_1);
            } else { // we are under the considered edge
                // "b" is the cost of the cell on the bottom-right of p_1
                cell_ind_b = cell_idx_bottom_right(p_1);
                cell_ind_c = cell_idx_bottom_left(p_1);
            }
        }

    } else { // p lies on a horizontal edge
        if (p_2.y > p_1.y) { // we need to traverse the cell at the top
            if (p.x > p_1.x) { // we are on the right of the considered edge
                // "b" is the cost of the cell on the bottom-right of p_1
                cell_ind_b = cell_idx_bottom_right(p_1);
                cell_ind_c = cell_idx_top_right(p_1);
            } else { // we are on the left of the considered edge
                // "b" is the cost of the cell on the bottom-left of p_1
                cell_ind_b = cell_idx_bottom_left(p_1);
                cell_ind_c = cell_idx_top_left(p_1);
            }
        } else { // we need to traverse the cell at the bottom
            if (p.x > p_1.x) { // we are on the right of the considered edge
                // "b" is the cost of the cell on the top-right of p_1
                cell_ind_b = cell_idx_top_right(p_1);
                cell_ind_c = cell_idx_bottom_right(p_1);
            } else { // we are on the left of the considered edge
                // "b" is the cost of the cell on the top-left of p_1
                cell_ind_b = cell_idx_top_left(p_1);
                cell_ind_c = cell_idx_bottom_left(p_1);
            }
        }
    }
    */
    if (t.p0.x == t.p1.x) { // p lies on a vertical edge
        cell_ind_b = cell_idx_4n(t.p1, t.p0.y > t.p1.y, t.p2.x < t.p1.x);
        cell_ind_c = cell_idx_4n(t.p1, t.p0.y > t.p1.y, t.p2.x > t.p1.x);
    } else { // p lies on a horizontal edge
        cell_ind_b = cell_idx_4n(t.p1, t.p2.y < t.p1.y, t.p0.x < t.p1.x);
        cell_ind_c = cell_idx_4n(t.p1, t.p2.y > t.p1.y, t.p0.x < t.p1.x);
    }

    t.b = node_grid_.getValWithConfigurationSpace(cell_ind_b);
    t.c = node_grid_.getValWithConfigurationSpace(cell_ind_c);
    return {t.b, t.c};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromCorner(const Position &p,
                                                                                   const Position &p_a,
                                                                                   const Position &p_b) {
    std::vector<Position> positions;
    float min_cost = INFINITY;

    assert(isVertex(p));
    assert(isVertex(p_a));
    assert(isVertex(p_b));

    TraversalParams cell;
    cell.p0 = p;
    bool cond = node_grid_.isDiagonalContinuous(p, p_a);
    cell.p1 = cond ? p_b : p_a;
    cell.p2 = cond ? p_a : p_b;

    assert((cell.p0.x == cell.p1.x) || (cell.p0.y == cell.p1.y));
    assert((cell.p0.x != cell.p2.x) && (cell.p0.y != cell.p2.y));

    cell.g1 = getG(cell.p1.castToNode());
    cell.g2 = getG(cell.p2.castToNode());
    if (cell.g1 == INFINITY && cell.g2 == INFINITY)
        return {{/*EMPTY*/}, INFINITY};
    getBC(cell);
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
    } else if (type == TYPE_II) {
        positions = TraversalTypeII::Corner::additions(cell);
    } else if (type == TYPE_III) {
        positions = TraversalTypeIII::Corner::additions(cell);
    } else if (type == TYPE_A) {
        positions = TraversalTypeA::Corner::additions(cell);
    } else if (type == TYPE_B) {
        positions = TraversalTypeB::Corner::additions(cell);
    }

    return {positions, min_cost};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromContiguousEdge(const Position &p,
                                                                                           const Position &p_a,
                                                                                           const Position &p_b) {
    std::vector<Position> positions;
    float min_cost;
    TraversalParams cell1;
    bool cond = (p.x == p_a.x || p.y == p_a.y);
    cell1.p0 = p;
    cell1.p1 = cond ? p_a : p_b; // lies on the same edge of p
    cell1.p2 = cond ? p_b : p_a;

    cell1.g1 = getG(cell1.p1.castToNode());
    cell1.g2 = getG(cell1.p2.castToNode());
    if (cell1.g1 == INFINITY && cell1.g2 == INFINITY)
        return {{/*EMPTY*/}, INFINITY};
    getBC(cell1);
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

    if (type == TYPE_I)
        positions = TraversalTypeI::ContiguousEdge::additions(cell1);
    else if (type == TYPE_II)
        positions = TraversalTypeII::ContiguousEdge::additions(cell1);
    else if (type == TYPE_III)
        positions = TraversalTypeIII::ContiguousEdge::additions(cell1);
    else if (type == TYPE_A)
        positions = TraversalTypeA::ContiguousEdge::additions(cell1);
    else if (type == TYPE_B)
        positions = TraversalTypeB::ContiguousEdge::additions(cell1);

    return {positions, min_cost};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromOppositeEdge(const Position &p,
                                                                                         const Position &p_a,
                                                                                         const Position &p_b) {

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

    cell1.g1 = cell2.g2 = getG(p_a.castToNode());
    cell1.g2 = cell2.g1 = getG(p_b.castToNode());
    if (cell1.g1 == INFINITY && cell2.g2 == INFINITY)
        return {{/*EMPTY*/}, INFINITY};
    getBC(cell1);
    getBC(cell2);
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

    if (type == TYPE_I__1)
        positions = TraversalTypeI::OppositeEdge::additions(cell1);
    else if (type == TYPE_I__2)
        positions = TraversalTypeI::OppositeEdge::additions(cell2);
    else if (type == TYPE_II__1)
        positions = TraversalTypeII::OppositeEdge::additions(cell1);
    else if (type == TYPE_II__2)
        positions = TraversalTypeII::OppositeEdge::additions(cell2);
    else if (type == TYPE_III__1)
        positions = TraversalTypeIII::OppositeEdge::additions(cell1);
    else if (type == TYPE_III__2)
        positions = TraversalTypeIII::OppositeEdge::additions(cell2);
    else if (type == TYPE_A__1)
        positions = TraversalTypeA::OppositeEdge::additions(cell1);
    else if (type == TYPE_A__2)
        positions = TraversalTypeA::OppositeEdge::additions(cell2);

    return {positions, min_cost};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromEdge(const Position &p,
                                                                                 const Position &p_a,
                                                                                 const Position &p_b) {

    assert(!isVertex(p));
    assert(isVertex(p_a));
    assert(isVertex(p_b));

    bool cond_1 = (p.x == p_a.x || p.y == p_a.y);
    bool cond_2 = (p.x == p_b.x || p.y == p_b.y);
    if (cond_1 || cond_2) {
        assert(cond_1 xor cond_2);
        return computeOptimalCellTraversalFromContiguousEdge(p, p_a, p_b);
    } else {
        return computeOptimalCellTraversalFromOppositeEdge(p, p_a, p_b);
    }
}

FieldDPlanner::path_additions FieldDPlanner::getPathAdditions(const Position &p, int lookahead_dist_remaining) {
    float min_cost = INFINITY;
    path_additions min_pa;
    path_additions temp_pa;
    float lookahead_cost;
    std::cout << "\np     " << std::to_string(p.x) << ", " << std::to_string(p.y)
              << (isVertex(p) ? " (Corner)" : " (Edge)") << std::endl << std::endl;
    //TODO use priority queue instead of 1-step-lookaed for ALL edges (Otte et al.)
    for (const auto &[p_a, p_b] : node_grid_.nbrsContinuous(p)) {
        // look ahead `lookahead_dist_remaining` planning steps into the future for best action
        if (isVertex(p)) {
            temp_pa = computeOptimalCellTraversalFromCorner(p, p_a, p_b);
        } else {
            temp_pa = computeOptimalCellTraversalFromEdge(p, p_a, p_b);
        }

        std::cout << "p_a   " << p_a.x << ", " << p_a.y << ", g=" << getG(p_a.castToNode()) << std::endl;
        std::cout << "p_b   " << p_b.x << ", " << p_b.y << ", g=" << getG(p_b.castToNode()) << std::endl;
        std::cout << "cost: " << temp_pa.second << std::endl;
        for (auto addition: temp_pa.first) {
            std::cout << "step  " << std::to_string(addition.x) << ", " << std::to_string(addition.y) << std::endl;
        }

        if ((lookahead_dist_remaining <= 0) || temp_pa.first.empty())
            lookahead_cost = temp_pa.second;
        else
            lookahead_cost = getPathAdditions(temp_pa.first.back(), lookahead_dist_remaining - 1).second;

        if (lookahead_cost < min_cost) {
            min_cost = lookahead_cost;
            min_pa = temp_pa;
        }
    }

    std::cout << "\nfinal choice: " << std::endl;
    std::cout << "cost: " << std::to_string(min_pa.second) << std::endl;
    std::cout << "p     " << std::to_string(p.x) << ", " << std::to_string(p.y) << std::endl;
    for (auto addition: min_pa.first) {
        std::cout << "step  " << std::to_string(addition.x) << ", " << std::to_string(addition.y) << std::endl;
    }

    return min_pa;
}

bool FieldDPlanner::isWithinRangeOfGoal(const Position &p) {
    int x, y;
    std::tie(x, y) = node_grid_.goal_.getIndex();
    float distance_to_goal = std::hypot(x - p.x, y - p.y);
    float goal_radius = goal_dist_ / node_grid_.resolution_;
    return distance_to_goal <= goal_radius;
}

void FieldDPlanner::insert_or_assign(Node s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    if (expanded_map_.find(s) != expanded_map_.end())
        expanded_map_.erase(s);

    expanded_map_.insert(std::make_pair(s, std::make_tuple(g, rhs)));
}

float FieldDPlanner::getG(const Node &s) {
    // return g value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (expanded_map_.find(s) != expanded_map_.end())
        return std::get<0>(expanded_map_.at(s));
    else
        return INFINITY;
}

float FieldDPlanner::getRHS(const Node &s) {
    // return rhs value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (expanded_map_.find(s) != expanded_map_.end())
        return std::get<1>(expanded_map_.at(s));
    else
        return INFINITY;
}
