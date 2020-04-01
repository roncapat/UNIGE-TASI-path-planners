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
    num_nodes_updated = updateNodesAroundUpdatedCells();
    if (num_nodes_updated > 0) std::cout << num_nodes_updated << " nodes updated" << std::endl;

    // only update the graph if nodes have been updated
    if ((num_nodes_updated > 0) || initialize_search) {
        num_nodes_expanded = computeShortestPath();
        if (num_nodes_expanded > 0) std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
        initialize_search = false;
    } else {
        num_nodes_expanded = 0;
    }

    if (publish_expanded_)
        publish_expanded_set();

    constructOptimalPath();
    publish_path();

    return LOOP_OK;
}

void FieldDPlanner::publish_expanded_set() {

    float max_g = -std::numeric_limits<float>::infinity();

    for (std::pair<Node, std::tuple<float, float>> e : expanded_map_) {
        // ignore infinite g-values when getting the max value
        if (std::get<0>(e.second) == std::numeric_limits<float>::infinity())
            continue;
        max_g = std::max(max_g, std::get<0>(e.second));
    }

    std::vector<std::tuple<int, int, float, float>> expanded;
    for (std::pair<Node, std::tuple<float, float>> e : expanded_map_) {

        auto x = static_cast<float>(std::get<0>(e.first.getIndex()) - x_initial_) * map_->resolution;
        auto y = static_cast<float>(std::get<1>(e.first.getIndex()) - y_initial_) * map_->resolution;
        auto g = std::get<0>(e.second);
        auto rhs = std::get<1>(e.second);

        expanded.emplace_back(x, y, g, rhs);
    }

    expanded_cb({expanded, num_nodes_expanded, num_nodes_updated});
}

void FieldDPlanner::set_map(const MapPtr &msg) {
    map_ = msg;  // update current map

    if (initialize_graph_) {
        // initial x and y coordinates of the graph search problem
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

    // re-initialize graph search problem if goal has changed
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

void FieldDPlanner::setGoalDistance(float goal_dist) {
    this->goal_dist_ = goal_dist;
}

CostComputation FieldDPlanner::computeCost(const Node &s, const Node &s_a, const Node &s_b) {
    return this->computeCost(Position(s), Position(s_a), Position(s_b));
}

CostComputation FieldDPlanner::computeCost(const Position &p, const Position &p_a, const Position &p_b) {
    Position p1;  // nearest neighbor
    Position p2;  // diagonal neighbor

    if (node_grid_.isDiagonalContinuous(p, p_a)) {
        p1 = p_b;
        p2 = p_a;
    } else {
        p1 = p_a;
        p2 = p_b;
    }

    // ensure that p and p1 are neighbors along an edge and that p and p2 are continuously diagonal
    assert((p.x == p1.x) || (p.y == p1.y));
    assert((p.x != p2.x) && (p.y != p2.y));

    float g_p1 = getEdgePositionCost(p1);  // path cost of nearest neighbor
    float g_p2 = getEdgePositionCost(p2);  // path cost of diagonal neighbor

    float d_p1 = 1.0f;      // distance to nearest neighbor
    float d_p2 = sqrtf(2);  // distance to diagonal
    float d_n = 1.0f;       // distance between consecutive neighbors (edge length)

    // traversal cost of position p and a diagonal position p2
    // in units of (cost/distance)
    float c = node_grid_.getContinuousTraversalCost(p, p2);
    // traversal cost of position p and p1, a non-diaginal neighbor of p
    // in units of (cost/distance)
    float b = node_grid_.getContinuousTraversalCost(p, p1);

    // travel distances
    float x = 0.0f;
    float y = 0.0f;

    // path cost of node s
    float v_s;

    if (std::max(c, b) == std::numeric_limits<float>::infinity()) {
        // infinite traversal cost, cells likely occupied
        v_s = std::numeric_limits<float>::infinity();
    } else if (g_p1 <= g_p2) {
        // cheapest to travel directly to nearest neighbor (non-diagonal)
        x = d_p1;
        v_s = (std::min(c, b) * x) + g_p1;
    } else {
        float f = g_p1 - g_p2;  // cost of going from s1 to s2

        if (f <= b) {
            if (c <= f) {
                // cheapest to go directly to diagonal cell
                x = d_p1;
                y = d_n;
                v_s = (c * d_p2) + g_p2;
            } else {
                // travel diagonally to point along edge
                x = d_p1;
                float toComp = f / sqrtf((c * c) - (f * f));
                y = std::min(toComp, d_n);
                v_s = c * sqrtf((x * x) + (y * y)) + (f * (d_n - y)) + g_p2;
            }
        } else {
            if (c <= b) {
                // cheapest to go directly to diagonal cell
                x = d_p1;
                y = d_n;
                v_s = (c * d_p2) + g_p2;
            } else {
                // travel along edge then to s2
                float toComp = b / sqrtf((c * c) - (b * b));
                x = d_p1 - std::min(toComp, 1.0f);
                v_s = c * sqrtf((d_n * d_n) + ((d_p1 - x) * (d_p1 - x))) + (b * x) + g_p2;
                y = -1.0f;
            }
        }
    }

    return CostComputation(x, y, v_s);
}

float FieldDPlanner::getEdgePositionCost(const Position &p) {
    if (isVertex(p))
        return getG(p.castToNode());
    else {
        Position p_a(ceilf(p.x), ceilf(p.y));    // get position of first neighbor
        Position p_b(floorf(p.x), floorf(p.y));  // get position of second neighbor

        assert(p_a != p_b);

        float d_a = std::hypot(p_a.x - p.x, p_a.y - p.y); // distance to first neighbor
        float d_b = std::hypot(p_b.x - p.x, p_b.y - p.y); // distance to second neighbor

        assert(((d_a + d_b) - 1.0f) < 1e-5);

        float g_a = getG(p_a.castToNode());  // path cost of p_a
        float g_b = getG(p_b.castToNode());  // path cost of p_b

        return ((d_b * g_a) + (d_a * g_b));  // return linearly interpolated path cost
    }
}

bool FieldDPlanner::isVertex(const Position &p) {
    bool is_vertex = (ceilf(p.x) == p.x) && (ceilf(p.y) == p.y);
    bool satisfies_bounds = (p.x >= 0) && (p.x <= node_grid_.length_) && (p.y >= 0) && (p.y <= node_grid_.width_);
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
    this->node_grid_.key_modifier_ = 0.0f;
    expanded_map_.clear();
    priority_queue_.clear();
    node_grid_.updated_cells_.clear();

    insert_or_assign(node_grid_.start_, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    insert_or_assign(node_grid_.goal_, std::numeric_limits<float>::infinity(), 0.0f);
    priority_queue_.insert(node_grid_.goal_, this->calculateKey(node_grid_.goal_));
}

void FieldDPlanner::updateNode(const Node &s) {
    // s never visited before, add to unordered map with g(s) = rhs(s) = inf
    if (expanded_map_.find(s) == expanded_map_.end()) {
        insert_or_assign(s, std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity());
    } else {
        /**
        looks for a node in the priority queue and removes it if found
        same as calling: if priority_queue_.contains(s) priority_queue_.remove(s);
        */
        priority_queue_.remove(s);
    }

    // update rhs value of Node s
    if (s != node_grid_.goal_) {
        float min_rhs = std::numeric_limits<float>::infinity();
        for (std::tuple<Node, Node> connbr : node_grid_.consecutiveNeighbors(s))
            min_rhs = std::min(min_rhs, this->computeCost(s, std::get<0>(connbr), std::get<1>(connbr)).cost);

        insert_or_assign(s, getG(s), min_rhs);
    }

    // insert node into priority queue if it is locally inconsistent
    if (getG(s) != getRHS(s)) {
        priority_queue_.insert(s, calculateKey(s));
    }
}

int FieldDPlanner::computeShortestPath() {
    // if the start node is occupied, return immediately. No path exists
    if (node_grid_.getMinTraversalCost(node_grid_.start_) == std::numeric_limits<float>::infinity()) {
        std::cerr << "Start node occupied. No path is possible." << std::endl;
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
            insert_or_assign(top_node, std::numeric_limits<float>::infinity(), getRHS(top_node));
            for (Node nbr : node_grid_.nbrs(top_node))
                updateNode(nbr);
            updateNode(top_node);
        }
    }
    return expanded;
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

    return to_update.size();
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
    } while (!isWithinRangeOfGoal(curr_pos) && (min_cost != std::numeric_limits<float>::infinity()) &&
        (curr_step < max_steps));

    if (min_cost == std::numeric_limits<float>::infinity()) {
        std::cerr << "[Extraction] No valid path exists" << std::endl;
        path_.clear();
    } else if (curr_step >= max_steps) {
        std::cerr << "[Extraction] Maximum step number reached" << std::endl;
        path_.clear();
    }
}

// p must be aligned with p_1, p_1 aligned with p_2, p and P_2 diagonal neighbors
std::pair<float, float> FieldDPlanner::getBC(const Position &p, const Position &p_1, const Position &p_2) {
    std::tuple<int, int> cell_ind_b, cell_ind_c;
    float _b, _c;
    if (p.x == p_1.x) { // p lies on a vertical edge
        if (p_2.x > p_1.x) { // we need to traverse the cell on the right
            if (p.y > p_1.y) { // we are over the considered edge
                // "b" is the cost of the cell on the top-left of p_1
                cell_ind_b = std::make_tuple(p_1.x - 1, p_1.y);
                cell_ind_c = std::make_tuple(p_1.x, p_1.y);
            } else { // we are under the considered edge
                // "b" is the cost of the cell on the bottom-left of p_1
                cell_ind_b = std::make_tuple(p_1.x - 1, p_1.y - 1);
                cell_ind_c = std::make_tuple(p_1.x, p_1.y - 1);
            }
        } else { // we need to traverse the cell on the left
            if (p.y > p_1.y) { // we are over the considered edge
                // "b" is the cost of the cell on the top-right of p_1
                cell_ind_b = std::make_tuple(p_1.x, p_1.y);
                cell_ind_c = std::make_tuple(p_1.x - 1, p_1.y);
            } else { // we are under the considered edge
                // "b" is the cost of the cell on the bottom-right of p_1
                cell_ind_b = std::make_tuple(p_1.x, p_1.y - 1);
                cell_ind_c = std::make_tuple(p_1.x - 1, p_1.y - 1);
            }
        }

    } else { // p lies on a horizontal edge
        if (p_2.y > p_1.y) { // we need to traverse the cell at the top
            if (p.x > p_1.x) { // we are on the right of the considered edge
                // "b" is the cost of the cell on the bottom-right of p_1
                cell_ind_b = std::make_tuple(p_1.x, p_1.y - 1);
                cell_ind_c = std::make_tuple(p_1.x, p_1.y);
            } else { // we are on the left of the considered edge
                // "b" is the cost of the cell on the bottom-left of p_1
                cell_ind_b = std::make_tuple(p_1.x - 1, p_1.y - 1);
                cell_ind_c = std::make_tuple(p_1.x, p_1.y - 1);
            }
        } else { // we need to traverse the cell at the bottom
            if (p.x > p_1.x) { // we are on the right of the considered edge
                // "b" is the cost of the cell on the top-right of p_1
                cell_ind_b = std::make_tuple(p_1.x, p_1.y);
                cell_ind_c = std::make_tuple(p_1.x - 1, p_1.y);
            } else { // we are on the left of the considered edge
                // "b" is the cost of the cell on the top-left of p_1
                cell_ind_b = std::make_tuple(p_1.x, p_1.y - 1);
                cell_ind_c = std::make_tuple(p_1.x - 1, p_1.y - 1);
            }
        }
    }
    _b = node_grid_.getValWithConfigurationSpace(cell_ind_b);
    _c = node_grid_.getValWithConfigurationSpace(cell_ind_c);
    return {_b, _c};
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromCorner(const Position &p,
                                                                                   const Position &p_a,
                                                                                   const Position &p_b) {
    std::vector<Position> positions;  // positions to add to path
    float min_cost = std::numeric_limits<float>::infinity();

    Position p_1, p_2;
    bool cond = node_grid_.isDiagonalContinuous(p, p_a);
    p_1 = cond ? p_b : p_a;
    p_2 = cond ? p_a : p_b;

    // ensure that p and p1 are neighbors along an edge and that p and p2 are continuously diagonal
    assert((p.x == p_1.x) || (p.y == p_1.y));
    assert((p.x != p_2.x) && (p.y != p_2.y));

    float _b1, _c, _f, _g_s1, _g_s2;
    std::tie(_b1, _c) = getBC(p, p_1, p_2);
    _g_s1 = getG(p_1.castToNode());
    _g_s2 = getG(p_2.castToNode());
    if (_g_s1 == std::numeric_limits<float>::infinity() && _g_s2 == std::numeric_limits<float>::infinity())
        return std::make_pair(positions, min_cost);
    _f = _g_s1 - _g_s2;
    int type;

    if (_c > _b1) {
        float t = std::min(_f, _b1);
        min_cost = _g_s2 + _c * std::sqrt(2);
        type = 3;                     // Type A
        if (_f < 0 or _f * _f < (_c * _c - _b1 * _b1)) {
            min_cost = _g_s1 + _b1;
            type = 2;                           // Type III
        } else if (_c > t * std::sqrt(2)) {
            min_cost = _g_s2 + t + std::sqrt(_c * _c - t * t);
            type = _f < _b1;     // Type I or II (check _f>_b)
        }
    } else {
        if (_f > _c / std::sqrt(2)) {
            min_cost = _g_s2 + _c * std::sqrt(2);
            type = 3;                 // Type A
        } else if (_f < 0) {
            min_cost = _g_s1 + _c;
            type = 4;                            // Type B
        } else {
            min_cost = _g_s1 + std::sqrt(_c * _c - _f * _f);
            type = 1;   // Type II
        }
    }

    float _x, _y;
    switch (type) {
        case 0: // Type I
            _x = 1 - _b1 / std::sqrt(_c * _c - _b1 * _b1);
            if (p.x == p_1.x) { // p lies on a vertical edge
                positions.emplace_back(p.x, p.y + (p_1.y - p.y) * _x);
            } else {            // p lies on a horizontal edge
                positions.emplace_back(p.x + (p_1.x - p.x) * _x, p.y);
            }
            positions.push_back(p_2);
            break;
        case 1: // Type II
            _y = _f / std::sqrt(_c * _c - _f * _f);
            if (p.x == p_1.x) { // p lies on a vertical edge
                positions.emplace_back(p_1.x + (p_2.x - p_1.x) * _y, p_1.y);
            } else {            // p lies on a horizontal edge
                positions.emplace_back(p_1.x, p_1.y + (p_2.y - p_1.y) * _y);
            }
            break;
        case 2: // Type III
            positions.push_back(p_1);
            break;
        case 3: // Type A
            positions.push_back(p_2);
            break;
        case 4: // Type B
            positions.push_back(p_1);
            break;
    }

    return std::make_pair(positions, min_cost);
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversalFromEdge(const Position &p, const Position &p_a,
                                                                                 const Position &p_b) {
    std::vector<Position> positions;  // positions to add to path

    float _b1, _c, _b2, _f, _p, _q, _g_s1, _g_s2, min_cost;
    bool cond_1 = (p.x == p_a.x || p.y == p_a.y);
    bool cond_2 = (p.x == p_b.x || p.y == p_b.y);
    if (cond_1 || cond_2) { // "contiguous" edge case
        const Position &p_1 = cond_1 ? p_a : p_b; // lies on the same edge of p
        const Position &p_2 = cond_1 ? p_b : p_a;
        assert((p.x == p_1.x) || (p.y == p_1.y));
        assert((p.x != p_2.x) && (p.y != p_2.y));

        std::tie(_b1, _c) = getBC(p, p_1, p_2);
        _q = 1 - std::abs(p_1.y - p.y) - std::abs(p_1.x - p.x);
        _g_s1 = getG(p_1.castToNode());
        _g_s2 = getG(p_2.castToNode());
        _f = _g_s1 - _g_s2;

        //c1, c2, c3, cA, cB;
        std::array<float, 5> costs{};
        costs.fill(std::numeric_limits<float>::infinity());
        if (_c > _b1 * std::sqrt(1 + (1 / ((1 - _q) * (1 - _q)))))
            costs[0] = _g_s2 + (1 - _q) * _b1 + std::sqrt(_c * _c - _b1 * _b1); // Type I
        if (_f > 0 and _c > _f * std::sqrt(1 + ((1 - _q) * (1 - _q))))
            costs[1] = _g_s2 + (1 - _q) * std::sqrt(_c * _c - _f * _f) + _f; // Type II
        if (_c > _b1)
            costs[2] = _g_s2 + (1 - _q) * _b1 + _f;                          // Type III
        costs[3] = _g_s2 + _c * std::sqrt(1 - ((1 - _q) * (1 - _q)));         // Type A
        costs[4] = _g_s2 + _c * std::sqrt(1 - (_q * _q)) + _f;            // Type B

        auto min = std::min_element(costs.begin(), costs.end());
        min_cost = *min;
        int min_idx = std::distance(costs.begin(), min);

        // Compute spatial parameters if either Type I or II
        // "x" can be seen as displacement from s towards s_1
        // "y" can be seen as displacement from s_1 towards s_2
        // Append path points to the result set
        float _x = 0, _y = 0;
        switch (min_idx) {
            case 0: // Type I
                _x = 1 - _q - _b1 / std::sqrt(_c * _c - _b1 * _b1);
                break;
            case 1: // Type II
                _y = (1 - _q) * _f / std::sqrt(_c + _c - _f * _f);
                break;
            default: break;
        }

        switch (min_idx) {
            case 0:
                if (p.x == p_1.x) { // p lies on a vertical edge
                    positions.emplace_back(p.x, p.y + (p_1.y - p.y) / std::abs((p_1.y - p.y)) * _x);
                } else {            // p lies on a horizontal edge
                    positions.emplace_back(p.x + (p_1.x - p.x) / std::abs((p_1.x - p.x)) * _x, p.y);
                }
                positions.push_back(p_2);
                break;
            case 1:
                if (p.x == p_1.x) { // p lies on a vertical edge
                    positions.emplace_back(p_1.x + (p_2.x - p_1.x) * _y, p_1.y);
                } else {            // p lies on a horizontal edge
                    positions.emplace_back(p_1.x, p_1.y + (p_2.y - p_1.y) * _y);
                }
                break;
            case 2:positions.push_back(p_1);
                break;
            case 3:positions.push_back(p_2);
                break;
            case 4:positions.push_back(p_1);
                break;
        }
    } else { // "opposite" edge case
        const Position &p_1 = p_a;
        const Position &p_2 = p_b;
        assert((p.x != p_1.x) && (p.y != p_1.y));
        assert((p.x != p_2.x) && (p.y != p_2.y));

        // align p with p_1 and p_2 before getBC();
        Position pb = p, pd = p;
        if (p_1.x == p_2.x) { // vertical edge, p must move up or down
            pb.y = p_1.y;
            pd.y = p_2.y;
        } else {              // horizontal edge, p must move left or right
            pb.x = p_1.x;
            pd.x = p_2.x;
        }

        float __c;
        std::tie(_b1, _c) = getBC(pb, p_1, p_2);
        std::tie(_b2, __c) = getBC(pd, p_2, p_1);
        assert(_c == __c);

        _p = std::abs(p_1.y - pb.y) + std::abs(p_1.x - pb.x);
        _g_s1 = getG(p_1.castToNode());
        _g_s2 = getG(p_2.castToNode());
        _f = _g_s1 - _g_s2;

        // Use _f, _p, _b1, _g_s2;
        // Use neg(_f), 1-_p, _b2, _g_s1;

        //c1, c2, c3, cA, cB;
        std::array<float, 7> costs{};
        costs.fill(std::numeric_limits<float>::infinity());
        if (_c > _b1 * std::sqrt(1 + ((1 + _p) * (1 + _p))))
            costs[0] = _g_s2 + _b1 + (1 + _p) * std::sqrt(_c * _c - _b1 * _b1);    // Type I
        if (_c > _b2 * std::sqrt(1 + ((2 - _p) * (2 - _p))))
            costs[1] = _g_s1 + _b2 + (2 - _p) * std::sqrt(_c * _c - _b2 * _b2);    // Type I
        if (_f > 0 and _c > _f * std::sqrt(1 + (1 / ((1 - _p) * (1 - _p)))))
            costs[2] = _g_s2 + std::sqrt(_c * _c - _f * _f) + (1 - _p) * _f;       // Type II
        else if (_f < 0 and _c > -_f * std::sqrt(1 + (1 / (_p * _p))))
            costs[2] = _g_s2 + std::sqrt(_c * _c - _f * _f) + (1 - _p) * _f;       // Type II
        if (_c > _b1 * std::sqrt(1 + _p * _p))
            costs[3] = _g_s2 + _b1 + _p * std::sqrt(_c * _c - _b1 * _b1) + _f;     // Type III
        if (_c > _b2 * std::sqrt(1 + (1 - _p) * (1 - _p)))
            costs[4] = _g_s1 + _b2 + (1 - _p) * std::sqrt(_c * _c - _b2 * _b2) - _f; // Type III
        costs[5] = _g_s2 + _c * std::sqrt((1 - _p) * (1 - _p) - 1);                 // Type A
        costs[6] = _g_s2 + _c * std::sqrt(_p * _p - 1) + _f;                    // Type B

        auto min = std::min_element(costs.begin(), costs.end());
        min_cost = *min;
        int min_idx = std::distance(costs.begin(), min);

        // Compute spatial parameters if either Type I II or III
        // "x" can be seen:
        // - for type I, as displacement from s towards the edge (refer to "Otte et al." cell extension trick)
        // - for type III, as displacement from s towards the edge
        // "y" can be seen as displacement from s_1 towards s_2
        // Append path points to the result set
        float _x = 0, _y = 0;
        switch (min_idx) {
            case 0: // Type I
                _x = 1 - (1 + _p) * _b1 / std::sqrt(_c * _c - _b1 * _b1);
                break;
            case 1: // Type I
                _x = 1 - (2 - _p) * _b2 / std::sqrt(_c * _c - _b2 * _b2);
                break;
            case 2: // Type II
                _y = _p + _f / std::sqrt(__c * _c - _f * _f);
                break;
            case 3: // Type III
                _x = _p * _b1 / std::sqrt(_c * _c - _b1 * _b1);
                break;
            case 4: // Type III
                _x = (1 - _p) * _b2 / std::sqrt(_c * _c - _b2 * _b2);
                break;
            default: break;
        }

        switch (min_idx) {
            case 0: // Type I
                if (p.x == pb.x) { // p lies on a vertical edge
                    positions.emplace_back(pb.x + (p_1.x - pb.x) * (1 - _x) * _p, p_1.y);
                    positions.emplace_back(pb.x + (p_1.x - pb.x) * (1 - _x) * _p + _x, p_1.y);
                } else {            // p lies on a horizontal edge
                    positions.emplace_back(p_1.x, pb.y + (p_1.y - pb.y) * (1 - _x) * _p);
                    positions.emplace_back(p_1.x, pb.y + (p_1.y - pb.y) * (1 - _x) * _p + _x);
                }
                positions.push_back(p_2);
                break;
            case 1: // Type I
                if (p.x == pd.x) { // p lies on a vertical edge
                    positions.emplace_back(pd.x + (p_2.x - pd.x) * (1 - _x) * (1 - _p), p_2.y);
                    positions.emplace_back(pd.x + (p_2.x - pd.x) * (1 - _x) * (1 - _p) + _x, p_2.y);
                } else {            // p lies on a horizontal edge
                    positions.emplace_back(p_2.x, pd.y + (p_2.y - pd.y) * (1 - _x) * (1 - _p));
                    positions.emplace_back(p_2.x, pd.y + (p_2.y - pd.y) * (1 - _x) * (1 - _p) + _x);
                }
                positions.push_back(p_1);
                break;
            case 2: // Type II
                if (p.x == pb.x) { // p lies on a vertical edge
                    positions.emplace_back(p_1.x + (p_2.x - p_1.x) * _y, p_1.y);
                } else {           // p lies on a horizontal edge
                    positions.emplace_back(p_1.x, p_1.y + (p_2.y - p_1.y) * _y);
                }
                break;
            case 3: // Type III
                if (p.x == pb.x) { // p lies on a vertical edge
                    positions.emplace_back(pb.x + (p_1.x - pb.x) * _x, pb.y);
                } else {            // p lies on a horizontal edge
                    positions.emplace_back(pb.x, pb.y + (p_1.y - pb.y) * _x);
                }
                positions.push_back(p_1);
                break;
            case 4: // Type III
                if (p.x == pd.x) { // p lies on a vertical edge
                    positions.emplace_back(pd.x + (p_2.x - pd.x) * _x, pd.y);
                } else {            // p lies on a horizontal edge
                    positions.emplace_back(pd.x, pd.y + (p_2.y - pd.y) * _x);
                }
                positions.push_back(p_2);
                break;
            case 5: // Type A
                positions.push_back(p_2);
                break;
            case 6: // Type B
                positions.push_back(p_1);
                break;
        }
    }
    return std::make_pair(positions, min_cost);
}

FieldDPlanner::path_additions FieldDPlanner::getPathAdditions(const Position &p, int lookahead_dist_remaining) {
    float min_cost = std::numeric_limits<float>::infinity();
    path_additions min_pa;
    path_additions temp_pa;
    float lookahead_cost;
    std::cout << "\np     " << std::to_string(p.x) << ", " << std::to_string(p.y) << std::endl << std::endl;
    for (const auto &[p_a, p_b] : node_grid_.nbrsContinuous(p)) {
        // look ahead `lookahead_dist_remaining` planning steps into the future for best action
        if (isVertex(p))
            temp_pa = computeOptimalCellTraversalFromCorner(p, p_a, p_b);
        else
            temp_pa = computeOptimalCellTraversalFromEdge(p, p_a, p_b);
        std::cout << "\np_a   " << std::to_string(p_a.x) << ", " << std::to_string(p_a.y) << std::endl;
        std::cout << "p_b   " << std::to_string(p_b.x) << ", " << std::to_string(p_b.y) << std::endl;
        std::cout << "cost: " << std::to_string(temp_pa.second) << std::endl;
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
        return std::numeric_limits<float>::infinity();
}

float FieldDPlanner::getRHS(const Node &s) {
    // return rhs value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (expanded_map_.find(s) != expanded_map_.end())
        return std::get<1>(expanded_map_.at(s));
    else
        return std::numeric_limits<float>::infinity();
}
