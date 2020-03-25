#include "FieldDPlanner.h"
#include "NodeUtils.hpp"

FieldDPlanner::FieldDPlanner() {
    expanded_cloud.header.frame_id = "odom";
}

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

    constructOptimalPath();

    publish_path();
    if (publish_expanded_)
        publish_expanded_set(expanded_cloud);

    return LOOP_OK;
}

void FieldDPlanner::publish_expanded_set(pcl::PointCloud<pcl::PointXYZRGB> &expanded_cloud) {
    expanded_cloud.clear();

    float max_g = -std::numeric_limits<float>::infinity();

    for (std::pair<Node, std::tuple<float, float>> e : expanded_map_) {
        // ignore infinite g-values when getting the max value
        if (std::get<0>(e.second) == std::numeric_limits<float>::infinity())
            continue;
        max_g = std::max(max_g, std::get<0>(e.second));
    }

    for (std::pair<Node, std::tuple<float, float>> e : expanded_map_) {
        pcl::PointXYZRGB p;

        p.x = static_cast<float>(std::get<0>(e.first.getIndex()) - x_initial_) * map_->resolution;
        p.y = static_cast<float>(std::get<1>(e.first.getIndex()) - y_initial_) * map_->resolution;
        p.z = -0.05f;

        // set the node color to red if its g-value is inf. Otherwise scale.
        if (std::get<0>(e.second) == std::numeric_limits<float>::infinity()) {
            p.r = 255;
            p.g = 0;
            p.b = 0;
        } else {
            p.r = 0;
            p.g = 125;
            p.b = static_cast<uint8_t>((std::get<0>(e.second) / max_g) * 255.0f);
        }
        expanded_cloud.points.push_back(p);
    }

    expanded_cb({expanded_cloud, num_nodes_expanded, num_nodes_updated});
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

        float d_a = igvc::get_distance(p.x, p.y, p_a.x, p_a.y);  // distance to first neighbor
        float d_b = igvc::get_distance(p.x, p.y, p_b.x, p_b.y);  // distance to second neighbor

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
    if (node_grid_.getMinTraversalCost(node_grid_.start_) == std::numeric_limits<float>::infinity())
        return 0;

    int num_nodes_expanded = 0;
    while (((priority_queue_.topKey() < calculateKey(node_grid_.start_)) ||
        (std::fabs(getRHS(node_grid_.start_) - getG(node_grid_.start_)) > 1e-5)) &&
        (!priority_queue_.empty())) {
        Node top_node = priority_queue_.topNode();
        priority_queue_.pop();
        num_nodes_expanded++;

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

    return to_update.size();
}

void FieldDPlanner::constructOptimalPath() {
    path_.clear();

    Position curr_pos(node_grid_.start_);
    path_.push_back(curr_pos);

    float min_cost;
    path_additions pa;

    int curr_step = 0;
    int max_steps = static_cast<int>(2000.00f / (this->node_grid_.resolution_));

    do {
        // move one step and calculate the optimal path additions (min 1, max 2)
        pa = getPathAdditions(curr_pos, lookahead_dist_);
        path_.insert(path_.end(), pa.first.begin(), pa.first.end());
        min_cost = pa.second;
        curr_pos = path_.back();
        curr_step += 1;
    } while (!isWithinRangeOfGoal(curr_pos) && (min_cost != std::numeric_limits<float>::infinity()) &&
        (curr_step < max_steps));

    // no valid path found. Set path to empty
    if (min_cost == std::numeric_limits<float>::infinity()) {
        std::cerr << "No valid path exists" << std::endl;
        path_.clear();
    } else if (curr_step >= max_steps) {
        std::cerr << "Maximum step number reached" << std::endl;
        path_.clear();
    }
}

FieldDPlanner::path_additions FieldDPlanner::computeOptimalCellTraversal(const Position &p, const Position &p_a,
                                                                         const Position &p_b) {
    std::vector<Position> positions;  // positions to add to path
    Position p1, p2;                  // p1 - nearest neighbor; p2 - diagonal
    CostComputation traversal_computation = this->computeCost(p, p_a, p_b);

    if (node_grid_.isDiagonalContinuous(p, p_a))  // p_b is nearest neighbor
    {
        p1 = p_b;
        p2 = p_a;
    } else  // p_a is nearest neighbor
    {
        p1 = p_a;
        p2 = p_b;
    }

    // CASE 0: no valid path found (infinite cost/no consecutive neighbors)
    if (traversal_computation.cost == std::numeric_limits<float>::infinity())
        return std::make_pair(positions, traversal_computation.cost);

    // calculate the multiplier for the positions to be added to the path. This
    // step is required because x and y calculations are peformed independent of
    // the orientation of the consecutive neighbors used to obtain these values.
    // As such, x_multiplier and y_multiplier account for this.

    // CASE 1: travel along x then cut to s2
    if (traversal_computation.y < 0.0f) {
        positions.push_back(p2);
        traversal_computation.y = 0.0f;
    }

    if (p1.x != p.x)  // nearest neighbor lies to left or right of s
    {
        traversal_computation.x *= (p1.x > p.x) ? 1.0f : -1.0f;
        traversal_computation.y *= (p2.y > p.y) ? 1.0f : -1.0f;
    } else  // nearest neighbor lies above or below s
    {
        std::tie(traversal_computation.x, traversal_computation.y) =
            std::make_tuple(traversal_computation.y,
                            traversal_computation
                                .x);  // path additions must be flipped to account for relative orientation
        traversal_computation.y *= (p1.y > p.y) ? 1.0f : -1.0f;
        traversal_computation.x *= (p2.x > p.x) ? 1.0f : -1.0f;
    }

    // CASE 1: travel along x then cut to s2
    // CASE 2: travel directly to diagonal node (s2)
    // CASE 3: travel to nearest node
    // CASE 4: travel to point along edge
    positions.insert(positions.begin(), Position(p.x + traversal_computation.x, p.y + traversal_computation.y));

    return std::make_pair(positions, traversal_computation.cost);
}

FieldDPlanner::path_additions FieldDPlanner::getPathAdditions(const Position &p, int lookahead_dist_remaining) {
    float min_cost = std::numeric_limits<float>::infinity();
    path_additions min_pa;

    path_additions temp_pa;
    float lookahead_cost;

    Position p_a, p_b;  // temp positions
    for (std::pair<Position, Position> connbr : node_grid_.nbrsContinuous(p)) {
        // look ahead `lookahead_dist_remaining` planning steps into the future for best action
        std::tie(p_a, p_b) = connbr;
        temp_pa = computeOptimalCellTraversal(p, p_a, p_b);
        if ((lookahead_dist_remaining <= 0) || temp_pa.first.empty())
            lookahead_cost = temp_pa.second;
        else
            lookahead_cost = getPathAdditions(temp_pa.first.back(), lookahead_dist_remaining - 1).second;

        if (lookahead_cost < min_cost) {
            min_cost = lookahead_cost;
            min_pa = temp_pa;
        }
    }
    return min_pa;
}

bool FieldDPlanner::isWithinRangeOfGoal(const Position &p) {
    float distance_to_goal =
        igvc::get_distance(std::make_tuple(p.x, p.y),
                           static_cast<std::tuple<float, float>>(node_grid_.goal_.getIndex()));
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
