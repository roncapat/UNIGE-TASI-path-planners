#include "DynamicFastMarching.h"
#include <cmath>
#include <array>
#include <numeric>
#include <chrono>
#include <iostream>
#include <memory>

const float SQRT2 = 1.41421356237309504880168872420969807856967187537694f;

void DFMPlanner::init() {
    initialize_search = true;
}

int DFMPlanner::step() {
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
        PriorityQueue new_queue;
        for (const auto &elem: priority_queue)
            // Only heuristic changes, so either G or RHS is kept the same
            new_queue.insert(elem.cell, calculateKey(elem.cell, elem.key.second));
        priority_queue.swap(new_queue);
        // gather cells with updated edge costs and update affected nodes
        updateCells();
    }
    auto end = std::chrono::steady_clock::now();
    u_time = std::chrono::duration<float, std::milli>(end - begin).count();

    // only update the graph if nodes have been updated
    begin = std::chrono::steady_clock::now();
    if ((num_cells_updated > 0)) {
        computeShortestPath();
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

void DFMPlanner::set_map(const MapPtr &msg) {
    grid.initializeGraph(msg);
    initialize_graph_ = false;
}

void DFMPlanner::set_goal(const Position &point) {
    Cell new_goal(point);
    if (grid.goal_ != new_goal)
        new_goal_ = true;
    grid.setGoal(new_goal);
    grid.goal_pos_ = point;
    goal_set_ = true;
}

bool DFMPlanner::isVertex(const Position &p) {
    bool is_vertex = (ceilf(p.x) == p.x) && (ceilf(p.y) == p.y);
    bool satisfies_bounds = grid.isValid(p);
    return is_vertex && satisfies_bounds;
}

PriorityQueue::Key DFMPlanner::calculateKey(const Cell &s) {
    auto[g, rhs] = map.getKey(s);
    return calculateKey(s, g, rhs);
}

PriorityQueue::Key DFMPlanner::calculateKey(const Cell &s, float g, float rhs) {
    return calculateKey(s, std::min(g, rhs));
}

PriorityQueue::Key DFMPlanner::calculateKey(const Cell &s, float cost_so_far) {
    auto dist = std::hypot(grid.goal_pos_.x - s.x, grid.goal_pos_.y - s.y);
    return {cost_so_far + heuristic_multiplier * dist, cost_so_far};
    //return {cost_so_far, cost_so_far}; //FIXME no need pair key in FMM without heuristic
}

void DFMPlanner::initializeSearch() {
    num_cells_updated = 0;
    num_nodes_expanded = 0;
    map.clear();
    priority_queue.clear();
    grid.updated_cells_.clear();
    map.insert_or_assign(grid.start_, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_, INFINITY, 0);
    priority_queue.insert(grid.goal_, calculateKey(grid.goal_, 0));
    num_cells_updated = 1;
}

bool DFMPlanner::end_condition() {
    auto top_key = priority_queue.topKey();
    auto[g, rhs] = map.getKey(grid.start_);
    return ((top_key >= calculateKey(grid.start_, rhs)) and std::abs(rhs - g) > 1e-2);
}

unsigned long DFMPlanner::computeShortestPath() {

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Cell s = priority_queue.topCell();
        priority_queue.pop();
        ++expanded;
        //std::cout << "POP    " << s.x << " " << s.y << std::endl;
        // Get reference to the node
        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Cell &nbr : grid.neighbors(s))
                updateCell(nbr);
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Cell &nbr : grid.neighbors(s))
                updateCell(nbr);
            updateCell(s);
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    return num_nodes_expanded;
}

std::pair<std::shared_ptr<float[]>, std::shared_ptr<float[]>> DFMPlanner::costMapGradient() {
    std::shared_ptr<float[]> _gh(new float[grid.size_], std::default_delete<float[]>());
    std::shared_ptr<float[]> _gv(new float[grid.size_], std::default_delete<float[]>());
    auto gh = [&](int x, int y) -> float & { return _gh.get()[x * grid.width_ + y]; };
    auto gv = [&](int x, int y) -> float & { return _gv.get()[x * grid.width_ + y]; };
    for (int i = 0; i < grid.length_; ++i) {
        for (int j = 0; j < grid.width_; ++j) {
            Cell c(i, j);
            std::tie(gh(i, j), gv(i, j)) = gradientAtCell(c);
        }
    }
    return {_gh, _gv};
}

//https://helloacm.com/cc-function-to-compute-the-bilinear-interpolation/
float
BilinearInterpolation(float q11,
                      float q12,
                      float q21,
                      float q22,
                      float x1,
                      float x2,
                      float y1,
                      float y2,
                      float x,
                      float y) {
    float x2x1, y2y1, x2x, y2y, yy1, xx1;
    x2x1 = x2 - x1;
    y2y1 = y2 - y1;
    x2x = x2 - x;
    y2y = y2 - y;
    yy1 = y - y1;
    xx1 = x - x1;
    return 1.0 / (x2x1 * y2y1) * (
        q11 * x2x * y2y +
            q21 * xx1 * y2y +
            q12 * x2x * yy1 +
            q22 * xx1 * yy1
    );
}

std::tuple<float, float> DFMPlanner::interpolateGradient(const Position &c,
                                                         std::shared_ptr<float[]> _gh,
                                                         std::shared_ptr<float[]> _gv) {
    // Coords of nearest node
    int x2 = (int) std::lround(c.x);
    int y2 = (int) std::lround(c.y);
    int x1 = x2 - 1;
    int y1 = y2 - 1;
    auto gh = [&](int x, int y) -> float & { return _gh.get()[x * grid.width_ + y]; };
    auto gv = [&](int x, int y) -> float & { return _gv.get()[x * grid.width_ + y]; };
    //return {gh(x1, y1), gv(x1, y1)};
    auto blh = gh(x1, y1), tlh = gh(x1, y2), brh = gh(x2, y1), trh = gh(x2, y2);
    auto blv = gv(x1, y1), tlv = gv(x1, y2), brv = gv(x2, y1), trv = gv(x2, y2);
    assert(c.x >= (x1 + 0.5f) && c.x <= (x2 + 0.5f));
    assert(c.y >= (y1 + 0.5f) && c.y <= (y2 + 0.5f));
    #ifdef VERBOSE_EXTRACTION
    std::cout << "Interpolating gradient for (" << c.x << ", " << c.y << ")" << std::endl;
    std::cout << "Corners (x,y):\n"
              << "\t(" << x1 << ", " << y2 << ")" << "\t\t(" << x2 << ", " << y2 << ")\n\n"
              << "\t(" << x1 << ", " << y1 << ")" << "\t\t(" << x2 << ", " << y1 << ")\n";
    std::cout << "Values (h,v):\n"
              << "\t(" << tlh << ", " << tlv << ")" << "\t\t(" << trh << ", " << trv << ")\n\n"
              << "\t(" << blh << ", " << blv << ")" << "\t\t(" << brh << ", " << brv << ")\n";
    #endif
    //FIXME handle borders
    return {    //Interpolate WRT centers of the 4 nearest cells (thus, shift 0.5f)
        BilinearInterpolation(blh, tlh, brh, trh,
                              x1 + 0.5f, x2 + 0.5f, y1 + 0.5f, y2 + 0.5f,
                              c.x, c.y),
        BilinearInterpolation(blv, tlv, brv, trv,
                              x1 + 0.5f, x2 + 0.5f, y1 + 0.5f, y2 + 0.5f,
                              c.x, c.y)
    };
}

std::tuple<float, float> DFMPlanner::gradientAtCell(const Cell __c) {
    float g = map.getRHS(__c);
    if (g == INFINITY) {
        return {0, 0};
    }
    Cell t = grid.topCell(__c), b = grid.bottomCell(__c), l = grid.leftCell(__c), r = grid.rightCell(__c);
    float v_pre = map.getRHS(t);
    float v_post = map.getRHS(b);
    float h_pre = map.getRHS(l);
    float h_post = map.getRHS(r);

    float dx, dy;
    if (grid.isValid(t) and v_pre < INFINITY) {
        if (grid.isValid(b) and v_post < INFINITY) {
            dy = (-0.5f) * v_pre + (0.5f) * v_post; // df/dy (second order of accuracy, centered)
        } else {
            dy = std::max(-v_pre + g, 0.f); // df/dy (first order of accuracy, backward)
        }
    } else if (grid.isValid(b) and v_post < INFINITY) {
        dy = std::min(-g + v_post, 0.f); // df/dy (first order of accuracy, forward)
    } else {
        dy = 0;
    }

    if (grid.isValid(l) and h_pre < INFINITY) {
        if (grid.isValid(r) and h_post < INFINITY) {
            dx = (-0.5f) * h_pre + (0.5f) * h_post; // df/dx (second order of accuracy, centered)
        } else {
            dx = std::max(-h_pre + g, 0.f); // df/dx (first order of accuracy, backward)
        }
    } else if (grid.isValid(r) and h_post < INFINITY) {
        dx = std::min(-g + h_post, 0.f); // df/dx (first order of accuracy, forward)
    } else {
        dx = 0;
    }

    assert(!std::isnan(dx));
    assert(!std::isnan(dy));
    float abs = std::hypot(dx, dy);

    if (abs > 0) {
        dx = dx / abs;
        dy = dy / abs;
    }
    return {dx, dy};
}

void DFMPlanner::updateCell(const Cell &cell) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_)
        RHS(s_it) = computeOptimalCost(cell);

    enqueueIfInconsistent(s_it);
    //priority_queue.print();
}

float DFMPlanner::computeOptimalCost(const Cell &c) {
    float g_a_1 = minCost(grid.topCell(c), grid.bottomCell(c));
    float g_b_1 = minCost(grid.leftCell(c), grid.rightCell(c));
    if (g_a_1 > g_b_1) std::swap(g_a_1, g_b_1);
    if (g_a_1 == INFINITY and g_b_1 == INFINITY) return INFINITY;
    if (grid.getTraversalCost(c) == INFINITY) return INFINITY;
    auto tau = grid.getTraversalCost(c);
    if (tau > (g_b_1 - g_a_1)) {
        return (g_a_1 + g_b_1 + std::sqrt(2 * SQUARE(tau) - SQUARE(g_b_1 - g_a_1))) / 2.0f;
    } else {
        return g_a_1 + tau;
    }
}

float DFMPlanner::minCost(const Cell &a, const Cell &b) {
    return std::min(map.getG(a), map.getG(b));
}

unsigned long DFMPlanner::updateCells() {
    for (const Cell &cell : grid.updated_cells_) {
        updateCell(cell);
    }

    num_cells_updated = grid.updated_cells_.size();
    std::cout << num_cells_updated << " cells updated" << std::endl;
    return num_cells_updated;
}

void DFMPlanner::enqueueIfInconsistent(ExpandedMap::iterator it) {
    if (G(it) != RHS(it))
        priority_queue.insert_or_update(CELL(it), calculateKey(CELL(it), G(it), RHS(it)));
    else
        priority_queue.remove_if_present(CELL(it));
}

void DFMPlanner::constructOptimalPath() {
    path_.clear();
    cost_.clear();
    total_cost = 0;
    total_dist = 0;

    auto[gh, gv] = costMapGradient();
    path_.push_back(grid.start_pos_);

    char buf[30];

    float min_cost = 0;
    int curr_step = 0;
    // TODO do something better than this sh*t
    int max_steps = 3000;

    float alpha = 0.2;
    auto s = grid.start_pos_;
    while (std::hypot(grid.goal_pos_.x - s.x, grid.goal_pos_.y - s.y) > 0.8) {
        if (curr_step > max_steps) break;
        auto[sgx, sgy] = interpolateGradient(s, gv, gh);
        #ifdef VERBOSE_EXTRACTION
        std::sprintf(buf, "s = [ %7.4f %7.4f]", s.x, s.y);
        std::cout << "Step " << curr_step << " " << buf << std::flush;
        std::sprintf(buf, " g = [ %7.4f %7.4f]", sgx, sgy);
        std::cout << buf << std::endl;
        #endif
        if (std::abs(sgx) < 0.0001 && std::abs(sgy) < 0.0001) break;
        s = Position(s.x - alpha * sgx, s.y - alpha * sgy);
        path_.push_back(s);
        cost_.push_back(0); // TODO manage cost computation
        ++curr_step;
//        if (curr_step == 180) break;
    }

    path_.push_back(grid.goal_pos_);
    cost_.push_back(0);
    //std::reverse(path_.begin(), path_.end());

    if (min_cost == INFINITY) {
        std::cerr << "[Extraction] No valid path exists" << std::endl;
    } else if (curr_step >= max_steps) {
        std::cerr << "[Extraction] Maximum step number reached" << std::endl;
    }

    std::cout << "Found path. Cost: " << total_cost << " Distance: " << total_dist << std::endl;
}

bool DFMPlanner::goalReached(const Position &p) {
    return grid.goal_.x == p.x && grid.goal_.y == p.y;
}

DFMPlanner::ExpandedMap::iterator
DFMPlanner::ExpandedMap::find_or_init(const Cell &n) {
    iterator it = find(n);
    if (it == end()) { // Init node if not yet considered
        it = emplace(n, std::make_tuple(INFINITY, INFINITY, NULLCELL)).first;
    }
    return it;
}

DFMPlanner::ExpandedMap::iterator
DFMPlanner::ExpandedMap::insert_or_assign(const Cell &s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    auto it = find(s);
    if (it != end()) {
        std::get<0>(it->second) = g;
        std::get<1>(it->second) = rhs;
        return it;
    } else {
        [[maybe_unused]] auto[it, ok] = emplace(s, std::make_tuple(g, rhs, NULLCELL));
        assert(ok);
        return it;
    }
}

PriorityQueue::Key DFMPlanner::ExpandedMap::getKey(const Cell &s) {
    ExpandedMap::iterator it;
    if ((it = find(s)) != end())
        return {G(it), RHS(it)};
    else
        return {INFINITY, INFINITY};
}

float DFMPlanner::ExpandedMap::getG(const Cell &s) {
    ExpandedMap::iterator it;
    if ((it = find(s)) != end())
        return G(it);
    else
        return INFINITY;
}

float DFMPlanner::ExpandedMap::getRHS(const Cell &s) {
    ExpandedMap::iterator it;
    if ((it = find(s)) != end())
        return RHS(it);
    else
        return INFINITY;
}

bool DFMPlanner::consistent(const Cell &s) {
    auto[g, rhs] = map.getKey(s);
    return g == rhs;
}

bool DFMPlanner::consistent(const ExpandedMap::iterator &it) {
    return G(it) == RHS(it);
}
void DFMPlanner::set_start(const Position &pos) {
    grid.start_pos_ = pos; //TODO move start_pos in grid also for FD* ??? major refactor needed
    grid.start_ = Cell(pos);
    new_start = true;
}

void DFMPlanner::patch_map(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h) {
    grid.updateGraph(patch, x, y, w, h);
}
void DFMPlanner::set_occupancy_threshold(float threshold) { grid.setOccupancyThreshold(threshold); }
void DFMPlanner::set_heuristic_multiplier(float mult) { heuristic_multiplier = mult; }
void DFMPlanner::set_optimization_lvl(int lvl) { optimization_lvl = lvl; }
void DFMPlanner::set_first_run_trick(bool enable) { first_run_trick = enable; }
DFMPlanner::DFMPlanner() {}
