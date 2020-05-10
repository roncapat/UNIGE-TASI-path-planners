#include "DynamicFastMarching.h"
#include <cmath>
#include <array>
#include <numeric>
#include <chrono>
#include <iostream>
#include <memory>
#include <tuple>
#include <utility>
#include <LinearTraversalCostInterpolation.h>
#include <Interpolation.h>

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
        Queue new_queue;
        for (const auto &elem: priority_queue)
            // Only heuristic changes, so either G or RHS is kept the same
            new_queue.insert(elem.elem, calculateKey(elem.elem, elem.key.second));
        priority_queue.swap(new_queue);
        //new_queue.insert(grid.goal_cell_, calculateKey(grid.goal_cell_, 0));
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



    //computeRoughtPath();
    //computeRoughtPath(true);
    computeInterpolatedPath();
    //constructOptimalPath();


    end = std::chrono::steady_clock::now();
    e_time = std::chrono::duration<float, std::milli>(end - begin).count();
    std::cout << "Update time     = " << u_time << " ms" << std::endl;
    std::cout << "Planning time   = " << p_time << " ms" << std::endl;
    std::cout << "Extraction time = " << e_time << " ms" << std::endl;

    return LOOP_OK;
}

void DFMPlanner::set_map(const std::shared_ptr<uint8_t[]> &m, int w, int l) {
    grid.initializeGraph(m, w, l);
    initialize_graph_ = false;
}

void DFMPlanner::set_goal(const Position &point) {
    Node new_goal(point);
    if (grid.goal_node_ != new_goal)
        new_goal_ = true;
    grid.setGoal(new_goal);
    goal_set_ = true;
}

bool DFMPlanner::isVertex(const Position &p) {
    bool is_vertex = (ceilf(p.x) == p.x) && (ceilf(p.y) == p.y);
    bool satisfies_bounds = grid.isValid(p);
    return is_vertex && satisfies_bounds;
}

DFMPlanner::Queue::Key DFMPlanner::calculateKey(const Cell &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return calculateKey(s, g, rhs);
}

DFMPlanner::Queue::Key DFMPlanner::calculateKey(const Cell &s, float g, float rhs) {
    return calculateKey(s, std::min(g, rhs));
}

DFMPlanner::Queue::Key DFMPlanner::calculateKey(const Cell &s, float cost_so_far) {
    return {cost_so_far, cost_so_far};
}

DFMPlanner::Queue::Key DFMPlanner::calculateHeurKey(const Cell &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return calculateHeurKey(s, g, rhs);
}

DFMPlanner::Queue::Key DFMPlanner::calculateHeurKey(const Cell &s, float g, float rhs) {
    return calculateHeurKey(s, std::min(g, rhs));
}

DFMPlanner::Queue::Key DFMPlanner::calculateHeurKey(const Cell &s, float cost_so_far) {
    auto dist = grid.start_cell_.distance(s);
    return {cost_so_far + heuristic_multiplier * dist + 255, cost_so_far};
}

void DFMPlanner::initializeSearch() {
    num_cells_updated = 0;
    num_nodes_expanded = 0;
    map.clear();
    priority_queue.clear();
    grid.updated_cells_.clear();
    map.insert_or_assign(grid.start_cell_, INFINITY, INFINITY);
    map.insert_or_assign(grid.goal_cell_, INFINITY, 0);
    priority_queue.insert(grid.goal_cell_, calculateKey(grid.goal_cell_, 0));
    num_cells_updated = 1;
}

bool DFMPlanner::end_condition() {
    auto top_key = priority_queue.topKey();
    auto[g, rhs] = map.getGandRHS(grid.start_cell_);
    //auto [k1,k2]=calculateKey(grid.start_cell_, g, rhs);
    //std::cout<< "START KEY " << k1 << " " << k2<< std::endl;
    if ((top_key < calculateKey(grid.start_cell_, g, rhs)) or (rhs != g)) {
        return false;
    }
    return true; //STOP: all 4 conditions met
}

unsigned long DFMPlanner::computeShortestPath() {

    //TODO check initial point for traversability

    int expanded = 0;
    while ((not priority_queue.empty()) and not end_condition()) {
        // Pop head of queue
        Cell s = priority_queue.topValue();
#ifdef VERBOSE_EXTRACTION
        Queue::Key k = priority_queue.topKey();
        std::cout << "POP    " << s.x << " " << s.y << "   KEY " << k.first << " " << k.second << std::endl;
#endif
        priority_queue.pop();
        ++expanded;
        // Get reference to the node
        auto s_it = map.find(s);
        assert(s_it != map.end());

        if (G(s_it) > RHS(s_it)) { // Overconsistent
            G(s_it) = RHS(s_it);
            for (const Cell &nbr : grid.neighbors_8(s))
                updateCellDecreasedNeighbor(nbr, s);
        } else { // Underconsistent
            G(s_it) = INFINITY;
            for (const Cell &nbr : grid.neighbors_8(s))
                updateCell(nbr);
            updateCell(s);
        }
    }
    num_nodes_expanded = expanded;
    std::cout << num_nodes_expanded << " nodes expanded" << std::endl;
    return num_nodes_expanded;
}

void DFMPlanner::updateCellDecreasedNeighbor(const Cell &cell, const Cell &nbr) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_cell_)
        RHS(s_it) = std::min(RHS(s_it), computeOptimalCostDecreasedNeighbor(cell, nbr));

    enqueueIfInconsistent(s_it);
}

float DFMPlanner::computeOptimalCostDecreasedNeighbor(const Cell &c, const Cell &ca1) {
    float tau = grid.getCost(c);
    if (tau == INFINITY) return INFINITY;

    int dx = ca1.x - c.x;
    int dy = ca1.y - c.y;

    float g_a_1 = map.getG(ca1);
    float g_b_1;
    Cell cb1;

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
    if (g_a_1 > g_b_1) {
        std::swap(g_a_1, g_b_1);
    }
    float h = HYPOT(dx, dy);
    if (g_a_1 == INFINITY and g_b_1 == INFINITY) {
        stencil_cost = INFINITY;
    } else if ((tau * h) > (g_b_1 - g_a_1)) {
        stencil_cost = (g_a_1 + g_b_1 + std::sqrt(2 * SQUARE(tau * h) - SQUARE(g_b_1 - g_a_1))) / 2.0f;
    } else {
        stencil_cost = g_a_1 + tau * h;
    }

    return stencil_cost;
}

void DFMPlanner::updateCell(const Cell &cell) {
    auto s_it = map.find_or_init(cell);

    if (cell != grid.goal_cell_)
        RHS(s_it) = computeOptimalCost(cell);

    enqueueIfInconsistent(s_it);
}

float DFMPlanner::computeOptimalCost(const Cell &c) {
    float tau = grid.getCost(c);
    if (tau == INFINITY) return INFINITY;

    float stencil_ortho_cost = INFINITY;
    float stencil_diago_cost = INFINITY;

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
    if (g_a_1 > g_b_1) std::swap(g_a_1, g_b_1);
    if (g_a_1 == INFINITY and g_b_1 == INFINITY) {
        stencil_ortho_cost = INFINITY;
    } else if (tau > (g_b_1 - g_a_1)) {
        stencil_ortho_cost = (g_a_1 + g_b_1 + std::sqrt(2 * SQUARE(tau) - SQUARE(g_b_1 - g_a_1))) / 2.0f;
    } else {
        stencil_ortho_cost = g_a_1 + tau;
    }

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
    if (g_c_1 > g_d_1) std::swap(g_c_1, g_d_1);
    if (g_c_1 == INFINITY and g_d_1 == INFINITY) {
        stencil_diago_cost = INFINITY;
    } else if (tau > (g_d_1 - g_c_1)) {
        stencil_diago_cost = (g_c_1 + g_d_1 + std::sqrt(2 * SQUARE(tau * SQRT2) - SQUARE(g_d_1 - g_c_1))) / 2.0f;
    } else {
        stencil_diago_cost = g_c_1 + tau * SQRT2;
    }

    return std::min(stencil_diago_cost, stencil_ortho_cost);
}

std::pair<Cell, float> DFMPlanner::minCost(const Cell &a, const Cell &b) {
    auto ca = map.getG(a), cb = map.getG(b);
    if (ca < cb)
        return {a, ca};
    else
        return {b, cb};
}

unsigned long DFMPlanner::updateCells() {
    for (const Cell &cell : grid.updated_cells_) {
        updateCell(cell);
    }

    num_cells_updated = grid.updated_cells_.size();
    std::cout << num_cells_updated << " cells updated" << std::endl;
    return num_cells_updated;
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

std::tuple<float, float> DFMPlanner::interpolateGradient(const Position &c) {
    // Coords of nearest node
    int x2 = (int) std::lround(c.x);
    int y2 = (int) std::lround(c.y);
    int x1 = x2 - 1;
    int y1 = y2 - 1;
    assert(c.x >= (x1 + 0.5f) && c.x <= (x2 + 0.5f));
    assert(c.y >= (y1 + 0.5f) && c.y <= (y2 + 0.5f));

    if (x2 == grid.length_) {
        --x1, --x2;
    } else if (x2 == 0) {
        ++x1, ++x2;
    }

    if (y2 == grid.width_) {
        --y1, --y2;
    } else if (y2 == 0) {
        ++y1, ++y2;
    }

    auto[blv, blh] = gradientAtCell({x1, y1});
    auto[tlv, tlh] = gradientAtCell({x1, y2});
    auto[brv, brh] = gradientAtCell({x2, y1});
    auto[trv, trh] = gradientAtCell({x2, y2});
#ifdef VERBOSE_EXTRACTION
    std::cout << "Interpolating gradient for (" << c.x << ", " << c.y << ")" << std::endl;
    std::cout << "Cells (x,y):\n"
              << "\t(" << x1 << ", " << y1 << ")" << "\t\t(" << x1 << ", " << y2 << ")\n\n"
              << "\t(" << x2 << ", " << y1 << ")" << "\t\t(" << x2 << ", " << y2 << ")\n";
    std::cout << "Gradients (dx,dy):\n"
              << "\t(" << blh << ", " << blv << ")" << "\t\t(" << tlh << ", " << tlv << ")\n\n"
              << "\t(" << brh << ", " << brv << ")" << "\t\t(" << trh << ", " << trv << ")\n";
    std::cout << "G-values (u):\n"
              << "\t" << map.getG({x1 - 1, y1 - 1}) << "\t\t" << map.getG({x1 - 1, y1}) << "\t\t"
              << map.getG({x1 - 1, y2}) << "\t\t" << map.getG({x1 - 1, y2 + 1}) << "\n"
              << "\t" << map.getG({x1, y1 - 1}) << "\t\t" << map.getG({x1, y1}) << "\t\t" << map.getG({x1, y2})
              << "\t\t" << map.getG({x1, y2 + 1}) << "\n"
              << "\t" << map.getG({x2, y1 - 1}) << "\t\t" << map.getG({x2, y1}) << "\t\t" << map.getG({x2, y2})
              << "\t\t" << map.getG({x2, y2 + 1}) << "\n"
              << "\t" << map.getG({x2 + 1, y1 - 1}) << "\t\t" << map.getG({x2 + 1, y1}) << "\t\t"
              << map.getG({x2 + 1, y2}) << "\t\t" << map.getG({x2 + 1, y2 + 1}) << "\n";
    std::cout << "RHS-values (u):\n"
              << "\t" << map.getRHS({x1 - 1, y1 - 1}) << "\t\t" << map.getRHS({x1 - 1, y1}) << "\t\t"
              << map.getRHS({x1 - 1, y2}) << "\t\t" << map.getRHS({x1 - 1, y2 + 1}) << "\n"
              << "\t" << map.getRHS({x1, y1 - 1}) << "\t\t" << map.getRHS({x1, y1}) << "\t\t" << map.getRHS({x1, y2})
              << "\t\t" << map.getRHS({x1, y2 + 1}) << "\n"
              << "\t" << map.getRHS({x2, y1 - 1}) << "\t\t" << map.getRHS({x2, y1}) << "\t\t" << map.getRHS({x2, y2})
              << "\t\t" << map.getRHS({x2, y2 + 1}) << "\n"
              << "\t" << map.getRHS({x2 + 1, y1 - 1}) << "\t\t" << map.getRHS({x2 + 1, y1}) << "\t\t"
              << map.getRHS({x2 + 1, y2}) << "\t\t" << map.getRHS({x2 + 1, y2 + 1}) << "\n";
#endif
    //Interpolate WRT centers of the 4 nearest cells (thus, shift 0.5f)
    auto sgx = BilinearInterpolation(blh, tlh, brh, trh,
                                     x1 + 0.5f, x2 + 0.5f, y1 + 0.5f, y2 + 0.5f,
                                     c.x, c.y);
    auto sgy = BilinearInterpolation(blv, tlv, brv, trv,
                                     x1 + 0.5f, x2 + 0.5f, y1 + 0.5f, y2 + 0.5f,
                                     c.x, c.y);
    assert(not std::isnan(sgx));
    assert(not std::isnan(sgy));
    return {sgx, sgy};
}

std::tuple<float, float> DFMPlanner::gradientAtCell(const Cell __c) {
    float g = map.getG(__c);
    Cell t = __c.topCell(), b = __c.bottomCell(), l = __c.leftCell(), r = __c.rightCell();
    float v_pre = map.getG(t);
    float v_post = map.getG(b);
    float h_pre = map.getG(l);
    float h_post = map.getG(r);
    std::cout << g << std::endl;
    std::cout << v_pre << " " << v_post << " " << h_pre << " " << h_post << " " << std::endl;

    if (g == INFINITY) {
        return {0, 0};
    }
    std::cout << __c.x << " " << __c.y;
    float dx, dy;
    if (grid.isValid(t) and v_pre < INFINITY) {
        if (grid.isValid(b) and v_post < INFINITY) {
            dy = (-0.5f) * v_pre + (0.5f) * v_post; // df/dy (second order of accuracy, centered)
            std::cout << " " << dy << " A";
        } else {
            dy = std::max(-v_pre + g, 0.f); // df/dy (first order of accuracy, backward)
            std::cout << " " << dy << " B";
        }
    } else if (grid.isValid(b) and v_post < INFINITY) {
        dy = std::min(-g + v_post, 0.f); // df/dy (first order of accuracy, forward)
        std::cout << " " << dy << " C";
    } else {
        dy = 0;
        std::cout << " " << dy << " D";
    }

    if (grid.isValid(l) and h_pre < INFINITY) {
        if (grid.isValid(r) and h_post < INFINITY) {
            dx = (-0.5f) * h_pre + (0.5f) * h_post; // df/dx (second order of accuracy, centered)
            std::cout << " " << dx << " A";
        } else {
            dx = std::max(-h_pre + g, 0.f); // df/dx (first order of accuracy, backward)
            std::cout << " " << dx << " B";
        }
    } else if (grid.isValid(r) and h_post < INFINITY) {
        dx = std::min(-g + h_post, 0.f); // df/dx (first order of accuracy, forward)
        std::cout << " " << dx << " C";
    } else {
        dx = 0;
        std::cout << " " << dx << " D";
    }
    std::cout << std::endl;

    assert(!std::isnan(dx));
    assert(!std::isnan(dy));
    float abs = std::hypot(dx, dy);

    if (abs > 0) {
        dx = dx / abs;
        dy = dy / abs;
    }
    return {dx, dy};
}


void DFMPlanner::enqueueIfInconsistent(Map::iterator it) {
    if (G(it) != RHS(it))
        priority_queue.insert_or_update(ELEM(it), calculateKey(ELEM(it), G(it), RHS(it)));
    else
        priority_queue.remove_if_present(ELEM(it));
}

void DFMPlanner::computeRoughtPath(bool eight_if_true) {
    path_.clear();
    path_.clear();
    cost_.clear();
    total_cost = 0;
    total_dist = 0;
    std::vector<Cell> path_cells_;
    cost_.push_back(0);
    path_cells_.push_back(grid.start_cell_);
    while (path_cells_.back() != grid.goal_cell_) {
        float min_cost = 0;
        Cell min_cell;
        float dist = INFINITY;
        for (auto &c: (eight_if_true ? grid.neighbors_8(path_cells_.back()) : grid.neighbors_4(path_cells_.back()))) {
            float cost = map.getRHS(path_cells_.back()) - map.getRHS(c);
            if (cost > min_cost) {
                min_cost = cost,
                        min_cell = c;
                dist = c.distance(path_cells_.back());
                assert(dist <= SQRT2);
            }
        }
        total_cost += min_cost;
        total_dist += dist;
        path_cells_.push_back(min_cell);
        cost_.push_back(min_cost);
    }
    path_.push_back(grid.start_pos_);
    std::transform(path_cells_.begin(), path_cells_.end(), std::back_inserter(path_),
                   [](const Cell &c) { return Position(c.x + 0.5f, c.y + 0.5f); });
    path_.push_back(grid.goal_pos_);
    cost_.push_back(0);
}

void DFMPlanner::constructOptimalPath() {
    path_.clear();
    cost_.clear();
    total_cost = 0;
    total_dist = 0;

    //auto[gh, gv] = costMapGradient();
    path_.push_back(grid.start_pos_);

    float min_cost = 0;
    int curr_step = 0;
    float step_cost;
    // TODO do something better than this sh*t
    int max_steps = 3000;

    float alpha = 0.1;
    auto s = grid.start_pos_;
    while (grid.goal_pos_.distance(s) > 0.8) {
        if (curr_step > max_steps) break;
        auto[sgx, sgy] = interpolateGradient(s);
#ifdef VERBOSE_EXTRACTION
        std::sprintf(buf, "s = [ %7.4f %7.4f]", s.x, s.y);
        std::cout << "Step " << curr_step << " " << buf << std::flush;
        std::sprintf(buf, " g = [ %7.4f %7.4f]", sgx, sgy);
        std::cout << buf << std::endl;
#endif
        if (std::abs(sgx) < 0.0001 && std::abs(sgy) < 0.0001)
            break;
        s = Position(s.x - alpha * sgx, s.y - alpha * sgy);
        total_dist += path_.back().distance(s);
        auto pp = grid.getGridBoundariesTraversals(path_.back(), s);
        step_cost = computePathAdditionsCost(pp);
        total_cost += step_cost;
        path_.push_back(s);
        cost_.push_back(step_cost);
        ++curr_step;
        if (curr_step == 347) return;
    }
    auto pp = grid.getGridBoundariesTraversals(s, grid.goal_pos_);
    step_cost = computePathAdditionsCost(pp);
    total_cost += step_cost;
    total_dist += s.distance(grid.goal_pos_);
    path_.push_back(grid.goal_pos_);
    cost_.push_back(step_cost);
    //std::reverse(path_.begin(), path_.end());

    if (min_cost == INFINITY) {
        std::cerr << "[Extraction] No valid path exists" << std::endl;
    } else if (curr_step >= max_steps) {
        std::cerr << "[Extraction] Maximum step number reached" << std::endl;
    }

    std::cout << "Found path. Cost: " << total_cost << " Distance: " << total_dist << std::endl;
}

float DFMPlanner::computePathAdditionsCost(const std::vector<Position> &p) {
    float cost = 0;
    for (auto a = p.begin(); a < p.end() - 1; ++a) {
        auto b = a + 1;
        Position m = {(a->x + b->x) / 2, (a->y + b->y) / 2};
        // It is possible to have segments along edges (midpoint on edge)
        float weight;
        if (floorf(m.x) == m.x) {
            weight = std::min(
                    grid.getCost(Node((int) m.x, (int) ceilf(m.y)).neighborCell(false, false)),
                    grid.getCost(Node((int) m.x, (int) ceilf(m.y)).neighborCell(true, false)));
        } else if (floorf(m.y) == m.y) {
            weight = std::min(
                    grid.getCost(Node((int) ceilf(m.x), (int) m.y).neighborCell(false, false)),
                    grid.getCost(Node((int) ceilf(m.x), (int) m.y).neighborCell(false, true)));
        } else {
            weight = grid.getCost(Cell(m.x, m.y));
        }
        cost += weight * a->distance(*b);
        //assert(cost != INFINITY);
        assert(!std::isnan(cost));
        assert(cost > 0);

    }
    return cost;
}

bool DFMPlanner::goalReached(const Position &p) {
    return grid.goal_cell_.x == p.x && grid.goal_cell_.y == p.y;
}

bool DFMPlanner::consistent(const Cell &s) {
    auto[g, rhs] = map.getGandRHS(s);
    return g == rhs;
}

bool DFMPlanner::consistent(const Map::iterator &it) {
    return G(it) == RHS(it);
}

void DFMPlanner::set_start(const Position &pos) {
    grid.setStart(pos);
    start_nodes =
            {Node(pos).cellTopRight(), Node(pos).cellTopLeft(), Node(pos).cellBottomRight(),
             Node(pos).cellBottomLeft()};
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

float DFMPlanner::getInterpG(const Node &node) {
    //return getInterpRHS(node);
    return map.getG(Cell(node.x, node.y));
    std::vector<float> gs;
    gs.push_back(map.getG(node.cellTopLeft()));
    gs.push_back(map.getG(node.cellTopRight()));
    gs.push_back(map.getG(node.cellBottomLeft()));
    gs.push_back(map.getG(node.cellBottomRight()));
    float sum = 0;
    int count = 0;
    for (auto val: gs) {
        if (val < INFINITY) {
            sum += val;
            ++count;
        }
    }
    if (count == 0) return INFINITY;
    assert((sum / count) > 0);
    return sum / count;
}

float DFMPlanner::getInterpRHS(const Node &node) {
    std::vector<float> gs;
    gs.push_back(map.getRHS(node.cellTopLeft()));
    gs.push_back(map.getRHS(node.cellTopRight()));
    gs.push_back(map.getRHS(node.cellBottomLeft()));
    gs.push_back(map.getRHS(node.cellBottomRight()));
    float sum = 0;
    int count = 0;
    for (auto val: gs) {
        if (val < INFINITY) {
            sum += val;
            ++count;
        }
    }
    if (count == 0) return INFINITY;
    assert((sum / count) > 0);
    return sum / count;
}

void DFMPlanner::computeInterpolatedPath() {
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
            step_dist += p->distance(*(p + 1));
        }
        total_cost += step_cost;
        total_dist += step_dist;
        curr_step += 1;
        last = path_.back();
    } while (!goalReached(last) && (min_cost != INFINITY) &&
             (curr_step < max_steps));

    if (min_cost == INFINITY) {
        std::cerr << "[Extraction] No valid path exists" << std::endl;
        //path_.clear();
    } else if (curr_step >= max_steps) {
        std::cerr << "[Extraction] Maximum step number reached" << std::endl;
        //path_.clear();
    }
    std::cout << "Found path. Cost: " << total_cost << " Distance: " << total_dist << std::endl;
}

//TODO consider p0, p1 p2 center of cells. Interpolate traversabilities
// for obstacles, it is possible to still use a traversable average, since
// at least one of s1 and s2 will have g=inf (thus interpolation degenerates towards
// the other one, which should be limited).
void DFMPlanner::getBC(TraversalParams &t) {
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

    //t.b = std::accumulate(b.begin(), b.end(), 0)/4;
    //t.c = std::accumulate(c.begin(), c.end(), 0)/4;

    //t.b = *std::max_element(b.begin(), b.end());
    //t.c = *std::max_element(c.begin(), c.end());

    t.b = *std::min_element(b.begin(), b.end());
    t.c = *std::min_element(c.begin(), c.end());
}

path_additions DFMPlanner::traversalFromCorner(const Position &p,
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
    cell.g1 = map.getRHS(Cell(cell.p1));
    cell.g2 = map.getRHS(Cell(cell.p2));
    getBC(cell);

    return InterpolatedTraversal::traversalFromCorner(cell, step_cost);
}

path_additions DFMPlanner::traversalFromContiguousEdge(const Position &p,
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

    cell1.g1 = map.getRHS(Cell(cell1.p1));
    cell1.g2 = map.getRHS(Cell(cell1.p2));
    getBC(cell1);
    cell1.q = 1 - std::abs(cell1.p1.y - p.y) - std::abs(cell1.p1.x - p.x);

    return InterpolatedTraversal::traversalFromContiguousEdge(cell1, step_cost);
}

path_additions DFMPlanner::traversalFromOppositeEdge(const Position &p,
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

    cell1.g1 = cell2.g2 = map.getRHS(Cell(cell1.p1));
    cell1.g2 = cell2.g1 = map.getRHS(Cell(cell1.p2));

    getBC(cell1);
    getBC(cell2);
    cell1.p = std::abs(p.y - cell1.p0.y) + std::abs(p.x - cell1.p0.x);
    cell2.p = 1 - cell1.p;

    return InterpolatedTraversal::traversalFromOppositeEdge(cell1, cell2, step_cost);
}

path_additions DFMPlanner::traversalFromEdge(const Position &p,
                                             const Node &p_a,
                                             const Node &p_b,
                                             float &step_cost) {

    assert(!isVertex(p));

    bool cond_1 = p.aligned(p_a);
    bool cond_2 = p.aligned(p_b);
    if (cond_1 || cond_2) {
        assert(cond_1 xor cond_2);
        return traversalFromContiguousEdge(p, p_a, p_b, step_cost);
    } else {
        return traversalFromOppositeEdge(p, p_a, p_b, step_cost);
    }
}

path_additions DFMPlanner::getPathAdditions(const Position &p,
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
                  << ", G:" << getInterpG(p_a) << ", RHS:" << getInterpRHS(p_a) << " | "
                  << "X:" << p_b.x << ", Y:" << p_b.y
                  << ", G:" << getInterpG(p_b) << ", RHS:" << getInterpRHS(p_b)
                  << " || cost: " << temp_pa.cost_to_goal << std::endl;
        for (const auto &addition: temp_pa.steps) {
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
    for (const auto &addition: min_pa.steps) {
        if (lookahead and not do_lookahead) std::cout << "\t";
        std::cout << "step  " << std::to_string(addition.x) << ", " << std::to_string(addition.y) << std::endl
                  << std::endl;
    }
#endif
    return min_pa;
}

