#include "Graph.h"
#include <cmath>
#include <utility>
#include <algorithm>

void Graph::set_start(const Position &start) {
    start_pos_ = start;
    start_cell_ = Cell(start);
    start_node_ = Node(start);
}

void Graph::set_goal(const Position &goal) {
    goal_pos_ = goal;
    goal_cell_ = Cell(goal);
    goal_node_ = Node(goal);
}

void Graph::set_occupancy_threshold(float occupancy_threshold) {
    occupancy_threshold_uchar_ = occupancy_threshold * 255.0f;
}

void Graph::init(std::shared_ptr<uint8_t> image, int width, int length) {
    length_ = length;
    width_ = width;
    flength_ = static_cast<float>(length);
    fwidth_ = static_cast<float>(width);
    size_ = length_ * width_;
    map_ = std::move(image);
}

uint8_t &Graph::get(int x, int y) {
    return map_.get()[x * width_ + y];
}

void Graph::update(const std::shared_ptr<uint8_t> &patch, int x, int y, int w, int h) {
    updated_cells_.clear();
    assert(x >= 0);
    assert(y >= 0);
    assert((x + h - 1) < length_);
    assert((y + w - 1) < width_);

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            auto &mv = get(i + x, j + y);
            auto pv = patch.get()[i * w + j];
            if (mv != pv) updated_cells_.emplace_back(x + i, y + j);
            mv = pv;
        }
    }
}

bool Graph::is_valid(const Node &s) const {
    return (s.x <= length_) && (s.y <= width_) && (s.x >= 0) && (s.y >= 0);
}

bool Graph::is_valid(const Position &p) const {
    return (p.x >= 0.0f) && (p.x <= flength_) && (p.y >= 0.0f) && (p.y <= fwidth_);
}

bool Graph::is_valid(const Cell &c) const {
    return (c.x >= 0) && (c.x < length_) && (c.y >= 0) && (c.y < width_);
}

bool Graph::is_valid_vertex(const Position &p) const {
    bool is_vertex = (ceilf(p.x) == p.x) && (ceilf(p.y) == p.y);
    bool satisfies_bounds = is_valid(p);
    return is_vertex && satisfies_bounds;
}

std::vector<Node> Graph::neighbors_8(const Node &s, bool include_invalid) const {
    std::vector<Node> neighbors{
        s.top_node(), s.top_left_node(), s.left_node(), s.bottom_left_node(),
        s.bottom_node(), s.bottom_right_node(), s.right_node(), s.top_right_node()
    };

    if (not include_invalid)
        neighbors.erase(
            std::remove_if(neighbors.begin(), neighbors.end(),
                           [&](Node const &p) { return not is_valid(p); }),
            neighbors.end()
        );

    return neighbors;
}

std::vector<Node> Graph::neighbors_4(const Node &s, bool include_invalid) const {
    std::vector<Node> neighbors{
        s.top_node(), s.left_node(),
        s.bottom_node(), s.right_node()
    };

    if (not include_invalid)
        neighbors.erase(
            std::remove_if(neighbors.begin(), neighbors.end(),
                           [&](Node const &p) { return not is_valid(p); }),
            neighbors.end()
        );

    return neighbors;
}

std::vector<Node> Graph::neighbors_diag_4(const Node &s, bool include_invalid) const {
    std::vector<Node> neighbors{
        s.top_left_node(), s.bottom_left_node(),
        s.top_right_node(), s.bottom_right_node()
    };

    if (not include_invalid)
        neighbors.erase(
            std::remove_if(neighbors.begin(), neighbors.end(),
                           [&](Node const &p) { return not is_valid(p); }),
            neighbors.end()
        );

    return neighbors;
}

std::vector<Cell> Graph::neighbors_8(const Cell &s, bool include_invalid) const {
    std::vector<Cell> neighbors{
        s.top_cell(), s.top_left_cell(), s.left_cell(), s.bottom_left_cell(),
        s.bottom_cell(), s.bottom_right_cell(), s.right_cell(), s.top_right_cell()
    };

    if (not include_invalid)
        neighbors.erase(
            std::remove_if(neighbors.begin(), neighbors.end(),
                           [&](Cell const &p) { return not is_valid(p); }),
            neighbors.end()
        );

    return neighbors;
}

std::vector<Cell> Graph::neighbors_4(const Cell &s, bool include_invalid) const {
    std::vector<Cell> neighbors{
        s.top_cell(), s.left_cell(),
        s.bottom_cell(), s.right_cell()
    };

    if (not include_invalid)
        neighbors.erase(
            std::remove_if(neighbors.begin(), neighbors.end(),
                           [&](Cell const &p) { return not is_valid(p); }),
            neighbors.end()
        );

    return neighbors;
}

std::vector<Edge> Graph::consecutive_neighbors(const Position &p) const {
    std::vector<Node> neighbors;
    std::vector<Edge> consecutive_neighbors;

    float intpartx, intparty, decpartx, decparty;
    decpartx = std::modf(p.x, &intpartx);
    decparty = std::modf(p.y, &intparty);
    if (0.0f < decpartx and decpartx < 1.0f) { //non-integer, lies on horizontal edge - 6 neighbors (2 cells)
        neighbors.reserve(6);
        consecutive_neighbors.reserve(6);
        neighbors.emplace_back(intpartx, intparty);   // left
        neighbors.emplace_back(intpartx, intparty - 1.0f);  // bottom left
        neighbors.emplace_back(intpartx + 1.0f, intparty - 1.0f); // bottom right
        neighbors.emplace_back(intpartx + 1.0f, intparty);  // right
        neighbors.emplace_back(intpartx + 1.0f, intparty + 1.0f); // top right
        neighbors.emplace_back(intpartx, intparty + 1.0f);  // top left
    } else if (0.0f < decparty and decparty < 1.0f) { //non-integer, lies on vertical edge - 6 neighbors (2 cells)
        neighbors.reserve(6);
        consecutive_neighbors.reserve(6);
        neighbors.emplace_back(intpartx, intparty);   // bottom
        neighbors.emplace_back(intpartx + 1.0f, intparty);  // bottom right
        neighbors.emplace_back(intpartx + 1.0f, intparty+1.0f); // top right
        neighbors.emplace_back(intpartx, intparty+1.0f);  // top
        neighbors.emplace_back(intpartx - 1.0f, intparty+1.0f);         // top left
        neighbors.emplace_back(intpartx - 1.0f, intparty);  // bottom left
    } else { // 8 neighbors (4 cells)
        neighbors.reserve(8);
        consecutive_neighbors.reserve(8);
        neighbors.emplace_back(intpartx + 1.0f, intparty);         // right
        neighbors.emplace_back(intpartx + 1.0f, intparty + 1.0f);  // top right
        neighbors.emplace_back(intpartx, intparty + 1.0f);         // top
        neighbors.emplace_back(intpartx - 1.0f, intparty + 1.0f);  // top left
        neighbors.emplace_back(intpartx - 1.0f, intparty);         // left
        neighbors.emplace_back(intpartx - 1.0f, intparty - 1.0f);  // bottom left
        neighbors.emplace_back(intpartx, intparty - 1.0f);         // bottom
        neighbors.emplace_back(intpartx + 1.0f, intparty - 1.0f);  // bottom right
    }

    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (is_valid(neighbors[i])) {
            if (is_valid(neighbors[(i + 1) % neighbors.size()])) {
                consecutive_neighbors.emplace_back(neighbors[i], neighbors[(i + 1) % neighbors.size()]);
            } else {
                ++i; //next edge is also invalid, skip
            }
        }
    }

    return consecutive_neighbors;
}

std::vector<Edge> Graph::consecutive_neighbors(const Node &s) const {
    std::vector<Node> neighbors;
    std::vector<Edge> consecutive_neighbors;
    neighbors.reserve(8);
    consecutive_neighbors.reserve(8);

    int intpartx = s.x, intparty = s.y;

    neighbors.emplace_back(intpartx + 1, intparty);      // right
    neighbors.emplace_back(intpartx + 1, intparty + 1);  // top right
    neighbors.emplace_back(intpartx, intparty + 1);      // top
    neighbors.emplace_back(intpartx - 1, intparty + 1);  // top left
    neighbors.emplace_back(intpartx - 1, intparty);      // left
    neighbors.emplace_back(intpartx - 1, intparty - 1);  // bottom left
    neighbors.emplace_back(intpartx, intparty - 1);      // bottom
    neighbors.emplace_back(intpartx + 1, intparty - 1);  // bottom right

    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (is_valid(neighbors[i])) {
            if (is_valid(neighbors[(i + 1) % neighbors.size()])) {
                consecutive_neighbors.emplace_back(neighbors[i], neighbors[(i + 1) % neighbors.size()]);
            } else {
                ++i; //next edge is also invalid, skip
            }
        }
    }

    return consecutive_neighbors;
}

optional<Node> Graph::ccw_neighbor(const Node &s, const Node &s_prime) const {
    int delta_x = s_prime.x - s.x + 1;
    int delta_y = s_prime.y - s.y + 1;

    static const int8_t lut_x_ccw[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};
    static const int8_t lut_y_ccw[3][3] = {{0, 1, 1}, {-1, 0, 1}, {-1, -1, 0}};

    int new_x = s.x + lut_x_ccw[delta_x][delta_y];
    int new_y = s.y + lut_y_ccw[delta_x][delta_y];

    Node ccw_neighbor(new_x, new_y);
    if (is_valid(ccw_neighbor)) return ccw_neighbor;
    else return nullopt;
}

optional<Node> Graph::cw_neighbor(const Node &s, const Node &s_prime) const {
    int delta_x = s_prime.x - s.x + 1;
    int delta_y = s_prime.y - s.y + 1;

    static const int8_t lut_x_cw[3][3] = {{0, -1, -1}, {1, 0, -1}, {1, 1, 0}};
    static const int8_t lut_y_cw[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};

    int new_x = s.x + lut_x_cw[delta_x][delta_y];
    int new_y = s.y + lut_y_cw[delta_x][delta_y];

    Node cw_neighbor(new_x, new_y);
    if (is_valid(cw_neighbor)) return cw_neighbor;
    else return nullopt;
}

float Graph::get_cost(const Cell &ind) const {
    if (!is_valid(ind))
        return INFINITY;
    auto cost = map_.get()[ind.x * width_ + ind.y];
    assert(cost>0);
    return (cost >= occupancy_threshold_uchar_) ? INFINITY : (float) cost;
}

Cell Graph::get_cell(const Node &a, const Node &b, const Node &c) {
    //Given 3 nodes around a cell, cell coords are min(xs) min(ys)
    int x = std::min(a.x, std::min(b.x, c.x));
    int y = std::min(a.y, std::min(b.y, c.y));
    Cell cell(x, y);
    assert(cell.has_node(a));
    assert(cell.has_node(b));
    assert(cell.has_node(c));
    return cell;
}

std::vector<Position> Graph::get_grid_boundaries_traversals(const Position &a, const Position &b) {
    std::vector<Position> xsplit;
    const Position &low_x = a.x < b.x ? a : b;
    const Position &high_x = a.x < b.x ? b : a;
    float x_min = low_x.x;
    float x_max = high_x.x;
    float x_cur = std::floor(x_min + 1);
    xsplit.push_back(low_x);
    if ((b.x - a.x) != 0) {
        float m = (b.y - a.y) / (b.x - a.x);
        float q = a.y - m * a.x;
        while (x_cur < x_max) {
            xsplit.emplace_back(x_cur, x_cur * m + q);
            ++x_cur;
        }
    }
    xsplit.push_back(high_x);

    if (low_x.y > high_x.y) std::reverse(xsplit.begin(), xsplit.end());
    std::vector<Position> ysplit;
    for (auto xp = xsplit.begin(); xp < xsplit.end() - 1; ++xp) {
        const Position &low_y = *xp;
        const Position &high_y = *(xp + 1);
        float y_min = low_y.y;
        float y_max = high_y.y;
        float y_cur = std::floor(y_min + 1);
        ysplit.push_back(low_y);
        if ((b.x - a.x) != 0) {
            while (y_cur < y_max) {
                float m = (b.y - a.y) / (b.x - a.x);
                float q = a.y - m * a.x;
                ysplit.emplace_back((y_cur - q) / m, y_cur), ++y_cur;
            }
        } else {
            while (y_cur < y_max) {
                ysplit.emplace_back(a.x, y_cur), ++y_cur;
            }
        }
    }
    ysplit.push_back(xsplit.back());
    return ysplit;
}

