#include "Graph.h"
#include <cmath>
#include <utility>
#include <algorithm>

void Graph::setOccupancyThreshold(float occupancy_threshold) {
    occupancy_threshold_uchar_ = occupancy_threshold * 255.0f;
}

//TODO should be parameter of type Pos (most generic)
void Graph::setStart(const Position &start) {
    start_pos_ = start;
    start_cell_ = {(int) std::floor(start.x), (int) std::floor(start.y)};
    start_node_ = {(int) std::round(start.x), (int) std::round(start.y)};
}

void Graph::setGoal(const Position &goal) {
    goal_pos_ = goal;
    goal_cell_ = {(int) std::floor(goal.x), (int) std::floor(goal.y)};
    goal_node_ = {(int) std::round(goal.x), (int) std::round(goal.y)};
}

void Graph::initializeGraph(std::shared_ptr<uint8_t[]> image, int width, int length) {
    length_ = length;
    width_ = width;
    flength_ = static_cast<float>(length);
    fwidth_ = static_cast<float>(width);
    size_ = length_ * width_;
    map_ = std::move(image);
}

void Graph::updateGraph(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h) {
    updated_cells_.clear();  // clear the vector of cells that need updating
    assert(x >= 0);
    assert(y >= 0);
    assert((x + h - 1) < length_);
    assert((y + w - 1) < width_);

    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            if (map_.get()[(i + x) * width_ + (y + j)] != patch.get()[i * w + j]) {
                updated_cells_.emplace_back(x + i, y + j);
            }
            map_.get()[(i + x) * width_ + (y + j)] = patch.get()[i * w + j];
        }
    }
}

bool Graph::isValid(const Node &s) {
    return (s.x <= length_) && (s.y <= width_) && (s.x >= 0) && (s.y >= 0);
}

bool Graph::isValid(const Position &p) {
    return (p.x >= 0.0f) && (p.x <= flength_) && (p.y >= 0.0f) && (p.y <= fwidth_);
}

bool Graph::isValid(const Cell &c) {
    return (c.x >= 0) && (c.x < length_) && (c.y >= 0) && (c.y < width_);
}

std::vector<Node> Graph::neighbors_8(const Node &s, bool include_invalid) {
    std::vector<Node> neighbors;
    neighbors.reserve(8);

    //TODO reason: if top left is invalid, bottom right is valid (as an example)
    // maybe optimizable then?

    // right
    Node r(s.x + 1, s.y);
    if (include_invalid || isValid(r))
        neighbors.push_back(std::move(r));

    //TODO use Node methods or create missing ones eg. .topRightNode()
    // top right
    Node tr(s.x + 1, s.y + 1);
    if (include_invalid || isValid(tr))
        neighbors.push_back(std::move(tr));

    // above
    Node t(s.x, s.y + 1);
    if (include_invalid || isValid(t))
        neighbors.push_back(std::move(t));

    // top left
    Node tl(s.x - 1, s.y + 1);
    if (include_invalid || isValid(tl))
        neighbors.push_back(std::move(tl));

    // left
    Node l(s.x - 1, s.y);
    if (include_invalid || isValid(l))
        neighbors.push_back(std::move(l));

    // bottom left
    Node bl(s.x - 1, s.y - 1);
    if (include_invalid || isValid(bl))
        neighbors.push_back(std::move(bl));

    // bottom
    Node b(s.x, s.y - 1);
    if (include_invalid || isValid(b))
        neighbors.push_back(std::move(b));

    // bottom right
    Node br(s.x + 1, s.y - 1);
    if (include_invalid || isValid(br))
        neighbors.push_back(std::move(br));

    return neighbors;
}

std::vector<Edge> Graph::consecutiveNeighbors(const Position &p) {
    std::vector<Node> neighbors;
    std::vector<Edge> consecutive_neighbors;

    float intpartx, intparty, decpartx, decparty;
    decpartx = std::modf(p.x, &intpartx);
    decparty = std::modf(p.y, &intparty);
    if (0 < decpartx and decpartx < 1) { //non-integer, lies on horizontal edge - 6 neighbors (2 cells)
        neighbors.reserve(6);
        consecutive_neighbors.reserve(6);
        neighbors.emplace_back(std::floor(p.x), intparty);   // left
        neighbors.emplace_back(std::floor(p.x), intparty - 1);  // bottom left
        neighbors.emplace_back(std::ceil(p.x), intparty - 1); // bottom right
        neighbors.emplace_back(std::ceil(p.x), intparty);  // right
        neighbors.emplace_back(std::ceil(p.x), intparty + 1);         // top right
        neighbors.emplace_back(std::floor(p.x), intparty + 1);  // top left
    } else if (0 < decparty and decparty < 1) { //non-integer, lies on vertical edge - 6 neighbors (2 cells)
        neighbors.reserve(6);
        consecutive_neighbors.reserve(6);
        neighbors.emplace_back(intpartx, std::floor(p.y));   // bottom
        neighbors.emplace_back(intpartx + 1, std::floor(p.y));  // bottom right
        neighbors.emplace_back(intpartx + 1, std::ceil(p.y)); // top right
        neighbors.emplace_back(intpartx, std::ceil(p.y));  // top
        neighbors.emplace_back(intpartx - 1, std::ceil(p.y));         // top left
        neighbors.emplace_back(intpartx - 1, std::floor(p.y));  // bottom left
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
        if (isValid(neighbors[i])) {
            if (isValid(neighbors[(i + 1) % neighbors.size()])) {
                consecutive_neighbors.emplace_back(neighbors[i], neighbors[(i + 1) % neighbors.size()]);
            } else {
                ++i; //next edge is also invalid, skip
            }
        }
    }

    return consecutive_neighbors;
}

std::vector<Edge> Graph::consecutiveNeighbors(const Node &s) {
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
        if (isValid(neighbors[i])) {
            if (isValid(neighbors[(i + 1) % neighbors.size()])) {
                consecutive_neighbors.emplace_back(neighbors[i], neighbors[(i + 1) % neighbors.size()]);
            } else {
                ++i; //next edge is also invalid, skip
            }
        }
    }

    return consecutive_neighbors;
}

Node Graph::counterClockwiseNeighbor(const Node &s, const Node &sp) {
    int delta_x = sp.x - s.x + 1;
    int delta_y = sp.y - s.y + 1;

    static const int8_t lut_x_ccw[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};
    static const int8_t lut_y_ccw[3][3] = {{0, 1, 1}, {-1, 0, 1}, {-1, -1, 0}};

    int new_x = s.x + lut_x_ccw[delta_x][delta_y];
    int new_y = s.y + lut_y_ccw[delta_x][delta_y];

    Node cc_neighbor(new_x, new_y);
    return isValid(cc_neighbor) ? cc_neighbor : Node{false};
}

Node Graph::clockwiseNeighbor(const Node &s, const Node &sp) {
    int delta_x = sp.x - s.x + 1;
    int delta_y = sp.y - s.y + 1;

    static const int8_t lut_x_cw[3][3] = {{0, -1, -1}, {1, 0, -1}, {1, 1, 0}};
    static const int8_t lut_y_cw[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};

    int new_x = s.x + lut_x_cw[delta_x][delta_y];
    int new_y = s.y + lut_y_cw[delta_x][delta_y];

    Node c_neighbor(new_x, new_y);
    return isValid(c_neighbor) ? c_neighbor : Node{false};
}

float Graph::getTraversalCost(const Cell &c) {
    if (!isValid(c))
        return INFINITY;
    auto cost = map_[c.x * width_ + c.y];
    return (cost >= occupancy_threshold_uchar_) ? INFINITY : (float) cost;
}

std::vector<Cell> Graph::neighbors_4(const Cell &s, bool include_invalid) {
    std::vector<Cell> neighbors;
    neighbors.reserve(4);

    // right
    auto r = s.rightCell();
    if (include_invalid || isValid(r))
        neighbors.push_back(std::move(r));

    // above
    auto t = s.topCell();
    if (include_invalid || isValid(t))
        neighbors.push_back(std::move(t));

    // left
    auto l = s.leftCell();
    if (include_invalid || isValid(l))
        neighbors.push_back(std::move(l));

    // bottom
    auto b = s.bottomCell();
    if (include_invalid || isValid(b))
        neighbors.push_back(std::move(b));

    assert(neighbors.size() >= 2);
    return neighbors;
}

std::vector<Cell> Graph::neighbors_8(const Cell &s, bool include_invalid) {
    std::vector<Cell> neighbors;
    neighbors.reserve(8);

    // right
    auto r = s.rightCell();
    if (include_invalid || isValid(r))
        neighbors.push_back(std::move(r));

    auto tr = s.topRightCell();
    if (include_invalid || isValid(tr))
        neighbors.push_back(std::move(tr));

    // above
    auto t = s.topCell();
    if (include_invalid || isValid(t))
        neighbors.push_back(std::move(t));

    // above
    auto tl = s.topLeftCell();
    if (include_invalid || isValid(tl))
        neighbors.push_back(std::move(tl));

    // left
    auto l = s.leftCell();
    if (include_invalid || isValid(l))
        neighbors.push_back(std::move(l));

    auto bl = s.bottomLeftCell();
    if (include_invalid || isValid(bl))
        neighbors.push_back(std::move(bl));

    // bottom
    auto b = s.bottomCell();
    if (include_invalid || isValid(b))
        neighbors.push_back(std::move(b));

    auto br = s.bottomRightCell();
    if (include_invalid || isValid(br))
        neighbors.push_back(std::move(br));

    assert(neighbors.size() >= 2);
    return neighbors;
}

Cell Graph::getCell(const Node &a, const Node &b, const Node &c) {
    //Given 3 nodes around a cell, cell coords are min(xs) min(ys)
    int x = std::min(a.x, std::min(b.x, c.x));
    int y = std::min(a.y, std::min(b.y, c.y));
    Cell cell(x, y);
    assert(cell.hasNode(a));
    assert(cell.hasNode(b));
    assert(cell.hasNode(c));
    return cell;
}

std::vector<Position> Graph::getGridBoundariesTraversals(const Position &a, const Position &b) {
    std::vector<Position> xsplit;
    const Position &low_x = a.x < b.x ? a : b;
    const Position &high_x = a.x < b.x ? b : a;
    float x_min = low_x.x;
    float x_max = high_x.x;
    float x_cur = std::floor(x_min + 1);
    xsplit.push_back(low_x);

    if ((b.x - a.x) != 0) {
        //y=mx+q
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
