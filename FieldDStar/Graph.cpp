#include "Graph.h"
#include <cmath>

void Graph::setOccupancyThreshold(float occupancy_threshold) {
    this->occupancy_threshold_uchar_ = occupancy_threshold * 255.0f;
}

void Graph::setGoal(const Node& goal) {
    this->goal_ = goal;
}

void Graph::initializeGraph(const MapPtr &msg) {
    updated_cells_.clear();
    length_ = msg->length;
    width_ = msg->width;
    flength_ = static_cast<float>(msg->length);
    fwidth_ = static_cast<float>(msg->width);
    size_ = length_ * width_;
    start_ = Node(msg->x, msg->y);
    map_ = msg->image;
}

void Graph::updateGraph(const std::shared_ptr<uint8_t[]>& patch, int x, int y, int w, int h) {
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
/*
    for (int i = 0; i < width_; ++i) {
        for (int j = 0; j < length_; ++j) {
            std::cout << int(map_.get()[i*width_+j]) << " ";
        }
        std::cout << std::endl;
    }
*/
}

bool Graph::isValidNode(const Node &s) {
    return (s.x <= length_) && (s.y <= width_) && (s.x >= 0) && (s.y >= 0);
}

bool Graph::isValidPosition(const Position &p) {
    return (p.x >= 0.0f) && (p.x <= flength_) && (p.y >= 0.0f) && (p.y <= fwidth_);
}

bool Graph::isValidCell(const Cell &c) {
    return (c.x >= 0) && (c.x < length_) && (c.y >= 0) && (c.y < width_);
}

bool Graph::unaligned(const Node &s, const Node &sp) {
    return ((s.x != sp.x) && (s.y != sp.y));
}

bool Graph::unaligned(const Position &p, const Position &sp) {
    return ((p.x != sp.x) && (p.y != sp.y));
}

std::vector<Node> Graph::neighbors(const Node &s, bool include_invalid) {
    std::vector<Node> neighbors;
    neighbors.reserve(8);

    // right
    Node r(s.x + 1, s.y);
    if (include_invalid || isValidNode(r))
        neighbors.push_back(std::move(r));

    // top right
    Node tr(s.x + 1, s.y + 1);
    if (include_invalid || isValidNode(tr))
        neighbors.push_back(std::move(tr));

    // above
    Node t(s.x, s.y + 1);
    if (include_invalid || isValidNode(t))
        neighbors.push_back(std::move(t));

    // top left
    Node tl(s.x - 1, s.y + 1);
    if (include_invalid || isValidNode(tl))
        neighbors.push_back(std::move(tl));

    // left
    Node l(s.x - 1, s.y);
    if (include_invalid || isValidNode(l))
        neighbors.push_back(std::move(l));

    // bottom left
    Node bl(s.x - 1, s.y - 1);
    if (include_invalid || isValidNode(bl))
        neighbors.push_back(std::move(bl));

    // bottom
    Node b(s.x, s.y - 1);
    if (include_invalid || isValidNode(b))
        neighbors.push_back(std::move(b));

    // bottom right
    Node br(s.x + 1, s.y - 1);
    if (include_invalid || isValidNode(br))
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
        if (isValidNode(neighbors[i])) {
            if (isValidNode(neighbors[(i + 1) % neighbors.size()])) {
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

    int intpartx = 0, intparty = 0;
    std::tie(intpartx, intparty) = s;

    neighbors.emplace_back(intpartx + 1, intparty);      // right
    neighbors.emplace_back(intpartx + 1, intparty + 1);  // top right
    neighbors.emplace_back(intpartx, intparty + 1);      // top
    neighbors.emplace_back(intpartx - 1, intparty + 1);  // top left
    neighbors.emplace_back(intpartx - 1, intparty);      // left
    neighbors.emplace_back(intpartx - 1, intparty - 1);  // bottom left
    neighbors.emplace_back(intpartx, intparty - 1);      // bottom
    neighbors.emplace_back(intpartx + 1, intparty - 1);  // bottom right

    for (size_t i = 0; i < neighbors.size(); ++i) {
        if (isValidNode(neighbors[i])) {
            if (isValidNode(neighbors[(i + 1) % neighbors.size()])) {
                consecutive_neighbors.emplace_back(neighbors[i], neighbors[(i + 1) % neighbors.size()]);
            } else {
                ++i; //next edge is also invalid, skip
            }
        }
    }

    return consecutive_neighbors;
}

Node Graph::counterClockwiseNeighbor(const Node& s, const Node& sp) {
    int delta_x = sp.x - s.x + 1;
    int delta_y = sp.y - s.y + 1;

    static const int8_t lut_x_ccw[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};
    static const int8_t lut_y_ccw[3][3] = {{0, 1, 1}, {-1, 0, 1}, {-1, -1, 0}};

    int new_x = s.x + lut_x_ccw[delta_x][delta_y];
    int new_y = s.y + lut_y_ccw[delta_x][delta_y];

    Node cc_neighbor(new_x, new_y);
    return isValidNode(cc_neighbor) ? cc_neighbor : Node{false};
}

Node Graph::clockwiseNeighbor(const Node& s, const Node& sp) {
    int delta_x = sp.x - s.x + 1;
    int delta_y = sp.y - s.y + 1;

    static const int8_t lut_x_cw[3][3] = {{0, -1, -1}, {1, 0, -1}, {1, 1, 0}};
    static const int8_t lut_y_cw[3][3] = {{-1, -1, 0}, {-1, 0, 1}, {0, 1, 1}};

    int new_x = s.x + lut_x_cw[delta_x][delta_y];
    int new_y = s.y + lut_y_cw[delta_x][delta_y];

    Node c_neighbor(new_x, new_y);
    return isValidNode(c_neighbor) ? c_neighbor : Node{false};
}

float Graph::getTraversalCost(const Cell &c) {
    if (!isValidCell(c))
        return INFINITY;
    return (map_[c.x * width_ + c.y]);
}

float Graph::euclideanHeuristic(const Node &s) {
    return std::hypot(start_.x - s.x, start_.y - s.y);
}

std::vector<Node> Graph::getNodesAroundCell(const Cell &cell) {
    auto top = cell.x;
    auto left = cell.y;
    auto bottom = top + 1;
    auto right = left + 1;

    top = std::max(top, 0);
    left = std::max(left, 0);
    bottom = std::min(bottom, length_ - 1);
    right = std::min(right, width_ - 1);

    return {{top, left}, {top, right}, {bottom, left}, {bottom, right}};
}
Position::Position(float x, float y) {
    this->x = x;
    this->y = y;
}
Position::Position(const Position &other) : pair(other) {}
Position::Position(const std::pair<float, float> &other) : pair(other) {}
Position::Position(const Node &n) {
    this->x = static_cast<float>(n.x);
    this->y = static_cast<float>(n.y);
}
Position &Position::operator=(const Position &other) {
    if (this == &other) return *this;
    x = other.x;
    y = other.y;
    return *this;
}

Node::Node(bool valid) : valid(valid) {}
Node::Node(int x, int y) {
    this->x = x;
    this->y = y;
}
Node::Node(const Node &other) : pair(other) {}
Node::Node(const std::pair<int, int> &other) : pair(other) {}
Node::Node(const Position &n) {
    x = static_cast<int>(roundf(n.x));
    y = static_cast<int>(roundf(n.y));
}
Node &Node::operator=(const Node &other) {
    if (this == &other) return *this;
    x = other.x;
    y = other.y;
    valid = other.valid;
    return *this;
}
bool Node::isValid() { return valid; }
void Node::setValidity(bool is_valid) { this->valid = is_valid; }

Cell::Cell(int x, int y) {
    this->x = x;
    this->y = y;
}
Cell::Cell(const Cell &other) : pair(other) {}
Cell::Cell(const std::pair<int, int> &other) : pair(other) {}
Cell::Cell(const Position &n) {
    x = static_cast<int>(roundf(n.x));
    y = static_cast<int>(roundf(n.y));
}
Cell &Cell::operator=(const Cell &other) {
    if (this == &other) return *this;
    x = other.x;
    y = other.y;
    return *this;
}