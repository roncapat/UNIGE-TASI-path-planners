#include "Node.h"
#include "Position.h"
#include "Cell.h"

Node::Node(int x, int y) : x(x), y(y) {}

Node::Node(const Node &other) = default;

Node::Node(Node &&other) noexcept = default;

Node::Node(const std::pair<int, int> &other) : x(other.first), y(other.second) {}

Node::Node(const Position &n) {
    x = static_cast<int>(std::roundf(n.x));
    y = static_cast<int>(std::roundf(n.y));
}

Node &Node::operator=(const Node &other) = default;
Node &Node::operator=(Node &&other) noexcept = default;

bool Node::operator==(const Node &other) const {
    if (this == &other) return true;
    return (x == other.x) and (y == other.y);
}

bool Node::operator!=(const Node &other) const { return not(*this == other); }

Node Node::top_node() const { return {x - 1, y}; }

Node Node::top_left_node() const { return {x - 1, y - 1}; }

Node Node::top_right_node() const { return {x - 1, y + 1}; }

Node Node::bottom_node() const { return {x + 1, y}; }

Node Node::bottom_left_node() const { return {x + 1, y - 1}; }

Node Node::bottom_right_node() const { return {x + 1, y + 1}; }

Node Node::left_node() const { return {x, y - 1}; }

Node Node::right_node() const { return {x, y + 1}; }

Cell Node::bottom_left_cell() const { return {x, y - 1}; }

Cell Node::bottom_right_cell() const { return {x, y}; }

Cell Node::top_left_cell() const { return {x - 1, y - 1}; }

Cell Node::top_right_cell() const { return {x - 1, y}; }

Cell Node::neighbor_cell(bool bottom_TOP, bool left_RIGHT) const {
    if (bottom_TOP)
        return left_RIGHT ? top_right_cell() : top_left_cell();
    else
        return left_RIGHT ? bottom_right_cell() : bottom_left_cell();
}

std::vector<Cell> Node::cells() const {
    return {top_left_cell(), top_right_cell(), bottom_left_cell(), bottom_right_cell()};
}

float Node::distance(const Node &n) const {
    return std::hypot(x - n.x, y - n.y);
}

float Node::distance(const Position &n) const {
    return std::hypot((float)x - n.x, (float)y - n.y);
}

bool Node::aligned(const Node &p) const {
    return ((x == p.x) || (y == p.y));
}
