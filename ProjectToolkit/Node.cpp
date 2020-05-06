//
// Created by patrick on 07/05/20.
//

#include "Node.h"
#include "Position.h"
#include "Cell.h"

Node::Node(bool valid) : valid(valid) {}
Node::Node(int x, int y) {
    this->x = x;
    this->y = y;
}
Node::Node(const Node &other) : x(other.x), y(other.y) {}
Node::Node(const std::pair<int, int> &other) : x(other.first), y(other.second) {}
Node::Node(const Position &n) {
    x = static_cast<int>(std::roundf(n.x));
    y = static_cast<int>(std::roundf(n.y));
}
Node &Node::operator=(const Node &other) {
    if (this == &other) return *this;
    x = other.x;
    y = other.y;
    valid = other.valid;
    return *this;
}

void Node::setValidity(bool is_valid) { this->valid = is_valid; }

bool Node::operator==(const Node &other) const {
    if (this == &other) return true;
    return (x == other.x) and (y == other.y);
}
bool Node::operator!=(const Node &other) const { return not(*this == other); }

bool Node::isValid() const { return valid; }

Node Node::topNode() const { return Node(x - 1, y); }
Node Node::bottomNode() const { return Node(x + 1, y); }
Node Node::leftNode() const { return Node(x, y - 1); }
Node Node::rightNode() const { return Node(x, y + 1); }

Cell Node::cellBottomLeft() const { return {x, y - 1}; }
Cell Node::cellBottomRight() const { return {x, y}; }
Cell Node::cellTopLeft() const { return {x - 1, y - 1}; }
Cell Node::cellTopRight() const { return {x - 1, y}; }

Cell Node::neighborCell(bool bottom_TOP, bool left_RIGHT) const {
    if (bottom_TOP)
        return left_RIGHT ? cellTopRight() : cellTopLeft();
    else
        return left_RIGHT ? cellBottomRight() : cellBottomLeft();
}
std::vector<Cell> Node::cells() const {
    return {cellTopLeft(), cellTopRight(), cellBottomLeft(), cellBottomRight()};
}

float Node::distance(const Node &n) const {
    return std::hypot(x - n.x, y - n.y);
}
bool Node::aligned(const Node &p) const {
    return ((x == p.x) || (y == p.y));
}
