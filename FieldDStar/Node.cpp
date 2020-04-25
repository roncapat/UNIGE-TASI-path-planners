#include "Node.h"
#include <cmath>

Node::Node(bool valid) :valid(valid){}

Node::Node(int x, int y) {
    x_ = x;
    y_ = y;
    ind_ = std::make_tuple(x, y);
}
Node::Node(std::tuple<int, int> ind) : Node(std::get<0>(ind), std::get<1>(ind)) {
}

Node::~Node() = default;

void Node::setIndex(int x, int y) {
    x_ = x;
    y_ = y;
    ind_ = std::make_tuple(x, y);
}

void Node::setIndex(std::tuple<int, int> ind) {
    setIndex(std::get<0>(ind), std::get<1>(ind));
}

std::tuple<int, int> Node::getIndex() const {
    return ind_;
}

float Node::distTo(std::tuple<float, float> position) {
    return std::hypot(x_ - std::get<0>(position), y_ - std::get<1>(position));
}

bool Node::operator==(const Node &other) const {
    return getIndex() == other.getIndex();
}

bool Node::operator!=(const Node &other) const {
    return !(*this == other);
}

Node &Node::operator=(const Node &node) = default;

std::ostream &operator<<(std::ostream &stream, const Node &n) {
    int x, y;
    std::tie(x, y) = n.getIndex();
    stream << "[" << x << ", " << y << "]";
    return stream;
}

std::ostream &operator<<(std::ostream &stream, const std::unordered_set<Node> &uset) {
    stream << "{";
    for (auto const &i : uset) {
        stream << i << ", ";
    }
    stream << "}";

    return stream;
}
