#include "Node.h"
#include <cmath>

Node::Node(bool valid) {
    this->valid = valid;
}

Node::Node(int x, int y) {
    this->x_ = x;
    this->y_ = y;
    this->ind_ = std::make_tuple(x, y);
}
Node::Node(std::tuple<int, int> ind) : Node(std::get<0>(ind), std::get<1>(ind)) {
}

Node::~Node() = default;

void Node::setIndex(int x, int y) {
    this->x_ = x;
    this->y_ = y;
    this->ind_ = std::make_tuple(x, y);
}

void Node::setIndex(std::tuple<int, int> ind) {
    this->setIndex(std::get<0>(ind), std::get<1>(ind));
}

std::tuple<int, int> Node::getIndex() const {
    return this->ind_;
}

void Node::setBptr(std::tuple<int, int> bptr) {
    this->bptr_ = bptr;
}

void Node::setBptr(const Node &bptr) {
    this->bptr_ = bptr.getIndex();
}

std::tuple<int, int> Node::getBptr() const {
    return this->bptr_;
}

float Node::distTo(std::tuple<float, float> position) {
    return std::hypot(x_ - std::get<0>(position), y_ - std::get<1>(position));
}

bool Node::operator==(const Node &other) const {
    return this->getIndex() == other.getIndex();
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
