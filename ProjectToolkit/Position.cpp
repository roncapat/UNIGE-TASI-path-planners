//
// Created by patrick on 07/05/20.
//

#include "Position.h"
#include "Node.h"
#include "Cell.h"

Position::Position(float x, float y) : x(x), y(y) {}
Position::Position(const Position &other) = default;
Position::Position(const Node &n) {
    this->x = static_cast<float>(n.x);
    this->y = static_cast<float>(n.y);
}
Position::Position(const std::pair<float, float> &other) : x(other.first), y(other.second) {}
Position::Position(const Cell &n) : Position(n.centerPosition()) {}

Position &Position::operator=(const Position &other) {
    if (this == &other) return *this;
    x = other.x;
    y = other.y;
    return *this;
}

bool Position::operator==(const Position &other) const {
    if (this == &other) return true;
    return (x == other.x) and (y == other.y);
}

bool Position::operator!=(const Position &other) const {return not (*this == other);}

float Position::distance(const Position &n) const {
    return std::hypot(x - n.x, y - n.y);
}
bool Position::aligned(const Position &p) const {
    return ((x == p.x) || (y == p.y));
}
