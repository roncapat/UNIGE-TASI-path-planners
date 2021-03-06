//
// Created by patrick on 07/05/20.
//

#include <cmath>
#include "Position.h"
#include "Node.h"
#include "Cell.h"

Position::Position(float x, float y) : x(x), y(y) {}
Position::Position(const Position &other) = default;
Position::Position( Position &&other) noexcept= default;
Position::Position(const Node &n) {
    this->x = static_cast<float>(n.x);
    this->y = static_cast<float>(n.y);
}
Position::Position(const std::pair<float, float> &other) : x(other.first), y(other.second) {}
Position::Position(const Cell &n) : Position(n.center()) {}

Position &Position::operator=(const Position &other) = default;
Position &Position::operator=(Position &&other) noexcept= default;

bool Position::operator==(const Position &other) const {
    return (this == &other) or ((x == other.x) and (y == other.y));
}

bool Position::operator!=(const Position &other) const {return not (*this == other);}

float Position::distance(const Position &n) const {
    return std::hypot(x - n.x, y - n.y);
}
bool Position::aligned(const Position &p) const {
    return ((x == p.x) || (y == p.y));
}
