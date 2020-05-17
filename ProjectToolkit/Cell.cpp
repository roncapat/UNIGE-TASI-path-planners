//
// Created by patrick on 07/05/20.
//

#include "Node.h"
#include "Position.h"
#include "Cell.h"

Cell::Cell() : x(-1), y(-1) {}

Cell::Cell(int x, int y) : x(x), y(y) {}

Cell::Cell(const Cell &other) = default;

Cell::Cell(Cell &&other) noexcept = default;

Cell::Cell(const std::pair<int, int> &other) : x(other.first), y(other.second) {}

Cell::Cell(const Position &n) {
    x = static_cast<int>(roundf(n.x));
    y = static_cast<int>(roundf(n.y));
}

Cell &Cell::operator=(const Cell &other) = default;
Cell &Cell::operator=(Cell &&other) noexcept = default;

bool Cell::operator==(const Cell &other) const {
    if (this == &other) return true;
    return (x == other.x) and (y == other.y);
}

bool Cell::operator!=(const Cell &other) const { return not(*this == other); }

Cell Cell::topCell() const { return {x - 1, y}; }

Cell Cell::topLeftCell() const { return {x - 1, y - 1}; }

Cell Cell::topRightCell() const { return {x - 1, y + 1}; }

Cell Cell::bottomCell() const { return {x + 1, y}; }

Cell Cell::bottomLeftCell() const { return {x + 1, y - 1}; }

Cell Cell::bottomRightCell() const { return {x + 1, y + 1}; }

Cell Cell::leftCell() const { return {x, y - 1}; }

Cell Cell::rightCell() const { return {x, y + 1}; }

Node Cell::topLeftNode() const { return {x, y}; }

Node Cell::topRightNode() const { return {x + 1, y}; }

Node Cell::bottomLeftNode() const { return {x, y + 1}; }

Node Cell::bottomRightNode() const { return {x + 1, y + 1}; }

Position Cell::centerPosition() const { return Position((float)x + 0.5f, (float)y + 0.5f); }

std::vector<Node> Cell::cornerNodes() const {
    return {topLeftNode(), topRightNode(), bottomLeftNode(), bottomRightNode()};
}

bool Cell::hasNode(const Node &n) const {
    return (((n.x == x) or (n.x == (x + 1)))
            and ((n.y == y) or (n.y == (y + 1))));
}

float Cell::distance(const Cell &n) const {
    return std::hypot(x - n.x, y - n.y);
}