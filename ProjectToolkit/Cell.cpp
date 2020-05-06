//
// Created by patrick on 07/05/20.
//

#include "Node.h"
#include "Position.h"
#include "Cell.h"

Cell::Cell(int x, int y): x(x), y(y){}

Cell::Cell(const Cell &other) = default;

Cell::Cell(const std::pair<int, int> &other) : x(other.first), y(other.second) {}

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

bool Cell::operator==(const Cell &other) const {
    if (this == &other) return true;
    return (x == other.x) and (y == other.y);
}

bool Cell::operator!=(const Cell &other) const { return not(*this == other); }

Cell Cell::topCell() const { return Cell(x - 1, y); }
Cell Cell::topLeftCell() const { return Cell(x - 1, y - 1); }
Cell Cell::topRightCell() const { return Cell(x - 1, y + 1); }
Cell Cell::bottomCell() const { return Cell(x + 1, y); }
Cell Cell::bottomLeftCell() const { return Cell(x + 1, y - 1); }
Cell Cell::bottomRightCell() const { return Cell(x + 1, y + 1); }
Cell Cell::leftCell() const { return Cell(x, y - 1); }
Cell Cell::rightCell() const { return Cell(x, y + 1); }

Node Cell::topLeftNode() const { return Node(x, y); }
Node Cell::topRightNode() const { return Node(x + 1, y); }
Node Cell::bottomLeftNode() const { return Node(x, y + 1); }
Node Cell::bottomRightNode() const { return Node(x + 1, y + 1); }

Position Cell::centerPosition() const { return Position(x + 0.5f, y + 0.5f); }
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
