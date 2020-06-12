//
// Created by patrick on 07/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_CELL_H
#define RONCAPAT_GLOBAL_PLANNERS_CELL_H

#include <robin_hood.h>
#include <utility>
#include <vector>

class Node;

class Position;

class Cell {
public:
    int x, y;

    Cell();

    Cell(int x, int y);

    Cell(const Cell &other);

    Cell(Cell &&other) noexcept;

    explicit Cell(const std::pair<int, int> &other);

    explicit Cell(const Position &n);  // Cell containing Position
    Cell &operator=(const Cell &other);

    Cell &operator=(Cell &&other) noexcept;

    [[nodiscard]] bool operator==(const Cell &other) const;

    [[nodiscard]] bool operator!=(const Cell &other) const;

    [[nodiscard]] Cell top_cell() const;

    [[nodiscard]] Cell top_left_cell() const;

    [[nodiscard]] Cell top_right_cell() const;

    [[nodiscard]] Cell bottom_cell() const;

    [[nodiscard]] Cell bottom_left_cell() const;

    [[nodiscard]] Cell bottom_right_cell() const;

    [[nodiscard]] Cell left_cell() const;

    [[nodiscard]] Cell right_cell() const;

    [[nodiscard]] Node top_left_node() const;

    [[nodiscard]] Node top_right_node() const;

    [[nodiscard]] Node bottom_left_node() const;

    [[nodiscard]] Node bottom_right_node() const;

    [[nodiscard]] Position center() const;

    [[nodiscard]] std::vector<Node> corners() const;

    [[nodiscard]] bool has_node(const Node &n) const;;

    [[nodiscard]] float distance(const Cell &n) const;
};

namespace robin_hood {
    template<>
    struct hash<Cell> {
        std::size_t operator()(const Cell &n) const {
            std::size_t result = 17;
            result = 31 * result + hash<int>()(n.x);
            result = 31 * result + hash<int>()(n.y);
            return result;
        }
    };
}
#endif //RONCAPAT_GLOBAL_PLANNERS_CELL_H
