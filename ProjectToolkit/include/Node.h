//
// Created by patrick on 07/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_NODE_H
#define RONCAPAT_GLOBAL_PLANNERS_NODE_H

#include <utility>
#include <vector>
#include <cmath>
#include <robin_hood.h>

class Cell;

class Position;

class Node {
public:
    int x{}, y{};

    Node() = default;

    explicit Node(bool valid);

    Node(int x, int y);

    Node(const Node &other);

    Node(Node &&other) noexcept;

    explicit Node(const std::pair<int, int> &other);

    explicit Node(const Position &n);  //Round to nearest node
    Node &operator=(const Node &other);

    Node &operator=(Node &&other) noexcept;

    void setValidity(bool is_valid);

    [[nodiscard]] bool operator==(const Node &other) const;

    [[nodiscard]] bool operator!=(const Node &other) const;

    [[nodiscard]] bool is_valid() const;

    [[nodiscard]] Node top_node() const;

    [[nodiscard]] Node top_left_node() const;

    [[nodiscard]] Node top_right_node() const;

    [[nodiscard]] Node bottom_node() const;

    [[nodiscard]] Node bottom_left_node() const;

    [[nodiscard]] Node bottom_right_node() const;

    [[nodiscard]] Node left_node() const;

    [[nodiscard]] Node right_node() const;

    [[nodiscard]] Cell bottom_left_cell() const;

    [[nodiscard]] Cell bottom_right_cell() const;

    [[nodiscard]] Cell top_left_cell() const;

    [[nodiscard]] Cell top_right_cell() const;

    [[nodiscard]] Cell neighbor_cell(bool bottom_TOP, bool left_RIGHT) const;

    [[nodiscard]] std::vector<Cell> cells() const;

    [[nodiscard]] float distance(const Node &n) const;

    [[nodiscard]] float distance(const Position &n) const;

    [[nodiscard]] bool aligned(const Node &p) const;

private:
    bool valid = true;
};

namespace robin_hood {
    template<>
    struct hash<Node> {
        std::size_t operator()(const Node &n) const {
            std::size_t result = 17;
            result = 31 * result + hash<int>()(n.x);
            result = 31 * result + hash<int>()(n.y);
            return result;
        }
    };
}

#endif //RONCAPAT_GLOBAL_PLANNERS_NODE_H
