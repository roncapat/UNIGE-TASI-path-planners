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

    [[nodiscard]] bool isValid() const;

    [[nodiscard]] Node topNode() const;

    [[nodiscard]] Node topLeftNode() const;

    [[nodiscard]] Node topRightNode() const;

    [[nodiscard]] Node bottomNode() const;

    [[nodiscard]] Node bottomLeftNode() const;

    [[nodiscard]] Node bottomRightNode() const;

    [[nodiscard]] Node leftNode() const;

    [[nodiscard]] Node rightNode() const;

    [[nodiscard]] Cell cellBottomLeft() const;

    [[nodiscard]] Cell cellBottomRight() const;

    [[nodiscard]] Cell cellTopLeft() const;

    [[nodiscard]] Cell cellTopRight() const;

    [[nodiscard]] Cell neighborCell(bool bottom_TOP, bool left_RIGHT) const;

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
