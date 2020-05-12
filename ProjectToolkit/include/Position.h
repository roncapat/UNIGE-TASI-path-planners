//
// Created by patrick on 07/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_POSITION_H
#define RONCAPAT_GLOBAL_PLANNERS_POSITION_H

#include <utility>
#include <cmath>
#include <vector>

class Node;
class Cell;

class Position {
 public:
  float x{}, y{};
  Position() = default;
  Position(float x, float y);
  Position(const Position &other);
  Position(const Node &n); // NOLINT(google-explicit-constructor)
  explicit Position(const std::pair<float, float> &other);
  explicit Position(const Cell &n); //Center of cell
  Position &operator=(const Position &other);
  [[nodiscard]] bool operator==(const Position &other) const;
  [[nodiscard]] bool operator!=(const Position &other) const;
  [[nodiscard]] float distance(const Position &n) const;
  [[nodiscard]] bool aligned(const Position &p) const;
};

#endif //RONCAPAT_GLOBAL_PLANNERS_POSITION_H
