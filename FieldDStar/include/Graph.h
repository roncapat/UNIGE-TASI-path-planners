#ifndef GRAPH_H
#define GRAPH_H

#include "Node.h"
#include "Map.h"

#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>


struct Position {
  float x;
  float y;

  Node castToNode() const {
      return {static_cast<int>(roundf(x)), static_cast<int>(roundf(y))};
  }

  Position() = default;

  Position(float x, float y) : x(x), y(y) {}

  Position(std::tuple<float, float> position) : Position(std::get<0>(position), std::get<1>(position)) {}

  Position(const Node &n) : Position(static_cast<std::tuple<float, float>>(n.getIndex())) {}

  Position &operator=(const Position &other) = default;

  // Cells equal if their corresponding indices are equal
  bool operator==(const Position &other) const {
      return (this->x == other.x) && (this->y == other.y);
  }

  bool operator!=(const Position &other) const {
      return !(*this == other);
  }
};

struct Cell {
  int x;
  int y;

  Cell(int x, int y) : x(x), y(y) {}

  Cell(std::tuple<int, int> cell) : Cell(std::get<0>(cell), std::get<1>(cell)) {}

  Cell &operator=(const Cell &other) = default;

  bool operator==(const Cell &other) const {
      return (this->x == other.x) && (this->y == other.y);
  }

  bool operator!=(const Cell &other) const {
      return !(*this == other);
  }
};

class Graph {
 public:
  std::shared_ptr<uint8_t[]> map_;  // Map is the current, most up-to-date occupancy grid.

  Node start_;  // start node in the search problem
  Node goal_;   // goal node in the search problem

  std::vector<Cell> updated_cells_;

  int length_;
  int width_;
  int size_;
  float flength_;
  float fwidth_;

  void setOccupancyThreshold(float occupancy_threshold);

  void setGoal(std::tuple<int, int> goal);
  void setGoal(Node goal);

  void initializeGraph(const MapPtr &msg);

  bool isValidNode(const Node &s);

  bool isValidPosition(const Position &p);

  bool isValidCell(const std::tuple<int, int> &ind);

  bool isDiagonal(const Node &s, const Node &s_prime);

  bool isDiagonalContinuous(const Position &p, const Position &p_prime);

  std::vector<Node> neighbors(const Node &s, bool include_invalid = false);

  std::vector<std::pair<Position, Position>> consecutiveNeighbors(const Position &p);

  Node counterClockwiseNeighbor(Node s, Node s_prime);

  Node clockwiseNeighbor(Node s, Node s_prime);

  std::vector<std::tuple<Node, Node>> consecutiveNeighbors(const Node &s);

  float getTraversalCost(const std::tuple<int, int> &ind);

  float euclideanHeuristic(const Node &s);
  float euclideanHeuristic(const std::tuple<int, int> &ind);

  std::vector<Node> getNodesAroundCell(const Cell &cell);
  void updateGraph(std::shared_ptr<uint8_t[]> patch, int x, int y, int w, int h);

 private:
  float occupancy_threshold_uchar_ = 178.5f;
};

#endif  // GRAPHSEARCH_H
