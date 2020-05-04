#ifndef GRAPH_H
#define GRAPH_H

#include "Map.h"
#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>

class Node;
class Cell;
class Position;

class Position : public std::pair<float, float> {
 public:
  float &x = std::get<0>(*this);
  float &y = std::get<1>(*this);

  Position() = default;
  Position(float x, float y);
  Position(const Position &other);
  explicit Position(const std::pair<float, float> &other);
  explicit Position(const Node &n);
  Position &operator=(const Position &other);
};

class Node : public std::pair<int, int> {
 public:
  int &x = std::get<0>(*this);
  int &y = std::get<1>(*this);

  Node() = default;
  explicit Node(bool valid);
  Node(int x, int y);
  Node(const Node &other);
  explicit Node(const std::pair<int, int> &other);
  explicit Node(const Position &n);
  Node &operator=(const Node &other);
  bool isValid();
  void setValidity(bool is_valid);
  Cell cellBottomLeft();
  Cell cellBottomRight();
  Cell cellTopLeft();
  Cell cellTopRight();
  Cell neighborCell(bool bottom_TOP, bool left_RIGHT);
 private:
  bool valid = true;


};

class Cell : public std::pair<int, int> {
 public:
  int &x = std::get<0>(*this);
  int &y = std::get<1>(*this);

  Cell() = default;
  Cell(int x, int y);
  Cell(const Cell &other);
  explicit Cell(const std::pair<int, int> &other);
  explicit Cell(const Position &n);
  Cell &operator=(const Cell &other);
  [[nodiscard]] Cell topCell() const {return Cell(x-1, y);}
  [[nodiscard]] Cell bottomCell()const {return Cell(x+1, y);}
  [[nodiscard]] Cell leftCell()const {return Cell(x, y-1);}
  [[nodiscard]] Cell rightCell()const {return Cell(x, y+1);}
};

namespace std {
template<>
struct hash<Node> {
  std::size_t operator()(const Node &n) const {
      std::size_t result = 17;
      result = 31 * result + hash<int>()(n.x);
      result = 31 * result + hash<int>()(n.y);
      return result;
  }
};

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

typedef std::pair<Node, Node> Edge;

class Graph {
 public:
  Graph();
  std::shared_ptr<uint8_t[]> map_;  // Occupancy grid

  Cell start_;
  Cell goal_;
  Position goal_pos_;
  Position start_pos_;

  std::vector<Cell> updated_cells_;

  int length_;
  int width_;
  int size_;
  float flength_;
  float fwidth_;

  void setOccupancyThreshold(float occupancy_threshold);

  void setGoal(const Cell &goal);

  void initializeGraph(const MapPtr &msg);

  bool isValid(const Node &s);

  bool isValid(const Position &p);

  bool isValid(const Cell &ind);

  static bool unaligned(const Node &s, const Node &sp);

  static bool unaligned(const Position &p, const Position &sp);

  std::vector<Cell> neighbors(const Cell &s, bool include_invalid = false);

  std::vector<Edge> consecutiveNeighbors(const Position &p);
  std::vector<Edge> consecutiveNeighbors(const Node &s);

  Node counterClockwiseNeighbor(const Node& s, const Node& s_prime);

  Node clockwiseNeighbor(const Node& s, const Node& s_prime);

  float getTraversalCost(const Cell &ind);

  float euclideanHeuristic(const Node &s);

  std::vector<Node> getNodesAroundCell(const Cell &cell);
  void updateGraph(const std::shared_ptr<uint8_t[]>& patch, int x, int y, int w, int h);

  int occupancy_threshold_uchar_ = 254;
};

#endif  // GRAPHSEARCH_H