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

struct Position : public std::pair<float, float> {
  float &x = std::get<0>(*this);
  float &y = std::get<1>(*this);

  Position() = default;
  Position(float x, float y);
  Position(const Position &other);
  explicit Position(const std::pair<float, float> &other);
  explicit Position(const Node &n);
  Position &operator=(const Position &other);;
};

class Node : public std::pair<int, int> {
 private:
  bool valid = true;
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
}

struct Cell : std::pair<int, int> {
 public:
  int &x = std::get<0>(*this);
  int &y = std::get<1>(*this);

  Cell(int x, int y);
  Cell(const Cell &other);
  explicit Cell(const std::pair<int, int> &other);
  explicit Cell(const Position &n);
  Cell &operator=(const Cell &other);
};

typedef std::pair<Node, Node> Edge;

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

  void setGoal(Node goal);

  void initializeGraph(const MapPtr &msg);

  bool isValidNode(const Node &s);

  bool isValidPosition(const Position &p);

  bool isValidCell(const std::tuple<int, int> &ind);

  bool unaligned(const Node &s, const Node &sp);

  bool isDiagonalContinuous(const Position &p, const Position &p_prime);

  std::vector<Node> neighbors(const Node &s, bool include_invalid = false);

  std::vector<Edge> consecutiveNeighbors(const Position &p);

  Node counterClockwiseNeighbor(Node s, Node s_prime);

  Node clockwiseNeighbor(Node s, Node s_prime);

  std::vector<std::tuple<Node, Node>> consecutiveNeighbors(const Node &s);

  float getTraversalCost(const std::tuple<int, int> &ind);

  float euclideanHeuristic(const Node &s);

  std::vector<Node> getNodesAroundCell(const Cell &cell);
  void updateGraph(std::shared_ptr<uint8_t[]> patch, int x, int y, int w, int h);

 private:
  float occupancy_threshold_uchar_ = 178.5f;
};

#endif  // GRAPHSEARCH_H
