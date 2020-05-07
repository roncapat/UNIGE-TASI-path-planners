#ifndef GRAPH_H
#define GRAPH_H

#include <cassert>
#include <cmath>
#include <limits>
#include <queue>
#include <tuple>
#include <utility>
#include <vector>
#include <memory>
#include "Node.h"
#include "Position.h"
#include "Cell.h"

typedef std::pair<Node, Node> Edge;

class Graph {
 public:
  std::shared_ptr<uint8_t[]> map_;
  std::vector<Cell> updated_cells_;

  Cell start_cell_, goal_cell_;
  Node start_node_, goal_node_;
  Position start_pos_, goal_pos_;

  int length_, width_;
  float flength_, fwidth_; //Used to avoid casts (is it useful?)
  int size_;

  int occupancy_threshold_uchar_ = 254;

  void setStart(const Position &start);
  void setGoal(const Position &goal);

  void setOccupancyThreshold(float occupancy_threshold);

  void initializeGraph(std::shared_ptr<uint8_t[]> image, int width, int length);
  void updateGraph(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h);

  float getTraversalCost(const Cell &ind);

  bool isValid(const Node &s);
  bool isValid(const Position &p);
  bool isValid(const Cell &c);

  std::vector<Node> neighbors_8(const Node &s, bool include_invalid = false);
  std::vector<Cell> neighbors_8(const Cell &s, bool include_invalid = false);
  std::vector<Cell> neighbors_4(const Cell &s, bool include_invalid = false);

  std::vector<Edge> consecutiveNeighbors(const Position &p);
  std::vector<Edge> consecutiveNeighbors(const Node &s);

  Node counterClockwiseNeighbor(const Node &s, const Node &s_prime);
  Node clockwiseNeighbor(const Node &s, const Node &s_prime);

  static Cell getCell(const Node &a, const Node &b, const Node &c);
  static std::vector<Position> getGridBoundariesTraversals(const Position &a, const Position &b);
  uint8_t &get(int x, int y);
};

#endif  // GRAPHSEARCH_H
