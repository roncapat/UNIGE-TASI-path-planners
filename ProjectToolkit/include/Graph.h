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

  void set_start(const Position &start);
  void set_goal(const Position &goal);

  void set_occupancy_threshold(float occupancy_threshold);

  void init(std::shared_ptr<uint8_t[]> image, int width, int length);
  void update(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h);

  float get_cost(const Cell &ind) const;

  bool is_valid(const Node &s) const;
  bool is_valid(const Position &p) const;
  bool is_valid(const Cell &c) const;
  bool is_valid_vertex(const Position &p) const;

  std::vector<Node> neighbors_8(const Node &s, bool include_invalid = false) const;
  std::vector<Cell> neighbors_8(const Cell &s, bool include_invalid = false) const;
  std::vector<Node> neighbors_4(const Node &s, bool include_invalid = false) const;
  std::vector<Cell> neighbors_4(const Cell &s, bool include_invalid = false) const;
  std::vector<Node> neighbors_diag_4(const Node &s, bool include_invalid = false) const;

  std::vector<Edge> consecutive_neighbors(const Position &p) const;
  std::vector<Edge> consecutive_neighbors(const Node &s) const;

  Node ccw_neighbor(const Node &s, const Node &s_prime) const;
  Node cw_neighbor(const Node &s, const Node &s_prime) const;

  static Cell get_cell(const Node &a, const Node &b, const Node &c);
  static std::vector<Position> get_grid_boundaries_traversals(const Position &a, const Position &b);
  uint8_t &get(int x, int y);

};

#endif  // GRAPHSEARCH_H
