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
  Position(const Node &n); //Allow implicit cast to broader concept
  explicit Position(const Cell &n); //Center of cell
  explicit Position(const std::pair<float, float> &other);
  Position &operator=(const Position &other);
  [[nodiscard]] float distance(const Position &n) const {
      return std::hypot(x - n.x, y - n.y);
  }
  [[nodiscard]] bool aligned(const Position &p) const {
      return ((x == p.x) || (y == p.y));
  }
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
  explicit Node(const Position &n);  //Round to nearest node
  Node &operator=(const Node &other);
  bool isValid() const;
  void setValidity(bool is_valid);
  [[nodiscard]] Node topNode() const { return Node(x - 1, y); }
  [[nodiscard]] Node bottomNode() const { return Node(x + 1, y); }
  [[nodiscard]] Node leftNode() const { return Node(x, y - 1); }
  [[nodiscard]] Node rightNode() const { return Node(x, y + 1); }
  [[nodiscard]] Cell cellBottomLeft() const;
  [[nodiscard]] Cell cellBottomRight() const;
  [[nodiscard]] Cell cellTopLeft() const;
  [[nodiscard]] Cell cellTopRight() const;
  [[nodiscard]] Cell neighborCell(bool bottom_TOP, bool left_RIGHT) const;
  [[nodiscard]] inline std::vector<Cell> cells() const;
  [[nodiscard]] float distance(const Node &n) const {
      return std::hypot(x - n.x, y - n.y);
  }
  [[nodiscard]] bool aligned(const Node &p) const{
      return ((x == p.x) || (y == p.y));
  }
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
  [[nodiscard]] inline Cell topCell() const { return Cell(x - 1, y); }
  [[nodiscard]] inline Cell topLeftCell() const { return Cell(x - 1, y-1); }
  [[nodiscard]] inline Cell topRightCell() const { return Cell(x - 1, y+1); }
  [[nodiscard]] inline Cell bottomCell() const { return Cell(x + 1, y); }
  [[nodiscard]] inline Cell bottomLeftCell() const { return Cell(x + 1, y-1); }
  [[nodiscard]] inline Cell bottomRightCell() const { return Cell(x + 1, y+1); }
  [[nodiscard]] inline Cell leftCell() const { return Cell(x, y - 1); }
  [[nodiscard]] inline Cell rightCell() const { return Cell(x, y + 1); }
  [[nodiscard]] inline Node topLeftNode() const { return Node(x, y); }
  [[nodiscard]] inline Node topRightNode() const { return Node(x + 1, y); }
  [[nodiscard]] inline Node bottomLeftNode() const { return Node(x, y + 1); }
  [[nodiscard]] inline Node bottomRightNode() const { return Node(x + 1, y + 1); }
  [[nodiscard]] inline Position centerPosition() const {return Position(x+0.5f, y+0.5f);}
  [[nodiscard]] inline std::vector<Node> corners() const {
      return {topLeftNode(), topRightNode(), bottomLeftNode(), bottomRightNode()};
  }
  [[nodiscard]] bool hasNode(const Node &n) const {
      return (((n.x == x) or (n.x == (x + 1)))
          and ((n.y == y) or (n.y == (y + 1))));
  };
  [[nodiscard]] float distance(const Cell &n) const {
      return std::hypot(x - n.x, y - n.y);
  }
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
  std::shared_ptr<uint8_t[]> map_;
  std::vector<Cell> updated_cells_;

  Cell start_cell_;
  Cell goal_cell_;
  Node start_node_;
  Node goal_node_;
  Position start_pos_;
  Position goal_pos_;

  int length_;
  int width_;
  int size_;
  float flength_;
  float fwidth_;

  void setOccupancyThreshold(float occupancy_threshold);
  void setStart(const Position &start);
  void setGoal(const Position &goal);
  void initializeGraph(std::shared_ptr<uint8_t[]> image, int width, int length);
  void updateGraph(const std::shared_ptr<uint8_t[]> &patch, int x, int y, int w, int h);
  float getTraversalCost(const Cell &ind);
  bool isValid(const Node &s);
  bool isValid(const Position &p);
  bool isValid(const Cell &c);

  std::vector<Node> neighbors_8(const Node &s, bool include_invalid = false);
  std::vector<Cell> neighbors_4(const Cell &s, bool include_invalid = false);

  std::vector<Edge> consecutiveNeighbors(const Position &p);
  std::vector<Edge> consecutiveNeighbors(const Node &s);

  Node counterClockwiseNeighbor(const Node &s, const Node &s_prime);
  Node clockwiseNeighbor(const Node &s, const Node &s_prime);

  int occupancy_threshold_uchar_ = 254;

  static Cell getCell(const Node &a, const Node &b, const Node &c);
  static std::vector<Position> getGridBoundariesTraversals(const Position &a, const Position &b);

  std::vector<Cell> neighbors_8(const Cell &s, bool include_invalid=false);
};

#endif  // GRAPHSEARCH_H
