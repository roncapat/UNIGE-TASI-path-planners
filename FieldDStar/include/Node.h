/**
The Node object represents a vertex on the graph.

In Field D*, Nodes are located on the corners of grid cells (as opposed to the
center, like in A*, D*, or D*lite). The Node is principle currency of Field D*
since, with Nodes, we can define a cost field from which we obtain an
optimal path.

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date: Dec. 15th, 2018
*/

#ifndef NODE_H
#define NODE_H

#include <functional>
#include <iostream>
#include <limits>
#include <tuple>
#include <unordered_set>

class Node {
 public:

  Node(bool valid = true);
  Node(int x, int y);
  Node(std::tuple<int, int> ind);
  ~Node();
  void setIndex(int x, int y);
  void setIndex(std::tuple<int, int> ind);
  std::tuple<int, int> getIndex() const;
  float distTo(std::tuple<float, float> position);

  bool operator==(const Node &other) const;
  bool operator!=(const Node &other) const;
  Node &operator=(const Node &node);
  bool valid = true;

 private:
  int x_;
  int y_;

  std::tuple<int, int> ind_;
};

namespace std {
template<>
struct hash<Node> {
  std::size_t operator()(const Node &node) const {
      auto[x, y] = node.getIndex();
      std::size_t result = 17;
      result = 31 * result + hash<int>()(x);
      result = 31 * result + hash<int>()(y);
      return result;
  }
};
}

#endif  // NODE_H
