#ifndef RONCAPAT_PLANNER_TOOLKIT_NODE_H
#define RONCAPAT_PLANNER_TOOLKIT_NODE_H

#include <utility>
#include <vector>
#include <robin_hood.h>

class Cell;
class Position;

/**
 * Represents an integer position on a uniform, cartesian grid.
 * Every Cell of the Grid has 4 Nodes as corners.
 * Integer coordinates are also referred as "grid ticks" or "indexes".
 */
class Node {
 public:
  /** Indexes. */
  int x{}, y{};

  /** Default constructor. Zero-initialises indexes. */
  Node() = default;

  /**
   * Parametrized construtor.
   * @param x First index.
   * @param y Second index.
   */
  Node(int x, int y);

  /**
   * Copy constructor.
   * @param other the Node object whose indexes have to be copied.
   */
  Node(const Node &other);

  /**
   * Move constructor.
   * @param other the temporary Node object to move data from.
   */
  Node(Node &&other) noexcept;

  /**
   * Cast a pair of indexes to a Node object.
   * @param other the pair of indexes to reify.
   */
  explicit Node(const std::pair<int, int> &other);

  /**
   * Round a generic Position instance to the nearest Node.
   * @param p the Position instance to round.
   */
  explicit Node(const Position &p);

  /**
   * Copy assignment operator.
   * @param other the Node instance to copy indexes from.
   * @return A reference to itself (for assignment operator chaining).
   */
  Node &operator=(const Node &other);

  /**
   * Move assignment operator.
   * @param other the (temporary) Node instance to copy coordinates from.
   * @return A reference to itself (for assignment operator chaining).
   */
  Node &operator=(Node &&other) noexcept;

  /**
   * Compares two Node instances for equality of their indexes.
   * @param other the other instance to compare with.
   * @return true if both indexes match, false otherwise.
   */
  [[nodiscard]] bool operator==(const Node &other) const;

  /**
   * Compares two Node instances for inequality of their indexes.
   * @param other the other instance to compare with.
   * @return true if at least one index differs, false otherwise.
   */
  [[nodiscard]] bool operator!=(const Node &other) const;

  /**
   * The first neighbour node in N direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour node.
   */
  [[nodiscard]] Node top_node() const;

  /**
 * The first neighbour node in N_W direction.
 * @note X axis goes N->S, Y axis W->E.
 * @return The neighbour node.
 */
  [[nodiscard]] Node top_left_node() const;

  /**
 * The first neighbour node in N_E direction.
 * @note X axis goes N->S, Y axis W->E.
 * @return The neighbour node.
 */
  [[nodiscard]] Node top_right_node() const;

  /**
   * The first neighbour node in S direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour node.
   */
  [[nodiscard]] Node bottom_node() const;

  /**
  * The first neighbour node in S_W direction.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour node.
  */
  [[nodiscard]] Node bottom_left_node() const;

  /**
  * The first neighbour node in S_E direction.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour node.
  */
  [[nodiscard]] Node bottom_right_node() const;

  /**
  * The first neighbour node in W direction.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour node.
  */
  [[nodiscard]] Node left_node() const;

  /**
  * The first neighbour node in E direction.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour node.
  */
  [[nodiscard]] Node right_node() const;

  /**
  * The neighbour cell in S_W direction.
  * The Node is corner N_E of the computed cell.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour cell.
  */
  [[nodiscard]] Cell bottom_left_cell() const;

  /**
  * The neighbour cell in S_E direction.
  * The Node is corner N_W of the computed cell.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour cell.
  */
  [[nodiscard]] Cell bottom_right_cell() const;

  /**
  * The neighbour cell in N_W direction.
  * The Node is corner S_E of the computed cell.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour cell.
  */
  [[nodiscard]] Cell top_left_cell() const;

  /**
  * The neighbour cell in N_E direction.
  * The Node is corner S_W of the computed cell.
  * @note X axis goes N->S, Y axis W->E.
  * @return The neighbour cell.
  */
  [[nodiscard]] Cell top_right_cell() const;

  /**
  * The neighbour cell in N_E direction.
  * @note X axis goes N->S, Y axis W->E.
  * @param bottom_TOP true for N direction, false for S.
  * @param left_RIGHT true for E direction, false for W.
  * @return The neighbour cell.
  */
  [[nodiscard]] Cell neighbor_cell(bool bottom_TOP, bool left_RIGHT) const;

  /**
   * The neighbour cells of the node.
   * @return The 4 neighbour cells of the node.
   */
  [[nodiscard]] std::vector<Cell> cells() const;
  //TODO maybe return only valid cells? Or an std::array instance?
  // Should this class be map-limit agnostic (infinite domain in every direction)

  /**
   * Compute the euclidean distance from another Node instance.
   * @param n the other instance to compute distance from.
   * @return the euclidean distance.
   */
  [[nodiscard]] float distance(const Node &n) const;

  /**
   * Compute the euclidean distance from another Position instance.
   * @param n the Position instance to compute distance from.
   * @return the euclidean distance.
   */
  [[nodiscard]] float distance(const Position &n) const;

  /**
   * Checks horizontal or vertical alignment with another Node instance.
   * @param p the other instance to check alignment with.
   * @return true if at least one coordinate matches, false otherwise.
   */
  [[nodiscard]] bool aligned(const Node &p) const;
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

#endif //RONCAPAT_PLANNER_TOOLKIT_NODE_H
