#ifndef RONCAPAT_PLANNER_TOOLKIT_CELL_H
#define RONCAPAT_PLANNER_TOOLKIT_CELL_H

#include <utility>
#include <vector>
#include <robin_hood.h>

class Node;
class Position;

/**
 * Represents a cell of a uniform, cartesian grid.
 * Every Cell of the Grid has 4 Nodes as corners.
 * Each cell is charecterized by two spatial indexes.
 */
class Cell {
 public:
  /** Indexes. */
  int x{}, y{};

  /** Default constructor. Zero-initialises indexes. */
  Cell();

  /**
   * Parametrized construtor.
   * @param x First index.
   * @param y Second index.
   */
  Cell(int x, int y);

  /**
   * Copy constructor.
   * @param other the Cell object whose indexes have to be copied.
   */
  Cell(const Cell &other);

  /**
   * Move constructor.
   * @param other the temporary Cell object to move data from.
   */
  Cell(Cell &&other) noexcept;

  /**
   * Cast a pair of indexes to a Cell object.
   * @param other the pair of indexes to reify.
   */
  explicit Cell(const std::pair<int, int> &other);

  /**
   * Get the Cell containing the specified position in the grid.
   * @param p the position whose cell in grid has to be determined.
   */
  explicit Cell(const Position &p);  // Cell containing Position

  /**
   * Copy assignment operator.
   * @param other the Cell instance to copy indexes from.
   * @return A reference to itself (for assignment operator chaining).
   */
  Cell &operator=(const Cell &other);

  /**
   * Move assignment operator.
   * @param other the (temporary) Cell instance to copy coordinates from.
   * @return A reference to itself (for assignment operator chaining).
   */
  Cell &operator=(Cell &&other) noexcept;

  /**
   * Compares two Cell instances for equality of their indexes.
   * @param other the other instance to compare with.
   * @return true if both indexes match, false otherwise.
   */
  [[nodiscard]] bool operator==(const Cell &other) const;

  /**
   * Compares two Cell instances for inequality of their indexes.
   * @param other the other instance to compare with.
   * @return true if at least one index differs, false otherwise.
   */
  [[nodiscard]] bool operator!=(const Cell &other) const;

  /**
   * The first neighbour Cell in N direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell top_cell() const;

  /**
   * The first neighbour Cell in N_W direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell top_left_cell() const;

  /**
   * The first neighbour Cell in N_E direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell top_right_cell() const;

  /**
   * The first neighbour Cell in S direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell bottom_cell() const;

  /**
   * The first neighbour Cell in S_W direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell bottom_left_cell() const;

  /**
   * The first neighbour Cell in S_E direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell bottom_right_cell() const;

  /**
   * The first neighbour Cell in W direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell left_cell() const;

  /**
   * The first neighbour Cell in E direction.
   * @note X axis goes N->S, Y axis W->E.
   * @return The neighbour Cell.
   */
  [[nodiscard]] Cell right_cell() const;

  /**
   * The Node corner in N_W direction.
   * The Cell is at S_E of the computed Node.
   * @note X axis goes N->S, Y axis W->E.
   * @return The requested corner.
   */
  [[nodiscard]] Node top_left_node() const;

  /**
   * The Node corner in N_E direction.
   * The Cell is at S_W of the computed Node.
   * @note X axis goes N->S, Y axis W->E.
   * @return The requested corner.
   */
  [[nodiscard]] Node top_right_node() const;

  /**
   * The Node corner in S_W direction.
   * The Cell is at N_E of the computed Node.
   * @note X axis goes N->S, Y axis W->E.
   * @return The requested corner.
   */
  [[nodiscard]] Node bottom_left_node() const;

  /**
   * The Node corner in S_E direction.
   * The Cell is at N_W of the computed Node.
   * @note X axis goes N->S, Y axis W->E.
   * @return The requested corner.
   */
  [[nodiscard]] Node bottom_right_node() const;

  /**
   * The center of the cell.
   * @return The Position of the point at the center of the cell.
   */
  [[nodiscard]] Position center() const;

  /**
   * The corners of the Cell.
   * @return The 4 corner Nodes of the Cell.
   */
  [[nodiscard]] std::vector<Node> corners() const;

  /**
   * Check if the Node instance is a corner of this Cell.
   * @param n the node.
   * @return true if the Node is a corner of the Cell, false otherwise.
   */
  [[nodiscard]] bool has_node(const Node &n) const;;


  /**
   * Compute the euclidean distance from another Cell instance.
   * The distance is calculated between the two centers of the cells.
   * @param c the Cell instance to compute distance from.
   * @return the euclidean distance.
   */
  [[nodiscard]] float distance(const Cell &c) const;
};

namespace robin_hood {
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
#endif //RONCAPAT_PLANNER_TOOLKIT_CELL_H
