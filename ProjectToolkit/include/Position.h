#ifndef RONCAPAT_PLANNER_TOOLKIT_POSITION_H
#define RONCAPAT_PLANNER_TOOLKIT_POSITION_H

#include <utility>
class Node;
class Cell;

/** Represents a generic position over a 2D carthesian surface. */
class Position {
 public:
  /** Coordinates. */
  float x{}, y{};

  /** Default constructor. Zero-initialises coordinates. */
  Position() = default;

  /**
   * Parametrized construtor.
   * @param x First coordinate.
   * @param y Second coordinate.
   */
  Position(float x, float y);

  /**
   * Copy constructor.
   * @param other the Position object whose coordinates have to be copied.
   */
  Position(const Position &other);

  /**
   * Move constructor.
   * @param other the temporary Position object to move data from.
   */
  Position(Position &&other) noexcept;

  /**
   * Cast a grid Node to a simple Position object.
   * @param n the Node instance to downgrade.
   */
  Position(const Node &n); // NOLINT(google-explicit-constructor)

  /**
   * Cast a pair of coordinates to a Position object.
   * @param other the pair of coordinates to reify.
   */
  explicit Position(const std::pair<float, float> &other);

  /**
   * Cast a grid Cell to a simple position object (representing the center of the cell).
   * @param c the Cell instance whose center has to be computed.
   */
  explicit Position(const Cell &c);

  /**
   * Copy assignment operator.
   * @param other the Position instance to copy coordinates from.
   * @return A reference to itself (for assignment operator chaining).
   */
  Position &operator=(const Position &other);

  /**
   * Move assignment operator.
   * @param other the (temporary) Position instance to copy coordinates from.
   * @return A reference to itself (for assignment operator chaining).
   */
  Position &operator=(Position &&other) noexcept;

  /**
   * Compares two Position instances for equality of their coordinates.
   * @param other the other instance to compare with.
   * @return true if both coordinates match exaclty, false otherwise.
   */
  [[nodiscard]] bool operator==(const Position &other) const;

  /**
   * Compares two Position instances for inequality of their coordinates.
   * @param other the other instance to compare with.
   * @return true if at least one coordinate differs, false otherwise.
   */
  [[nodiscard]] bool operator!=(const Position &other) const;

  /**
   * Compute the euclidean distance from another Position instance.
   * @param n the other instance to compute distance from.
   * @return the euclidean distance.
   */
  [[nodiscard]] float distance(const Position &n) const;

  /**
   * Checks horizontal or vertical alignment with another Position instance.
   * @param p the other instance to check alignment with.
   * @return true if at least one coordinate matches, false otherwise.
   */
  [[nodiscard]] bool aligned(const Position &p) const;
};

#endif //RONCAPAT_PLANNER_TOOLKIT_POSITION_H
