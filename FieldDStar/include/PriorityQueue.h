/**
Priority Queue implementation for Field D* path planning algorithm. Specializes
std::priority_queue to provide Node-specific helper methods for manipulating
the priority queue and a comparator to deal with the ordering of key-value pairs.

The PriorityQueue class orders states based on ascending order of key value.
Because the key value of each state contains two quantities a lexicographic
ordering is used, where key(s) < key(s') iff the first element of key(s) is less
than the first element of key(s') or the first element of key(s) equals the first
element of key(s') and the second element of key(s) is less than the second
element of key(s').

Each State is composed of the following:
1. x and y index representing the node on the graph
2. f-value and g-value

Author: Alejandro Escontrela <aescontrela3@gatech.edu>
Date Created: December 20th, 2018

                    Because this software was written
                         during christmas 2018:

                                     *
                                     ^
                                    ^^^
                                   ^^^^^
                                  ^^^#^^^
                                 ^^^^^$^^^
                                ^^^^^^^^^^^
                               ^^J^^^^^^^0^^
                              ^^^^^^^^^^^^^^^
                             ^^^^^@^^^^^^^^^&^
                            ^^^^^^^^^^^^^^^^^^^
                                ||       ||
                                ||       ||
                            ||_______________||

                     yes, some of the ascii characters
                               are ornaments

                                                - Alejandro
*/

#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <algorithm>
#include <functional>
#include <set>
#include <sstream>
#include <string>
#include <tuple>
#include <utility>

#include "Node.h"

/**
  key defined as <f1(s), f2(s)>
  where...
  f1(s) = min(g(s), rhs(s)) + h(s_start, s) + K_M
  f2(s)min(g(s), rhs(s))
*/
typedef std::pair<float, float> Key;

class PriorityQueue {
 public:
  /**
  Default no-arg constructor for priority queue. Initializes priority queue
  with compFunctor Comparator. This comparator sorts the priority queue based
  on the lexicographic ordering of the key values.
  */
  PriorityQueue();
  /**
  Checks whether or not the priority queue contains a state with the specified
  index.

  @param[in] ind tuple representing the indices of the state
  @return whether or not the priority queue contains an entry for that state
  */
  bool contains(const Node &n);  // O(n)
  /**
  Insert a state into the priority queue.

  @param[in] item state to insert into the priority queue
  */
  void insert(const Node &n, Key k);  // O(n)
  /**
  Clears the priority queue's contents.
  */
  void clear();
  /**
  Remove an item from the priority queue for the specified node.

  @param[in] n node to remove from the priority queue
  @return whether such an item was found and removed
  */
  void remove(const Node &n);  // O(n)
  /**
  Pops the top (least-cost) state from the priority queue. This has the effect of
  reducing the size of the priority queue by one. If the priority queue is
  empty, nothing happens.
  */
  void pop();  // O(1)
  /**
  Returns the key of the least cost state from the priority queue

  @return least-cost key from the priority queue
  */
  Key topKey();  // O(1)
  /**
  Returns the node of the least cost state from the priority queue

  @return least-cost node from the priority queue
  */
  Node topNode();  // O(1)
  /**
  Returns current size of the priority queue.

  @return number of states in priority queue
  */
  int size();  // O(1)

  /**
  Checks if the priority queue has no elements

  @return bool indicating whether or not the priority queue is empty
  */
  bool empty();

 private:
  /**
  Finds first state in the priority queue with the specified index. If found,
  returns an iterator to this state in the priority queue. Otherwise, returns
  iterator to the end of the priority queue.

  @param[in] ind index of state to find in the priority queue
  @return iterator to the found element or iterator to end of priority queue if no such element found
  */
  std::set<std::pair<Node, Key>>::iterator find(const Node &n);

  // Defining a lambda function to compare two entries. Compares using key.
  // Comparison logic for priority queue entries
  typedef std::function<bool(std::pair<Node, Key>, std::pair<Node, Key>)> Comparator;
  Comparator compFunctor = [](const std::pair<Node, Key> &elem1, const std::pair<Node, Key> &elem2) {
    return elem1.second <= elem2.second;
  };

  std::set<std::pair<Node, Key>, Comparator> pq_;
};

#endif
