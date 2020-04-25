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
#include <boost/heap/binomial_heap.hpp>
/**
  key defined as <f1(s), f2(s)>
  where...
  f1(s) = min(g(s), rhs(s)) + h(s_start, s) + K_M
  f2(s)min(g(s), rhs(s))
*/
typedef std::pair<float, float> Key;

class PriorityQueue {
 public:
  typedef std::pair<Node, Key> ElemType;
 private:
  struct comparator {
    inline bool operator()
        (const ElemType &c1, const ElemType &c2) const {
        return c1.second >= c2.second;
    }
  };

  typedef boost::heap::compare<comparator> CompareOption;
  typedef boost::heap::mutable_<true> MutableOption;
  typedef boost::heap::binomial_heap<ElemType, CompareOption, MutableOption> QueueType;
  QueueType __queue;
  std::unordered_map<Node, QueueType::handle_type> __handles;

 public:
  typedef QueueType::handle_type HandleType;
  typedef QueueType::iterator IteratorType;

  IteratorType begin() { return __queue.begin(); };
  IteratorType end() { return __queue.end(); };
  void swap(PriorityQueue &other) {
      __queue.swap(other.__queue);
      __handles.swap(other.__handles);
  };
  void insert_or_update(const Node& n, Key k);

  void remove_if_present(const Node &n);

  /**
  Default no-arg constructor for priority queue. Initializes priority queue
  with compFunctor Comparator. This comparator sorts the priority queue based
  on the lexicographic ordering of the key values.
  */
  PriorityQueue(){};
  // O(n)
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
};

#endif
