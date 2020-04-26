#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <algorithm>
#include <functional>
#include <set>
#include <tuple>
#include <utility>

#include "Graph.h"
#include <boost/heap/binomial_heap.hpp>

class PriorityQueue {
 public:
  typedef std::pair<float, float> Key;
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

  PriorityQueue() {};

  IteratorType begin() { return __queue.begin(); };
  IteratorType end() { return __queue.end(); };
  void swap(PriorityQueue &other);
  void insert_or_update(const Node &n, const Key &k);
  void remove_if_present(const Node &n);
  void insert(const Node &n, Key k);
  void clear();
  void pop();
  Key topKey();
  Node topNode();
  int size();
  bool empty();
};

#endif
