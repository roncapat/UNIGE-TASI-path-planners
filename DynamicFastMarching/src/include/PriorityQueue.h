#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <algorithm>
#include <functional>
#include <set>
#include <utility>

#include "Graph.h"
#include <boost/heap/fibonacci_heap.hpp>

class PriorityQueue {
 public:
  PriorityQueue(){};
  typedef std::pair<float, float> Key;

  class ElemType {
   public:
    Cell cell;
    Key key;
    ElemType(const Cell &cell, const Key &key) : cell(cell), key(key){};
  };

 private:
  struct comparator {
    inline bool operator()
        (const ElemType &c1, const ElemType &c2) const {
        return c1.key >= c2.key;
    }
  };

  typedef boost::heap::compare<comparator> CompareOption;
  typedef boost::heap::mutable_<true> MutableOption;
  typedef boost::heap::fibonacci_heap<ElemType, CompareOption, MutableOption> QueueType;
  QueueType __queue;
  std::unordered_map<Cell, QueueType::handle_type> __handles;

 public:
  typedef QueueType::handle_type HandleType;
  typedef QueueType::iterator IteratorType;

  IteratorType begin() {
      return __queue.begin();
  };
  IteratorType end() {
      return __queue.end();
  };

  void swap(PriorityQueue &other);
  void insert_or_update(const Cell &n, const Key &k);
  void remove_if_present(const Cell &n);
  void insert(const Cell &n, const Key &k);
  void clear();
  void pop();
  Key topKey();
  Cell topCell();
  int size();
  bool empty();
  bool contains(const Cell&c);
};

#endif
