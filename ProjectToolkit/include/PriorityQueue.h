#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <algorithm>
#include <functional>
#include <set>
#include <utility>
#include <unordered_map>

#include "Graph.h"
#include <boost/heap/fibonacci_heap.hpp>

template <typename K, typename T>
class PriorityQueue {
 public:
  typedef K Key;
  typedef T Value;

  class ElemType {
   public:
    Value elem;
    Key key;
    ElemType(const Value &cell, const Key &key) : elem(cell), key(key){};
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
  std::unordered_map<Value, typename QueueType::handle_type> __handles;

 public:
  typedef typename QueueType::handle_type HandleType;
  typedef typename QueueType::iterator IteratorType;

  PriorityQueue() = default;

  IteratorType begin() {
      return __queue.begin();
  };
  IteratorType end() {
      return __queue.end();
  };

  void swap(PriorityQueue &other);
  void insert_or_update(const Value &n, const Key &k);
  void remove_if_present(const Value &n);
  void insert(const Value &n, const Key &k);
  void clear();
  void pop();
  Key topKey();
  Value topValue();
  int size();
  bool empty();
};
#include "impl/PriorityQueue_impl.h"
#endif
