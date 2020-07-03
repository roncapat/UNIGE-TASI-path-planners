#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <boost/heap/fibonacci_heap.hpp>
#include <robin_hood.h>

template<typename K, typename V>
class PriorityQueue {
 public:
  using Key = K;
  using Value = V;

 private:
  struct ElemType {
    Value elem;
    Key key;
  };

  struct comparator {
    inline bool operator()
        (const ElemType &c1, const ElemType &c2) const {
        return c1.key >= c2.key;
    }
  };

  template<typename T, class ...Options>
  using heap = boost::heap::fibonacci_heap<T, Options...>;
  template<typename A, typename B>
  using map = robin_hood::unordered_flat_map<A, B>;

  using CompareOption = boost::heap::compare<comparator>;
  using MutableOption = boost::heap::mutable_<true>;

  using QueueType = heap<ElemType, CompareOption, MutableOption>;
  using MapType = map<Value, typename QueueType::handle_type>;

  QueueType queue;
  MapType handles;

 public:
  PriorityQueue() = default;

  using IteratorType = typename QueueType::iterator;
  using OrderedIteratorType = typename QueueType::ordered_iterator;

  IteratorType begin();
  IteratorType end();
  OrderedIteratorType ordered_begin();
  OrderedIteratorType ordered_end();

  void pop();
  void clear();
  void swap(PriorityQueue &other);
  void insert(const Value &n, const Key &k);
  void insert_or_update(const Value &n, const Key &k);
  void remove_if_present(const Value &n);

  const Key &top_key() const;
  const Value &top_value() const;
  int size() const;
  bool empty() const;
};

#include "impl/PriorityQueue_impl.h"
#endif /* PRIORITYQUEUE_H */
