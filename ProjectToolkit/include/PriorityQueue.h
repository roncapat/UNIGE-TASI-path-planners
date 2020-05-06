#ifndef PRIORITYQUEUE_H
#define PRIORITYQUEUE_H

#include <algorithm>
#include <functional>
#include <set>
#include <utility>
#include <unordered_map>

#include "Graph.h"
#include <boost/heap/fibonacci_heap.hpp>

template <typename T>
class PriorityQueue {
 public:
  typedef std::pair<float, float> Key;
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

template <typename T>
void PriorityQueue<T>::insert(const Value &n, const Key &k) {
    HandleType handle = __queue.emplace(n, k);
    __handles[n] = handle;
}

template <typename T>
void PriorityQueue<T>::clear() {
    __queue.clear();
    __handles.clear();
}

template <typename T>
void PriorityQueue<T>::remove_if_present(const Value &n) {
    auto h_it = __handles.find(n);
    if (h_it != __handles.end()) {
        __queue.erase(h_it->second);
        __handles.erase(h_it);
    }
}

template <typename T>
void PriorityQueue<T>::insert_or_update(const Value &n, const Key &k) {
    auto h_it = __handles.find(n);
    if (h_it != __handles.end()) {
        __queue.update(h_it->second, ElemType(n, k));
    } else insert(n, k);
}

template <typename T>
void PriorityQueue<T>::pop() {
    __handles.erase(__queue.top().elem);
    __queue.pop();
}

template <typename T>
typename PriorityQueue<T>::Key PriorityQueue<T>::topKey() {
    return __queue.top().key;
}

template <typename T>
typename PriorityQueue<T>::Value PriorityQueue<T>::topValue() {
    return __queue.top().elem;
}

template <typename T>
int PriorityQueue<T>::size() {
    return __queue.size();
}

template <typename T>
bool PriorityQueue<T>::empty() {
    return __queue.empty();
}

template <typename T>
void PriorityQueue<T>::swap(PriorityQueue<T> &other) {
    __queue.swap(other.__queue);
    __handles.swap(other.__handles);
}

#endif
