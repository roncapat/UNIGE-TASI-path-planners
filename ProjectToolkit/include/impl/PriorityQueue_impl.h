
#include <PriorityQueue.h>

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
