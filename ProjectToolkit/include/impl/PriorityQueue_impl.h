
#include <PriorityQueue.h>

template<typename K, typename T>
void PriorityQueue<K,T>::insert(const Value &n, const Key &k) {
    HandleType handle = __queue.emplace(n, k);
    __handles[n] = handle;
}

template<typename K, typename T>
void PriorityQueue<K,T>::clear() {
    __queue.clear();
    __handles.clear();
}

template<typename K, typename T>
void PriorityQueue<K,T>::remove_if_present(const Value &n) {
    auto h_it = __handles.find(n);
    if (h_it != __handles.end()) {
        __queue.erase(h_it->second);
        __handles.erase(h_it);
    }
}

template<typename K, typename T>
void PriorityQueue<K,T>::insert_or_update(const Value &n, const Key &k) {
    auto h_it = __handles.find(n);
    if (h_it != __handles.end()) {
        __queue.update(h_it->second, ElemType(n, k));
    } else insert(n, k);
}

template<typename K, typename T>
void PriorityQueue<K,T>::pop() {
    __handles.erase(__queue.top().elem);
    __queue.pop();
}

template<typename K, typename T>
const typename PriorityQueue<K,T>::Key& PriorityQueue<K,T>::topKey() {
    return __queue.top().key;
}

template<typename K, typename T>
const typename PriorityQueue<K,T>::Value& PriorityQueue<K,T>::topValue() {
    return __queue.top().elem;
}

template<typename K, typename T>
int PriorityQueue<K,T>::size() {
    return __queue.size();
}

template<typename K, typename T>
bool PriorityQueue<K,T>::empty() {
    return __queue.empty();
}

template<typename K, typename T>
void PriorityQueue<K,T>::swap(PriorityQueue<K,T> &other) {
    __queue.swap(other.__queue);
    __handles.swap(other.__handles);
}
