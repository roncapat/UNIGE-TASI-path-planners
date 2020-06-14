
#include <PriorityQueue.h>

template<typename K, typename T>
void PriorityQueue<K,T>::insert(const Value &n, const Key &k) {
    handles[n] = queue.emplace(n, k);
}

template<typename K, typename T>
void PriorityQueue<K,T>::clear() {
    queue.clear();
    handles.clear();
}

template<typename K, typename T>
void PriorityQueue<K,T>::remove_if_present(const Value &n) {
    auto h_it = handles.find(n);
    if (h_it != handles.end()) {
        queue.erase(h_it->second);
        handles.erase(h_it);
    }
}

template<typename K, typename T>
void PriorityQueue<K,T>::insert_or_update(const Value &n, const Key &k) {
    auto h_it = handles.find(n);
    if (h_it != handles.end()) {
        queue.update(h_it->second, ElemType(n, k));
    } else insert(n, k);
}

template<typename K, typename T>
void PriorityQueue<K,T>::pop() {
    handles.erase(queue.top().elem);
    queue.pop();
}

template<typename K, typename T>
const typename PriorityQueue<K,T>::Key& PriorityQueue<K,T>::top_key() {
    return queue.top().key;
}

template<typename K, typename T>
const typename PriorityQueue<K,T>::Value& PriorityQueue<K,T>::top_value() {
    return queue.top().elem;
}

template<typename K, typename T>
int PriorityQueue<K,T>::size() {
    return queue.size();
}

template<typename K, typename T>
bool PriorityQueue<K,T>::empty() {
    return queue.empty();
}

template<typename K, typename T>
void PriorityQueue<K,T>::swap(PriorityQueue<K,T> &other) {
    queue.swap(other.queue);
    handles.swap(other.handles);
}
