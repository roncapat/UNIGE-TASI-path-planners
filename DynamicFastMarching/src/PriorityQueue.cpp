#include "PriorityQueue.h"

void PriorityQueue::insert(const Cell &n, const Key &k) {
    HandleType handle = __queue.emplace(n, k);
    __handles[n] = handle;
}

void PriorityQueue::clear() {
    __queue.clear();
    __handles.clear();
}

void PriorityQueue::remove_if_present(const Cell &n) {
    auto h_it = __handles.find(n);
    if (h_it != __handles.end()) {
        __queue.erase(h_it->second);
        __handles.erase(h_it);
    }
}

void PriorityQueue::insert_or_update(const Cell &n, const Key &k) {
    auto h_it = __handles.find(n);
    if (h_it != __handles.end()) {
        __queue.update(h_it->second, ElemType(n, k));
    } else insert(n, k);
}

void PriorityQueue::pop() {
    __handles.erase(__queue.top().cell);
    __queue.pop();
}

PriorityQueue::Key PriorityQueue::topKey() {
    return __queue.top().key;
}

Cell PriorityQueue::topCell() {
    return __queue.top().cell;
}

int PriorityQueue::size() {
    return __queue.size();
}

bool PriorityQueue::empty() {
    return __queue.empty();
}

void PriorityQueue::swap(PriorityQueue &other) {
    __queue.swap(other.__queue);
    __handles.swap(other.__handles);
}
bool PriorityQueue::contains(const Cell &c) {
    return __handles.find(c) != __handles.end();
}
