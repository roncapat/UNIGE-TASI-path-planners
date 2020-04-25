#include "PriorityQueue.h"

PriorityQueue::PriorityQueue() : pq_(compFunctor) {
}

bool PriorityQueue::contains(const Node &n) {
    return find(n) != pq_.end();
}

void PriorityQueue::insert(const Node &n, Key k) {
    pq_.insert({n, k});
}

void PriorityQueue::clear() {
    pq_.clear();
}

void PriorityQueue::remove(const Node &n) {
    auto it = this->find(n);
    if (it != pq_.end())
        pq_.erase(it);
}

void PriorityQueue::pop() {
    if (this->size() > 0)
        pq_.erase(pq_.begin());
}

Key PriorityQueue::topKey() {
    return pq_.begin()->second;
}

Node PriorityQueue::topNode() {
    return pq_.begin()->first;
}

int PriorityQueue::size() {
    return pq_.size();
}

bool PriorityQueue::empty() {
    return pq_.empty();
}

std::set<std::pair<Node, Key>>::iterator PriorityQueue::find(const Node &n) {
    return std::find_if(pq_.begin(),
                        pq_.end(),
                        [&n](auto &e) { return e.first == n; });
}