#include <cmath>
template<typename E, typename I>
typename ExpandedMap<E,I>::iterator
ExpandedMap<E,I>::find_or_init(const ElemType &n) {
    iterator it = find(n);
    if (it == end()) { // Init node if not yet considered
        it = emplace(n, std::make_tuple(INFINITY, INFINITY, NULLINFO)).first;
    }
    return it;
}

template<typename E, typename I>
typename ExpandedMap<E,I>::iterator
ExpandedMap<E,I>::insert_or_assign(const ElemType &s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    auto it = find(s);
    if (it != end()) {
        std::get<0>(it->second) = g;
        std::get<1>(it->second) = rhs;
        return it;
    } else {
        bool ok;
        std::tie(it, ok) = emplace(s, std::make_tuple(g, rhs, NULLINFO));
        assert(ok); (void) ok;
        return it;
    }
}

template<typename E, typename I>
std::pair<float, float> ExpandedMap<E,I>::getGandRHS(const ElemType &s) {
    iterator it;
    if ((it = find(s)) != end())
        return {G(it), RHS(it)};
    else
        return {INFINITY, INFINITY};
}

template<typename E, typename I>
float ExpandedMap<E,I>::getG(const ElemType &s) {
    iterator it;
    if ((it = find(s)) != end())
        return G(it);
    else
        return INFINITY;
}

template<typename E, typename I>
float ExpandedMap<E,I>::getRHS(const ElemType &s) {
    iterator it;
    if ((it = find(s)) != end())
        return RHS(it);
    else
        return INFINITY;
}

template<typename iterator>
const auto &ELEM(const iterator &map_it) { return (map_it)->first; }

template<typename iterator>
float &G(const iterator &map_it) { return std::get<0>((map_it)->second); }

template<typename iterator>
float &RHS(const iterator &map_it) { return std::get<1>((map_it)->second); }

template<typename iterator>
auto &INFO(const iterator &map_it) { return std::get<2>((map_it)->second); }
