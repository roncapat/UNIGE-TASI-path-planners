#include <cmath>
#include <ExpandedMap.h>

template<typename E, typename I>
typename ExpandedMap<E, I>::iterator
ExpandedMap<E, I>::find_or_init(const ElemType &n) {
    auto idx = get_bucket_idx(n);
    assert(idx >= 0 and idx < (signed)buckets.size());
    auto it = buckets[idx].find(n);
    if (it == buckets[idx].end()) // Init node if not yet considered
        it = buckets[idx].emplace(n, std::make_tuple(INFINITY, INFINITY, NULLINFO)).first;
    return it;
}

template<typename E, typename I>
typename ExpandedMap<E, I>::iterator
ExpandedMap<E, I>::insert_or_assign(const ElemType &s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    auto idx = get_bucket_idx(s);
    assert(idx >= 0 and idx < (signed)buckets.size());
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end()) {
        std::get<0>(it->second) = g;
        std::get<1>(it->second) = rhs;
        return it;
    } else {
        bool ok;
        std::tie(it, ok) = buckets[idx].emplace(s, std::make_tuple(g, rhs, NULLINFO));
        assert(ok);
        (void) ok;
        return it;
    }
}

template<typename E, typename I>
std::pair<float, float> ExpandedMap<E, I>::get_g_rhs(const ElemType &s) const {
    auto idx = get_bucket_idx(s);
    if (idx == -1 or idx >= (signed)buckets.size()) return {INFINITY, INFINITY};
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end())
        return {G(it), RHS(it)};
    else
        return {INFINITY, INFINITY};
}

template<typename E, typename I>
float ExpandedMap<E, I>::get_g(const ElemType &s) const {
    auto idx = get_bucket_idx(s);
    if (idx == -1 or idx >= (signed)buckets.size()) return INFINITY;
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end())
        return G(it);
    else
        return INFINITY;
}

template<typename E, typename I>
float ExpandedMap<E, I>::get_rhs(const ElemType &s) const {
    auto idx = get_bucket_idx(s);
    if (idx == -1 or idx >= (signed)buckets.size()) return INFINITY;
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end())
        return RHS(it);
    else
        return INFINITY;
}

#include <Interpolation.h>
template<typename E, typename I>
float ExpandedMap<E, I>::get_interp_rhs(const Node &s) const {
    if constexpr (std::is_same_v<E, Cell>){
        Cell p(std::floor(s.x - 0.5), std::floor(s.y - 0.5));
        float a = get_rhs(p.bottom_cell()), b = get_rhs(p), c = get_rhs(p.bottom_right_cell()), d = get_rhs(p.right_cell());
        return (a+b+c+d)*0.25;
    } else if constexpr (std::is_same_v<E,Node>){
        return get_rhs(s);
    } else assert(0);
}

template<typename ElemType_, typename InfoType_>
ExpandedMap<ElemType_, InfoType_>::ExpandedMap(const unsigned int x, const unsigned int y, const unsigned char bits)
    : bits(bits) {
    init(x, y, bits);
}
template<typename E, typename I>
std::optional<typename ExpandedMap<E, I>::iterator> ExpandedMap<E, I>::find(const E &n) {
    auto idx = get_bucket_idx(n);
    if (idx == -1 or idx >= (signed)buckets.size()) return {};
    iterator it = buckets[idx].find(n);
    if (it != buckets[idx].end())
        return it;
    else return {};
}

template<typename E, typename I>
size_t ExpandedMap<E, I>::size() {
    size_t size = 0;
    for (auto b : buckets) size += b.size();
    return size;
}
template<typename E, typename I>
void ExpandedMap<E, I>::clear() noexcept {
    buckets.clear();
}
template<typename E, typename I>
void ExpandedMap<E, I>::init(unsigned int x, unsigned int y, unsigned char bits) {
    this->bits = bits;
    dim_x = (x >> bits) + 1;
    dim_y = (y >> bits) + 1;
    //std::cout << dim_x << " " << dim_y << std::endl;
    buckets.reserve(dim_x * dim_y);
    for (unsigned int i = 0; i < (dim_x * dim_y); ++i)
        buckets.emplace_back();
}
template<typename ElemType_, typename InfoType_>
bool ExpandedMap<ElemType_, InfoType_>::consistent(const ElemType &s) {
    auto[g, rhs] = get_g_rhs(s);
    return g == rhs;
}

template<typename iterator>
const auto &ELEM(const iterator &map_it) { return (map_it)->first; }

template<typename iterator, std::enable_if_t<not is_const_iterator<iterator>::value, int>>
float &G(const iterator &map_it) { return std::get<0>((map_it)->second); }

template<typename iterator, std::enable_if_t<is_const_iterator<iterator>::value, int>>
const float &G(const iterator &map_it) { return std::get<0>((map_it)->second); }

template<typename iterator, std::enable_if_t<not is_const_iterator<iterator>::value, int>>
float &RHS(const iterator &map_it) { return std::get<1>((map_it)->second); }

template<typename iterator, std::enable_if_t<is_const_iterator<iterator>::value, int>>
const float &RHS(const iterator &map_it) { return std::get<1>((map_it)->second); }

template<typename iterator, std::enable_if_t<not is_const_iterator<iterator>::value, int>>
auto &INFO(const iterator &map_it) { return std::get<2>((map_it)->second); }

template<typename iterator, std::enable_if_t<is_const_iterator<iterator>::value, int>>
const auto &INFO(const iterator &map_it) { return std::get<2>((map_it)->second); }

template<typename iterator>
bool &CONSISTENT(const iterator &map_it) { return G(map_it) == RHS(map_it); }