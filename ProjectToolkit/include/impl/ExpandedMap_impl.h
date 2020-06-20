#include <cmath>
#include <ExpandedMap.h>
#include <Interpolation.h>

template<typename E, typename I>
typename ExpandedMap<E, I>::nodeptr
ExpandedMap<E, I>::find_or_init(const ElemType &n) {
    auto idx = get_bucket_idx(n);
    assert(check_bucket_existence(idx));
    auto it = buckets[idx].find(n);
    bool ret;
    if (it == buckets[idx].end()) { // Init node if not yet considered
        std::tie(it, ret) = buckets[idx].emplace(n, std::make_tuple(INFINITY, INFINITY, NULLINFO));
        assert(ret);
        (void) ret;
    }
    return &(*it);
}

template<typename E, typename I>
typename ExpandedMap<E, I>::nodeptr
ExpandedMap<E, I>::insert_or_assign(const ElemType &s, float g, float rhs) {
    // re-assigns value of node in unordered map or inserts new entry
    auto idx = get_bucket_idx(s);
    assert(check_bucket_existence(idx));
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end()) {
        std::get<0>(it->second) = g;
        std::get<1>(it->second) = rhs;
        return &(*it);
    } else {
        bool ok;
        std::tie(it, ok) = buckets[idx].emplace(s, std::make_tuple(g, rhs, NULLINFO));
        assert(ok);
        (void) ok;
        return &(*it);
    }
}

template<typename E, typename I>
std::pair<float, float> ExpandedMap<E, I>::get_g_rhs(const ElemType &s) const {
    auto idx = get_bucket_idx(s);
    if (not check_bucket_existence(idx)) return {INFINITY, INFINITY};
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end())
        return {G(it), RHS(it)};
    else
        return {INFINITY, INFINITY};
}

template<typename E, typename I>
float ExpandedMap<E, I>::get_g(const ElemType &s) const {
    auto idx = get_bucket_idx(s);
    if (not check_bucket_existence(idx)) return INFINITY;
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end())
        return G(&(*it));
    else
        return INFINITY;
}

template<typename E, typename I>
float ExpandedMap<E, I>::get_rhs(const ElemType &s) const {
    auto idx = get_bucket_idx(s);
    if (not check_bucket_existence(idx)) return INFINITY;
    auto it = buckets[idx].find(s);
    if (it != buckets[idx].end())
        return RHS(it);
    else
        return INFINITY;
}

template<typename E, typename I>
float ExpandedMap<E, I>::get_interp_rhs(const Node &s, tag<Cell>) const {
    Cell p(std::floor(s.x - 0.5), std::floor(s.y - 0.5));
    float a = get_rhs(p.bottom_cell()), b = get_rhs(p), c = get_rhs(p.bottom_right_cell()), d = get_rhs(p.right_cell());
    return (a + b + c + d) * 0.25f;
}

template<typename E, typename I>
float ExpandedMap<E, I>::get_interp_rhs(const Node &s, tag<Node>) const {
    return get_rhs(s);
}

template<typename ElemType_, typename InfoType_>
ExpandedMap<ElemType_, InfoType_>::ExpandedMap(const unsigned int x, const unsigned int y, const unsigned char bits) {
    init(x, y, bits);
}
template<typename E, typename I>
optional<typename ExpandedMap<E, I>::nodeptr> ExpandedMap<E, I>::find(const E &n) {
    auto idx = get_bucket_idx(n);
    if (idx == -1 or idx >= (signed) buckets.size()) return {};
    auto it = buckets[idx].find(n);
    if (it != buckets[idx].end()) {
        return &(*it);
    } else return {};
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
    float g, rhs;
    std::tie(g, rhs) = get_g_rhs(s);
    return g == rhs;
}
template<typename ElemType_, typename InfoType_>
float ExpandedMap<ElemType_, InfoType_>::get_interp_rhs(const Node &s) const {
    return get_interp_rhs(s, tag<ElemType>());
}

template<typename nodeptr>
auto ELEM(const nodeptr &it) -> typename const_ref<decltype((it)->first)>::type {
    return (it)->first;
}
//const auto &ELEM(const nodeptr &it) { return (it)->first; } //C++17

template<typename nodeptr, typename use_for_refs_to_mutable<nodeptr>::type >
float &G(const nodeptr &it) {
    return std::get<0>((it)->second);
}

template<typename nodeptr, typename use_for_refs_to_const<nodeptr>::type >
const float &G(const nodeptr &it) {
    return std::get<0>((it)->second);
}

template<typename nodeptr, typename use_for_refs_to_mutable<nodeptr>::type >
float &RHS(const nodeptr &it) {
    return std::get<1>((it)->second);
}

template<typename nodeptr, typename use_for_refs_to_const<nodeptr>::type >
const float &RHS(const nodeptr &it) {
    return std::get<1>((it)->second);
}

template<typename nodeptr, typename use_for_refs_to_mutable<nodeptr>::type >
auto INFO(const nodeptr &it) -> typename mut_ref<decltype(std::get<2>((it)->second))>::type {
    return std::get<2>((it)->second);
}
//auto &INFO(const nodeptr &it) { return std::get<2>((it)->second); } //C++17

template<typename nodeptr, typename use_for_refs_to_const<nodeptr>::type >
auto INFO(const nodeptr &it) -> typename const_ref<decltype(std::get<2>((it)->second))>::type {
    return std::get<2>((it)->second);
}
//const auto &INFO(const nodeptr &it) { return std::get<2>((it)->second); } //C++17

template<typename nodeptr>
bool CONSISTENT(const nodeptr &it) {
    return G(it) == RHS(it);
}