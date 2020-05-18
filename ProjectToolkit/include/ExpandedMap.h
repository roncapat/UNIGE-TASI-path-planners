#ifndef RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
#define RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H

#include <robin_hood.h>

template<typename ElemType_, typename InfoType_>
using hashmap_ = robin_hood::unordered_node_map<ElemType_, std::tuple<float, float, InfoType_>>;

template<typename ElemType_, typename InfoType_>
class ExpandedMap : public hashmap_<ElemType_, InfoType_> {
public:
    using hashmap_<ElemType_, InfoType_>::begin;
    using hashmap_<ElemType_, InfoType_>::end;
    using hashmap_<ElemType_, InfoType_>::find;
    using hashmap_<ElemType_, InfoType_>::emplace;
    using typename hashmap_<ElemType_, InfoType_>::iterator;
    using typename hashmap_<ElemType_, InfoType_>::const_iterator;
    typedef ElemType_ ElemType;
    typedef InfoType_ InfoType;
public:
    const InfoType NULLINFO = InfoType{};

    iterator find_or_init(const ElemType &n);

    iterator insert_or_assign(const ElemType &s, float g, float rhs);

    float getG(const ElemType &s) const;

    float getRHS(const ElemType &s) const;

    std::pair<float, float> getGandRHS(const ElemType &s) const;

    bool consistent(const Node &s) {
        auto[g, rhs] = getGandRHS(s);
        return g == rhs;
    }
};

template <typename IT>
using is_const_iterator =
std::is_const<typename std::remove_reference<typename std::iterator_traits<IT>::reference>::type>;

template<class iterator>
static inline const auto &ELEM(const iterator &map_it);

template<typename iterator, std::enable_if_t<not is_const_iterator<iterator>::value, int> = 0>
static inline float &G(const iterator &map_it);

template<typename iterator, std::enable_if_t<is_const_iterator<iterator>::value, int> = 0>
static inline const float &G(const iterator &map_it);

template<typename iterator, std::enable_if_t<not is_const_iterator<iterator>::value, int> = 0>
static inline float &RHS(const iterator &map_it);

template<typename iterator, std::enable_if_t<is_const_iterator<iterator>::value, int> = 0>
static inline const float &RHS(const iterator &map_it);

template<typename iterator, std::enable_if_t<not is_const_iterator<iterator>::value, int> = 0>
static inline auto &INFO(const iterator &map_it);

template<typename iterator, std::enable_if_t<is_const_iterator<iterator>::value, int> = 0>
static inline const auto &INFO(const iterator &map_it);

template<class iterator>
static inline bool &CONSISTENT(const iterator &map_it);

#include "impl/ExpandedMap_impl.h"

#endif //RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
