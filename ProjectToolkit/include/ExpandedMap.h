#ifndef RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
#define RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H

#include <robin_hood.h>

template<typename ElemType_, typename InfoType_>
using hashmap_ = robin_hood::unordered_node_map<ElemType_, std::tuple<float, float, InfoType_>>;

template<typename ElemType_, typename InfoType_>
class ExpandedMap : public hashmap_<ElemType_,InfoType_> {
 public:
  using hashmap_<ElemType_,InfoType_>::begin;
  using hashmap_<ElemType_,InfoType_>::end;
  using hashmap_<ElemType_,InfoType_>::find;
  using hashmap_<ElemType_,InfoType_>::emplace;
  using typename hashmap_<ElemType_,InfoType_>::iterator;
  typedef ElemType_ ElemType;
  typedef InfoType_ InfoType;
 public:
  const InfoType NULLINFO = InfoType{};
  iterator find_or_init(const ElemType &n);
  iterator insert_or_assign(const ElemType &s, float g, float rhs);
  float getG(const ElemType &s);
  float getRHS(const ElemType &s);
  std::pair<float, float> getGandRHS(const ElemType &s);
};

template<class iterator>
static inline const auto &ELEM(const iterator &map_it);

template<class iterator>
static inline float &G(const iterator &map_it);

template<class iterator>
static inline float &RHS(const iterator &map_it);

template<class iterator>
static inline auto &INFO(const iterator &map_it);
#include "impl/ExpandedMap_impl.h"
#endif //RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
