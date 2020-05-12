//
// Created by patrick on 08/05/20.
//

#ifndef RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
#define RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H

#include <unordered_map>

template<typename ElemType_, typename InfoType_>
class ExpandedMap : public std::unordered_map<ElemType_, std::tuple<float, float, InfoType_>> {
 public:
  using std::unordered_map<ElemType_, std::tuple<float, float, InfoType_>>::begin;
  using std::unordered_map<ElemType_, std::tuple<float, float, InfoType_>>::end;
  using std::unordered_map<ElemType_, std::tuple<float, float, InfoType_>>::find;
  using std::unordered_map<ElemType_, std::tuple<float, float, InfoType_>>::emplace;
  using typename std::unordered_map<ElemType_, std::tuple<float, float, InfoType_>>::iterator;
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
using ElemType = typename std::iterator_traits<iterator>::value_type::first_type;

template<class iterator>
using TupleType = typename std::iterator_traits<iterator>::value_type::second_type;

template <class iterator>
using InfoType = typename std::tuple_element<2,TupleType<iterator>>::type;

template<class iterator>
static inline const ElemType<iterator> &ELEM(const iterator &map_it);

template<class iterator>
static inline float &G(const iterator &map_it);

template<class iterator>
static inline float &RHS(const iterator &map_it);

template<class iterator>
static inline InfoType<iterator> &INFO(const iterator &map_it);
#include "impl/ExpandedMap_impl.h"
#endif //RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
