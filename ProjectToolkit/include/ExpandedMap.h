#ifndef RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H
#define RONCAPAT_GLOBAL_PLANNERS_EXPANDEDMAP_H

#include <robin_hood.h>

template<typename ElemType_, typename InfoType_>
using hashmap_ = robin_hood::unordered_node_map<ElemType_, std::tuple<float, float, InfoType_>>;

template<typename ElemType_, typename InfoType_>
class ExpandedMap {
 public:
  typedef ElemType_ ElemType;
  typedef InfoType_ InfoType;
  using iterator = typename hashmap_<ElemType_, InfoType_>::iterator;
  using const_iterator = typename hashmap_<ElemType_, InfoType_>::const_iterator;

  ExpandedMap() = default;
  ExpandedMap(unsigned int x,
              unsigned int y,
              unsigned char bits /* tile size: 2^bits * 2^bits */);
  std::vector<hashmap_<ElemType_, InfoType_>> buckets;
 private:
  inline int get_bucket_idx(const ElemType &s) const {
      if (s.x < 0 or s.y < 0) return -1;
      return (s.y >> bits) * dim_x + (s.x >> bits);
  }

  inline bool check_bucket_existence(int id) const {
      return (id >= 0 and id < (signed)buckets.size());
  }
  unsigned char bits; /* tile size: 2^bits * 2^bits */
  unsigned char dim_x, dim_y;
 public:
  const InfoType NULLINFO = InfoType{};

  iterator find_or_init(const ElemType &n);

  iterator insert_or_assign(const ElemType &s, float g, float rhs);

  std::optional<iterator> find(const ElemType &n);

  size_t size();

  void init(unsigned int x, unsigned int y, unsigned char bits);
  void clear() noexcept;

  float get_g(const ElemType &s) const;
  float get_rhs(const ElemType &s) const;
  float get_interp_rhs(const Node &s) const;
  std::pair<float, float> get_g_rhs(const ElemType &s) const;

  bool consistent(const ElemType &s);
};

template<typename IT>
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
