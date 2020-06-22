#ifndef RONCAPAT_PLANNER_TOOLKIT_EXPANDEDMAP_H
#define RONCAPAT_PLANNER_TOOLKIT_EXPANDEDMAP_H

#include <robin_hood.h>
#include "Macros.h"

template<typename>
struct tag {};

struct empty {};

template<typename ElemType_, typename InfoType_>
class ExpandedMap {
  using value_ = typename std::conditional<std::is_same<InfoType_, void>::value,
                                           std::tuple<float, float>,
                                           std::tuple<float, float, InfoType_>>::type;
  using hashmap_ = robin_hood::unordered_node_map<ElemType_, value_>;
 public:
  typedef ElemType_ ElemType;
  typedef InfoType_ InfoType;
  using nodeptr = robin_hood::pair<const ElemType_, value_> *;
  using const_nodeptr = robin_hood::pair<const ElemType_, value_> const *;
 private:
  using iterator = typename hashmap_::iterator;
  using const_iterator = typename hashmap_::const_iterator;
 public:
  ExpandedMap() = default;
  ExpandedMap(unsigned int x,
              unsigned int y,
              unsigned char bits /* tile size: 2^bits * 2^bits */);
  std::vector<hashmap_> buckets;
 private:
  inline int get_bucket_idx(const ElemType &s) const {
      if (s.x < 0 or s.y < 0) return -1;
      return (s.y >> bits) * dim_x + (s.x >> bits);
  }

  inline bool check_bucket_existence(int id) const {
      return (id >= 0 and id < (signed) buckets.size());
  }
  unsigned char bits; /* tile size: 2^bits * 2^bits */
  unsigned char dim_x, dim_y;
 public:
  nodeptr find_or_init(const ElemType &n);
  nodeptr insert_or_assign(const ElemType &s, float g, float rhs);
  optional<nodeptr> find(const ElemType &n);
  size_t size() const;
  void init(unsigned int x, unsigned int y, unsigned char bits);
  void clear() noexcept;
  float get_g(const ElemType &s) const;
  float get_rhs(const ElemType &s) const;
  float get_interp_rhs(const Node &s) const;
  std::pair<float, float> get_g_rhs(const ElemType &s) const;
  bool consistent(const ElemType &s) const;
 private:
  float get_interp_rhs(const Node &s, tag<Node>) const;
  float get_interp_rhs(const Node &s, tag<Cell>) const;
  template<bool cond, typename U>
  using resolvedType = typename std::enable_if<cond, U>::type;
  template<typename U = InfoType_>
  resolvedType<std::is_same<U, void>::value, nodeptr> insert(const ElemType &s, float g, float rhs, int bucket_idx);
  template<typename U = InfoType_>
  resolvedType<not std::is_same<U, void>::value, nodeptr> insert(const ElemType &s, float g, float rhs, int bucket_idx);
};

/* this checks if we are dealing with a "const iterator" or a "raw pointer to a const" */
/* iterator_traits are specialized in the standard to play nicely also with raw pointers */
template<typename IT>
using refers_to_const = std::is_const<typename std::remove_reference<typename std::iterator_traits<IT>::reference>::type>;

/* shorthand for SFINAE selection of the read-only accessor variant */
template<typename IT>
using use_for_refs_to_const = std::enable_if<refers_to_const<IT>::value, int>;

/* shorthand for SFINAE selection of the read-write accessor variant */
template<typename IT>
using use_for_refs_to_mutable = std::enable_if<not refers_to_const<IT>::value, int>;


template<typename T>
using const_ref = std::add_lvalue_reference<const T>;

template<typename T>
using mut_ref = std::add_lvalue_reference<T>;

template<class nodeptr>
static inline auto ELEM(const nodeptr &it) -> typename const_ref<decltype((it)->first)>::type;

template<typename nodeptr, typename use_for_refs_to_mutable<nodeptr>::type = 0>
static inline float &G(const nodeptr &it);

template<typename nodeptr, typename use_for_refs_to_const<nodeptr>::type = 0>
static inline const float &G(const nodeptr &it);

template<typename nodeptr, typename use_for_refs_to_mutable<nodeptr>::type = 0>
static inline float &RHS(const nodeptr &it);

template<typename nodeptr, typename use_for_refs_to_const<nodeptr>::type = 0>
static inline const float &RHS(const nodeptr &it);

template<typename nodeptr, typename use_for_refs_to_mutable<nodeptr>::type = 0>
static inline auto INFO(const nodeptr &it) -> typename mut_ref<decltype(std::get<2>((it)->second))>::type;

template<typename nodeptr, typename use_for_refs_to_const<nodeptr>::type = 0>
static inline auto INFO(const nodeptr &it) -> typename const_ref<decltype(std::get<2>((it)->second))>::type;

template<class nodeptr>
static inline bool CONSISTENT(const nodeptr &it);

#include "impl/ExpandedMap_impl.h"

#endif //RONCAPAT_PLANNER_TOOLKIT_EXPANDEDMAP_H
