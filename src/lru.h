#ifndef __LRU_H__
#define __LRU_H__

#include <boost/container/flat_map.hpp>
#include <boost/container/list.hpp>
#include <list>
#include <optional>

#include "envoy.h"

EVY_NAMESPACE_BEGIN

/**
 * An simple LRU implementation in envoy
 * I'll not directly use boost's implementation since it is not thread-safe.
 * @note Keys should be different
 */
template <typename TKey, typename TItem>
class LRU {
public:
  using key_type  = TKey;
  using item_type = TItem;
  using list_type = boost::container::list<std::pair<TKey, TItem>>;
  using map_type =
      boost::container::flat_map<TKey, typename list_type::iterator>;

  LRU(std::size_t capacity) : m_capacity(capacity) {}
  ~LRU() = default;
  std::size_t capacity() const { return m_capacity; }
  std::size_t size() const { return m_map.size(); }
  bool        empty() const { return m_map.empty(); }
  bool exists(const TKey &key) const { return m_map.find(key) != m_map.end(); }

  void insert(const TKey &key, const TItem &item) {
    // check if the key already exists
    if (!exists(key)) {
      // insert the key
      if (size() >= capacity()) {
        evict();
      }

      m_list.push_front(std::make_pair(key, item));
      m_map[key] = m_list.begin();
    }
  }

  std::optional<TItem> get(const TKey &key) {
    auto value = m_map.find(key);
    if (value == m_map.end()) {
      return std::nullopt;
    } else {
      // value->second: the iteration
      m_list.splice(m_list.begin(), m_list, value->second);
      return value->second->second; // the TItem
    }
  }

private:
  void evict() {
    auto evict_item = --m_list.end();
    m_map.erase(evict_item->first);
    m_list.pop_back();
  }

  std::size_t m_capacity;
  list_type   m_list;
  map_type    m_map;
};

EVY_NAMESPACE_END

#endif
