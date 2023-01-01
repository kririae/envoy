#ifndef __RESOURCE_MANAGER_H__
#define __RESOURCE_MANAGER_H__

#include <oneapi/tbb/cache_aligned_allocator.h>
#include <oneapi/tbb/scalable_allocator.h>

#include <boost/container/flat_map.hpp>

#include "envoy_common.h"
#include "pages.h"
#include "stats.h"

EVY_NAMESPACE_BEGIN

namespace detail_ {
struct DestructorBase {
  virtual ~DestructorBase() {}
};
template <typename T>
requires(!std::is_array_v<T>) struct Destructor : public DestructorBase {
  Destructor(T *p, size_t alignment) : m_p(p), m_alignment(alignment) {}
  ~Destructor() override { std::destroy_at(m_p); }
  T          *m_p;
  std::size_t m_alignment;
};
template <typename T, typename T_ = std::remove_extent_t<T>>
requires(std::is_unbounded_array_v<T>) struct ArrayDestructor
    : public DestructorBase {
  ArrayDestructor(T_ *p, size_t n, size_t alignment)
      : m_p(p), m_n(n), m_alignment(alignment) {}
  ~ArrayDestructor() override { std::destroy_n(m_p, m_n); }
  T_    *m_p;
  size_t m_n, m_alignment;
};
};  // namespace detail_

/**
 * Generic Memory Resource
 */
struct GResource {
  GResource() = default;  // use local scalable_resource
  // GResource(std::pmr::memory_resource *upstream)
  //   : m_upstream(upstream), m_mem_resource(&m_upstream) {}
  GResource(const GResource &)             = delete;
  GResource &operator()(const GResource &) = delete;
  virtual ~GResource() { release(); }

  /**
   * @brief Allocate memory using the memory resource manager and the
   * constructor parameters
   * @note The interface is exactly the same as make_unique since C++ 14.
   * @tparam T
   * @tparam Args
   * @return T* A pointer to the allocated memory fragment
   */
  template <typename T, size_t Align = alignof(T), typename... Args>
  std::enable_if_t<!std::is_array<T>::value, T *> alloc(Args &&...args) {
    auto allocator = std::pmr::polymorphic_allocator<T>(&m_mem_resource);
    T   *mem = static_cast<T *>(allocator.allocate_bytes(sizeof(T), Align));
    assert(((size_t)(void *)mem) % Align == 0);
    allocator.construct(
        mem, std::forward<Args>(
                 args)...);  // instead of placement new and new_object
                             // if (!std::is_trivially_destructible_v<T>)
    m_destructors.emplace(static_cast<ptr_t>(mem),
                          new detail_::Destructor<T>(mem, Align));
    stats::RecResourceAllocateInfo(sizeof(T));
    return mem;
  }

  /**
   * @brief Allocate array using the memory resource manager and the size of the
   * array
   * @see The previous `alloc`
   * @note The implementation is quite different from unique_ptr, which accepts
   * a constructor for every element in the array.
   *
   * @tparam T
   * @tparam Args
   * @param n The number of elements in the array.
   * @param args
   * @return T* A pointer to the allocated memory fragment
   */
  template <typename T, size_t Align = alignof(T),
            typename T_ = std::remove_extent_t<T>, typename... Args>
  std::enable_if_t<std::is_unbounded_array_v<T>, T_ *> alloc(size_t n,
                                                             Args &&...args) {
    auto allocator = std::pmr::polymorphic_allocator<T_>(&m_mem_resource);
    T_  *mem =
        static_cast<T_ *>(allocator.allocate_bytes(sizeof(T_) * n, Align));
    assert(((size_t)(void *)mem) % Align == 0);
    if constexpr (sizeof...(args) == 0) {
      new (mem) T_[n];
    } else {
      for (size_t i = 0; i < n; ++i)
        allocator.construct(mem + i, std::forward<Args>(args)...);
    }  // if constexpr
    m_destructors.emplace(static_cast<ptr_t>(mem),
                          new detail_::ArrayDestructor<T>(mem, n, Align));
    stats::RecResourceAllocateInfo(sizeof(T_) * n);
    return mem;
  }

  /**
   * @brief dealloc the memories related to the pointer and invoke the
   * destructors
   *
   * @tparam T
   * @param ptr
   */
  template <typename T>
  void dealloc(T *ptr) {
    auto allocator = std::pmr::polymorphic_allocator<T>(&m_mem_resource);
    auto iterator  = m_destructors.find(ptr);
    if (iterator == m_destructors.end()) return;

    detail_::DestructorBase *dbase = iterator->second;
    detail_::Destructor<T>  *d = dynamic_cast<detail_::Destructor<T> *>(dbase);
    if (d != nullptr) {
      // the DestructorBase is Destructor
      assert(ptr == d->m_p);
      allocator.deallocate_bytes(ptr, sizeof(T), d->m_alignment);
    } else {
      detail_::ArrayDestructor<T[]> *darray =
          dynamic_cast<detail_::ArrayDestructor<T[]> *>(dbase);
      assert(ptr == darray->m_p);
      allocator.deallocate_bytes(ptr, sizeof(T) * darray->m_n,
                                 darray->m_alignment);
    }

    delete dbase;
    m_destructors.erase(iterator);  // then destructor is invoked
  }

  /**
   * @brief Deallocate and destruct all resource allocated
   */
  void release() {
    for (auto &i : m_destructors) delete i.second;
    m_destructors.clear();
    m_mem_resource.release();
  }
  void printStat() { Info("PrintStat() is not implemented yet"); }

protected:
  using ptr_t = void *;
  boost::container::flat_map<ptr_t, detail_::DestructorBase *> m_destructors;
  tbb::cache_aligned_resource                                  m_upstream{
      oneapi::tbb::scalable_memory_resource()};
  std::pmr::unsynchronized_pool_resource m_mem_resource{&m_upstream};
};

// The naive implementation of Page Resource
struct StaticPageResource {
protected:
  using ptr_t = void *;
  boost::container::flat_map<uint64_t, ptr_t> m_pages;
};

EVY_NAMESPACE_END

#endif
