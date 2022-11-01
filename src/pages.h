#ifndef __PAGES_H__
#define __PAGES_H__

#include "envoy_common.h"
#include "geometry.h"
#include "zpp_bits.h"

EVY_NAMESPACE_BEGIN

constexpr uint32_t    PAGE_SHIFT = 12;
constexpr std::size_t PAGE_SIZE  = (1 << PAGE_SHIFT);

/**
 * @brief An abstraction(view) for system page
 *
 * @tparam T The type to fill this page
 * @tparam N Variable page size for different platforms
 */
template <typename T, std::size_t N = PAGE_SIZE>
struct alignas(PAGE_SIZE) SysPage {
public:
  using ptr_t = SysPage *;

  SysPage() = default;
  SysPage(uint64_t rm_page_id) : addr(this), rm_page_id(rm_page_id) {}

  // Bytes Serialization, hope it works
  friend zpp::bits::access;
  constexpr static auto serialize(auto &archive, auto &self) {
    using archive_type = std::remove_cvref_t<decltype(archive)>;
    auto result        = archive(zpp::bits::as_bytes(self));
    if constexpr (archive_type::kind() == zpp::bits::kind::in)
      self.addr = &self;
    return result;
  }

  static constexpr std::size_t get_element_num() {
    // TODO: consider alignment
    const std::size_t element_size = N - sizeof(ptr_t) - sizeof(rm_page_id);
    const std::size_t element_num =
        std::floor(static_cast<float>(element_size) / sizeof(T));
    return element_num;
  }

  static constexpr std::size_t sys_page_size = N;
  static constexpr std::size_t element_num   = get_element_num();
  ptr_t addr{nullptr};    /* This field is intended to be managed by
                                  resource_manager */
  uint64_t rm_page_id{0}; /* page id to track in resource_manager */
  T        elements[element_num];
};

EVY_NAMESPACE_END

#endif
