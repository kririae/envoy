#ifndef __ENVOY_H__
#define __ENVOY_H__

#include <spdlog/spdlog.h>
#include <spdlog/fmt/ostr.h>

#include <filesystem>

#undef NDEBUG

#if !defined(EVY_NAMESPACE_BEGIN)
#define EVY_NAMESPACE_BEGIN namespace envoy {
#endif

#if !defined(EVY_NAMESPACE_END)
#define EVY_NAMESPACE_END }  // namespace envoy
#endif

#define Info(...)              \
  do {                         \
    spdlog::info(__VA_ARGS__); \
  } while (false)

#define Warn(...)              \
  do {                         \
    spdlog::warn(__VA_ARGS__); \
  } while (false)

#define Critical(...)              \
  do {                             \
    spdlog::critical(__VA_ARGS__); \
  } while (false)

#define Error(...)              \
  do {                          \
    spdlog::error(__VA_ARGS__); \
  } while (false)

// #define EVY_FORCEINLINE __always_inline
#define EVY_FORCEINLINE inline

#ifndef EVY_TEST_ASSET_PATH
#define EVY_TEST_ASSET_PATH "."
#endif

EVY_FORCEINLINE std::filesystem::path __get_asset_path(
    const std::string &asset_name) {
  const std::filesystem::path asset_path = EVY_TEST_ASSET_PATH;
  return asset_path / asset_name;
}

#endif
