-- General Requirements
add_requires("args", "spdlog", "boost", "onetbb", { system = false })

-- Specific Requirements
add_requires("miniply master", "linalg main", "glm", "xsimd", "zpp_bits v4.4.12", "lz4", { system = false })
add_requires("metis", "embree", { system = false })

-- Special case for LGPL-3.0
add_requires("cgal", { system = false, configs = { shared = true } })

target("envoy.core")
  set_kind("shared")
  add_files("mesh.cpp", "pages.cpp", "bvh.cpp", "stats.cpp", "packer.cpp")
  add_packages(
    "args",
    "spdlog",
    "boost",
    "onetbb", --[[ general requirements ]]
    "miniply",
    "linalg",
    "glm",
    "xsimd",
    "zpp_bits",
    "lz4",
    "metis", 
    "embree", --[[ specific requirements ]]
    "cgal",
    { public = true }
  )
  add_includedirs(os.scriptdir(), { public = true })

target("envoy")
  set_kind("binary")
  add_files("envoy.cpp")
  add_deps("envoy.core")
