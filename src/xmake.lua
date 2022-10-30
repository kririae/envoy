-- General Requirements
add_requires("fmt", "spdlog", "tbb", "glm", "pacman::cgal", "embree")
-- To add boost, python is required
add_requires("boost")

-- Specific Requirements
add_requires("miniply master", "linalg main", "xsimd")
add_requires("zpp_bits v4.4.12", "lz4")

target("envoy.core")
	set_kind("static")
	add_packages("fmt", "spdlog", "miniply", "linalg", { public = true })
	add_packages("zpp_bits", "lz4", { public = true })
	add_packages("tbb", "pacman::cgal", "xsimd", "embree", { public = true })
	add_files("mesh.cpp", "pages.cpp", "bvh.cpp")
	add_includedirs(os.scriptdir(), { public = true })

target("envoy")
	set_kind("binary")
	add_files("envoy.cpp")
	add_deps("envoy.core")