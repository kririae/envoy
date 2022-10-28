-- General Requirements
add_requires("fmt", "spdlog", "tbb", "glm", "pacman::cgal")
-- To add boost, python is required
add_requires("boost")

-- Specific Requirements
add_requires("miniply master", "linalg main", "xsimd")

target("envoy.core")
	set_kind("static")
	add_packages("fmt", "spdlog", "miniply", "linalg", { public = true })
	add_packages("tbb", "pacman::cgal", "xsimd", { public = true })
	add_files("mesh.cpp")
	add_includedirs(os.scriptdir(), { public = true })

target("envoy")
	set_kind("binary")
	add_files("envoy.cpp")
	add_deps("envoy.core")