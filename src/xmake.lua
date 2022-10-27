-- General Requirements
add_requires("fmt", "spdlog", "tbb", "glm", "pacman::cgal")
-- To add boost, python is required
add_requires("boost")

-- Specific Requirements
add_requires("miniply master")

target("envoy.lib")
	set_kind("static")
	add_packages("fmt", "spdlog", "miniply", "glm", { public = true })
	add_packages("tbb", "pacman::cgal", { public = true })
	add_files("mesh.cpp")
	add_includedirs(os.scriptdir(), { public = true })

target("envoy")
	set_kind("binary")
	add_files("envoy.cpp")
	add_deps("envoy.lib")