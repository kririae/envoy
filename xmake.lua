set_project("envoy")
set_languages("c++20")
add_rules("mode.debug", "mode.release", "mode.asan", "mode.tsan", "mode.lsan", "mode.ubsan")
add_vectorexts("avx", "avx2")

includes("packages/*.lua")

add_defines("EVY_TEST_ASSET_PATH=\"" .. path.join(os.projectdir(), "assets") .. "\"")
includes("src")
includes("tests")
