set_project("envoy")
set_languages("c++20")
add_rules("mode.debug", "mode.release")
add_vectorexts("avx", "avx2")
add_vectorexts("sse", "sse2", "sse3", "ssse3")

includes("packages/*.lua")

add_defines("EVY_TEST_ASSET_PATH=\"" .. path.join(os.projectdir(), "assets") .. "\"")
includes("src")
includes("tests")
