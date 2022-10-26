set_project("envoy")
set_languages("c++20")
add_rules("mode.debug", "mode.release")

includes("packages/linalg.lua")
includes("packages/miniply.lua")

add_defines("EVY_TEST_ASSET_PATH=\"" .. path.join(os.projectdir(), "assets") .. "\"")
includes("src")
includes("tests")
