package("miniply")
  set_description("A fast and easy-to-use PLY parsing library in a single c++11 header and cpp file")
  set_urls("https://github.com/vilya/miniply.git")

  on_install("linux", function(package)
    local configs = {}
    io.writefile("xmake.lua", [[
      add_rules("mode.release", "mode.debug")
      target("miniply")
        set_kind("$(kind)")
        add_files("miniply.cpp")
        add_headerfiles("miniply.h")
    ]])
    if package:config("shared") then
      configs.kind = "shared"
    end
    import("package.tools.xmake").install(package, configs)
  end)
package_end()