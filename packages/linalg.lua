package("linalg")
  set_kind("library", { headeronly = true })
  set_description("linalg.h is a single header, public domain, short vector math library for C++")
  set_urls("https://github.com/sgorsten/linalg.git", { version = "main" })

  on_install(function(package)
    os.cp("*.h", package:installdir("include"))
  end)
package_end()
