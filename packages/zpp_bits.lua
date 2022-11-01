package("zpp_bits")
  set_kind("library", { headeronly = true })
  set_description("A lightweight C++20 serialization and RPC library")
  set_urls("https://github.com/eyalz800/zpp_bits.git", { version = "v4.4.12" })

  on_install(function(package)
    os.cp("*.h", package:installdir("include"))
  end)
package_end()
