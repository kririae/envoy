package("args")
  set_kind("library", { headeronly = true })
  set_description("A simple header-only C++ argument parser library. Supposed to be flexible and powerful, and attempts to be compatible with the functionality of the Python standard argparse library (though not necessarily the API).")
  set_urls("https://github.com/Taywee/args.git", { version = "master" })

  on_install(function(package)
    os.cp("*.hxx", package:installdir("include"))
  end)
package_end()
