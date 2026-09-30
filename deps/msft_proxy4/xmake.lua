package("msft_proxy4")
    set_kind("library", {headeronly = true})
    set_homepage("https://github.com/microsoft/proxy")
    set_description("Proxy: Easy Polymorphism in C++")
    set_license("MIT")

    add_urls("https://github.com/ngcpp/proxy/releases/download/$(version)/proxy-$(version).tgz")
    add_versions("4.1.0", "3b96e426df094fdc1da9b2bfb997c04574fe2ab2d022df834f2be2ab8339ddb5")

    on_install(function (package)
        os.cp("proxy", package:installdir("include"))
    end)

    on_test(function (package)
        assert(package:has_cxxincludes("proxy/proxy.h", {configs = {languages = "c++20"}}))
    end)
package_end()

add_requires("msft_proxy4 4.1.0")
