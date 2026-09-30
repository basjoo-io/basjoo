package("zpp_bits")
    set_kind("library", {headeronly = true})
    set_homepage("https://github.com/eyalz800/zpp_bits")
    set_description("A lightweight C++20 serialization and RPC library")
    set_license("MIT")

    add_urls("https://github.com/eyalz800/zpp_bits/archive/refs/tags/v$(version).tar.gz")
    add_versions("4.7.6", "536b40cf921e2a88739668785181f09de1fab4be1b3ca9172f2702fc3700abf9")
    add_patches(
        "4.7.6",
        path.join(os.scriptdir(), "exclude_complex_from_member_autodetect.patch"),
        "a1ec125975ea646579d9da48e603a57283e12ba11389e1ab38a76182f8486a32"
    )

    on_install(function (package)
        os.cp("zpp_bits.h", package:installdir("include"))
    end)

    on_test(function (package)
        assert(package:has_cxxincludes("zpp_bits.h", {configs = {languages = "c++20"}}))
    end)
package_end()

add_requires("zpp_bits 4.7.6")
