package("pocketfft")
    set_kind("library", {headeronly = true})
    set_homepage("https://github.com/mreineck/pocketfft")
    set_description("FFT implementation based on FFTPack, but with several improvements")
    set_license("BSD-3-Clause")

    add_urls("https://github.com/mreineck/pocketfft.git")
    add_versions("2024.11.30", "0fa0ef591e38c2758e3184c6c23e497b9f732ffa")

    add_defines("POCKETFFT_NO_MULTITHREADING")

    on_install(function (package)
        os.cp("pocketfft_hdronly.h", package:installdir("include"))
    end)

    on_test(function (package)
        assert(package:check_cxxsnippets({test = [[
            #include <pocketfft_hdronly.h>
            void test() {
                pocketfft::shape_t var;
            }
        ]]}, {configs = {languages = "c++11"}}))
    end)
package_end()

add_requires("pocketfft 2024.11.30")
