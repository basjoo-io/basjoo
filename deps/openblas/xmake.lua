package("openblas")
    set_homepage("https://www.openblas.net/")
    set_description("An optimized BLAS library based on GotoBLAS2 1.13 BSD version")
    set_license("BSD-3-Clause")

    add_urls("https://github.com/OpenMathLib/OpenBLAS/releases/download/v$(version)/OpenBLAS-$(version).tar.gz")
    add_versions("0.3.34", "cd7e129868320cc2d033afa920e31202dfe0b8066a5b66661900ccc0f197dfed")

    add_deps("cmake")
    add_deps("ninja")

    add_configs("shared", {description = "Build OpenBLAS as a shared library.", default = false, type = "boolean"})

    set_policy("package.cmake_generator.ninja", true)

    add_includedirs("include", "include/openblas")

    if is_plat("linux", "macosx") then
        add_syslinks("pthread")
    end

    on_load(function (package)
        package:add("links", "openblas")
        if (package:tool("fc") or ""):find("flang", 1, true) then
            package:add("syslinks", "flang_rt.runtime", "FortranDecimal", {public = true})
        else
            package:add("syslinks", "gfortran", "quadmath", {public = true})
        end
    end)

    on_install("linux", function (package)
        import("linux", {rootdir = os.scriptdir()}).install(package)
    end)

    on_install("macosx", function (package)
        import("darwin", {rootdir = os.scriptdir()}).install(package)
    end)

    on_install("windows", function (package)
        import("windows", {rootdir = os.scriptdir()}).install(package)
    end)

    on_test(function (package)
        assert(package:check_csnippets({test = [[
            void test() {
                double A[6] = {1.0, 2.0, 1.0, -3.0, 4.0, -1.0};
                double B[6] = {1.0, 2.0, 1.0, -3.0, 4.0, -1.0};
                double C[9] = {.5, .5, .5, .5, .5, .5, .5, .5, .5};
                cblas_dgemm(CblasColMajor, CblasNoTrans, CblasTrans, 3, 3, 2, 1, A, 3, B, 3, 2, C, 3);
            }
        ]]}, {includes = "cblas.h"}))
    end)
package_end()

add_requires("openblas 0.3.34", {configs = {shared = (get_config("kind") == "shared")}})

target("blas_lapack")
    add_packages("openblas", {public = true})
    add_defines("BASJOO_USE_BLAS_LAPACK", {public = true})
    if is_plat("windows") then
        add_defines("HAVE_LAPACK_CONFIG_H", "LAPACK_COMPLEX_CPP", {public = true})
        add_cxflags("/wd4190", {public = true})
    else
        add_cxflags("-Wno-c99-extensions", {public = true})
    end
target_end()
