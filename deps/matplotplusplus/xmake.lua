package("matplotplusplus")
    set_homepage("https://alandefreitas.github.io/matplotplusplus/")
    set_description("A C++ Graphics Library for Data Visualization")
    set_license("MIT")

    add_urls("https://github.com/alandefreitas/matplotplusplus/archive/refs/tags/v$(version).tar.gz")
    add_versions("1.2.2", "c7434b4fea0d0cc3508fd7104fafbb2fa7c824b1d2ccc51c52eaee26fc55a9a0")

    add_deps("cmake")
    add_deps("ninja")
    if is_config("basjoo_use_blas_lapack", "openblas") then
        add_deps("openblas 0.3.34", {configs = {shared = (get_config("kind") == "shared")}})
    end

    add_configs("shared", {description = "Build Matplot++ as a shared library.", default = false, type = "boolean"})

    set_policy("package.cmake_generator.ninja", true)
    add_patches("1.2.2", os.scriptdir() .. "/disable_warning_as_error.patch", "f88ae43afeea778d1edb61f2f13443d06cf7e7725630fe3fb09ec824aab8b744")
    add_patches("1.2.2", os.scriptdir() .. "/deprecate_is_trivial.patch", "9042aebed0b6f1fe37d87f3c8c3895b3d9d180ec2ad09dba307190e46739e47a")
    add_patches("1.2.2", os.scriptdir() .. "/lapack_fallback_to_blas.patch", "5574221acbf7b17268d8987df3cc7b347604433333f666a0d303d2ea354cc7ec")

    add_links("matplot")
    on_load(function (package)
        if not package:config("shared") then
            package:add("links", "nodesoup")
        end
    end)

    on_install(function (package)
        local configs = {
            "-DCMAKE_COMPILE_WARNING_AS_ERROR=OFF",
            "-DMATPLOTPP_BUILD_WITH_PEDANTIC_WARNINGS=OFF",
            "-DMATPLOTPP_BUILD_TESTS=OFF",
            "-DMATPLOTPP_BUILD_EXAMPLES=OFF",
            "-DMATPLOTPP_BUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"),
        }
        local openblas_dep = package:dep("openblas")
        if openblas_dep then
            table.insert(configs, "-DCMAKE_PREFIX_PATH=" .. openblas_dep:installdir())
        end
        import("package.tools.cmake").install(package, configs)
        local nodesoup_lib = path.join(package:installdir("lib"), "Matplot++", is_plat("windows") and "nodesoup.lib" or "libnodesoup.a")
        if os.isfile(nodesoup_lib) then
            os.cp(nodesoup_lib, package:installdir("lib"))
        end
    end)

    on_test(function (package)
        assert(package:has_cxxincludes("matplot/matplot.h", {configs = {languages = "c++17"}}))
    end)
package_end()

add_requires("matplotplusplus 1.2.2", {configs = {shared = (get_config("kind") == "shared")}})
