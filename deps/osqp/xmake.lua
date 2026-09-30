package("osqp")
    set_homepage("https://osqp.org/")
    set_description("The Operator Splitting QP Solver")
    set_license("Apache-2.0")

    add_urls("https://github.com/osqp/osqp/releases/download/v$(version)/osqp-v$(version)-src.tar.gz")
    add_versions("1.0.0", "ec0bb8fd34625d0ea44274ab3e991aa56e3e360ba30935ae62476557b101c646")

    add_deps("cmake")
    add_deps("ninja")
    add_deps("qdldl 0.1.9", {configs = {shared = is_config("kind", "shared")}})

    add_configs("shared", {description = "Build osqp as a shared library.", default = false, type = "boolean"})

    set_policy("package.cmake_generator.ninja", true)

    add_includedirs("include", "include/osqp")

    on_install(function (package)
        local configs = {
            "-DCMAKE_POLICY_VERSION_MINIMUM=3.10",
            "-DCMAKE_COMPILE_WARNING_AS_ERROR=OFF",
            "-DOSQP_USE_LONG=OFF",
            "-DOSQP_USE_FLOAT=OFF",
            "-DOSQP_BUILD_DEMO_EXE=OFF",
            "-DOSQP_ENABLE_PRINTING=OFF",
            "-DOSQP_ENABLE_PROFILING=OFF",
            "-DOSQP_ENABLE_INTERRUPT=OFF",
            "-DOSQP_BUILD_STATIC_LIB=" .. (package:config("shared") and "OFF" or "ON"),
            "-DOSQP_BUILD_SHARED_LIB=" .. (package:config("shared") and "ON" or "OFF"),
        }

        local qdldl_dep = package:dep("qdldl")
        if qdldl_dep then
            local qdldl_sourcedir = path.join(qdldl_dep:cachedir(), "source")
            if os.isdir(qdldl_sourcedir) then
                table.insert(configs, "-DFETCHCONTENT_SOURCE_DIR_QDLDL=" .. qdldl_sourcedir)
            end
        end

        import("package.tools.cmake").install(package, configs)
    end)

    on_test(function (package)
        assert(package:has_cfuncs("osqp_setup", {includes = "osqp.h"}))
    end)
package_end()

add_requires("osqp 1.0.0", {configs = {shared = is_config("kind", "shared")}})
