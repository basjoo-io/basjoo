package("qdldl")
    set_homepage("https://github.com/osqp/qdldl")
    set_description("A free LDL factorisation routine")
    set_license("Apache-2.0")

    add_urls("https://github.com/osqp/qdldl/archive/refs/tags/v$(version).tar.gz")
    add_versions("0.1.9", "7d1285b2db15cf2730dc83b3d16ed28412f558591108cca4f28d4438bf72ceed")

    add_deps("cmake")
    add_deps("ninja")

    add_configs("shared", {description = "Build qdldl as a shared library.", default = false, type = "boolean"})

    set_policy("package.cmake_generator.ninja", true)

    add_includedirs("include/qdldl")

    on_install(function (package)
        import("package.tools.cmake").install(package, {
            "-DCMAKE_POLICY_DEFAULT_CMP0069=NEW",
            "-DCMAKE_COMPILE_WARNING_AS_ERROR=OFF",
            "-DQDLDL_LONG=OFF",
            "-DQDLDL_FLOAT=OFF",
            "-DQDLDL_BUILD_DEMO_EXE=OFF",
            "-DQDLDL_UNITTESTS=OFF",
            "-DQDLDL_BUILD_STATIC_LIB=" .. (package:config("shared") and "OFF" or "ON"),
            "-DQDLDL_BUILD_SHARED_LIB=" .. (package:config("shared") and "ON" or "OFF"),
        })
    end)

    on_test(function (package)
        assert(package:has_cincludes("qdldl.h"))
    end)
package_end()

add_requires("qdldl 0.1.9", {configs = {shared = is_config("kind", "shared")}})
