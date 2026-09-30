package("stdexec")
    set_kind("library", {headeronly = true})
    set_homepage("https://github.com/NVIDIA/stdexec")
    set_description("`std::execution`, the proposed C++ framework for asynchronous and parallel programming.")
    set_license("Apache-2.0")

    add_urls("https://github.com/NVIDIA/stdexec/archive/e8c349f3f3425b9341306bc56615fc5279a15cf4.tar.gz")
    add_versions("2026.07.06", "b497a9ddbf3dd81633b64297af8f2e84b40022c7d229a23c9b9653fa86bca628")

    add_deps("cmake")
    add_deps("ninja")
    add_deps("boost 1.90.0", {configs = {
        cmake = true,
        asio = true,
        multiprecision = true,
        system = true,
        filesystem = false,
        shared = is_config("kind", "shared")
    }})
    add_deps("taskflow 4.1.0")

    set_policy("package.cmake_generator.ninja", true)

    on_install(function (package)
        if package:has_tool("cxx", "cl") then
            package:add("cxxflags", "/Zc:__cplusplus", "/Zc:preprocessor")
        end

        local configs = {
            "-DSTDEXEC_BUILD_EXAMPLES=OFF",
            "-DSTDEXEC_BUILD_TESTS=OFF",
            "-DSTDEXEC_INSTALL=ON",
            "-DSTDEXEC_ENABLE_TASKFLOW=ON",
            "-DSTDEXEC_ENABLE_ASIO=ON",
            "-DSTDEXEC_ASIO_IMPLEMENTATION=boost",
            "-DBUILD_SHARED_LIBS=" .. (package:config("shared") and "ON" or "OFF"),
        }

        local function depsourcedir(name)
            local dep = package:dep(name)
            if dep then
                local sourcedir = path.join(dep:cachedir(), "source")
                if os.isdir(sourcedir) then
                    return sourcedir
                end
            end
        end
        local boost_sourcedir = depsourcedir("boost")
        if boost_sourcedir then
            table.insert(configs, "-DCPM_Boost_SOURCE=" .. boost_sourcedir)
        end
        local taskflow_sourcedir = depsourcedir("taskflow")
        if taskflow_sourcedir then
            table.insert(configs, "-DCPM_Taskflow_SOURCE=" .. taskflow_sourcedir)
        end

        import("package.tools.cmake").install(package, configs)
    end)

    on_test(function (package)
        assert(package:has_cxxincludes("exec/asio/use_sender.hpp", {configs = {languages = "c++20"}}))
    end)
package_end()

add_requires("stdexec 2026.07.06", {configs = {
    shared = is_config("kind", "shared"),
}})
