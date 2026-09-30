set_project("basjoo")
set_version("0.0.1", {soname = true})
set_description("The Fundamental Math Basis for Autonomous Driving Vehicles and Robotics.")
set_xmakever("3.1.0")

includes("xmake/toolchains/*.lua")

if not get_config("toolchain") then
    if is_host("linux") then
        set_toolchains("linux-gcc-x64")
    elseif is_host("macosx") then
        set_toolchains("darwin-clang-arm64")
    elseif is_host("windows") then
        set_toolchains("windows-msvc-x64")
    end
end

set_allowedplats("linux", "macosx", "windows")
set_allowedarchs("x86_64", "x64", "arm64")

set_defaultmode("release")
add_rules("mode.debug", "mode.release", "mode.releasedbg", "mode.minsizerel")

-- The toolchains already pick -O2 for these modes, but mode.release and mode.releasedbg append
-- -O3 after it unless the target sets an optimization level, and the last one wins. Pin -O2 so
-- that xmake builds what the CMake presets build.
if is_mode("release", "releasedbg") then
    set_optimize("faster")
end

set_languages("c23", "cxx23")
set_encodings("utf-8")
set_warnings("more", "error")

set_policy("build.ccache", true)
set_policy("build.warning", true)
set_policy("package.keep_source", true)

option("basjoo_check_params")
    set_default(false)
    set_showmenu(true)
    set_description("Enable parameters checking")
option_end()

option("basjoo_build_testing")
    set_default(true)
    set_showmenu(true)
    set_description("Enable testing")
option_end()

option("basjoo_enable_install")
    set_default(true)
    set_showmenu(true)
    set_description("Enable install")
option_end()

option("basjoo_use_boost_unordered")
    set_default(true)
    set_showmenu(true)
    set_description("Enable Boost.unordered")
option_end()

option("basjoo_use_blas_lapack")
    set_default("openblas")
    set_showmenu(true)
    set_values("openblas", "netlib", "mkl", "n")
    set_description("BLAS/LAPACK library to use (openblas, netlib, mkl, n)")
option_end()

if is_config("basjoo_check_params", true) then
    add_defines("BASJOO_CHECK_PARAMS=1")
else
    add_defines("BASJOO_CHECK_PARAMS=0")
end

includes("xmake/rules.lua")
includes("deps/xmake.lua")

includes("src/xmake.lua")
if is_config("basjoo_build_testing", true) then
    includes("tests/xmake.lua")
end

add_rules("plugin.compile_commands.autoupdate", {outputdir = os.scriptdir(), lsp = "clangd"})
